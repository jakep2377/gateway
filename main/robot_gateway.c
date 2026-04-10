#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/uart.h"

#include "ra01s.h"

#include "driver/gpio.h"
#include "display.h"

// ---------------- LoRa settings ----------------
#define LORA_FREQ_HZ       915000000
#define LORA_TX_POWER_DBM  22
#define LORA_TCXO_VOLT     3.3f
#define LORA_USE_LDO       true

#define LORA_SF            7
#define LORA_BW            4
#define LORA_CR            1
#define LORA_PREAMBLE      8
#define LORA_PAYLOAD_LEN   0
#define LORA_CRC_ON        true
#define LORA_INVERT_IRQ    false

// ---------------- UART settings ----------------
#define GW_UART            UART_NUM_1
#define GW_UART_BAUD       115200

#define GW_UART_TX_GPIO    6
#define GW_UART_RX_GPIO    7

// UART RX buffering
#define UART_RX_BUF_SIZE   1024
#define UART_LINE_MAX      512
#define LORA_MAX_PAYLOAD   255
#define UART_STREAM_IDLE_FLUSH_MS        120
#define UART_STREAM_SHORT_IDLE_FLUSH_MS  250
#define UART_STREAM_MIN_FLUSH_BYTES      4
#define UART_BINARY_DROP_MAX_BYTES       3
#define UART_BINARY_HEX_MAX_BYTES        24
#define LORA_STREAM_CHUNK_MAX            150
#define LORA_STREAM_HEADER_RESERVE       24
#define LORA_TX_RETRY_COUNT 5
#define LORA_TX_RETRY_DELAY_MS 80
#define GATEWAY_HEARTBEAT_INTERVAL_MS    5000

static const char *TAG = "ROBOT_GW";
static bool display_available = false;
static char s_last_downlink[32] = "none";
static char s_last_uplink[32] = "none";
static uint32_t s_lora_rx_count = 0;
static uint32_t s_uart_forward_count = 0;

typedef struct {
    bool is_stream;
    bool has_end;
    uint32_t seq;
    const char *payload;
} stream_frame_t;

// Simple mutex around LoRa send
static SemaphoreHandle_t lora_tx_lock;

static bool rx_seq_init = false;
static uint32_t rx_seq_expected = 0;
static char rx_stream_buf[UART_LINE_MAX];
static int rx_stream_len = 0;

static void lora_send_text(const char *text);

// ---------------- Helpers ----------------

static char* trim_inplace(char *s) {
    if (!s) return s;

    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;

    size_t n = strlen(s);
    while (n > 0) {
        char c = s[n - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            s[n - 1] = '\0';
            n--;
        } else {
            break;
        }
    }

    return s;
}

static void make_printable(const char *in, size_t in_len, char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
    if (!in || in_len == 0) {
        out[0] = '\0';
        return;
    }

    size_t j = 0;
    for (size_t i = 0; i < in_len && j + 1 < out_size; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c >= 32 && c <= 126) {
            out[j++] = (char)c;
        } else {
            out[j++] = '.';
        }
    }
    out[j] = '\0';
}

static void update_preview_text(const char *input, char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
    if (!input) {
        snprintf(out, out_size, "none");
        return;
    }

    char printable[64];
    make_printable(input, strnlen(input, sizeof(printable) - 1), printable, sizeof(printable));
    char *trimmed = trim_inplace(printable);
    snprintf(out, out_size, "%.*s", (int)((strlen(trimmed) < out_size - 1) ? strlen(trimmed) : out_size - 1), trimmed);
    if (out[0] == '\0') {
        snprintf(out, out_size, "none");
    }
}

static size_t sanitize_uart_payload(const char *in, size_t in_len, char *out, size_t out_size, bool *had_nonprintable)
{
    if (had_nonprintable) *had_nonprintable = false;
    if (!out || out_size == 0) return 0;
    if (!in || in_len == 0) {
        out[0] = '\0';
        return 0;
    }

    size_t j = 0;
    bool saw_nonprintable = false;
    for (size_t i = 0; i < in_len && j + 1 < out_size; i++) {
        unsigned char c = (unsigned char)in[i];

        if (c == '\0' || c == '\r') {
            continue;
        }
        if (c == '\t') {
            c = ' ';
        }

        if (c >= 32 && c <= 126) {
            out[j++] = (char)c;
        } else {
            saw_nonprintable = true;
        }
    }
    out[j] = '\0';

    char *trimmed = trim_inplace(out);
    if (trimmed != out) {
        memmove(out, trimmed, strlen(trimmed) + 1);
    }

    if (had_nonprintable) *had_nonprintable = saw_nonprintable;
    return strlen(out);
}

static size_t format_uart_binary_hex(const uint8_t *in, size_t in_len, char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    if (!in || in_len == 0) {
        out[0] = '\0';
        return 0;
    }

    size_t limit = (in_len < UART_BINARY_HEX_MAX_BYTES) ? in_len : UART_BINARY_HEX_MAX_BYTES;
    size_t j = 0;

    int prefix = snprintf(out, out_size, "HEX:");
    if (prefix < 0 || (size_t)prefix >= out_size) {
        out[0] = '\0';
        return 0;
    }
    j = (size_t)prefix;

    for (size_t i = 0; i < limit && j + 2 < out_size; i++) {
        int w = snprintf(out + j, out_size - j, "%02X", in[i]);
        if (w != 2) break;
        j += 2;
    }

    if (limit < in_len && j + 4 < out_size) {
        memcpy(out + j, "...", 4);
        j += 3;
    }

    return j;
}

static void forward_uart_payload_to_lora(const char *raw, int raw_len, bool use_stream_frame,
                                         bool final_chunk, uint32_t *stream_seq, const char *reason)
{
    if (!raw || raw_len <= 0) return;

    char cleaned[UART_LINE_MAX];
    bool had_nonprintable = false;
    size_t clean_len = sanitize_uart_payload(raw, (size_t)raw_len, cleaned, sizeof(cleaned), &had_nonprintable);

    if (clean_len == 0) {
        if (had_nonprintable) {
            if (raw_len <= UART_BINARY_DROP_MAX_BYTES) {
                ESP_LOGI(TAG, "Ignoring tiny binary UART %s (%d raw bytes)", reason, raw_len);
                return;
            }

            char hex_msg[LORA_MAX_PAYLOAD + 1];
            size_t hex_len = format_uart_binary_hex((const uint8_t *)raw, (size_t)raw_len,
                                                    hex_msg, sizeof(hex_msg));
            if (hex_len > 0) {
                ESP_LOGI(TAG, "UART %s binary (%d raw bytes)", reason, raw_len);
                lora_send_text(hex_msg);
            }
        }
        return;
    }

    if (!use_stream_frame && clean_len <= LORA_MAX_PAYLOAD) {
        ESP_LOGI(TAG, "UART %s direct (%u bytes%s)",
                 reason, (unsigned)clean_len, had_nonprintable ? ", sanitized" : "");
        update_preview_text(cleaned, s_last_uplink, sizeof(s_last_uplink));
        s_uart_forward_count++;
        lora_send_text(cleaned);
        return;
    }

    char framed[LORA_MAX_PAYLOAD + 1];
    int w = snprintf(framed, sizeof(framed), "S:%lu:%c:%.*s\n",
                     (unsigned long)(*stream_seq)++,
                     final_chunk ? 'E' : 'M',
                     (int)clean_len, cleaned);
    if (w > 0) {
        ESP_LOGI(TAG, "UART %s seq=%lu (%u bytes%s)",
                 reason,
                 (unsigned long)(*stream_seq - 1),
                 (unsigned)clean_len,
                 had_nonprintable ? ", sanitized" : "");
        update_preview_text(cleaned, s_last_uplink, sizeof(s_last_uplink));
        s_uart_forward_count++;
        lora_send_text(framed);
    }
}

static stream_frame_t parse_stream_frame(char *text)
{
    stream_frame_t out = {
        .is_stream = false,
        .has_end = true,
        .seq = 0,
        .payload = text
    };

    if (!text || strncmp(text, "S:", 2) != 0) {
        return out;
    }

    char *p = text + 2;
    char *endptr = NULL;
    unsigned long seq = strtoul(p, &endptr, 10);
    if (endptr == p || !endptr || *endptr != ':') {
        return out;
    }

    out.is_stream = true;
    out.seq = (uint32_t)seq;

    char *payload = endptr + 1;

    // New format: S:<seq>:<M|E>:<payload>
    if ((payload[0] == 'M' || payload[0] == 'E') && payload[1] == ':') {
        out.has_end = (payload[0] == 'E');
        out.payload = payload + 2;
    } else {
        // Backward compatibility: S:<seq>:<payload> treated as end-of-line chunk
        out.has_end = true;
        out.payload = payload;
    }

    return out;
}

static void validate_pin_map(void) {
    if (GW_UART_TX_GPIO == GW_UART_RX_GPIO) {
        ESP_LOGE(TAG, "Invalid UART pin map: TX and RX are both GPIO%d", GW_UART_TX_GPIO);
    }

    const int lora_pins[] = {
        CONFIG_MISO_GPIO,
        CONFIG_MOSI_GPIO,
        CONFIG_SCLK_GPIO,
        CONFIG_NSS_GPIO,
        CONFIG_RST_GPIO,
        CONFIG_BUSY_GPIO
    };

    for (size_t i = 0; i < (sizeof(lora_pins) / sizeof(lora_pins[0])); i++) {
        if (GW_UART_TX_GPIO == lora_pins[i] || GW_UART_RX_GPIO == lora_pins[i]) {
            ESP_LOGW(TAG, "Potential pin conflict: UART pin overlaps LoRa SPI/control GPIO %d", lora_pins[i]);
        }
    }
}

static void lora_setup(void) {
    ESP_LOGI(TAG, "LoRaInit...");
    LoRaInit();

    int rc = LoRaBegin(LORA_FREQ_HZ, LORA_TX_POWER_DBM, LORA_TCXO_VOLT, LORA_USE_LDO);
    if (rc != 0) {
        ESP_LOGE(TAG, "LoRaBegin failed (%d). Check SX1262 pins/TCXO/freq.", rc);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    LoRaConfig(LORA_SF, LORA_BW, LORA_CR,
               LORA_PREAMBLE, LORA_PAYLOAD_LEN, LORA_CRC_ON, LORA_INVERT_IRQ);

    ESP_LOGI(TAG, "LoRa ready @ %d Hz", (int)LORA_FREQ_HZ);
}

static void uart_setup(void) {
    validate_pin_map();

    uart_config_t cfg = {
        .baud_rate = GW_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(GW_UART, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GW_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GW_UART, GW_UART_TX_GPIO, GW_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(gpio_pullup_en(GW_UART_RX_GPIO));
    ESP_ERROR_CHECK(gpio_pulldown_dis(GW_UART_RX_GPIO));

    ESP_LOGI(TAG, "UART ready: UART%d TX=%d RX=%d @ %d",
             (int)GW_UART, GW_UART_TX_GPIO, GW_UART_RX_GPIO, GW_UART_BAUD);
}

// Send a LoRa packet safely from any task
static void lora_send_text(const char *text) {
    if (!text) return;
    size_t n = strnlen(text, UART_LINE_MAX - 1);
    if (n == 0) return;

    if (n > LORA_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "LoRa payload too long (%u), truncating to %u", (unsigned)n, (unsigned)LORA_MAX_PAYLOAD);
        n = LORA_MAX_PAYLOAD;
    }

    xSemaphoreTake(lora_tx_lock, portMAX_DELAY);
    bool sent = false;
    for (int retry = 0; retry < LORA_TX_RETRY_COUNT; retry++) {
        bool send_ok = LoRaSend((uint8_t*)text, (uint8_t)n, SX126x_TXMODE_SYNC);
        if (send_ok) {
            sent = true;
            break;
        }
        ESP_LOGW(TAG, "LoRa uplink retry %d/%d failed", retry + 1, LORA_TX_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(LORA_TX_RETRY_DELAY_MS));
    }
    xSemaphoreGive(lora_tx_lock);

    if (!sent) {
        ESP_LOGE(TAG, "LoRa uplink failed after %d attempts", LORA_TX_RETRY_COUNT);
    }
}

// ---------------- Tasks ----------------

// LoRa RX -> UART TX + ACK
static void lora_rx_task(void *arg) {
    (void)arg;
    uint8_t rx[255];
    char rx_log[255];

    while (1) {
        uint8_t n = LoRaReceive(rx, sizeof(rx));
        if (n > 0) {
            // Ensure it's printable as a C string
            if (n >= sizeof(rx)) n = sizeof(rx) - 1;
            rx[n] = '\0';
            make_printable((const char *)rx, n, rx_log, sizeof(rx_log));
            update_preview_text(rx_log, s_last_downlink, sizeof(s_last_downlink));
            s_lora_rx_count++;
            ESP_LOGI(TAG, "LoRa RX (%d): %s", n, rx_log);

            // Forward to STM32 as a line-based command
            char line[UART_LINE_MAX];
            char rx_copy[UART_LINE_MAX];
            strncpy(rx_copy, (char*)rx, sizeof(rx_copy) - 1);
            rx_copy[sizeof(rx_copy) - 1] = '\0';
            char *cmd = trim_inplace(rx_copy);

            stream_frame_t frame = parse_stream_frame(cmd);
            const char *uart_payload = frame.payload ? frame.payload : "";

            if (frame.is_stream) {
                if (!rx_seq_init) {
                    rx_seq_expected = frame.seq;
                    rx_seq_init = true;
                } else {
                    if (frame.seq != rx_seq_expected) {
                        rx_stream_len = 0;
                        rx_seq_expected = frame.seq;
                    }
                }

                size_t payload_len = strlen(uart_payload);
                if (payload_len > 0 && uart_payload[payload_len - 1] == '\n') {
                    payload_len--;
                }

                int remain = (int)sizeof(rx_stream_buf) - 1 - rx_stream_len;
                if (remain > 0 && payload_len > 0) {
                    size_t copy_len = payload_len > (size_t)remain ? (size_t)remain : payload_len;
                    memcpy(&rx_stream_buf[rx_stream_len], uart_payload, copy_len);
                    rx_stream_len += (int)copy_len;
                    rx_stream_buf[rx_stream_len] = '\0';
                }

                rx_seq_expected = frame.seq + 1;

                if (!frame.has_end) {
                    char ack[80];
                    snprintf(ack, sizeof(ack), "GWRX:%.69s", (char*)rx);
                    lora_send_text(ack);
                    continue;
                }

                int w = snprintf(line, sizeof(line), "%s\r\n", rx_stream_buf);
                if (w > 0) {
                    uart_write_bytes(GW_UART, line, w);
                    ESP_LOGI(TAG, "UART TX -> STM32: %s", line);
                }
                rx_stream_len = 0;
                rx_stream_buf[0] = '\0';
            } else {
                // Pass through legacy/non-stream payload as one line
                int w = snprintf(line, sizeof(line), "%s\r\n", uart_payload);
                if (w > 0) {
                    uart_write_bytes(GW_UART, line, w);
                    ESP_LOGI(TAG, "UART TX -> STM32: %s", line);
                }
            }

            // Send a gateway receipt marker back over LoRa without pretending the
            // STM32 already applied the command. The real `ACK:` should come from
            // the STM32 response that is bridged back on UART RX.
            char ack[80];
            snprintf(ack, sizeof(ack), "GWRX:%.69s", (char*)rx);
            lora_send_text(ack);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Periodic gateway heartbeat -> LoRa TX
static void gateway_heartbeat_task(void *arg) {
    (void)arg;
    char heartbeat[64];

    vTaskDelay(pdMS_TO_TICKS(1500));
    while (1) {
        snprintf(heartbeat, sizeof(heartbeat), "GW:ALIVE:%lu",
                 (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        lora_send_text(heartbeat);
        vTaskDelay(pdMS_TO_TICKS(GATEWAY_HEARTBEAT_INTERVAL_MS));
    }
}

static void gateway_display_task(void *arg) {
    (void)arg;

    while (1) {
        if (display_available) {
            display_show_gateway_status(
                "READY",
                "ONLINE",
                "BRIDGE",
                s_last_downlink,
                s_last_uplink,
                s_lora_rx_count,
                s_uart_forward_count);
        }
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

// UART RX -> LoRa TX
static void uart_rx_task(void *arg) {
    (void)arg;
    uint8_t buf[UART_RX_BUF_SIZE];
    char line[UART_LINE_MAX];
    int line_len = 0;
    bool line_chunked = false;
    uint32_t stream_seq = 0;
    const int stream_payload_max = (LORA_MAX_PAYLOAD > LORA_STREAM_HEADER_RESERVE)
        ? (LORA_MAX_PAYLOAD - LORA_STREAM_HEADER_RESERVE)
        : LORA_MAX_PAYLOAD;
    const int chunk_max = (stream_payload_max < LORA_STREAM_CHUNK_MAX)
        ? stream_payload_max
        : LORA_STREAM_CHUNK_MAX;
    TickType_t last_byte_tick = 0;

    while (1) {
        int r = uart_read_bytes(GW_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (r > 0) {
            for (int i = 0; i < r; i++) {
                char c = (char)buf[i];

                // Build stream/chunks; newline flushes the current chunk.
                if (c == '\r' || c == '\0') continue;

                if (c == '\n') {
                    if (line_len > 0) {
                        forward_uart_payload_to_lora(line, line_len, line_chunked, true,
                                                     &stream_seq, line_chunked ? "RX final chunk" : "RX");
                    }
                    line_len = 0;
                    line_chunked = false;
                    last_byte_tick = 0;
                } else {
                    if (line_len < chunk_max) {
                        line[line_len++] = c;
                        last_byte_tick = xTaskGetTickCount();
                    } else {
                        // Stream is longer than one LoRa packet: send current chunk and continue.
                        forward_uart_payload_to_lora(line, line_len, true, false,
                                                     &stream_seq, "stream chunk");
                        line_chunked = true;
                        line_len = 0;
                        line[line_len++] = c;
                        last_byte_tick = xTaskGetTickCount();
                    }
                }
            }
        }

        if (line_len > 0 && last_byte_tick != 0) {
            TickType_t now = xTaskGetTickCount();
            TickType_t idle_flush_ticks = pdMS_TO_TICKS(
                (line_len < UART_STREAM_MIN_FLUSH_BYTES)
                    ? UART_STREAM_SHORT_IDLE_FLUSH_MS
                    : UART_STREAM_IDLE_FLUSH_MS);

            if ((now - last_byte_tick) >= idle_flush_ticks) {
                forward_uart_payload_to_lora(line, line_len, line_chunked, true,
                                             &stream_seq, "idle flush");
                line_len = 0;
                line_chunked = false;
                last_byte_tick = 0;
            }
        }
    }
}

void app_main(void) {
    lora_tx_lock = xSemaphoreCreateMutex();

    uart_setup();
    lora_setup();
    display_available = display_init();

    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);
    xTaskCreate(gateway_heartbeat_task, "gw_heartbeat", 2048, NULL, 3, NULL);
    if (display_available) {
        xTaskCreate(gateway_display_task, "gw_display", 3072, NULL, 2, NULL);
    }

    ESP_LOGI(TAG, "Robot gateway running: LoRa <-> UART bridge active");
}