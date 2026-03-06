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
#define UART_STREAM_IDLE_FLUSH_MS  30
#define LORA_STREAM_CHUNK_MAX 150

static const char *TAG = "ROBOT_GW";

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
    LoRaSend((uint8_t*)text, (uint8_t)n, SX126x_TXMODE_SYNC);
    xSemaphoreGive(lora_tx_lock);
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
                    snprintf(ack, sizeof(ack), "ACK:%.70s", (char*)rx);
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

            // Send ACK back over LoRa so base station can confirm delivery
            char ack[80];
            snprintf(ack, sizeof(ack), "ACK:%.70s", (char*)rx);
            lora_send_text(ack);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// UART RX -> LoRa TX
static void uart_rx_task(void *arg) {
    (void)arg;
    uint8_t buf[UART_RX_BUF_SIZE];
    char line[UART_LINE_MAX];
    char framed[LORA_MAX_PAYLOAD + 1];
    int line_len = 0;
    uint32_t stream_seq = 0;
    const int chunk_max = LORA_STREAM_CHUNK_MAX;
    TickType_t last_byte_tick = 0;

    while (1) {
        int r = uart_read_bytes(GW_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (r > 0) {
            for (int i = 0; i < r; i++) {
                char c = (char)buf[i];

                // Build stream/chunks; newline flushes the current chunk.
                if (c == '\r') continue;

                if (c == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0) {
                        int w = snprintf(framed, sizeof(framed), "S:%lu:E:%.*s\n",
                                         (unsigned long)stream_seq++, line_len, line);
                        if (w > 0) {
                            ESP_LOGI(TAG, "UART RX chunk seq=%lu (%d bytes)",
                                     (unsigned long)(stream_seq - 1), line_len);
                            lora_send_text(framed);
                        }
                    }
                    line_len = 0;
                } else {
                    if (line_len < chunk_max) {
                        line[line_len++] = c;
                        last_byte_tick = xTaskGetTickCount();
                    } else {
                        // Stream is longer than one LoRa packet: send current chunk and continue.
                        line[line_len] = '\0';
                        int w = snprintf(framed, sizeof(framed), "S:%lu:M:%.*s\n",
                                         (unsigned long)stream_seq++, line_len, line);
                        if (w > 0) {
                            ESP_LOGI(TAG, "UART stream chunk seq=%lu (%d bytes)",
                                     (unsigned long)(stream_seq - 1), line_len);
                            lora_send_text(framed);
                        }

                        line_len = 0;
                        line[line_len++] = c;
                        last_byte_tick = xTaskGetTickCount();
                    }
                }
            }
        }

        if (line_len > 0 && last_byte_tick != 0) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_byte_tick) >= pdMS_TO_TICKS(UART_STREAM_IDLE_FLUSH_MS)) {
                line[line_len] = '\0';
                int w = snprintf(framed, sizeof(framed), "S:%lu:E:%.*s\n",
                                 (unsigned long)stream_seq++, line_len, line);
                if (w > 0) {
                    ESP_LOGI(TAG, "UART idle flush seq=%lu (%d bytes)",
                             (unsigned long)(stream_seq - 1), line_len);
                    lora_send_text(framed);
                }
                line_len = 0;
                last_byte_tick = 0;
            }
        }
    }
}

void app_main(void) {
    lora_tx_lock = xSemaphoreCreateMutex();

    uart_setup();
    lora_setup();

    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Robot gateway running: LoRa <-> UART bridge active");
}