#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/uart.h"

#include "ra01s.h"

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
#define UART_LINE_MAX      256

static const char *TAG = "ROBOT_GW";

// Simple mutex around LoRa send
static SemaphoreHandle_t lora_tx_lock;

// ---------------- Helpers ----------------

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

    ESP_LOGI(TAG, "UART ready: UART%d TX=%d RX=%d @ %d",
             (int)GW_UART, GW_UART_TX_GPIO, GW_UART_RX_GPIO, GW_UART_BAUD);
}

// Send a LoRa packet safely from any task
static void lora_send_text(const char *text) {
    if (!text) return;
    size_t n = strnlen(text, 240);
    if (n == 0) return;

    xSemaphoreTake(lora_tx_lock, portMAX_DELAY);
    LoRaSend((uint8_t*)text, (uint8_t)n, SX126x_TXMODE_SYNC);
    xSemaphoreGive(lora_tx_lock);
}

// ---------------- Tasks ----------------

// LoRa RX -> UART TX + ACK
static void lora_rx_task(void *arg) {
    (void)arg;
    uint8_t rx[255];

    while (1) {
        uint8_t n = LoRaReceive(rx, sizeof(rx));
        if (n > 0) {
            // Ensure it's printable as a C string
            if (n >= sizeof(rx)) n = sizeof(rx) - 1;
            rx[n] = '\0';

            ESP_LOGI(TAG, "LoRa RX (%d): %s", n, (char*)rx);

            // Forward to STM32 as a line-based command
            char line[UART_LINE_MAX];
            int w = snprintf(line, sizeof(line), "CMD:%s\n", (char*)rx);
            if (w > 0) {
                uart_write_bytes(GW_UART, line, w);
                ESP_LOGI(TAG, "UART TX -> STM32: %s", line);
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
    int line_len = 0;

    while (1) {
        int r = uart_read_bytes(GW_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (r > 0) {
            for (int i = 0; i < r; i++) {
                char c = (char)buf[i];

                // Build line until newline
                if (c == '\r') continue;

                if (c == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0) {
                        ESP_LOGI(TAG, "UART RX <- STM32: %s", line);
                        lora_send_text(line);
                    }
                    line_len = 0;
                } else {
                    if (line_len < (UART_LINE_MAX - 1)) {
                        line[line_len++] = c;
                    } else {
                        // Overflow: reset
                        line_len = 0;
                    }
                }
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