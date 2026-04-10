#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "ssd1306.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define OLED_VEXT 36
#define OLED_I2C_PORT I2C_NUM_0
#define OLED_TEXT_LINE_LEN 21

static SSD1306_t dev;
static const char *TAG = "GW_DISPLAY";

static void write_line(int row, const char *text, char *cache, size_t cache_size) {
    char buffer[OLED_TEXT_LINE_LEN + 1];
    snprintf(buffer, sizeof(buffer), "%-21.21s", text ? text : "");
    if (strncmp(buffer, cache, cache_size) == 0) {
        return;
    }
    ssd1306_display_text(&dev, row, buffer, strlen(buffer), false);
    snprintf(cache, cache_size, "%s", buffer);
}

static void compact_text(const char *input, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    out[0] = '\0';
    if (!input || !input[0]) {
        snprintf(out, out_size, "none");
        return;
    }

    const char *start = input;
    if (strncmp(start, "CMD:", 4) == 0 || strncmp(start, "ACK:", 4) == 0 || strncmp(start, "GWRX:", 5) == 0) {
        const char *prefix_end = strchr(start, ':');
        start = prefix_end ? prefix_end + 1 : start;
        while (*start == ':') start++;
    }
    while (*start == ' ') start++;

    size_t len = 0;
    while (start[len] && start[len] != '\r' && start[len] != '\n' && len < out_size - 1) {
        len++;
    }
    snprintf(out, out_size, "%.*s", (int)len, start);
}

static void compact_state_label(const char *input, char *out, size_t out_size) {
    char token[16];
    compact_text(input, token, sizeof(token));

    char lower[16];
    size_t i = 0;
    for (; token[i] && i < sizeof(lower) - 1; ++i) {
        lower[i] = (char)tolower((unsigned char)token[i]);
    }
    lower[i] = '\0';

    if (strcmp(lower, "connected") == 0 || strcmp(lower, "online") == 0 || strcmp(lower, "ready") == 0 || strcmp(lower, "ok") == 0) {
        snprintf(out, out_size, "READY");
    } else if (strstr(lower, "degrad") || strstr(lower, "stale") || strstr(lower, "warn")) {
        snprintf(out, out_size, "WARN");
    } else if (strcmp(lower, "none") == 0 || strstr(lower, "off") != NULL || strstr(lower, "disconn") != NULL) {
        snprintf(out, out_size, "OFF");
    } else if (strstr(lower, "idle")) {
        snprintf(out, out_size, "IDLE");
    } else {
        size_t j = 0;
        for (; token[j] && j < out_size - 1; ++j) {
            out[j] = (char)toupper((unsigned char)token[j]);
        }
        out[j] = '\0';
        if (out[0] == '\0') {
            snprintf(out, out_size, "--");
        }
    }
}

static bool oled_probe(void) {
    const uint8_t addresses[] = {0x3C, 0x3D};
    for (size_t i = 0; i < sizeof(addresses); ++i) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            dev._address = addresses[i];
            ESP_LOGI(TAG, "OLED found at address 0x%02X", addresses[i]);
            return true;
        }
    }
    ESP_LOGW(TAG, "OLED probe failed at 0x3C and 0x3D");
    return false;
}

bool display_init(void) {
    gpio_config_t vext_conf = {
        .pin_bit_mask = 1ULL << OLED_VEXT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&vext_conf));
    ESP_ERROR_CHECK(gpio_set_level(OLED_VEXT, 0));
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_config_t rst_conf = {
        .pin_bit_mask = 1ULL << OLED_RST,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_conf));
    ESP_ERROR_CHECK(gpio_set_level(OLED_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(gpio_set_level(OLED_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_master_init(&dev, OLED_SDA, OLED_SCL, -1);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!oled_probe()) {
        return false;
    }

    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "NAVIGATOR GW", 12, false);
    ssd1306_display_text(&dev, 4, "PLEASE WAIT", 11, false);
    vTaskDelay(pdMS_TO_TICKS(220));
    ssd1306_clear_screen(&dev, false);
    ESP_LOGI(TAG, "Gateway display initialized");
    return true;
}

void display_show_gateway_status(const char *mode,
                                 const char *lora_status,
                                 const char *uart_status,
                                 const char *last_downlink,
                                 const char *last_uplink,
                                 uint32_t lora_rx_count,
                                 uint32_t uart_forward_count) {
    static char cache[8][OLED_TEXT_LINE_LEN + 1] = {{0}};
    char line[OLED_TEXT_LINE_LEN + 1];
    char mode_text[12];
    char lora_text[12];
    char uart_text[12];
    char downlink_text[22];
    char uplink_text[22];

    compact_state_label(mode, mode_text, sizeof(mode_text));
    compact_state_label(lora_status, lora_text, sizeof(lora_text));
    compact_state_label(uart_status, uart_text, sizeof(uart_text));
    compact_text(last_downlink, downlink_text, sizeof(downlink_text));
    compact_text(last_uplink, uplink_text, sizeof(uplink_text));

    write_line(0, "NAVIGATOR GW", cache[0], sizeof(cache[0]));

    snprintf(line, sizeof(line), "STATE:%-.10s", mode_text);
    write_line(1, line, cache[1], sizeof(cache[1]));

    snprintf(line, sizeof(line), "LORA:%-.5s RX:%02lu", lora_text, (unsigned long)(lora_rx_count % 100));
    write_line(2, line, cache[2], sizeof(cache[2]));

    snprintf(line, sizeof(line), "UART:%-.5s TX:%02lu", uart_text, (unsigned long)(uart_forward_count % 100));
    write_line(3, line, cache[3], sizeof(cache[3]));

    write_line(4, "BASE -> ROBOT", cache[4], sizeof(cache[4]));
    write_line(5, downlink_text, cache[5], sizeof(cache[5]));
    write_line(6, "ROBOT -> BASE", cache[6], sizeof(cache[6]));
    write_line(7, uplink_text, cache[7], sizeof(cache[7]));
}
