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
#include "esp_idf_version.h"

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define OLED_VEXT 36
#define OLED_I2C_PORT I2C_NUM_0
#define OLED_TEXT_LINE_LEN 16
#define OLED_PAGE_SWITCH_MS 7000
#define OLED_SCROLL_STEP_MS 350

static SSD1306_t dev;
static const char *TAG = "GW_DISPLAY";

static void write_line_ex(int row, const char *text, char *cache, size_t cache_size, bool invert) {
    char buffer[OLED_TEXT_LINE_LEN + 1];
    snprintf(buffer, sizeof(buffer), "%-16.16s", text ? text : "");
    if (!invert && strncmp(buffer, cache, cache_size) == 0) {
        return;
    }
    ssd1306_display_text(&dev, row, buffer, strlen(buffer), invert);
    snprintf(cache, cache_size, "%s", buffer);
}

static void write_line(int row, const char *text, char *cache, size_t cache_size) {
    write_line_ex(row, text, cache, cache_size, false);
}

static const char *display_value_start(const char *input) {
    static const char empty[] = "none";

    if (!input || !input[0]) {
        return empty;
    }

    const char *start = input;
    if (strncmp(start, "CMD:", 4) == 0 ||
        strncmp(start, "ACK:", 4) == 0 ||
        strncmp(start, "GWRX:", 5) == 0 ||
        strncmp(start, "GWTX:", 5) == 0) {
        const char *prefix_end = strchr(start, ':');
        start = prefix_end ? prefix_end + 1 : start;
        while (*start == ':' || *start == ' ') {
            start++;
        }
    }

    return *start ? start : empty;
}

static void compact_token_label(const char *input, char *out, size_t out_size) {
    if (!out || out_size == 0) {
        return;
    }

    out[0] = '\0';
    const char *start = display_value_start(input);

    size_t len = 0;
    while (start[len] && start[len] != ',' && start[len] != ' ' && start[len] != '\r' && start[len] != '\n' && len < out_size - 1) {
        len++;
    }

    snprintf(out, out_size, "%.*s", (int)len, start);
    if (out[0] == '\0') {
        snprintf(out, out_size, "none");
    }
}

static void copy_windowed_text(const char *input, char *out, size_t out_size, size_t offset) {
    if (!out || out_size == 0) {
        return;
    }

    out[0] = '\0';
    const char *start = display_value_start(input);
    size_t width = out_size - 1;
    size_t len = strlen(start);

    if (len <= width) {
        snprintf(out, out_size, "%s", start);
        return;
    }

    size_t max_start = len - width;
    size_t begin = offset % (max_start + 1);
    snprintf(out, out_size, "%.*s", (int)width, start + begin);
}

static void compact_state_label(const char *input, char *out, size_t out_size) {
    char token[16];
    compact_token_label(input, token, sizeof(token));

    char lower[16];
    size_t i = 0;
    for (; token[i] && i < sizeof(lower) - 1; ++i) {
        lower[i] = (char)tolower((unsigned char)token[i]);
    }
    lower[i] = '\0';

    if (strcmp(lower, "connected") == 0 || strcmp(lower, "online") == 0 || strcmp(lower, "ready") == 0 || strcmp(lower, "ok") == 0) {
        snprintf(out, out_size, "OK");
    } else if (strstr(lower, "bridge") || strstr(lower, "link") || strstr(lower, "forward")) {
        snprintf(out, out_size, "LINK");
    } else if (strstr(lower, "setup") || strstr(lower, "config") || strcmp(lower, "ap") == 0) {
        snprintf(out, out_size, "SETUP");
    } else if (strstr(lower, "degrad") || strstr(lower, "stale") || strstr(lower, "warn")) {
        snprintf(out, out_size, "WARN");
    } else if (strcmp(lower, "none") == 0 || strstr(lower, "off") != NULL || strstr(lower, "disconn") != NULL) {
        snprintf(out, out_size, "OFF");
    } else if (strstr(lower, "idle") != NULL) {
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

static bool state_needs_alert(const char *label) {
    return label && (strcmp(label, "WARN") == 0 || strcmp(label, "OFF") == 0);
}

static bool oled_probe(void) {
    const uint8_t addresses[] = {0x3C, 0x3D};

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (dev._i2c_bus_handle == NULL) {
        ESP_LOGW(TAG, "OLED probe skipped because the I2C bus handle is not ready");
        return false;
    }

    for (size_t i = 0; i < sizeof(addresses); ++i) {
        esp_err_t ret = i2c_master_probe(dev._i2c_bus_handle, addresses[i], 50);
        if (ret == ESP_OK) {
            dev._address = addresses[i];
            ESP_LOGI(TAG, "OLED found at address 0x%02X", addresses[i]);
            return true;
        }
    }
#else
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
#endif

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
    static int last_page = -1;
    char line[OLED_TEXT_LINE_LEN + 1];
    char detail[OLED_TEXT_LINE_LEN + 1];
    char mode_label[12];
    char lora_label[12];
    char uart_label[12];
    char downlink_label[14];
    char uplink_label[14];
    TickType_t now = xTaskGetTickCount();
    size_t scroll_step = (size_t)(now / pdMS_TO_TICKS(OLED_SCROLL_STEP_MS));

    compact_state_label(mode, mode_label, sizeof(mode_label));
    compact_state_label(lora_status, lora_label, sizeof(lora_label));
    compact_state_label(uart_status, uart_label, sizeof(uart_label));
    compact_token_label(last_downlink, downlink_label, sizeof(downlink_label));
    compact_token_label(last_uplink, uplink_label, sizeof(uplink_label));

    bool show_alert = state_needs_alert(lora_label) || state_needs_alert(uart_label);
    int page_count = show_alert ? 3 : 2;
    int page = (int)((now / pdMS_TO_TICKS(OLED_PAGE_SWITCH_MS)) % page_count);

    if (page != last_page) {
        memset(cache, 0, sizeof(cache));
        ssd1306_clear_screen(&dev, false);
        last_page = page;
    }

    if (page == 0) {
        write_line(0, "GATEWAY P1", cache[0], sizeof(cache[0]));

        snprintf(line, sizeof(line), "MODE:%-.10s", mode_label);
        write_line(1, line, cache[1], sizeof(cache[1]));

        snprintf(line, sizeof(line), "L:%-.4s U:%-.4s", lora_label, uart_label);
        write_line(2, line, cache[2], sizeof(cache[2]));

        write_line(3, "BASE->ROBOT", cache[3], sizeof(cache[3]));
        copy_windowed_text(last_downlink, detail, sizeof(detail), scroll_step);
        write_line(4, detail, cache[4], sizeof(cache[4]));

        write_line(5, "ROBOT->BASE", cache[5], sizeof(cache[5]));
        copy_windowed_text(last_uplink, detail, sizeof(detail), scroll_step + 5);
        write_line(6, detail, cache[6], sizeof(cache[6]));

        snprintf(line, sizeof(line), "RX:%02lu TX:%02lu",
                 (unsigned long)(lora_rx_count % 100),
                 (unsigned long)(uart_forward_count % 100));
        write_line(7, line, cache[7], sizeof(cache[7]));
    } else if (page == 1) {
        write_line(0, "GW DETAIL P2", cache[0], sizeof(cache[0]));

        write_line(1, "DOWNLINK", cache[1], sizeof(cache[1]));
        copy_windowed_text(last_downlink, detail, sizeof(detail), scroll_step + 9);
        write_line(2, detail, cache[2], sizeof(cache[2]));

        write_line(3, "UPLINK", cache[3], sizeof(cache[3]));
        copy_windowed_text(last_uplink, detail, sizeof(detail), scroll_step + 13);
        write_line(4, detail, cache[4], sizeof(cache[4]));

        write_line(5, "SHORT STATUS", cache[5], sizeof(cache[5]));
        snprintf(line, sizeof(line), "M:%.4s L:%.4s", mode_label, lora_label);
        write_line(6, line, cache[6], sizeof(cache[6]));
        snprintf(line, sizeof(line), "UART:%.8s", uart_label);
        write_line(7, line, cache[7], sizeof(cache[7]));
    } else {
        write_line_ex(0, "!!! ALERT !!!", cache[0], sizeof(cache[0]), true);

        snprintf(line, sizeof(line), "LORA:%-.10s", lora_label);
        write_line_ex(1, line, cache[1], sizeof(cache[1]), true);

        snprintf(line, sizeof(line), "UART:%-.10s", uart_label);
        write_line_ex(2, line, cache[2], sizeof(cache[2]), true);

        snprintf(line, sizeof(line), "RX:%02lu TX:%02lu",
                 (unsigned long)(lora_rx_count % 100),
                 (unsigned long)(uart_forward_count % 100));
        write_line_ex(3, line, cache[3], sizeof(cache[3]), true);

        snprintf(line, sizeof(line), "DOWN:%-.11s", downlink_label);
        write_line_ex(4, line, cache[4], sizeof(cache[4]), true);

        snprintf(line, sizeof(line), "UP:%-.13s", uplink_label);
        write_line_ex(5, line, cache[5], sizeof(cache[5]), true);

        write_line_ex(6, "CHECK LINKS", cache[6], sizeof(cache[6]), true);
        write_line_ex(7, "AND POWER", cache[7], sizeof(cache[7]), true);
    }
}
