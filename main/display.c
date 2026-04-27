/* display.c — SSD1306 OLED rendering for the robot gateway.
 *
 * Drives a 128×64 SSD1306 panel over I2C using the esp-idf-ssd1306 component.
 * The panel is treated as an 8-row × 16-character text grid (8×8 px font).
 *
 * All rendering is done through display_show_gateway_status(), which is meant
 * to be called from a low-priority task at ~700 ms intervals.  The function
 * manages its own dirty-line cache, page switching, and long-text scrolling
 * without requiring any caller-side state.
 */

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

/* --- Hardware pin assignments (Heltec WiFi LoRa 32 V3 layout) --- */
#define OLED_SDA 17          /* I2C data line to SSD1306 */
#define OLED_SCL 18          /* I2C clock line to SSD1306 */
#define OLED_RST 21          /* Active-low reset for SSD1306 */
#define OLED_VEXT 36         /* GPIO that gates the 3.3 V OLED supply rail */
#define OLED_I2C_PORT I2C_NUM_0

/* --- Display geometry / timing constants --- */
#define OLED_TEXT_LINE_LEN 16        /* Visible characters per row (128 px / 8 px font) */
#define OLED_PAGE_SWITCH_MS 7000     /* How long each display page stays visible (ms) */
#define OLED_SCROLL_STEP_MS 350      /* Horizontal scroll advances one character per step (ms) */

static SSD1306_t dev;
static const char *TAG = "GW_DISPLAY";

/* Write one 16-character row to the panel.
 * If invert is false the function compares the new content against the
 * per-row cache and skips the I2C write when nothing has changed, reducing
 * bus traffic on a ~700 ms refresh loop.  Inverted (alert) rows are always
 * redrawn because the inversion flag is not stored in the cache. */
static void write_line_ex(int row, const char *text, char *cache, size_t cache_size, bool invert) {
    char buffer[OLED_TEXT_LINE_LEN + 1];
    snprintf(buffer, sizeof(buffer), "%-16.16s", text ? text : "");
    if (!invert && strncmp(buffer, cache, cache_size) == 0) {
        return;
    }
    ssd1306_display_text(&dev, row, buffer, strlen(buffer), invert);
    snprintf(cache, cache_size, "%s", buffer);
}

/* Convenience wrapper: non-inverted write with dirty-line caching. */
static void write_line(int row, const char *text, char *cache, size_t cache_size) {
    write_line_ex(row, text, cache, cache_size, false);
}

/* Strip well-known gateway label prefixes (CMD:, ACK:, GWRX:, GWTX:) so the
 * OLED shows only the payload value rather than the protocol tag.  Returns a
 * pointer into the original string — no allocation. */
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

/* Extract the first whitespace/comma-delimited token after stripping any
 * protocol prefix.  Used for status fields where only the leading word is
 * meaningful (e.g. "MANUAL,1234" → "MANUAL"). */
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

/* Return a fixed-width window into a long string, advancing by 'offset'
 * characters.  When the text fits within out_size-1 characters it is copied
 * verbatim; otherwise the window wraps around so long payloads scroll across
 * the 16-character display row on successive render calls. */
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

/* Normalise a raw status string to a short, fixed-vocabulary label that fits
 * the 16-character grid.  Recognised semantic groups:
 *   connected/online/ready/ok  → "OK"
 *   bridge/link/forward        → "LINK"
 *   setup/config/ap            → "SETUP"
 *   degraded/stale/warn        → "WARN"
 *   none/off/disconn           → "OFF"
 *   idle                       → "IDLE"
 *   anything else              → upper-cased token (or "--" if empty)
 */
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

/* Returns true when the compact label warrants showing the alert page.
 * "WARN" means a degraded link; "OFF" means a completely absent link. */
static bool state_needs_alert(const char *label) {
    return label && (strcmp(label, "WARN") == 0 || strcmp(label, "OFF") == 0);
}

/* Probe both common SSD1306 I2C addresses (0x3C and 0x3D) and store the
 * responding address in dev._address.  Uses the ESP-IDF v5 master-probe API
 * when available, falling back to a manual start/stop sequence on v4. */
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
    /* Enable VEXT to power the OLED panel; the rail needs ~100 ms to stabilise
     * before the reset sequence can begin. */
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

    /* Hardware reset: pull RST low for 20 ms, then release.  The SSD1306
     * datasheet requires at least 3 µs; 20 ms gives plenty of margin. */
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
    /* Brief splash so the operator can confirm the display is working. */
    ssd1306_display_text(&dev, 2, "NAVIGATOR GW", 12, false);
    ssd1306_display_text(&dev, 4, "PLEASE WAIT", 11, false);
    vTaskDelay(pdMS_TO_TICKS(220));
    ssd1306_clear_screen(&dev, false);
    ESP_LOGI(TAG, "Gateway display initialized");
    return true;
}

void display_show_gateway_status(const char *mode,
                                 const char *lora_status,
                                 const char *cmd_status,
                                 const char *uart_status,
                                 const char *last_downlink,
                                 const char *last_uplink,
                                 const char *alert_headline,
                                 const char *alert_detail,
                                 uint32_t lora_rx_count,
                                 uint32_t uart_forward_count,
                                 uint32_t tx_drop_count,
                                 uint32_t uart_overflow_count) {
    /* Per-row dirty cache; cleared whenever the active page changes so that a
     * full redraw is forced after the page-transition screen clear. */
    static char cache[8][OLED_TEXT_LINE_LEN + 1] = {{0}};
    static int last_page = -1;
    char line[OLED_TEXT_LINE_LEN + 1];
    char detail[OLED_TEXT_LINE_LEN + 1];
    char mode_label[12];
    char lora_label[12];
    char cmd_label[12];
    char uart_label[12];
    char downlink_label[14];
    char uplink_label[14];
    TickType_t now = xTaskGetTickCount();
    /* scroll_step increments once per OLED_SCROLL_STEP_MS; passed to
     * copy_windowed_text() as the horizontal offset so long strings pan
     * automatically across the 16-character row. */
    size_t scroll_step = (size_t)(now / pdMS_TO_TICKS(OLED_SCROLL_STEP_MS));

    /* Normalise all raw status strings to short display labels. */
    compact_state_label(mode, mode_label, sizeof(mode_label));
    compact_state_label(lora_status, lora_label, sizeof(lora_label));
    compact_state_label(cmd_status, cmd_label, sizeof(cmd_label));
    compact_state_label(uart_status, uart_label, sizeof(uart_label));
    compact_token_label(last_downlink, downlink_label, sizeof(downlink_label));
    compact_token_label(last_uplink, uplink_label, sizeof(uplink_label));

    /* Determine how many pages to cycle through.  The alert page (page 2) is
     * added only when a fault condition is present so normal operation stays
     * on a comfortable two-page rotation. */
    bool show_alert = state_needs_alert(lora_label)
        || state_needs_alert(uart_label)
        || tx_drop_count > 0
        || uart_overflow_count > 0;
    int page_count = show_alert ? 3 : 2;
    int page = (int)((now / pdMS_TO_TICKS(OLED_PAGE_SWITCH_MS)) % page_count);

    /* On a page transition: clear the panel and invalidate the dirty cache so
     * every row is redrawn from scratch on the new page. */
    if (page != last_page) {
        memset(cache, 0, sizeof(cache));
        ssd1306_clear_screen(&dev, false);
        last_page = page;
    }

    /* --- Page 0: high-level gateway status --- */
    if (page == 0) {
        write_line(0, "GW STATUS P1", cache[0], sizeof(cache[0]));

        snprintf(line, sizeof(line), "MODE:%-.10s", mode_label);
        write_line(1, line, cache[1], sizeof(cache[1]));

        /* Pack LoRa and CMD labels side-by-side on one row. */
        snprintf(line, sizeof(line), "L:%-.4s C:%-.4s", lora_label, cmd_label);
        write_line(2, line, cache[2], sizeof(cache[2]));

        snprintf(line, sizeof(line), "UART:%-.8s", uart_label);
        write_line(3, line, cache[3], sizeof(cache[3]));

        /* Rows 4–7: scrolling previews of the most recent downlink/uplink
         * payloads.  The two streams use different scroll offsets (+5) so
         * they don't appear to move in lockstep. */
        write_line(4, "BASE->ROBOT", cache[4], sizeof(cache[4]));
        copy_windowed_text(last_downlink, detail, sizeof(detail), scroll_step);
        write_line(5, detail, cache[5], sizeof(cache[5]));

        write_line(6, "ROBOT->BASE", cache[6], sizeof(cache[6]));
        copy_windowed_text(last_uplink, detail, sizeof(detail), scroll_step + 5);
        write_line(7, detail, cache[7], sizeof(cache[7]));

    /* --- Page 1: numeric counters and full message previews --- */
    } else if (page == 1) {
        write_line(0, "GW DETAIL P2", cache[0], sizeof(cache[0]));

        /* Counters are modulo 100 so they always fit the two-digit field. */
        snprintf(line, sizeof(line), "RX:%02lu TX:%02lu",
                 (unsigned long)(lora_rx_count % 100),
                 (unsigned long)(uart_forward_count % 100));
        write_line(1, line, cache[1], sizeof(cache[1]));

        snprintf(line, sizeof(line), "DROP:%02lu OF:%02lu",
                 (unsigned long)(tx_drop_count % 100),
                 (unsigned long)(uart_overflow_count % 100));
        write_line(2, line, cache[2], sizeof(cache[2]));

        write_line(3, "DOWNLINK", cache[3], sizeof(cache[3]));
        copy_windowed_text(last_downlink, detail, sizeof(detail), scroll_step + 9);
        write_line(4, detail, cache[4], sizeof(cache[4]));

        write_line(5, "UPLINK", cache[5], sizeof(cache[5]));
        copy_windowed_text(last_uplink, detail, sizeof(detail), scroll_step + 13);
        write_line(6, detail, cache[6], sizeof(cache[6]));
        snprintf(line, sizeof(line), "M:%.4s C:%.4s", mode_label, cmd_label);
        write_line(7, line, cache[7], sizeof(cache[7]));

    /* --- Page 2: alert page (all rows inverted for high visual contrast) --- */
    } else {
        write_line_ex(0, "!!! GW ALERT !", cache[0], sizeof(cache[0]), true);

        copy_windowed_text(alert_headline, detail, sizeof(detail), scroll_step);
        write_line_ex(1, detail, cache[1], sizeof(cache[1]), true);

        snprintf(line, sizeof(line), "LORA:%-.10s", lora_label);
        write_line_ex(2, line, cache[2], sizeof(cache[2]), true);

        snprintf(line, sizeof(line), "CMD:%-.11s", cmd_label);
        write_line_ex(3, line, cache[3], sizeof(cache[3]), true);

        snprintf(line, sizeof(line), "UART:%-.10s", uart_label);
        write_line_ex(4, line, cache[4], sizeof(cache[4]), true);

        copy_windowed_text(alert_detail, detail, sizeof(detail), scroll_step + 3);
        write_line_ex(5, detail, cache[5], sizeof(cache[5]), true);

        snprintf(line, sizeof(line), "D:%02lu O:%02lu",
                 (unsigned long)(tx_drop_count % 100),
                 (unsigned long)(uart_overflow_count % 100));
        write_line_ex(6, line, cache[6], sizeof(cache[6]), true);

        snprintf(line, sizeof(line), "RX:%02lu TX:%02lu",
                 (unsigned long)(lora_rx_count % 100),
                 (unsigned long)(uart_forward_count % 100));
        write_line_ex(7, line, cache[7], sizeof(cache[7]), true);
    }
}
