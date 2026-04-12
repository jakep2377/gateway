#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t width;
    uint8_t height;
    uint8_t pages;
    uint8_t _address;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2c_master_bus_handle_t _i2c_bus_handle;
    i2c_master_dev_handle_t _i2c_dev_handle;
#else
    i2c_port_t _i2c_port;
#endif
} SSD1306_t;

void i2c_master_init(SSD1306_t *dev, int sda_gpio, int scl_gpio, int reset_gpio);
void ssd1306_init(SSD1306_t *dev, int width, int height);
void ssd1306_clear_screen(SSD1306_t *dev, bool invert);
void ssd1306_display_text(SSD1306_t *dev, int page, const char *text, int text_len, bool invert);

#ifdef __cplusplus
}
#endif
