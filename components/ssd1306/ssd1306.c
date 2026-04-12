#include "ssd1306.h"

#include <ctype.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SSD1306_I2C_PORT I2C_NUM_0
#define SSD1306_I2C_FREQ_HZ 400000
#define SSD1306_MAX_CHUNK 16
#define SSD1306_GLYPH_WIDTH 6

static const char *TAG = "ssd1306";

static void glyph_for_char(char c, uint8_t out[5]) {
    memset(out, 0, 5);
    c = (char)toupper((unsigned char)c);

    switch (c) {
        case '0': out[0] = 0x3E; out[1] = 0x51; out[2] = 0x49; out[3] = 0x45; out[4] = 0x3E; break;
        case '1': out[0] = 0x00; out[1] = 0x42; out[2] = 0x7F; out[3] = 0x40; out[4] = 0x00; break;
        case '2': out[0] = 0x62; out[1] = 0x51; out[2] = 0x49; out[3] = 0x49; out[4] = 0x46; break;
        case '3': out[0] = 0x22; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x36; break;
        case '4': out[0] = 0x18; out[1] = 0x14; out[2] = 0x12; out[3] = 0x7F; out[4] = 0x10; break;
        case '5': out[0] = 0x2F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x31; break;
        case '6': out[0] = 0x3E; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x32; break;
        case '7': out[0] = 0x01; out[1] = 0x71; out[2] = 0x09; out[3] = 0x05; out[4] = 0x03; break;
        case '8': out[0] = 0x36; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x36; break;
        case '9': out[0] = 0x26; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x3E; break;
        case 'A': out[0] = 0x7E; out[1] = 0x09; out[2] = 0x09; out[3] = 0x09; out[4] = 0x7E; break;
        case 'B': out[0] = 0x7F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x36; break;
        case 'C': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x41; out[3] = 0x41; out[4] = 0x22; break;
        case 'D': out[0] = 0x7F; out[1] = 0x41; out[2] = 0x41; out[3] = 0x22; out[4] = 0x1C; break;
        case 'E': out[0] = 0x7F; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x41; break;
        case 'F': out[0] = 0x7F; out[1] = 0x09; out[2] = 0x09; out[3] = 0x09; out[4] = 0x01; break;
        case 'G': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x49; out[3] = 0x49; out[4] = 0x7A; break;
        case 'H': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x08; out[3] = 0x08; out[4] = 0x7F; break;
        case 'I': out[0] = 0x00; out[1] = 0x41; out[2] = 0x7F; out[3] = 0x41; out[4] = 0x00; break;
        case 'J': out[0] = 0x20; out[1] = 0x40; out[2] = 0x41; out[3] = 0x3F; out[4] = 0x01; break;
        case 'K': out[0] = 0x7F; out[1] = 0x08; out[2] = 0x14; out[3] = 0x22; out[4] = 0x41; break;
        case 'L': out[0] = 0x7F; out[1] = 0x40; out[2] = 0x40; out[3] = 0x40; out[4] = 0x40; break;
        case 'M': out[0] = 0x7F; out[1] = 0x02; out[2] = 0x0C; out[3] = 0x02; out[4] = 0x7F; break;
        case 'N': out[0] = 0x7F; out[1] = 0x04; out[2] = 0x08; out[3] = 0x10; out[4] = 0x7F; break;
        case 'O': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x41; out[3] = 0x41; out[4] = 0x3E; break;
        case 'P': out[0] = 0x7F; out[1] = 0x09; out[2] = 0x09; out[3] = 0x09; out[4] = 0x06; break;
        case 'Q': out[0] = 0x3E; out[1] = 0x41; out[2] = 0x51; out[3] = 0x21; out[4] = 0x5E; break;
        case 'R': out[0] = 0x7F; out[1] = 0x09; out[2] = 0x19; out[3] = 0x29; out[4] = 0x46; break;
        case 'S': out[0] = 0x26; out[1] = 0x49; out[2] = 0x49; out[3] = 0x49; out[4] = 0x32; break;
        case 'T': out[0] = 0x01; out[1] = 0x01; out[2] = 0x7F; out[3] = 0x01; out[4] = 0x01; break;
        case 'U': out[0] = 0x3F; out[1] = 0x40; out[2] = 0x40; out[3] = 0x40; out[4] = 0x3F; break;
        case 'V': out[0] = 0x1F; out[1] = 0x20; out[2] = 0x40; out[3] = 0x20; out[4] = 0x1F; break;
        case 'W': out[0] = 0x7F; out[1] = 0x20; out[2] = 0x18; out[3] = 0x20; out[4] = 0x7F; break;
        case 'X': out[0] = 0x63; out[1] = 0x14; out[2] = 0x08; out[3] = 0x14; out[4] = 0x63; break;
        case 'Y': out[0] = 0x03; out[1] = 0x04; out[2] = 0x78; out[3] = 0x04; out[4] = 0x03; break;
        case 'Z': out[0] = 0x61; out[1] = 0x51; out[2] = 0x49; out[3] = 0x45; out[4] = 0x43; break;
        case ':': out[0] = 0x00; out[1] = 0x36; out[2] = 0x36; out[3] = 0x00; out[4] = 0x00; break;
        case '-': out[0] = 0x08; out[1] = 0x08; out[2] = 0x08; out[3] = 0x08; out[4] = 0x08; break;
        case '>': out[0] = 0x00; out[1] = 0x41; out[2] = 0x22; out[3] = 0x14; out[4] = 0x08; break;
        case '!': out[0] = 0x00; out[1] = 0x00; out[2] = 0x5F; out[3] = 0x00; out[4] = 0x00; break;
        case '.': out[0] = 0x00; out[1] = 0x60; out[2] = 0x60; out[3] = 0x00; out[4] = 0x00; break;
        case ',': out[0] = 0x00; out[1] = 0x80; out[2] = 0x60; out[3] = 0x00; out[4] = 0x00; break;
        case '/': out[0] = 0x20; out[1] = 0x10; out[2] = 0x08; out[3] = 0x04; out[4] = 0x02; break;
        case ' ': default: break;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static esp_err_t ssd1306_write(SSD1306_t *dev, uint8_t control, const uint8_t *payload, size_t payload_len) {
    if (!dev || !dev->_i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[SSD1306_MAX_CHUNK + 1];
    buffer[0] = control;

    while (payload_len > 0) {
        size_t chunk = payload_len > SSD1306_MAX_CHUNK ? SSD1306_MAX_CHUNK : payload_len;
        memcpy(&buffer[1], payload, chunk);
        esp_err_t err = i2c_master_transmit(dev->_i2c_dev_handle, buffer, chunk + 1, -1);
        if (err != ESP_OK) {
            return err;
        }
        payload += chunk;
        payload_len -= chunk;
    }
    return ESP_OK;
}
#else
static esp_err_t ssd1306_write(SSD1306_t *dev, uint8_t control, const uint8_t *payload, size_t payload_len) {
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    if (payload_len > 0) {
        i2c_master_write(cmd, (uint8_t *)payload, payload_len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}
#endif

static esp_err_t ssd1306_command(SSD1306_t *dev, uint8_t command) {
    return ssd1306_write(dev, 0x00, &command, 1);
}

static esp_err_t ssd1306_set_pos(SSD1306_t *dev, uint8_t page, uint8_t column) {
    esp_err_t err = ssd1306_command(dev, 0xB0 | (page & 0x07));
    if (err != ESP_OK) {
        return err;
    }
    err = ssd1306_command(dev, 0x00 | (column & 0x0F));
    if (err != ESP_OK) {
        return err;
    }
    return ssd1306_command(dev, 0x10 | ((column >> 4) & 0x0F));
}

void i2c_master_init(SSD1306_t *dev, int sda_gpio, int scl_gpio, int reset_gpio) {
    (void)reset_gpio;
    if (!dev) {
        return;
    }

    memset(dev, 0, sizeof(*dev));
    dev->width = 128;
    dev->height = 64;
    dev->pages = 8;
    dev->_address = 0x3C;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2c_master_bus_config_t bus_config = {
        .i2c_port = SSD1306_I2C_PORT,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &dev->_i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        dev->_i2c_bus_handle = NULL;
    }
#else
    dev->_i2c_port = SSD1306_I2C_PORT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SSD1306_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(dev->_i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(dev->_i2c_port, conf.mode, 0, 0, 0));
#endif
}

void ssd1306_init(SSD1306_t *dev, int width, int height) {
    if (!dev) {
        return;
    }

    dev->width = (uint8_t)width;
    dev->height = (uint8_t)height;
    dev->pages = (uint8_t)(height / 8);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (!dev->_i2c_bus_handle) {
        return;
    }

    if (!dev->_i2c_dev_handle) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = dev->_address,
            .scl_speed_hz = SSD1306_I2C_FREQ_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(dev->_i2c_bus_handle, &dev_cfg, &dev->_i2c_dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add SSD1306 device: %s", esp_err_to_name(err));
            return;
        }
    }
#endif

    const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, (uint8_t)(height - 1), 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, (height == 64) ? 0x12 : 0x02,
        0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };

    for (size_t i = 0; i < sizeof(init_seq); ++i) {
        if (ssd1306_command(dev, init_seq[i]) != ESP_OK) {
            ESP_LOGW(TAG, "SSD1306 init command %u failed", (unsigned)i);
            return;
        }
    }
}

void ssd1306_clear_screen(SSD1306_t *dev, bool invert) {
    if (!dev) {
        return;
    }

    uint8_t line[128];
    memset(line, invert ? 0xFF : 0x00, sizeof(line));

    for (uint8_t page = 0; page < dev->pages; ++page) {
        if (ssd1306_set_pos(dev, page, 0) != ESP_OK) {
            return;
        }
        if (ssd1306_write(dev, 0x40, line, dev->width) != ESP_OK) {
            return;
        }
    }
}

void ssd1306_display_text(SSD1306_t *dev, int page, const char *text, int text_len, bool invert) {
    if (!dev || !text || page < 0 || page >= dev->pages) {
        return;
    }

    if (text_len < 0) {
        text_len = (int)strlen(text);
    }

    if (ssd1306_set_pos(dev, (uint8_t)page, 0) != ESP_OK) {
        return;
    }

    uint8_t buffer[128];
    size_t out = 0;
    int max_chars = dev->width / SSD1306_GLYPH_WIDTH;
    int chars = text_len < max_chars ? text_len : max_chars;

    for (int i = 0; i < chars && out + SSD1306_GLYPH_WIDTH <= sizeof(buffer); ++i) {
        uint8_t glyph[5];
        glyph_for_char(text[i], glyph);
        for (int j = 0; j < 5; ++j) {
            buffer[out++] = invert ? (uint8_t)~glyph[j] : glyph[j];
        }
        buffer[out++] = invert ? 0xFF : 0x00;
    }

    while (out < dev->width) {
        buffer[out++] = invert ? 0xFF : 0x00;
    }

    (void)ssd1306_write(dev, 0x40, buffer, out);
}
