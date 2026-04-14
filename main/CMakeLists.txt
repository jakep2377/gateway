idf_component_register(SRCS "display.c" "robot_gateway.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_uart esp_wifi esp_event esp_http_server esp_netif nvs_flash lwip ssd1306 ra01s)
