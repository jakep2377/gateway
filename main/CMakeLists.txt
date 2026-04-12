idf_component_register(SRCS "display.c" "robot_gateway.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_uart ssd1306 ra01s)
