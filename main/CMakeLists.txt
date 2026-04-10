idf_component_register(SRCS "display.c" "robot_gateway.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio esp_driver_i2c ssd1306)
