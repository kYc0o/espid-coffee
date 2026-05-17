idf_component_register(
    SRCS        "src/ssd1306_display.cpp"
    INCLUDE_DIRS "include"
    REQUIRES    hal_cpp
                nixy4__u8g2
                esp_driver_i2c
                log
)
