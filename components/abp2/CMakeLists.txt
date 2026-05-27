# abp2/ — Honeywell ABP2 digital pressure sensor.
#
# Contains:
# - pure frame decoder (host-testable)
# - I2C polling driver for ESP-IDF targets

idf_component_register(
    SRCS
        "src/abp2_decoder.cpp"
        "src/abp2_driver.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        hal_cpp
        esp_driver_gpio
        esp_driver_i2c
        freertos
        log
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
