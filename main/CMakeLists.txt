idf_component_register(
    SRCS
        "app_main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_system
        esp_timer
        log
        freertos
        nvs_flash
        esp_http_server
        esp_driver_gpio
        hal_cpp
        tsic306
        ssd1306
        brew
        wifi
        heater
        pid
        abp2
)

# Modern C++ for application code.
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)

# Build a LittleFS image from the web assets in main/data/ and embed it
# as a flashable binary that idf.py flash will write to the 'storage'
# partition automatically.
littlefs_create_partition_image(storage "${CMAKE_CURRENT_LIST_DIR}/data"
    FLASH_IN_PROJECT)
