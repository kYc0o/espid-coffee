# hawkbit/ — Eclipse HawkBit DDI OTA client.

idf_component_register(
    SRCS
        "src/hawkbit_client.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_http_client
        esp_https_ota
        app_update
        heap
        mbedtls
        log
        freertos
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
