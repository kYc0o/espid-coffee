# tsic306/ — TSIC 306 temperature sensor.
#
# Pure-logic decoder (host-testable) + RMT-based capture driver.
# The decoder has no ESP-IDF deps; the driver depends on the RMT and
# esp_timer components.

idf_component_register(
    SRCS
        "src/tsic306_decoder.cpp"
        "src/tsic306_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES hal_cpp esp_driver_rmt esp_driver_gpio esp_timer freertos
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
