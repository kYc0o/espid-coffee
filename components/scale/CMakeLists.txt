# scale/ — dual-HX711 drip-tray weight scale.
#
# The ScaleCore weight math is pure C++17, header-only, and shared with
# host_tests. Hx711Scale adds the bit-bang reader for the two amplifiers.

idf_component_register(
    SRCS
        "src/hx711_scale.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        hal_cpp
        esp_driver_gpio
        freertos
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
