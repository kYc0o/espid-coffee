# pump/ — PSM (phase-angle) pump dimming.
#
# The PsmCore firing-delay logic is pure C++17, header-only, and shared
# with host_tests. PsmPump adds the zero-cross ISR + gptimer gate driver.

idf_component_register(
    SRCS
        "src/psm_pump.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        hal_cpp
        esp_driver_gpio
        esp_driver_gptimer
        esp_timer
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
