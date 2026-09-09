# valve/ — 3-way solenoid valve relay output.

idf_component_register(
    SRCS
        "src/solenoid_valve.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        hal_cpp
        esp_driver_gpio
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
