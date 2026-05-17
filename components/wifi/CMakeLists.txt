# wifi/ — esp_wifi-backed INetwork implementation.

idf_component_register(
    SRCS
        "src/esp_wifi_network.cpp"
    INCLUDE_DIRS "include"
    REQUIRES hal_cpp esp_wifi esp_netif esp_event log
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
