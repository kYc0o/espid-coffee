# scale/ — dual-HX711 drip-tray weight scale.
#
# The ScaleCore weight math is pure C++17, header-only, and shared with
# host_tests. The HX711 bit-bang driver is added in a later commit.

idf_component_register(
    INCLUDE_DIRS "include"
)
