# pump/ — PSM (phase-angle) pump dimming.
#
# The PsmCore firing-delay logic is pure C++17, header-only, and shared
# with host_tests. The zero-cross ISR + gate timer driver is added in a
# later commit.

idf_component_register(
    INCLUDE_DIRS "include"
)
