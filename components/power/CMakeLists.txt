# power/ — standby timer, daily wake schedule and deep-sleep planning
# (header-only).
#
# Pure C++17, no ESP-IDF / FreeRTOS dependencies. The same headers are
# pulled into host_tests for deterministic unit-testing.

idf_component_register(
    INCLUDE_DIRS "include"
)
