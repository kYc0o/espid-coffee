# pid/ — discrete PID controller (header-only, pure C++).
#
# No ESP-IDF dependencies: the same translation units are also pulled
# into the host_tests CMake project to be unit-tested without flashing.

idf_component_register(
    INCLUDE_DIRS "include"
)
