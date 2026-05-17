# ESPID-Coffee — host-side unit tests.
#
# Standalone CMake project (no ESP-IDF). Compiles pure-C++ logic units against
# gtest+gmock on the host (macOS/Linux) for fast TDD. Run with:
#     cmake -S host_tests -B host_tests/build -G Ninja
#     cmake --build host_tests/build
#     ctest --test-dir host_tests/build --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(espid_coffee_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---------------- GoogleTest + GoogleMock via FetchContent ----------------
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

# ---------------- Test binary ----------------
add_executable(espid_unit_tests
    test_sanity.cpp
)
target_link_libraries(espid_unit_tests PRIVATE
    GTest::gtest_main
    GTest::gmock
)

gtest_discover_tests(espid_unit_tests)
