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

# ---------------- Units under test ----------------
# We pull source files directly from components/. Headers are header-only or
# ESP-IDF-free, so this compiles cleanly on the host.
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(espid_tsic306_decoder STATIC
    ${REPO_ROOT}/components/tsic306/src/tsic306_decoder.cpp
)
target_include_directories(espid_tsic306_decoder PUBLIC
    ${REPO_ROOT}/components/tsic306/include
    ${REPO_ROOT}/components/hal_cpp/include
)

add_library(espid_abp2_decoder STATIC
    ${REPO_ROOT}/components/abp2/src/abp2_decoder.cpp
)
target_include_directories(espid_abp2_decoder PUBLIC
    ${REPO_ROOT}/components/abp2/include
    ${REPO_ROOT}/components/hal_cpp/include
)

# pid is header-only; expose its include dir as an INTERFACE target.
add_library(espid_pid INTERFACE)
target_include_directories(espid_pid INTERFACE
    ${REPO_ROOT}/components/pid/include
)

# brew is header-only (BrewButton + ShotTimer).
add_library(espid_brew INTERFACE)
target_include_directories(espid_brew INTERFACE
    ${REPO_ROOT}/components/brew/include
)

# heater slow-PWM core is header-only and host-testable.
add_library(espid_heater INTERFACE)
target_include_directories(espid_heater INTERFACE
    ${REPO_ROOT}/components/heater/include
)

# ---------------- Test binary ----------------
add_executable(espid_unit_tests
    test_sanity.cpp
    test_tsic306_decoder.cpp
    test_abp2_decoder.cpp
    test_pid.cpp
    test_brew.cpp
    test_heater.cpp
)
target_link_libraries(espid_unit_tests PRIVATE
    espid_tsic306_decoder
    espid_abp2_decoder
    espid_pid
    espid_brew
    espid_heater
    GTest::gtest_main
    GTest::gmock
)

gtest_discover_tests(espid_unit_tests)
