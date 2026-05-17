# ESPID-Coffee — top-level project file.
# Requires ESP-IDF v6.0.x or newer.
cmake_minimum_required(VERSION 3.16)

# Allow out-of-tree components (added in later phases).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

# ----------------------------------------------------------------------------
# Workaround: nixy4/u8g2 v0.1.4's component manifest lists `driver` in its
# REQUIRES, but ESP-IDF v6 split that monolithic component into
# `esp_driver_gpio`, `esp_driver_i2c`, etc.  Its port file
# (`src/port/esp32_hw_i2c.c`) includes `driver/gpio.h`, so the build fails
# unless `esp_driver_gpio` is in REQUIRES.  Patch the managed component
# CMakeLists in place (idempotent) so a fresh `idf.py reconfigure` succeeds
# without us having to vendor ~12 MB of u8g2 into the repo.
# Remove when nixy4/u8g2 ships a fixed release that lists esp_driver_gpio.
set(_u8g2_cmake "${CMAKE_CURRENT_LIST_DIR}/managed_components/nixy4__u8g2/CMakeLists.txt")
if(EXISTS "${_u8g2_cmake}")
    file(READ "${_u8g2_cmake}" _u8g2_text)
    if(NOT _u8g2_text MATCHES "esp_driver_gpio")
        string(REPLACE
            "REQUIRES driver esp_driver_i2c esp_driver_spi"
            "REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_spi"
            _u8g2_text "${_u8g2_text}")
        file(WRITE "${_u8g2_cmake}" "${_u8g2_text}")
        message(STATUS "Patched ${_u8g2_cmake}: +esp_driver_gpio in REQUIRES")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(espid_coffee
    VERSION 0.0.1
    LANGUAGES C CXX
)
