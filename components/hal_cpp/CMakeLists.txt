# hal_cpp/ — C++ hardware-abstraction layer for ESPID-Coffee.
#
# Header-only: defines Result<T>/Status and pure-virtual driver interfaces
# (clock, gpio, i2c, zacwire). Concrete drivers live in their own components.
#
# Renamed from "hal" to avoid colliding with ESP-IDF's built-in `hal`
# component, whose headers (e.g. hal/efuse_hal.h) are pulled in by core
# components such as esp_https_ota.

idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES esp_common
)
