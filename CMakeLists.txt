# ESPID-Coffee — top-level project file.
# Requires ESP-IDF v6.0.x or newer.
cmake_minimum_required(VERSION 3.16)

# Allow out-of-tree components (added in later phases).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(espid_coffee
    VERSION 0.0.1
    LANGUAGES C CXX
)
