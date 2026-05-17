# tsic306/ — TSIC 306 temperature sensor.
#
# Pure-logic decoder (host-testable) + RMT-based capture driver (added later).
# The decoder has no ESP-IDF deps; only the future driver does.

idf_component_register(
    SRCS "src/tsic306_decoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES hal_cpp
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)
