// ABP2 decoder — converts 7-byte digital frames into engineering values.
//
// The logic here is pure C++ and intentionally ESP-IDF free so it can be
// unit-tested on host. Driver code (I2C transport) lives separately.

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::abp2 {

enum class FrameStatus : std::uint8_t {
    kNormal = 0,
    kCommandMode = 1,
    kStaleData = 2,
    kDiagnostic = 3,
};

struct TransferFunction {
    // Typical ABP2 transfer counts for 10%-90% scaling over 24-bit output.
    std::uint32_t output_min_counts{1'677'722};
    std::uint32_t output_max_counts{15'099'494};

    // Sensor pressure span in PSI (device specific).
    float pressure_min_psi{0.0F};
    float pressure_max_psi{30.0F};
};

struct DecodedSample {
    FrameStatus status{FrameStatus::kNormal};
    std::uint32_t pressure_counts{0};
    std::uint32_t temperature_counts{0};
    float pressure_psi{0.0F};
    float pressure_bar{0.0F};
    float temperature_c{0.0F};
};

// Decode one ABP2 frame (7 bytes). Returns:
// - kInvalidArg for null input / invalid transfer function
// - kHardwareFault for diagnostic status frames
// - kRangeError for out-of-range counts
hal_cpp::Result<DecodedSample> decode_frame(const std::uint8_t raw[7],
                                            const TransferFunction& tf) noexcept;

}  // namespace espid::abp2
