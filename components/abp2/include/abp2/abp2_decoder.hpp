// ABP2 decoder — converts 7-byte digital frames into engineering values.
//
// The logic here is pure C++ and intentionally ESP-IDF free so it can be
// unit-tested on host. Driver code (I2C transport) lives separately.

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::abp2 {

// ABP2 status byte (first byte of the 7-byte read response):
//   bit 6  powered      1 = device answered normally  -> a healthy read is 0x40
//   bit 5  busy         conversion in progress, words not yet updated
//   bit 2  memory error
//   bit 0  math saturation
// These names follow the ABP2 datasheet. The older HSC/SSC parts encode bits
// 7:6 as normal/command/stale/diagnostic, and reading an ABP2 that way makes a
// healthy frame look like "command mode" and an all-zero (absent) frame look
// like "normal" — which is how a garbled read used to pass as 0.000 bar.
// Numeric values are unchanged so existing telemetry keeps its meaning.
enum class FrameStatus : std::uint8_t {
    kUnpowered  = 0,  // power bit clear: no response, or a garbled frame
    kPowered    = 1,  // normal
    kReserved   = 2,  // bit 7 set; an ABP2 never reports this
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
    FrameStatus status{FrameStatus::kPowered};
    bool busy{false};  // status bit 5: words not refreshed yet
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
