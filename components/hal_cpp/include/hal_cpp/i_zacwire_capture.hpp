// IZacwireCapture — ZACwire frame producer for TSIC 30x temperature sensors.
//
// Concrete impl uses the RMT peripheral to time the variable-width pulses on
// a single GPIO. The decoder consumes ZacwireFrame instances and produces
// temperature samples (see components/tsic306).

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

struct ZacwireFrame {
    // Two concatenated 8-bit packets, MSB-first. The 11-bit temperature value
    // occupies the lowest 11 bits of `raw` (see Tsic306Decoder).
    std::uint16_t raw;

    // Parity status for the two packets:
    //   bit 0 = packet 1 parity OK
    //   bit 1 = packet 2 parity OK
    // Both bits must be set for the frame to be considered valid.
    std::uint8_t  parity_mask;

    // Capture timestamp (from ISystemClock::now_us at frame completion).
    std::uint64_t timestamp_us;
};

class IZacwireCapture {
   public:
    virtual ~IZacwireCapture() = default;

    // Blocks for up to `timeout_ms` for the next complete frame. Returns
    // kTimeout when no frame arrives in time, kBusError on capture failure.
    virtual Result<ZacwireFrame> next_frame(std::uint32_t timeout_ms) noexcept = 0;

    // Discards any frame currently buffered (call after a long pause).
    virtual Status flush() noexcept = 0;
};

}  // namespace espid::hal_cpp
