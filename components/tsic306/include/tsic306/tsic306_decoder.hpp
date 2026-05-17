// ESPID-Coffee — TSIC 306 decoder (pure logic, host-testable).
//
// Converts a captured ZACwire frame into a temperature in °C, applying the
// TSIC 306 transfer function and basic sanity / rate-of-change filtering.
//
// Transfer function (TSIC 306, 11-bit value, range -50 .. +150 °C):
//     T = (raw11 / 2047.0) * 200.0 - 50.0
//
// Per-frame parity is the responsibility of the capture layer (it sets the
// parity_mask on each ZacwireFrame); we treat a frame as invalid if either
// parity bit is bad.

#pragma once

#include <cstdint>
#include <optional>

#include "hal_cpp/i_zacwire_capture.hpp"
#include "hal_cpp/result.hpp"

namespace espid::tsic306 {

struct DecoderConfig {
    // Maximum allowed |ΔT| between consecutive valid samples, in °C.
    // Mirrors clevercoffee's ZACwire(MAX_CHANGERATE=15). Set to 0 to disable.
    float max_change_rate_c = 15.0F;

    // Minimum / maximum plausible temperature (°C). Out-of-range frames are
    // rejected with Error::kRangeError before being smoothed.
    float min_valid_c = -50.0F;
    float max_valid_c = 150.0F;
};

class Tsic306Decoder {
   public:
    explicit Tsic306Decoder(DecoderConfig cfg = {}) noexcept : cfg_(cfg) {}

    // Decode a ZACwire frame to °C. Returns kCrcMismatch on parity failure,
    // kRangeError when the value is outside [min_valid_c, max_valid_c], and
    // kRangeError again when |ΔT| exceeds max_change_rate_c vs the last good
    // sample.
    hal_cpp::Result<float> decode(const hal_cpp::ZacwireFrame& frame) noexcept;

    // Returns the last successfully decoded temperature, if any.
    std::optional<float> last_value() const noexcept { return last_value_; }

    // Resets the change-rate filter (e.g. after a long pause).
    void reset() noexcept { last_value_.reset(); }

   private:
    DecoderConfig        cfg_;
    std::optional<float> last_value_{};
};

}  // namespace espid::tsic306
