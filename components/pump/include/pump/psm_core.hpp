// PsmCore -- pure phase-control (PSM) firing-delay logic for the pump.
//
// Given a mains zero-cross reference and a requested power percentage,
// computes how long after each zero-cross the triac gate must be fired.
// Full power fires immediately at the zero-cross; lower power delays the
// gate deeper into the half-cycle (classic phase-angle dimming). This
// file has no ESP-IDF dependencies and is host-testable.
//
// Timing model: a half-cycle spans 1e6 / (2 * mains_hz) microseconds.
// firing_delay_us scales linearly from 0 us at 100% to a full half-cycle
// at 0% -- matching the common AC-dimmer mapping used for the vibratory
// pump. The ISR/timer that consumes these values lives in the driver.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::pump {

class PsmCore {
   public:
    explicit PsmCore(std::uint32_t mains_frequency_hz = 50) noexcept {
        configure(mains_frequency_hz);
    }

    bool configure(std::uint32_t mains_frequency_hz) noexcept {
        if (mains_frequency_hz == 0) {
            return false;
        }
        half_period_us_ = 1'000'000U / (2U * mains_frequency_hz);
        return true;
    }

    std::uint32_t half_period_us() const noexcept { return half_period_us_; }

    bool set_power_pct(float power_pct) noexcept {
        if (!std::isfinite(power_pct) || power_pct < 0.0F || power_pct > 100.0F) {
            return false;
        }
        power_pct_ = power_pct;
        return true;
    }

    float power_pct() const noexcept { return power_pct_; }

    // True when the triac should be fired at all during a half-cycle.
    bool firing_enabled() const noexcept { return power_pct_ > 0.0F; }

    // Delay from the zero-cross edge to the gate pulse, in microseconds.
    // Full power -> 0 us; lower power fires later. The returned value is
    // only meaningful when firing_enabled() is true.
    std::uint32_t firing_delay_us() const noexcept {
        if (power_pct_ >= 100.0F) {
            return 0;
        }
        if (power_pct_ <= 0.0F) {
            return half_period_us_;
        }
        const float delay =
            static_cast<float>(half_period_us_) * (100.0F - power_pct_) / 100.0F;
        auto us = static_cast<std::uint32_t>(delay + 0.5F);
        if (us > half_period_us_) {
            us = half_period_us_;
        }
        return us;
    }

   private:
    std::uint32_t half_period_us_{10'000};  // 50 Hz default
    float         power_pct_{0.0F};
};

}  // namespace espid::pump
