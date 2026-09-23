// ShotThermalStats -- per-shot boiler temperature metrics for PID tuning.
//
// Pure C++17, host-testable.
//
// Records one shot window (shot start → end of the trailing phase) and
// reduces it to the numbers that matter when tuning the brew gains:
//   - how far the boiler dips below setpoint, and when;
//   - how far it overshoots after recovering from the dip;
//   - when it last left the ±band around setpoint (settling time);
//   - average heater duty while the shot ran and while trailing.
//
// Lifecycle: `begin()` at shot start, `sample()` every control step,
// `finish()` once the window closes. Not thread-safe.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::pid {

class ShotThermalStats {
   public:
    struct Result {
        float         setpoint_c       = 0.0F;
        float         start_temp_c     = 0.0F;
        float         min_temp_c       = 0.0F;
        std::uint32_t min_at_ms        = 0;     // relative to shot start
        float         max_after_min_c  = 0.0F;  // peak once the dip bottomed out
        std::uint32_t brew_ms          = 0;     // pump/shot duration
        std::uint32_t window_ms        = 0;     // shot + trailing
        std::uint32_t settled_at_ms    = 0;     // last re-entry into band (0 = never left)
        bool          settled          = true;  // in band at window end
        float         avg_duty_brew    = 0.0F;
        float         avg_duty_trail   = 0.0F;

        float drop_c() const noexcept      { return setpoint_c - min_temp_c; }
        float overshoot_c() const noexcept { return max_after_min_c - setpoint_c; }
    };

    void begin(std::uint64_t now_ms, float setpoint_c, float temp_c,
               float band_c) noexcept {
        r_                 = Result{};
        r_.setpoint_c      = setpoint_c;
        r_.start_temp_c    = temp_c;
        r_.min_temp_c      = temp_c;
        r_.max_after_min_c = temp_c;
        start_ms_          = now_ms;
        band_c_            = band_c;
        duty_sum_brew_     = 0.0F;
        duty_sum_trail_    = 0.0F;
        n_brew_            = 0;
        n_trail_           = 0;
        active_            = true;
    }

    void sample(std::uint64_t now_ms, float temp_c, float duty_pct,
                bool shot_running) noexcept {
        if (!active_ || !std::isfinite(temp_c)) {
            return;
        }
        const auto rel = static_cast<std::uint32_t>(now_ms - start_ms_);

        if (temp_c < r_.min_temp_c) {
            r_.min_temp_c      = temp_c;
            r_.min_at_ms       = rel;
            r_.max_after_min_c = temp_c;
        } else if (temp_c > r_.max_after_min_c) {
            r_.max_after_min_c = temp_c;
        }

        if (std::fabs(temp_c - r_.setpoint_c) > band_c_) {
            r_.settled       = false;
            r_.settled_at_ms = rel;
        } else if (!r_.settled) {
            r_.settled       = true;
            r_.settled_at_ms = rel;
        }

        if (shot_running) {
            duty_sum_brew_ += duty_pct;
            ++n_brew_;
            r_.brew_ms = rel;
        } else {
            duty_sum_trail_ += duty_pct;
            ++n_trail_;
        }
        r_.window_ms = rel;
    }

    // Closes the window and returns the summary.
    Result finish() noexcept {
        active_ = false;
        r_.avg_duty_brew =
            (n_brew_ > 0U) ? duty_sum_brew_ / static_cast<float>(n_brew_) : 0.0F;
        r_.avg_duty_trail =
            (n_trail_ > 0U) ? duty_sum_trail_ / static_cast<float>(n_trail_) : 0.0F;
        return r_;
    }

    bool active() const noexcept { return active_; }

   private:
    Result        r_{};
    std::uint64_t start_ms_       = 0;
    float         band_c_         = 0.5F;
    float         duty_sum_brew_  = 0.0F;
    float         duty_sum_trail_ = 0.0F;
    std::uint32_t n_brew_         = 0;
    std::uint32_t n_trail_        = 0;
    bool          active_         = false;
};

}  // namespace espid::pid
