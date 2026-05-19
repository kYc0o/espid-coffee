// SlowPwmCore -- pure slow-PWM timing logic.
//
// Given `now_ms`, period, and duty-percent, computes whether output should
// currently be ON. This file has no ESP-IDF dependencies and is host-testable.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::heater {

class SlowPwmCore {
   public:
    explicit SlowPwmCore(std::uint32_t period_ms = 1000) noexcept
        : period_ms_(period_ms) {}

    bool configure(std::uint32_t period_ms) noexcept {
        if (period_ms == 0) {
            return false;
        }
        period_ms_ = period_ms;
        started_ = false;
        origin_ms_ = 0;
        return true;
    }

    bool set_duty_pct(float duty_pct) noexcept {
        if (!std::isfinite(duty_pct) || duty_pct < 0.0F || duty_pct > 100.0F) {
            return false;
        }
        duty_pct_ = duty_pct;
        return true;
    }

    float duty_pct() const noexcept { return duty_pct_; }

    bool output_on(std::uint64_t now_ms) noexcept {
        if (period_ms_ == 0) {
            return false;
        }
        if (!started_) {
            origin_ms_ = now_ms;
            started_ = true;
        }

        if (duty_pct_ <= 0.0F) {
            return false;
        }
        if (duty_pct_ >= 100.0F) {
            return true;
        }

        const std::uint64_t elapsed = now_ms - origin_ms_;
        const std::uint32_t phase = static_cast<std::uint32_t>(elapsed % period_ms_);
        const std::uint32_t on_window = on_window_ms_();
        return phase < on_window;
    }

   private:
    std::uint32_t on_window_ms_() const noexcept {
        const float on_ms = (duty_pct_ * static_cast<float>(period_ms_)) / 100.0F;
        auto window = static_cast<std::uint32_t>(on_ms + 0.5F);
        if (window > period_ms_) {
            window = period_ms_;
        }
        return window;
    }

    std::uint32_t period_ms_{1000};
    float duty_pct_{0.0F};
    bool started_{false};
    std::uint64_t origin_ms_{0};
};

}  // namespace espid::heater
