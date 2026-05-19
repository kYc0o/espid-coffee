// SlowPwmSsrHeater -- GPIO-backed heater SSR output.
//
// Uses SlowPwmCore timing logic and applies it to one GPIO pin.

#pragma once

#include <cstdint>

#include "hal_cpp/i_heater.hpp"
#include "heater/slow_pwm_core.hpp"

namespace espid::heater {

class SlowPwmSsrHeater final : public hal_cpp::IHeater {
   public:
    SlowPwmSsrHeater() = default;
    ~SlowPwmSsrHeater() override = default;

    SlowPwmSsrHeater(const SlowPwmSsrHeater&) = delete;
    SlowPwmSsrHeater& operator=(const SlowPwmSsrHeater&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept override;
    hal_cpp::Status set_duty_pct(float duty_pct) noexcept override;
    hal_cpp::Status step(std::uint64_t now_ms) noexcept override;

    float duty_pct() const noexcept override;
    bool output_on() const noexcept override;

   private:
    hal_cpp::Status apply_output_(bool on) noexcept;

    Config cfg_{};
    SlowPwmCore pwm_{};
    bool initialised_{false};
    bool output_on_{false};
};

}  // namespace espid::heater
