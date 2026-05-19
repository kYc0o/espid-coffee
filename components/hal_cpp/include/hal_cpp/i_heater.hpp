// IHeater -- slow-PWM heater output contract.
//
// Implementations drive a heater SSR using a duty cycle in percent over
// a configured period. The caller owns timing and must call step(now_ms)
// periodically (for example from a control task).

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

class IHeater {
   public:
    struct Config {
        int           ssr_gpio      = -1;
        bool          active_high   = true;
        std::uint32_t pwm_period_ms = 1000;
    };

    virtual ~IHeater() = default;

    // Initialize GPIO/output state. Returns kAlreadyInit if called twice.
    virtual Status init(const Config& cfg) noexcept = 0;

    // Set output duty in [0, 100].
    virtual Status set_duty_pct(float duty_pct) noexcept = 0;

    // Advance output state for timestamp `now_ms`.
    virtual Status step(std::uint64_t now_ms) noexcept = 0;

    virtual float duty_pct() const noexcept = 0;
    virtual bool  output_on() const noexcept = 0;
};

}  // namespace espid::hal_cpp
