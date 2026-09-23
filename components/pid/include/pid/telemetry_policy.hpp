// TelemetryPolicy -- how often heater telemetry is worth publishing.
//
// Pure C++17, host-testable.
//
// Telemetry is only useful for tuning when it resolves the transients:
// the warm-up ramp, a shot and its recovery, steam, and any excursion
// away from setpoint. Steady-state idle needs a slow heartbeat only, so
// the broker (and the cellular/Wi-Fi link) is not flooded between shots.
//
// Modelled on ranciliopid's machine-state tagging, which samples fast
// during brew and cold start and slowly once the boiler is parked.

#pragma once

#include <cmath>
#include <cstdint>

#include "pid/heater_phase.hpp"

namespace espid::pid {

struct TelemetryPolicy {
    struct Config {
        std::uint32_t warmup_ms        = 600'000;  // fast window after boot
        std::uint32_t warmup_period_ms = 1'000;
        std::uint32_t brew_period_ms   = 500;      // brew + trailing
        std::uint32_t active_period_ms = 1'000;    // steam or off-setpoint
        std::uint32_t idle_period_ms   = 10'000;   // settled heartbeat
        float         settle_band_c    = 0.5F;
        float         active_slope_cps = 0.05F;
    };

    // Publish period for the current conditions. `err_c` is
    // setpoint - measurement; `slope_cps` is the filtered dT/dt.
    static std::uint32_t period_ms(const Config& cfg, std::uint64_t uptime_ms,
                                   HeaterPhase phase, float err_c,
                                   float slope_cps) noexcept {
        if (HeaterPhaseTracker::uses_brew_gains(phase)) {
            return cfg.brew_period_ms;
        }
        if (phase == HeaterPhase::kSteam) {
            return cfg.active_period_ms;
        }
        if (uptime_ms < cfg.warmup_ms) {
            return cfg.warmup_period_ms;
        }
        const bool off_setpoint = !std::isfinite(err_c) ||
                                  std::fabs(err_c) > cfg.settle_band_c;
        const bool moving = !std::isfinite(slope_cps) ||
                            std::fabs(slope_cps) > cfg.active_slope_cps;
        if (off_setpoint || moving) {
            return cfg.active_period_ms;
        }
        return cfg.idle_period_ms;
    }
};

}  // namespace espid::pid
