// plan_sleep() -- how long to deep-sleep to hit a wall-clock wake time.
//
// Pure C++17, host-testable.
//
// The board has no 32 kHz crystal, so deep sleep is timed by the internal
// RC oscillator; ESP-IDF calibrates it before sleeping, but it still drifts
// by up to a few percent with temperature -- tens of minutes over a day.
// Instead of one long sleep the planner stops short of the target by a
// margin proportional to the remaining time, the caller resynchronises the
// clock over NTP on that early wake and plans again. Each round shrinks the
// remaining time by the drift fraction, so after two short resync boots the
// last leg is within `precise_window_s` and is slept straight to the
// target.
//
// Stateless.

#pragma once

#include <cmath>
#include <cstdint>
#include <ctime>

namespace espid::power {

struct SleepPlanConfig {
    float         drift_fraction   = 0.03F;  // worst-case RC drift
    std::uint32_t min_margin_s     = 60;     // never stop short by less
    std::uint32_t precise_window_s = 120;    // sleep the last leg directly
};

struct SleepPlan {
    std::uint64_t sleep_us     = 0;      // 0: the wake time has come
    bool          resync_first = false;  // early wake: resync, then plan again
};

inline SleepPlan plan_sleep(std::time_t now, std::time_t target,
                            const SleepPlanConfig& cfg) noexcept {
    SleepPlan plan{};
    if (target <= now) {
        return plan;
    }
    const auto remaining_s = static_cast<std::uint64_t>(target - now);
    if (remaining_s <= cfg.precise_window_s) {
        plan.sleep_us = remaining_s * 1'000'000ULL;
        return plan;
    }
    // Rounded up: erring early only costs one more short resync.
    auto margin_s = static_cast<std::uint64_t>(std::ceil(
        static_cast<double>(remaining_s) * static_cast<double>(cfg.drift_fraction)));
    if (margin_s < cfg.min_margin_s) {
        margin_s = cfg.min_margin_s;
    }
    // A min_margin_s larger than the remaining time would leave nothing to
    // sleep; keep at least 1 s so every round makes progress.
    if (margin_s >= remaining_s) {
        margin_s = remaining_s - 1U;
    }
    plan.sleep_us     = (remaining_s - margin_s) * 1'000'000ULL;
    plan.resync_first = true;
    return plan;
}

}  // namespace espid::power
