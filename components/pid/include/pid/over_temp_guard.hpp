// OverTempGuard -- latched boiler over-temperature stop, steam-mode aware.
//
// Pure C++17, host-testable.
//
// Brewing and steaming run the boiler in very different temperature
// ranges, so each has its own stop / release pair. The subtle part is the
// transition out of steam: the boiler is still well above the brew stop
// threshold when the switch goes off, and applying brew limits straight
// away trips a stop on every normal steam session (observed 2026-09-24:
// steam off at 126.5 C, latched for 15.5 min until the boiler reached the
// 100 C release point).
//
// So leaving steam starts a cool-down during which the steam limits stay
// in force. It ends once the boiler has fallen `cooldown_margin_c` below
// the brew stop threshold -- the earliest point where re-arming the brew
// limits cannot be tripped by a sensor flicker at the boundary. Turning
// steam back on cancels it. ranciliopid handles the same transition with
// a dedicated kCoolDown machine state.
//
// Trip and release always use the limits in force for that sample, which
// preserves the existing mode-dependent release behaviour.
//
// Lifecycle: call `update()` once per control step. Non-finite
// temperatures neither trip, release nor end a cool-down, but steam edges
// are still tracked. Not thread-safe; drive from the control task.

#pragma once

#include <cmath>

namespace espid::pid {

class OverTempGuard {
   public:
    struct Limits {
        float stop_c  = 0.0F;  // latch at or above
        float reset_c = 0.0F;  // release at or below
    };

    struct Config {
        Limits brew{};
        Limits steam{};
        float  cooldown_margin_c = 0.0F;  // re-arm brew limits below brew.stop_c - margin
    };

    // What changed during one `update()`, for logging.
    struct Transitions {
        bool tripped          = false;
        bool released         = false;
        bool cooldown_started = false;
        bool cooldown_ended   = false;
    };

    explicit OverTempGuard(const Config& cfg) noexcept : cfg_(cfg) {}

    Transitions update(bool steam_active, float temp_c) noexcept {
        Transitions tr{};
        const bool finite = std::isfinite(temp_c);

        if (steam_active) {
            cooling_down_ = false;  // back in steam: its limits apply anyway
        } else if (was_steam_ && (!finite || temp_c >= cooldown_exit_c())) {
            // Steam just switched off with a hot (or unknown) boiler.
            cooling_down_       = true;
            tr.cooldown_started = true;
        }
        was_steam_ = steam_active;

        if (!finite) {
            return tr;
        }

        if (cooling_down_ && temp_c < cooldown_exit_c()) {
            cooling_down_     = false;
            tr.cooldown_ended = true;
        }

        const Limits& lim = active_limits(steam_active);
        if (!tripped_ && temp_c >= lim.stop_c) {
            tripped_    = true;
            tr.tripped  = true;
        } else if (tripped_ && temp_c <= lim.reset_c) {
            tripped_    = false;
            tr.released = true;
        }
        last_steam_ = steam_active;
        return tr;
    }

    bool tripped() const noexcept      { return tripped_; }
    bool cooling_down() const noexcept { return cooling_down_; }

    // Limits applied on the most recent finite sample.
    const Limits& limits() const noexcept { return active_limits(last_steam_); }

    float cooldown_exit_c() const noexcept {
        return cfg_.brew.stop_c - cfg_.cooldown_margin_c;
    }

   private:
    const Limits& active_limits(bool steam_active) const noexcept {
        return (steam_active || cooling_down_) ? cfg_.steam : cfg_.brew;
    }

    Config cfg_{};
    bool   tripped_      = false;
    bool   cooling_down_ = false;
    bool   was_steam_    = false;
    bool   last_steam_   = false;
};

}  // namespace espid::pid
