// HeaterPhaseTracker -- selects which heater PID gain set is active.
//
// Pure C++17, host-testable.
//
// Phases:
//   kIdle          normal gains, holding the brew setpoint.
//   kBrew          shot running: cold water enters the boiler and the
//                  puck adds flow resistance, so brew gains apply.
//   kBrewTrailing  brew gains held for `trailing_ms` after the shot stops;
//                  the boiler wall and sensor keep reporting the cold-water
//                  dip for a while after the pump stops.
//   kSteam         steam switch engaged; overrides everything else.
//
// Lifecycle: call `update()` once per control step with a monotonic
// clock. Not thread-safe; drive from the control task.

#pragma once

#include <cstdint>

namespace espid::pid {

enum class HeaterPhase : std::uint8_t {
    kIdle         = 0,
    kBrew         = 1,
    kBrewTrailing = 2,
    kSteam        = 3,
};

constexpr const char* to_string(HeaterPhase p) noexcept {
    switch (p) {
        case HeaterPhase::kIdle:         return "idle";
        case HeaterPhase::kBrew:         return "brew";
        case HeaterPhase::kBrewTrailing: return "trail";
        case HeaterPhase::kSteam:        return "steam";
    }
    return "?";
}

class HeaterPhaseTracker {
   public:
    HeaterPhase update(bool steam_active, bool shot_running,
                       std::uint64_t now_ms, std::uint32_t trailing_ms) noexcept {
        HeaterPhase next = HeaterPhase::kIdle;
        if (steam_active) {
            next = HeaterPhase::kSteam;
        } else if (shot_running) {
            next = HeaterPhase::kBrew;
        } else if (phase_ == HeaterPhase::kBrew) {
            next = (trailing_ms > 0U) ? HeaterPhase::kBrewTrailing
                                      : HeaterPhase::kIdle;
        } else if (phase_ == HeaterPhase::kBrewTrailing) {
            next = (now_ms - entered_ms_ < trailing_ms)
                       ? HeaterPhase::kBrewTrailing
                       : HeaterPhase::kIdle;
        }

        if (!started_ || next != phase_) {
            previous_   = phase_;
            phase_      = next;
            entered_ms_ = now_ms;
            changed_    = started_;
            started_    = true;
        } else {
            changed_ = false;
        }
        return phase_;
    }

    HeaterPhase   phase() const noexcept      { return phase_; }
    HeaterPhase   previous() const noexcept   { return previous_; }
    bool          changed() const noexcept    { return changed_; }
    std::uint64_t entered_ms() const noexcept { return entered_ms_; }

    static constexpr bool uses_brew_gains(HeaterPhase p) noexcept {
        return p == HeaterPhase::kBrew || p == HeaterPhase::kBrewTrailing;
    }

   private:
    HeaterPhase   phase_      = HeaterPhase::kIdle;
    HeaterPhase   previous_   = HeaterPhase::kIdle;
    std::uint64_t entered_ms_ = 0;
    bool          started_    = false;
    bool          changed_    = false;
};

}  // namespace espid::pid
