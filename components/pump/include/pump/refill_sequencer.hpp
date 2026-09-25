// RefillSequencer -- post-steam boiler top-up, stopped by pressure.
//
// Pure C++17, host-testable.
//
// Steaming lowers the boiler level: the first wand opening on a flooded
// boiler spits liquid, then every pitcher boils off water (~19 g for 125 g
// of milk, measured 2026-09-24). The Silvia's hot-water switch is not wired,
// so nothing tops the boiler up again except a shot -- which then takes the
// cold slug itself (a ~15 C boiler dip was measured when the startup fill
// did it instead).
//
// A refill pumps gently with the brew valve closed, so water can only go
// into the boiler through the check valve. While a steam space remains, the
// incoming water condenses it and pressure stays near the steam pressure;
// once the boiler is hydraulically full, pressure climbs quickly toward the
// pump's dead-head. The refill ends when pressure has stayed at or above
// `full_pressure_bar` for `confirm_ms`, after `max_duration_ms` as a
// backstop (e.g. the steam knob left open, so the boiler never pressurises),
// or as soon as the caller withdraws permission.
//
// A refill is bound to the switch press that started it: once finished it
// stays finished until the caller withdraws permission (switch released),
// so a switch left on can neither restart a refill nor fall through into a
// shot.
//
// Lifecycle: `start()` on the triggering press, then `step()` every pump
// loop iteration. Not thread-safe; drive from the pump task.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::pump {

class RefillSequencer {
   public:
    enum class State : std::uint8_t {
        kIdle     = 0,
        kRunning  = 1,
        kFinished = 2,  // waiting for the triggering switch to be released
    };

    enum class StopReason : std::uint8_t {
        kNone      = 0,
        kFull      = 1,  // pressure confirmed: boiler hydraulically full
        kTimeout   = 2,  // max duration reached without pressurising
        kCancelled = 3,  // permission withdrawn before full
    };

    struct Config {
        float         power_pct         = 0.0F;
        float         full_pressure_bar = 0.0F;
        std::uint32_t confirm_ms        = 0;
        std::uint32_t max_duration_ms   = 0;
    };

    explicit RefillSequencer(const Config& cfg) noexcept : cfg_(cfg) {}

    // Begin a refill. Ignored unless idle.
    void start(std::uint64_t now_ms) noexcept {
        if (state_ != State::kIdle) {
            return;
        }
        state_       = State::kRunning;
        reason_      = StopReason::kNone;
        started_ms_  = now_ms;
        stopped_ms_  = now_ms;
        above_since_ = 0;
        above_       = false;
    }

    // One pump-loop step. `keep_going` is the caller's permission (switch
    // still on, steam still off). Returns the pump power to apply now.
    float step(std::uint64_t now_ms, bool keep_going, bool pressure_usable,
               float pressure_bar) noexcept {
        finished_now_ = false;

        if (state_ == State::kFinished) {
            if (!keep_going) {
                state_ = State::kIdle;
            }
            return 0.0F;
        }
        if (state_ != State::kRunning) {
            return 0.0F;
        }

        if (!keep_going) {
            finish(now_ms, StopReason::kCancelled);
            state_ = State::kIdle;  // permission is already gone
            return 0.0F;
        }

        if (pressure_usable && std::isfinite(pressure_bar) &&
            pressure_bar >= cfg_.full_pressure_bar) {
            if (!above_) {
                above_       = true;
                above_since_ = now_ms;
            }
            if (now_ms - above_since_ >= cfg_.confirm_ms) {
                finish(now_ms, StopReason::kFull);
                return 0.0F;
            }
        } else {
            above_ = false;
        }

        if (now_ms - started_ms_ >= cfg_.max_duration_ms) {
            finish(now_ms, StopReason::kTimeout);
            return 0.0F;
        }
        return cfg_.power_pct;
    }

    State      state() const noexcept        { return state_; }
    StopReason reason() const noexcept       { return reason_; }
    bool       running() const noexcept      { return state_ == State::kRunning; }
    // True only on the step that ended the refill.
    bool       finished_now() const noexcept { return finished_now_; }

    std::uint32_t duration_ms() const noexcept {
        return static_cast<std::uint32_t>(stopped_ms_ - started_ms_);
    }

   private:
    void finish(std::uint64_t now_ms, StopReason reason) noexcept {
        state_        = State::kFinished;
        reason_       = reason;
        stopped_ms_   = now_ms;
        finished_now_ = true;
    }

    Config        cfg_{};
    State         state_        = State::kIdle;
    StopReason    reason_       = StopReason::kNone;
    std::uint64_t started_ms_   = 0;
    std::uint64_t stopped_ms_   = 0;
    std::uint64_t above_since_  = 0;
    bool          above_        = false;
    bool          finished_now_ = false;
};

constexpr const char* to_string(RefillSequencer::StopReason r) noexcept {
    switch (r) {
        case RefillSequencer::StopReason::kNone:      return "none";
        case RefillSequencer::StopReason::kFull:      return "full";
        case RefillSequencer::StopReason::kTimeout:   return "timeout";
        case RefillSequencer::StopReason::kCancelled: return "cancelled";
    }
    return "?";
}

}  // namespace espid::pump
