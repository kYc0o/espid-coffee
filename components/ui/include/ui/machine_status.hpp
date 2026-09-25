// MachineStatus -- what the OLED header tells the operator.
//
// Pure C++17, host-testable.
//
// The header line used to be a fixed "ESPID-Coffee". It now names what the
// machine is doing and, when the operator has to act, says what to do: the
// post-steam refill is driven by the brew switch, so the display has to
// prompt for it (press BREW) and for its end (switch BREW off), otherwise
// nobody would know the switch changed meaning after steaming.
//
// `select_status()` resolves the competing conditions by priority, safety
// first. `format_header()` renders the text; everything fits 16 characters
// of the 6x10 font, leaving room for the Wi-Fi label. `needs_attention()`
// marks the prompts the display should blink.
//
// SteamReadyTracker adds hysteresis to "steam ready" so the label does not
// flicker as the boiler temperature crosses a single threshold.
//
// MachineStatusTracker owns that state plus the "already refilled during
// this cool-down" flag and turns raw machine signals into a MachineStatus.
// The OLED and the MQTT status record each run one, so the dashboard keeps
// working when the display failed to initialise. `to_string()` gives the
// stable machine-readable names published over MQTT.
//
// Not thread-safe; each task drives its own instance.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace espid::ui {

enum class MachineStatus : std::uint8_t {
    kIdle              = 0,
    kFilling           = 1,  // startup fill in progress
    kFillFailed        = 2,  // startup fill never confirmed water
    kEmergencyStop     = 3,
    kBrewing           = 4,
    kSteamHeating      = 5,
    kSteamReady        = 6,
    kRefillPrompt      = 7,  // post-steam cool-down: press BREW to refill
    kRefilling         = 8,
    kRefillFullRelease = 9,   // refill confirmed full: switch BREW off
    kRefillTimeoutRelease = 10,  // refill timed out: switch BREW off
    kCoolingDown       = 11,  // post-steam, already refilled: wait to brew
};

struct StatusInputs {
    bool fill_complete   = true;
    bool fill_ok         = true;
    bool estop           = false;
    bool shot_running    = false;
    bool steam_active    = false;
    bool steam_ready     = false;
    bool steam_cooldown  = false;
    bool refill_running  = false;
    bool refill_waiting_release = false;  // finished, triggering switch still on
    bool refill_timed_out       = false;  // valid with refill_waiting_release
    bool refilled_this_cooldown = false;  // a refill confirmed full since steam off
};

constexpr MachineStatus select_status(const StatusInputs& in) noexcept {
    if (in.estop) {
        return MachineStatus::kEmergencyStop;
    }
    if (!in.fill_complete) {
        return MachineStatus::kFilling;
    }
    if (!in.fill_ok) {
        return MachineStatus::kFillFailed;
    }
    if (in.refill_running) {
        return MachineStatus::kRefilling;
    }
    if (in.refill_waiting_release) {
        return in.refill_timed_out ? MachineStatus::kRefillTimeoutRelease
                                   : MachineStatus::kRefillFullRelease;
    }
    if (in.shot_running) {
        return MachineStatus::kBrewing;
    }
    if (in.steam_active) {
        return in.steam_ready ? MachineStatus::kSteamReady
                              : MachineStatus::kSteamHeating;
    }
    if (in.steam_cooldown) {
        // After a confirmed refill the boiler is full; it only has to cool.
        return in.refilled_this_cooldown ? MachineStatus::kCoolingDown
                                         : MachineStatus::kRefillPrompt;
    }
    return MachineStatus::kIdle;
}

// Stable identifiers for telemetry and dashboards (unlike the header text,
// which carries a live counter while refilling).
constexpr const char* to_string(MachineStatus s) noexcept {
    switch (s) {
        case MachineStatus::kIdle:                 return "idle";
        case MachineStatus::kFilling:              return "filling";
        case MachineStatus::kFillFailed:           return "fill_failed";
        case MachineStatus::kEmergencyStop:        return "emergency_stop";
        case MachineStatus::kBrewing:              return "brewing";
        case MachineStatus::kSteamHeating:         return "steam_heating";
        case MachineStatus::kSteamReady:           return "steam_ready";
        case MachineStatus::kRefillPrompt:         return "refill_prompt";
        case MachineStatus::kRefilling:            return "refilling";
        case MachineStatus::kRefillFullRelease:    return "refill_full";
        case MachineStatus::kRefillTimeoutRelease: return "refill_timeout";
        case MachineStatus::kCoolingDown:          return "cooling_down";
    }
    return "unknown";
}

// Prompts that ask the operator to do something.
constexpr bool needs_attention(MachineStatus s) noexcept {
    return s == MachineStatus::kRefillPrompt ||
           s == MachineStatus::kRefillFullRelease ||
           s == MachineStatus::kRefillTimeoutRelease ||
           s == MachineStatus::kEmergencyStop ||
           s == MachineStatus::kFillFailed;
}

// Writes the header text for `s` into `buf`. `refill_elapsed_s` is only
// used while refilling. Returns the number of characters written.
inline int format_header(MachineStatus s, std::uint32_t refill_elapsed_s,
                         char* buf, std::size_t len) noexcept {
    if (buf == nullptr || len == 0U) {
        return 0;
    }
    const char* text = "ESPID-Coffee";
    switch (s) {
        case MachineStatus::kIdle:              text = "ESPID-Coffee"; break;
        case MachineStatus::kFilling:           text = "FILLING BOILER"; break;
        case MachineStatus::kFillFailed:        text = "FILL FAILED"; break;
        case MachineStatus::kEmergencyStop:     text = "EMERGENCY STOP"; break;
        case MachineStatus::kBrewing:           text = "BREWING"; break;
        case MachineStatus::kSteamHeating:      text = "STEAM heating"; break;
        case MachineStatus::kSteamReady:        text = "STEAM ready"; break;
        case MachineStatus::kRefillPrompt:      text = "BREW to refill"; break;
        case MachineStatus::kRefillFullRelease: text = "FULL: BREW off"; break;
        case MachineStatus::kRefillTimeoutRelease: text = "TIMEOUT:BREW off"; break;
        case MachineStatus::kCoolingDown:       text = "COOLING DOWN"; break;
        case MachineStatus::kRefilling:
            return std::snprintf(buf, len, "REFILLING %2us",
                                 static_cast<unsigned>(refill_elapsed_s));
    }
    return std::snprintf(buf, len, "%s", text);
}

// "Steam ready" with hysteresis: ready once within `on_below_c` of the
// setpoint, not ready again only after falling more than `off_below_c`
// below it. Resets whenever steam mode is off.
class SteamReadyTracker {
   public:
    SteamReadyTracker(float on_below_c, float off_below_c) noexcept
        : on_below_c_(on_below_c), off_below_c_(off_below_c) {}

    bool update(bool steam_active, float temp_c, float setpoint_c) noexcept {
        if (!steam_active || !std::isfinite(temp_c) || !std::isfinite(setpoint_c)) {
            ready_ = false;
            return ready_;
        }
        if (!ready_ && temp_c >= setpoint_c - on_below_c_) {
            ready_ = true;
        } else if (ready_ && temp_c < setpoint_c - off_below_c_) {
            ready_ = false;
        }
        return ready_;
    }

   private:
    float on_below_c_;
    float off_below_c_;
    bool  ready_ = false;
};

// Raw machine signals, before any interpretation.
struct RawStatusInputs {
    bool  fill_complete    = true;
    bool  fill_ok          = true;
    bool  estop            = false;
    bool  shot_running     = false;
    bool  steam_active     = false;
    float temp_c           = NAN;  // NaN when the sample is not valid
    float steam_setpoint_c = NAN;
    bool  steam_cooldown   = false;
    bool  refill_running   = false;
    bool  refill_finished  = false;  // finished, triggering switch still on
    bool  refill_full      = false;  // last refill stopped on confirmed pressure
    bool  refill_timed_out = false;  // last refill stopped on the timeout
};

class MachineStatusTracker {
   public:
    MachineStatusTracker(float steam_ready_on_below_c,
                         float steam_ready_off_below_c) noexcept
        : steam_ready_(steam_ready_on_below_c, steam_ready_off_below_c) {}

    MachineStatus update(const RawStatusInputs& raw) noexcept {
        // Each steam session gets its own cool-down; remember whether its
        // boiler has already been confirmed full.
        if (raw.steam_cooldown && !cooldown_was_) {
            refilled_this_cooldown_ = false;
        }
        if (raw.refill_finished && raw.refill_full) {
            refilled_this_cooldown_ = true;
        }
        cooldown_was_ = raw.steam_cooldown;

        StatusInputs in{};
        in.fill_complete          = raw.fill_complete;
        in.fill_ok                = raw.fill_ok;
        in.estop                  = raw.estop;
        in.shot_running           = raw.shot_running;
        in.steam_active           = raw.steam_active;
        in.steam_ready            = steam_ready_.update(raw.steam_active, raw.temp_c,
                                                        raw.steam_setpoint_c);
        in.steam_cooldown         = raw.steam_cooldown;
        in.refill_running         = raw.refill_running;
        in.refill_waiting_release = raw.refill_finished;
        in.refill_timed_out       = raw.refill_timed_out;
        in.refilled_this_cooldown = refilled_this_cooldown_;
        status_ = select_status(in);
        return status_;
    }

    MachineStatus status() const noexcept { return status_; }

   private:
    SteamReadyTracker steam_ready_;
    bool              cooldown_was_           = false;
    bool              refilled_this_cooldown_ = false;
    MachineStatus     status_                 = MachineStatus::kIdle;
};

}  // namespace espid::ui
