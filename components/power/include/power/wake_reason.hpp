// classify_wake() -- why did the chip boot, after deep sleep or otherwise.
//
// Pure C++17, host-testable.
//
// A switch wake is attributed by comparing the switch levels with the ones
// recorded at sleep time, not by the ESP32-S3 ext1 status register: real
// switch wakes on 2026-10-02 read back with no pin set in that register.
// The levels are also what tells a real wake from a glitch: a wake through
// the switch lines with both switches where they were (a pulse picked up
// by the wiring, 2026-10-05 07:29, nobody at the machine) is kSpurious,
// and the caller goes straight back to sleep.
//
// Stateless.

#pragma once

#include <cstdint>

namespace espid::power {

enum class WakeReason : std::uint8_t {
    kPowerOn     = 0,  // power-on, reset or OTA reboot: not a wake from sleep
    kSchedule    = 1,  // daily wake time reached
    kBrewSwitch  = 2,
    kSteamSwitch = 3,
    kOther       = 4,  // woke from sleep with no source we armed
    kClockResync = 5,  // early timer wake: resync the clock, sleep again
    kSpurious    = 6,  // switch-line wake with no switch moved: sleep again
};

struct WakeInputs {
    bool from_deep_sleep = false;  // reset reason is a deep-sleep wake
    bool context_valid   = false;  // the sleep context in RTC memory is ours
    bool by_switch_line  = false;  // ext0 or ext1 in the wake causes
    bool by_timer        = false;
    bool resync_leg      = false;  // the timer was set short of the target
    bool brew_changed    = false;  // level differs from the one at sleep time
    bool steam_changed   = false;
};

constexpr WakeReason classify_wake(const WakeInputs& in) noexcept {
    if (!in.from_deep_sleep || !in.context_valid) {
        return WakeReason::kPowerOn;
    }
    // A switch the operator moved wins over any other source.
    if (in.brew_changed) {
        return WakeReason::kBrewSwitch;
    }
    if (in.steam_changed) {
        return WakeReason::kSteamSwitch;
    }
    if (in.by_switch_line) {
        return WakeReason::kSpurious;
    }
    if (in.by_timer) {
        return in.resync_leg ? WakeReason::kClockResync : WakeReason::kSchedule;
    }
    return WakeReason::kOther;
}

constexpr const char* to_string(WakeReason r) noexcept {
    switch (r) {
        case WakeReason::kPowerOn:     return "power_on";
        case WakeReason::kSchedule:    return "schedule";
        case WakeReason::kBrewSwitch:  return "brew_switch";
        case WakeReason::kSteamSwitch: return "steam_switch";
        case WakeReason::kOther:       return "other";
        case WakeReason::kClockResync: return "clock_resync";
        case WakeReason::kSpurious:    return "spurious";
    }
    return "?";
}

}  // namespace espid::power
