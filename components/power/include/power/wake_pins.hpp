// select_wake_pins() -- deep-sleep wake configuration for the two switches.
//
// Pure C++17, host-testable.
//
// Both machine switches are latching and source 3V3 when on (active-high,
// pulled down), and either one must wake the machine when the operator
// flips it. The ESP32-S3 offers ext1 (several RTC pins, all waking on the
// same level) and ext0 (one RTC pin, one level). So every switch that is
// off at sleep time goes into the ext1 mask, waking on high; a switch left
// on (in practice steam) goes on ext0, waking when it is turned off.
//
// Two switches on cannot both be armed; the caller does not sleep while
// the brew switch is on, so that only rejects an impossible state.
//
// Stateless.

#pragma once

#include <cstdint>

namespace espid::power {

struct WakePins {
    int brew_gpio  = -1;
    int steam_gpio = -1;
};

struct WakeArming {
    bool          ok             = false;
    std::uint64_t ext1_high_mask = 0;   // wake when any of these goes high
    int           ext0_gpio      = -1;  // -1: ext0 unused
    int           ext0_level     = 0;   // level that wakes on ext0
};

inline WakeArming select_wake_pins(const WakePins& pins, bool brew_on,
                                   bool steam_on) noexcept {
    WakeArming a{};
    if (pins.brew_gpio < 0 || pins.steam_gpio < 0 || (brew_on && steam_on)) {
        return a;
    }
    const int  on_gpio   = brew_on ? pins.brew_gpio : (steam_on ? pins.steam_gpio : -1);
    const auto add_off   = [&a](int gpio) { a.ext1_high_mask |= 1ULL << gpio; };
    if (!brew_on) {
        add_off(pins.brew_gpio);
    }
    if (!steam_on) {
        add_off(pins.steam_gpio);
    }
    if (on_gpio >= 0) {
        a.ext0_gpio  = on_gpio;
        a.ext0_level = 0;
    }
    a.ok = true;
    return a;
}

}  // namespace espid::power
