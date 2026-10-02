// Deep sleep -- low-power standby with switch and daily-schedule wake.
//
// The machine stays mains-powered around the clock; standby is ESP32-S3
// deep sleep. Before sleeping every output is latched at its inactive level
// (heater SSR, valve, pump triac gate, TSIC supply) so nothing can energise
// or float while the CPU is off, and the two latching switches are armed as
// wake sources (see power/wake_pins.hpp). An optional RTC timer wakes the
// machine at the next scheduled time. Any wake is a full reboot, so the
// normal boot path (fill, heat) runs unchanged afterwards.
//
// Lifecycle:
//   - begin_boot() first thing in app_main: releases the latches left by
//     the previous sleep and reports why the chip woke.
//   - enter_deep_sleep() from the power task once the inactivity timer has
//     run out and the other tasks have parked (heater off, panel asleep).
//
// Threading: begin_boot() from app_main before any task starts;
// enter_deep_sleep() from a single task. wake_reason() is safe anywhere
// after begin_boot().

#pragma once

#include <cstdint>
#include <ctime>

namespace espid::app {

enum class WakeReason : std::uint8_t {
    kPowerOn     = 0,  // power-on, reset or OTA reboot: not a wake from sleep
    kSchedule    = 1,  // daily wake time reached
    kBrewSwitch  = 2,
    kSteamSwitch = 3,
    kOther       = 4,
};

const char* to_string(WakeReason r) noexcept;

WakeReason begin_boot() noexcept;
WakeReason wake_reason() noexcept;

// Sleeps until a switch is flipped or, if `wake_at` >= 0, until that epoch.
// Returns false (without sleeping) only if the wake sources cannot be
// armed -- both switches on at once.
bool enter_deep_sleep(std::time_t wake_at) noexcept;

}  // namespace espid::app
