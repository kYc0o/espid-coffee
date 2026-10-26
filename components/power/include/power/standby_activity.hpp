// standby_effect() -- what the machine's state means for the standby timer.
//
// Pure C++17, host-testable.
//
// Two kinds of condition keep the machine awake, and mixing them up has
// cost a night of heating twice:
//   - Operator activity restarts the countdown: a shot or a refill while it
//     runs, a steam switch change, a settings save.
//   - Background work only defers the sleep while it runs, leaving the
//     countdown alone: the startup fill and a hawkbit poll or download.
//     (Hawkbit polls every 5 min; restarting on it kept the machine awake
//     for good, 2026-10-02..05.)
// Anything else is neither. In particular the brew switch left on after a
// post-steam refill has finished is not activity: the refill is over and
// the switch only waits to be released (2026-10-07 and -13: 7 h and 11 h
// at 94.5 C). The machine may sleep with it on; switching it off wakes it.
//
// Stateless.

#pragma once

namespace espid::power {

struct MachineActivity {
    bool shot_running    = false;
    bool refill_running  = false;  // pumping now, not finished-and-waiting
    bool steam_changed   = false;  // steam switch edge since the last check
    bool settings_saved  = false;  // since the last check
    bool fill_running    = false;  // startup fill not complete yet
    bool ota_busy        = false;  // hawkbit poll or download in progress
};

struct StandbyEffect {
    bool restart_countdown = false;
    bool defer_sleep       = false;  // do not sleep now, even if due
};

constexpr StandbyEffect standby_effect(const MachineActivity& a) noexcept {
    StandbyEffect e{};
    e.restart_countdown =
        a.shot_running || a.refill_running || a.steam_changed || a.settings_saved;
    e.defer_sleep = e.restart_countdown || a.fill_running || a.ota_busy;
    return e;
}

}  // namespace espid::power
