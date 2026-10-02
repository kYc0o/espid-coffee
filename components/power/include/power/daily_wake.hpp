// next_daily_wake() -- the next local wall-clock time HH:MM as a UTC epoch.
//
// Pure C++17 on the C library time functions, host-testable.
//
// Uses the process time zone (TZ + tzset(), set once at boot), so a wake
// time stays at the same local hour across summer-time changes: the epoch
// gap between two wakes is 23 h or 25 h on the change days. A local time
// that does not exist on a spring-forward day (e.g. 02:30) is normalised
// by mktime() to the hour after.
//
// Returns the first such instant strictly after `now`, or -1 when the
// minute is out of range or the clock is not set (now before 2020: no NTP
// sync yet since power-on).
//
// Thread-safe apart from the TZ environment itself; localtime_r/mktime
// only.

#pragma once

#include <ctime>

namespace espid::power {

constexpr int         kMinutesPerDay = 24 * 60;
// 2020-01-01T00:00:00Z. Anything earlier means the RTC was never set.
constexpr std::time_t kClockValidAfter = 1577836800;

constexpr bool clock_is_set(std::time_t now) noexcept {
    return now >= kClockValidAfter;
}

inline std::time_t next_daily_wake(std::time_t now, int minute_of_day) noexcept {
    if (minute_of_day < 0 || minute_of_day >= kMinutesPerDay || !clock_is_set(now)) {
        return -1;
    }
    std::tm local{};
    if (localtime_r(&now, &local) == nullptr) {
        return -1;
    }
    // Today first, then the following days; two steps always suffice, the
    // loop only guards against mktime() normalising onto the same instant.
    for (int day = 0; day < 3; ++day) {
        std::tm want = local;
        want.tm_mday += day;
        want.tm_hour  = minute_of_day / 60;
        want.tm_min   = minute_of_day % 60;
        want.tm_sec   = 0;
        want.tm_isdst = -1;  // let mktime() decide for that date
        const std::time_t t = std::mktime(&want);
        if (t > now) {
            return t;
        }
    }
    return -1;
}

}  // namespace espid::power
