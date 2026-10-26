// WeeklySchedule -- a wake time (or none) for each day of the week.
//
// Pure C++17 on the C library time functions, host-testable.
//
// Day index follows struct tm::tm_wday: 0 = Sunday ... 6 = Saturday.
// next_weekly_wake() returns the first enabled wake strictly after `now`,
// looking up to a week ahead, in the process time zone (TZ + tzset()), so
// each day keeps its local hour across summer-time changes like
// next_daily_wake(). -1 when no day is enabled or the clock is not set.
//
// Stateless apart from the TZ environment.

#pragma once

#include <array>
#include <ctime>

#include "power/daily_wake.hpp"

namespace espid::power {

constexpr int kDaysPerWeek = 7;

struct WeeklySchedule {
    // Minute of day per weekday; -1 = no wake that day.
    std::array<int, kDaysPerWeek> minute_of_day{-1, -1, -1, -1, -1, -1, -1};

    static WeeklySchedule every_day(int minute) noexcept {
        WeeklySchedule s;
        s.minute_of_day.fill(minute);
        return s;
    }
    bool any_enabled() const noexcept {
        for (const int m : minute_of_day) {
            if (m >= 0 && m < kMinutesPerDay) {
                return true;
            }
        }
        return false;
    }
};

inline std::time_t next_weekly_wake(std::time_t now, const WeeklySchedule& s) noexcept {
    if (!clock_is_set(now) || !s.any_enabled()) {
        return -1;
    }
    std::tm local{};
    if (localtime_r(&now, &local) == nullptr) {
        return -1;
    }
    // Today through the same weekday next week (its time may be earlier
    // than now today but later a week on).
    for (int day = 0; day <= kDaysPerWeek; ++day) {
        const int m = s.minute_of_day[static_cast<std::size_t>((local.tm_wday + day) % kDaysPerWeek)];
        if (m < 0 || m >= kMinutesPerDay) {
            continue;
        }
        std::tm want = local;
        want.tm_mday += day;
        want.tm_hour  = m / 60;
        want.tm_min   = m % 60;
        want.tm_sec   = 0;
        want.tm_isdst = -1;
        const std::time_t t = std::mktime(&want);
        if (t > now) {
            return t;
        }
    }
    return -1;
}

}  // namespace espid::power
