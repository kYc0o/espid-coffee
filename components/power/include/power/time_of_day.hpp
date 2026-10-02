// Time-of-day text <-> minute of day, for the wake-time setting.
//
// Pure C++17, host-testable.
//
// Accepts "H:MM" or "HH:MM" (24 h); anything else -- seconds, AM/PM,
// out-of-range fields, trailing text -- is rejected, so a malformed web
// form value can never become a silently wrong wake time.
//
// Stateless.

#pragma once

#include <cstddef>
#include <cstdio>

namespace espid::power {

inline bool parse_time_of_day(const char* text, int* minute_of_day) noexcept {
    if (text == nullptr || minute_of_day == nullptr) {
        return false;
    }
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    std::size_t i = 0;
    int hours = 0;
    std::size_t hour_digits = 0;
    while (digit(text[i]) && hour_digits < 2) {
        hours = hours * 10 + (text[i] - '0');
        ++i;
        ++hour_digits;
    }
    if (hour_digits == 0 || text[i] != ':') {
        return false;
    }
    ++i;
    if (!digit(text[i]) || !digit(text[i + 1]) || text[i + 2] != '\0') {
        return false;
    }
    const int minutes = (text[i] - '0') * 10 + (text[i + 1] - '0');
    if (hours > 23 || minutes > 59) {
        return false;
    }
    *minute_of_day = hours * 60 + minutes;
    return true;
}

// Writes "HH:MM"; returns false if the minute is out of range or the
// buffer is too small (6 bytes needed).
inline bool format_time_of_day(int minute_of_day, char* out, std::size_t len) noexcept {
    if (out == nullptr || len < 6 || minute_of_day < 0 || minute_of_day >= 24 * 60) {
        return false;
    }
    const int n = std::snprintf(out, len, "%02d:%02d", minute_of_day / 60,
                                minute_of_day % 60);
    return n == 5;
}

}  // namespace espid::power
