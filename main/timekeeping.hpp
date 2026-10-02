// Timekeeping -- local time zone and SNTP wall-clock sync.
//
// Lifecycle: init_timezone() once at the very start of app_main (before
// anything formats local time); start_sntp() once the network is up. SNTP
// then resyncs on its own every CONFIG_LWIP_SNTP_UPDATE_DELAY.
//
// Threading: init/start from app_main only; clock_is_set() and
// local_time_text() are safe from any task.

#pragma once

#include <cstddef>
#include <cstdint>

namespace espid::app {

void init_timezone() noexcept;

// Idempotent. Returns false if the SNTP client could not be started.
bool start_sntp() noexcept;

// Blocks up to `timeout_ms` for the first sync after start_sntp().
bool wait_for_sntp(std::uint32_t timeout_ms) noexcept;

// True once the RTC holds a real date (synced since power-on; survives
// deep sleep).
bool clock_is_set() noexcept;

// "YYYY-MM-DD HH:MM" local time, or "unset" before the first sync.
void local_time_text(char* out, std::size_t len) noexcept;

}  // namespace espid::app
