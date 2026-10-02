// InactivityTimer -- when has the machine been idle long enough to sleep.
//
// Pure C++17, host-testable.
//
// Mirrors ranciliopid's standby timer: any operator action (shot, steam
// switch, refill, settings change) restarts the countdown; when it runs out
// the caller puts the machine to sleep. Conditions that must never be cut
// short (a shot or refill in progress, the brew switch held, an OTA
// download) are the caller's to report by calling touch() while they last.
//
// Lifecycle: construct at boot (boot counts as activity), touch() on every
// action, poll expired() from one task. Changing the timeout keeps the last
// activity, so lowering it can expire the timer immediately.
//
// Not thread-safe; drive from a single task.

#pragma once

#include <cstdint>

namespace espid::power {

class InactivityTimer {
   public:
    struct Config {
        bool          enabled    = false;
        std::uint32_t timeout_ms = 0;
    };

    InactivityTimer(const Config& cfg, std::uint64_t now_ms) noexcept
        : cfg_(cfg), last_activity_ms_(now_ms) {}

    void          set_config(const Config& cfg) noexcept { cfg_ = cfg; }
    const Config& config() const noexcept                { return cfg_; }

    void touch(std::uint64_t now_ms) noexcept { last_activity_ms_ = now_ms; }

    bool expired(std::uint64_t now_ms) const noexcept {
        return cfg_.enabled && idle_ms(now_ms) >= cfg_.timeout_ms;
    }

    // Time left before expiry; 0 once expired, and while disabled.
    std::uint32_t remaining_ms(std::uint64_t now_ms) const noexcept {
        if (!cfg_.enabled) {
            return 0;
        }
        const std::uint64_t idle = idle_ms(now_ms);
        return idle >= cfg_.timeout_ms
                   ? 0U
                   : static_cast<std::uint32_t>(cfg_.timeout_ms - idle);
    }

   private:
    std::uint64_t idle_ms(std::uint64_t now_ms) const noexcept {
        return now_ms > last_activity_ms_ ? now_ms - last_activity_ms_ : 0U;
    }

    Config        cfg_{};
    std::uint64_t last_activity_ms_ = 0;
};

}  // namespace espid::power
