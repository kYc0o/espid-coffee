// DutyAccumulator -- mean heater duty between two telemetry records.
//
// Pure C++17, host-testable.
//
// The control loop changes duty 5 times a second; telemetry publishes
// every 0.5-10 s. Publishing the instantaneous duty drops every change in
// between, so short heat pulses vanish and delivered energy cannot be
// integrated from the records (the 2026-09-24 idle-hunting analysis ran
// into exactly this). The producer adds every applied duty; the consumer
// takes the mean since its previous take.
//
// Sample count and duty sum (in 0.01 % units) share one 64-bit atomic, so
// a take always sees a count and a sum from the same instant -- no torn
// pair. If nobody takes for kMaxSamples (e.g. the broker is unreachable
// for hours) the window restarts rather than overflowing the sum.
//
// Threading: one producer task, one consumer task. Orderings are relaxed:
// the packed word is the only shared state.

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

namespace espid::pid {

class DutyAccumulator {
   public:
    // 100k samples at 10 000 (100.00 %) sum to 1e9, well inside 32 bits;
    // at 5 Hz that is ~5.5 h without a take.
    static constexpr std::uint32_t kMaxSamples = 100'000U;

    void add(float duty_pct) noexcept {
        const std::uint32_t centi = to_centi(duty_pct);
        std::uint64_t cur = packed_.load(std::memory_order_relaxed);
        for (;;) {
            std::uint32_t n = static_cast<std::uint32_t>(cur >> 32);
            std::uint32_t s = static_cast<std::uint32_t>(cur & 0xFFFF'FFFFU);
            if (n >= kMaxSamples) {  // stale window: start over
                n = 0U;
                s = 0U;
            }
            const std::uint64_t next =
                (static_cast<std::uint64_t>(n + 1U) << 32) | (s + centi);
            if (packed_.compare_exchange_weak(cur, next,
                                              std::memory_order_relaxed,
                                              std::memory_order_relaxed)) {
                return;
            }
        }
    }

    // Mean duty (%) since the previous take; false if nothing was added.
    bool take(float* mean_pct) noexcept {
        const std::uint64_t v = packed_.exchange(0U, std::memory_order_relaxed);
        const auto n = static_cast<std::uint32_t>(v >> 32);
        if (n == 0U || mean_pct == nullptr) {
            return false;
        }
        const auto s = static_cast<std::uint32_t>(v & 0xFFFF'FFFFU);
        *mean_pct = static_cast<float>(s) / (100.0F * static_cast<float>(n));
        return true;
    }

   private:
    static std::uint32_t to_centi(float duty_pct) noexcept {
        if (!std::isfinite(duty_pct) || duty_pct <= 0.0F) {
            return 0U;
        }
        if (duty_pct >= 100.0F) {
            return 10'000U;
        }
        return static_cast<std::uint32_t>(std::lround(duty_pct * 100.0F));
    }

    std::atomic<std::uint64_t> packed_{0U};
};

}  // namespace espid::pid
