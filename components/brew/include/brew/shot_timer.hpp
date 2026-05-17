// Shot timer — counts elapsed milliseconds between start() and stop().
//
// Pure logic, host-testable. All timestamps are caller-supplied
// monotonic milliseconds (typically from ISystemClock::now_ms()).
//
// State machine:
//   idle      → start(t)  → running
//   running   → stop(t)   → stopped (preserves elapsed_at_stop)
//   stopped   → start(t)  → running (resumes from elapsed_at_stop)
//   any state → reset()   → idle (elapsed = 0)
//
// Not thread-safe.

#pragma once

#include <cstdint>

namespace espid::brew {

class ShotTimer {
   public:
    ShotTimer() = default;

    void start(std::uint64_t now_ms) noexcept {
        if (running_) {
            return;
        }
        start_ms_ = now_ms;
        running_  = true;
    }

    void stop(std::uint64_t now_ms) noexcept {
        if (!running_) {
            return;
        }
        accumulated_ms_ += static_cast<std::uint32_t>(now_ms - start_ms_);
        running_         = false;
    }

    void reset() noexcept {
        running_        = false;
        start_ms_       = 0;
        accumulated_ms_ = 0;
    }

    bool running() const noexcept { return running_; }

    std::uint32_t elapsed_ms(std::uint64_t now_ms) const noexcept {
        if (running_) {
            return accumulated_ms_ +
                   static_cast<std::uint32_t>(now_ms - start_ms_);
        }
        return accumulated_ms_;
    }

   private:
    bool          running_        = false;
    std::uint64_t start_ms_       = 0;
    std::uint32_t accumulated_ms_ = 0;
};

}  // namespace espid::brew
