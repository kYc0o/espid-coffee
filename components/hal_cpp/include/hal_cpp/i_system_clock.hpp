// ISystemClock — monotonic time source abstraction.
//
// Concrete impl wraps esp_timer_get_time(); host fakes return canned values
// for deterministic testing of rate/age logic.

#pragma once

#include <cstdint>

namespace espid::hal_cpp {

class ISystemClock {
   public:
    virtual ~ISystemClock() = default;

    // Microseconds since boot (monotonic, never wraps in practical lifetimes).
    virtual std::uint64_t now_us() const noexcept = 0;

    virtual std::uint64_t now_ms() const noexcept { return now_us() / 1000U; }
    virtual std::uint64_t now_s()  const noexcept { return now_us() / 1000000U; }
};

}  // namespace espid::hal_cpp
