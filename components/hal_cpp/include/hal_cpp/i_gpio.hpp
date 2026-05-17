// IGpio — minimal GPIO contract used by buttons, relays, and discrete I/O.

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

enum class GpioDir : std::uint8_t {
    kInput,
    kOutput,
    kInputPullUp,
    kInputPullDown,
};

enum class GpioLevel : std::uint8_t {
    kLow  = 0,
    kHigh = 1,
};

class IGpio {
   public:
    virtual ~IGpio() = default;

    virtual Status            configure(int pin, GpioDir dir) noexcept = 0;
    virtual Status            write(int pin, GpioLevel level) noexcept = 0;
    virtual Result<GpioLevel> read(int pin) const noexcept = 0;
};

}  // namespace espid::hal_cpp
