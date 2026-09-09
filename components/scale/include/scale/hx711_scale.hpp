// Hx711Scale -- bit-bang reader for two HX711 amplifiers on a shared clock.
//
// One load cell per drip-tray side; both HX711 share the PD_SCK line and
// expose independent DOUT lines, so a single 24-clock burst reads both
// cells simultaneously. Raw sign-extended counts are handed to ScaleCore
// for calibration/tare/summation.
//
// The clock burst runs inside a critical section: an HX711 powers down if
// PD_SCK is held high for >60 us, so the shift must not be preempted.
//
// Not thread-safe. Drive read()/ready() from a single task.

#pragma once

#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::scale {

class Hx711Scale {
   public:
    struct Config {
        int          data1_gpio  = -1;
        int          data2_gpio  = -1;
        int          clock_gpio  = -1;
        std::uint8_t gain_pulses = 1;  // 1=ch A/128, 2=ch B/32, 3=ch A/64
    };

    Hx711Scale() = default;
    ~Hx711Scale() = default;

    Hx711Scale(const Hx711Scale&)            = delete;
    Hx711Scale& operator=(const Hx711Scale&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept;

    // Both DOUT lines low => a fresh conversion is ready to clock out.
    bool ready() const noexcept;

    // Clock out 24 bits from each cell (call only when ready()).
    hal_cpp::Status read(std::int32_t& raw1, std::int32_t& raw2) noexcept;

   private:
    Config cfg_{};
    bool   initialised_{false};
};

}  // namespace espid::scale
