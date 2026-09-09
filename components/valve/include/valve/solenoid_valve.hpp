// SolenoidValve -- 3-way solenoid valve on a single relay/SSR output.
//
// The valve is energised ("open") during brew and backflush and released
// ("closed") otherwise. Thin GPIO wrapper; no timing logic.

#pragma once

#include "hal_cpp/result.hpp"

namespace espid::valve {

class SolenoidValve {
   public:
    struct Config {
        int  gpio        = -1;
        bool active_high = true;
    };

    SolenoidValve() = default;
    ~SolenoidValve() = default;

    SolenoidValve(const SolenoidValve&)            = delete;
    SolenoidValve& operator=(const SolenoidValve&) = delete;

    // Configures the pin as an output and drives it to the closed state.
    hal_cpp::Status init(const Config& cfg) noexcept;

    hal_cpp::Status open() noexcept;   // energise (brew / backflush)
    hal_cpp::Status close() noexcept;  // release

    bool is_open() const noexcept { return open_; }

   private:
    hal_cpp::Status drive_(bool open) noexcept;

    Config cfg_{};
    bool   initialised_{false};
    bool   open_{false};
};

}  // namespace espid::valve
