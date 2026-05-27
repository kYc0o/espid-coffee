// ABP2 I2C polling driver.
//
// Lifecycle:
// - call init() once
// - call read_sample() periodically from a task
//
// Threading:
// - not thread-safe; use from one task or externally serialise.

#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "abp2/abp2_decoder.hpp"
#include "hal_cpp/result.hpp"

namespace espid::abp2 {

class Abp2Driver final {
   public:
    struct Config {
        int i2c_port{0};
        gpio_num_t sda{GPIO_NUM_8};
        gpio_num_t scl{GPIO_NUM_9};
        std::uint32_t i2c_freq_hz{400'000};
        std::uint8_t i2c_addr_7bit{0x28};
        std::uint32_t timeout_ms{50};
        std::uint32_t command_delay_ms{10};

        TransferFunction transfer{};
    };

    Abp2Driver() = default;
    ~Abp2Driver() = default;

    Abp2Driver(const Abp2Driver&) = delete;
    Abp2Driver& operator=(const Abp2Driver&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept;
    hal_cpp::Result<DecodedSample> read_sample() noexcept;
    bool last_raw_frame(std::uint8_t out[7]) const noexcept;

   private:
    Config cfg_{};
    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t dev_{nullptr};
    std::uint8_t last_raw_[7]{};
    bool has_last_raw_{false};
    bool owns_bus_{false};
    bool initialised_{false};
};

}  // namespace espid::abp2
