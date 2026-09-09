// SolenoidValve -- ESP-IDF GPIO glue.

#include "valve/solenoid_valve.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

namespace espid::valve {

hal_cpp::Status SolenoidValve::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    if (cfg.gpio < 0) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    cfg_ = cfg;

    gpio_config_t gpio_cfg{};
    gpio_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg_.gpio);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&gpio_cfg) != ESP_OK) {
        return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
    }

    initialised_ = true;
    return drive_(false);
}

hal_cpp::Status SolenoidValve::open() noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }
    return drive_(true);
}

hal_cpp::Status SolenoidValve::close() noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }
    return drive_(false);
}

hal_cpp::Status SolenoidValve::drive_(bool open) noexcept {
    const int active = cfg_.active_high ? 1 : 0;
    const int level = open ? active : (active != 0 ? 0 : 1);
    if (gpio_set_level(static_cast<gpio_num_t>(cfg_.gpio),
                       static_cast<std::uint32_t>(level)) != ESP_OK) {
        return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
    }
    open_ = open;
    return hal_cpp::Status{};
}

}  // namespace espid::valve
