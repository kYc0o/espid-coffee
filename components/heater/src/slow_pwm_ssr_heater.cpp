#include "heater/slow_pwm_ssr_heater.hpp"

#include "driver/gpio.h"
#include "esp_err.h"

namespace espid::heater {

namespace {

hal_cpp::Status hw_fault_status() noexcept {
    return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
}

}  // namespace

hal_cpp::Status SlowPwmSsrHeater::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    if (cfg.ssr_gpio < 0 || cfg.pwm_period_ms == 0) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    cfg_ = cfg;

    gpio_config_t gpio_cfg{};
    gpio_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg_.ssr_gpio);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t c = gpio_config(&gpio_cfg);
    if (c != ESP_OK) {
        return hw_fault_status();
    }

    if (!pwm_.configure(cfg_.pwm_period_ms)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    (void)pwm_.set_duty_pct(0.0F);

    const auto s = apply_output_(false);
    if (!s.ok()) {
        return s;
    }

    initialised_ = true;
    return hal_cpp::Status{};
}

hal_cpp::Status SlowPwmSsrHeater::set_duty_pct(float duty_pct) noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }
    if (!pwm_.set_duty_pct(duty_pct)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    return hal_cpp::Status{};
}

hal_cpp::Status SlowPwmSsrHeater::step(std::uint64_t now_ms) noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }

    const bool should_on = pwm_.output_on(now_ms);
    if (should_on == output_on_) {
        return hal_cpp::Status{};
    }
    return apply_output_(should_on);
}

float SlowPwmSsrHeater::duty_pct() const noexcept {
    return pwm_.duty_pct();
}

bool SlowPwmSsrHeater::output_on() const noexcept {
    return output_on_;
}

hal_cpp::Status SlowPwmSsrHeater::apply_output_(bool on) noexcept {
    const int level = (cfg_.active_high == on) ? 1 : 0;
    const esp_err_t w = gpio_set_level(static_cast<gpio_num_t>(cfg_.ssr_gpio), level);
    if (w != ESP_OK) {
        return hw_fault_status();
    }
    output_on_ = on;
    return hal_cpp::Status{};
}

}  // namespace espid::heater
