// Hx711Scale -- ESP-IDF bit-bang glue for dual HX711 read.

#include "scale/hx711_scale.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

namespace espid::scale {

namespace {

// Shared across instances (only one scale expected); guards the clock
// burst from preemption that would power the HX711 down.
portMUX_TYPE g_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

std::int32_t sign_extend24(std::uint32_t v) noexcept {
    if ((v & 0x0080'0000U) != 0U) {
        v |= 0xFF00'0000U;
    }
    return static_cast<std::int32_t>(v);
}

hal_cpp::Status hw_fault() noexcept {
    return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
}

}  // namespace

hal_cpp::Status Hx711Scale::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    if (cfg.data1_gpio < 0 || cfg.data2_gpio < 0 || cfg.clock_gpio < 0 ||
        cfg.gain_pulses < 1 || cfg.gain_pulses > 3) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    cfg_ = cfg;

    gpio_config_t clk_cfg{};
    clk_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg_.clock_gpio);
    clk_cfg.mode = GPIO_MODE_OUTPUT;
    clk_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    clk_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    clk_cfg.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&clk_cfg) != ESP_OK) {
        return hw_fault();
    }
    gpio_set_level(static_cast<gpio_num_t>(cfg_.clock_gpio), 0);

    gpio_config_t data_cfg{};
    data_cfg.pin_bit_mask = (1ULL << static_cast<unsigned>(cfg_.data1_gpio)) |
                            (1ULL << static_cast<unsigned>(cfg_.data2_gpio));
    data_cfg.mode = GPIO_MODE_INPUT;
    data_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    data_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    data_cfg.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&data_cfg) != ESP_OK) {
        return hw_fault();
    }

    initialised_ = true;
    return hal_cpp::Status{};
}

bool Hx711Scale::ready() const noexcept {
    if (!initialised_) {
        return false;
    }
    return gpio_get_level(static_cast<gpio_num_t>(cfg_.data1_gpio)) == 0 &&
           gpio_get_level(static_cast<gpio_num_t>(cfg_.data2_gpio)) == 0;
}

hal_cpp::Status Hx711Scale::read(std::int32_t& raw1, std::int32_t& raw2) noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }

    const auto clk = static_cast<gpio_num_t>(cfg_.clock_gpio);
    const auto d1 = static_cast<gpio_num_t>(cfg_.data1_gpio);
    const auto d2 = static_cast<gpio_num_t>(cfg_.data2_gpio);

    std::uint32_t v1 = 0;
    std::uint32_t v2 = 0;

    portENTER_CRITICAL(&g_hx711_mux);
    for (int i = 0; i < 24; ++i) {
        gpio_set_level(clk, 1);
        esp_rom_delay_us(1);
        v1 = (v1 << 1) | static_cast<std::uint32_t>(gpio_get_level(d1));
        v2 = (v2 << 1) | static_cast<std::uint32_t>(gpio_get_level(d2));
        gpio_set_level(clk, 0);
        esp_rom_delay_us(1);
    }
    // Extra pulses select the next channel/gain.
    for (std::uint8_t i = 0; i < cfg_.gain_pulses; ++i) {
        gpio_set_level(clk, 1);
        esp_rom_delay_us(1);
        gpio_set_level(clk, 0);
        esp_rom_delay_us(1);
    }
    portEXIT_CRITICAL(&g_hx711_mux);

    raw1 = sign_extend24(v1);
    raw2 = sign_extend24(v2);
    return hal_cpp::Status{};
}

}  // namespace espid::scale
