// SSD1306 OLED driver implementing IDisplay via u8g2.
//
// Wraps the nixy4/u8g2 ESP-IDF port (`u8x8_byte_esp32_hw_i2c` +
// `u8x8_gpio_and_delay_esp32_i2c`) and the
// `u8g2_Setup_ssd1306_i2c_128x64_noname_f` setup, exposing only the
// vendor-neutral IDisplay surface to the rest of the app.
//
// Threading: not thread-safe. All draw/flush calls must come from a
// single task. Use a dedicated display task or serialise externally.

#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "hal_cpp/i_display.hpp"
#include "hal_cpp/result.hpp"

extern "C" {
#include "u8g2.h"
#include "esp32_hw_i2c.h"
}

namespace espid::ssd1306 {

class Ssd1306Display final : public hal_cpp::IDisplay {
   public:
    struct Config {
        int           i2c_port      = 0;
        gpio_num_t    sda           = GPIO_NUM_8;
        gpio_num_t    scl           = GPIO_NUM_9;
        std::uint32_t clk_hz        = 400'000;
        std::uint8_t  dev_addr_7bit = 0x3C;
        // Optional hardware reset line. Set to GPIO_NUM_NC for "no
        // dedicated reset pin" (most generic 0.96" modules tie /RES
        // internally).
        gpio_num_t    reset_pin     = GPIO_NUM_NC;
    };

    // Panel dimensions for the 128x64 SSD1306 variant we set up.
    static constexpr std::uint16_t kWidth  = 128;
    static constexpr std::uint16_t kHeight = 64;

    Ssd1306Display() = default;
    ~Ssd1306Display() override;

    Ssd1306Display(const Ssd1306Display&)            = delete;
    Ssd1306Display& operator=(const Ssd1306Display&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept;

    // IDisplay
    hal_cpp::Status init()                                noexcept override;
    std::uint16_t   width()                         const noexcept override { return kWidth; }
    std::uint16_t   height()                        const noexcept override { return kHeight; }
    void            clear()                               noexcept override;
    void            draw_text(std::int16_t x, std::int16_t baseline_y,
                              hal_cpp::Font font, const char* text) noexcept override;
    hal_cpp::Status flush()                               noexcept override;

   private:
    Config                  cfg_{};
    u8g2_esp32_i2c_ctx_t    i2c_ctx_{};
    u8g2_t                  u8g2_{};
    bool                    initialised_ = false;
};

}  // namespace espid::ssd1306
