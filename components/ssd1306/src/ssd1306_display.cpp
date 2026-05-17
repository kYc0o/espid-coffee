// Ssd1306Display — IDisplay implementation backed by u8g2.

#include "ssd1306/ssd1306_display.hpp"

#include <cstring>

#include "esp_err.h"
#include "esp_log.h"

namespace espid::ssd1306 {

namespace {

constexpr const char* kTag = "ssd1306_drv";

// Pick a u8g2 font for each logical Font value.
const std::uint8_t* font_for(hal_cpp::Font f) noexcept {
    switch (f) {
        case hal_cpp::Font::kLarge:  return u8g2_font_logisoso24_tn;  // numbers only
        case hal_cpp::Font::kMedium: return u8g2_font_9x15B_tr;
        case hal_cpp::Font::kSmall:  // fall-through
        default:                     return u8g2_font_6x10_tr;
    }
}

}  // namespace

Ssd1306Display::~Ssd1306Display() {
    // u8g2_esp32_i2c_set_default_context() retains pointers into i2c_ctx_;
    // deinit would need the upstream lib to expose a teardown helper.
    // For now leave the bus alone — the device is only ever destroyed on
    // shutdown.
    initialised_ = false;
}

hal_cpp::Status Ssd1306Display::init() noexcept {
    // Default-pin overload — caller should prefer init(cfg). We initialise
    // with the struct defaults (GPIO 8/9, 0x3C).
    Config cfg{};
    return init(cfg);
}

hal_cpp::Status Ssd1306Display::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    cfg_ = cfg;

    // Fill the u8g2 ESP32-port I2C context.
    std::memset(&i2c_ctx_, 0, sizeof(i2c_ctx_));
    i2c_ctx_.cfg.i2c_port      = cfg_.i2c_port;
    i2c_ctx_.cfg.sda_pin       = static_cast<int>(cfg_.sda);
    i2c_ctx_.cfg.scl_pin       = static_cast<int>(cfg_.scl);
    i2c_ctx_.cfg.clk_hz        = cfg_.clk_hz;
    i2c_ctx_.cfg.dev_addr_7bit = cfg_.dev_addr_7bit;
    i2c_ctx_.cfg.timeout_ms    = 1000;
    i2c_ctx_.cfg.reset_pin     = (cfg_.reset_pin == GPIO_NUM_NC)
                                     ? U8G2_ESP32_PIN_UNUSED
                                     : static_cast<int>(cfg_.reset_pin);

    const esp_err_t err = u8g2_esp32_i2c_set_default_context(&i2c_ctx_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "u8g2_esp32_i2c_set_default_context failed: %s",
                 esp_err_to_name(err));
        return hal_cpp::Status(hal_cpp::Error::kBusError);
    }

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2_, U8G2_R0,
        u8x8_byte_esp32_hw_i2c,
        u8x8_gpio_and_delay_esp32_i2c);

    // u8x8 expects the 8-bit-shifted address.
    u8x8_SetI2CAddress(&u8g2_.u8x8,
                       static_cast<std::uint8_t>(cfg_.dev_addr_7bit << 1));

    u8g2_InitDisplay(&u8g2_);
    u8g2_SetPowerSave(&u8g2_, 0);
    u8g2_ClearBuffer(&u8g2_);
    u8g2_SendBuffer(&u8g2_);

    initialised_ = true;
    ESP_LOGI(kTag, "SSD1306 ready (sda=%d scl=%d addr=0x%02X @ %u Hz)",
             static_cast<int>(cfg_.sda), static_cast<int>(cfg_.scl),
             cfg_.dev_addr_7bit, static_cast<unsigned>(cfg_.clk_hz));
    return hal_cpp::Status{};
}

void Ssd1306Display::clear() noexcept {
    if (!initialised_) return;
    u8g2_ClearBuffer(&u8g2_);
}

void Ssd1306Display::draw_text(std::int16_t x, std::int16_t baseline_y,
                               hal_cpp::Font font, const char* text) noexcept {
    if (!initialised_ || text == nullptr) return;
    u8g2_SetFont(&u8g2_, font_for(font));
    u8g2_DrawStr(&u8g2_, x, baseline_y, text);
}

hal_cpp::Status Ssd1306Display::flush() noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }
    u8g2_SendBuffer(&u8g2_);
    return hal_cpp::Status{};
}

}  // namespace espid::ssd1306
