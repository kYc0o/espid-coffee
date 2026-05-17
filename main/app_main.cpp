// ESPID-Coffee — application entry point.
//
// Phase 0..2c: scaffold + HAL contracts + TSIC 306 decoder + RMT driver
// + SSD1306 OLED rendering.  sensor_task publishes the latest decoded
// temperature into an atomic snapshot; display_task renders that
// snapshot to the OLED at ~5 Hz.

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal_cpp/result.hpp"
#include "ssd1306/ssd1306_display.hpp"
#include "tsic306/tsic306_decoder.hpp"
#include "tsic306/tsic306_driver.hpp"

namespace {

constexpr const char* kTag = "espid";

// ZACwire data line. Verify against your wiring; GPIO 1 is the project
// default on the ESP32-S3 N16R8.
constexpr gpio_num_t     kTsicGpio       = GPIO_NUM_1;
constexpr std::uint32_t  kFrameTimeoutMs = 500;

// OLED I2C bus.  ESP32-S3 default I2C pins keep GPIO 0/2 free for
// strapping / future use.
constexpr gpio_num_t    kOledSda         = GPIO_NUM_8;
constexpr gpio_num_t    kOledScl         = GPIO_NUM_9;
constexpr std::uint8_t  kOledAddr7bit    = 0x3C;
constexpr std::uint32_t kDisplayPeriodMs = 200;  // ~5 Hz refresh.

// Atomic snapshot published by sensor_task and consumed by display_task.
// 32-bit values on ESP32-S3 are naturally aligned/atomic, so no spinlock
// is needed; `valid` acts as the publication barrier.
struct Snapshot {
    std::atomic<float>         temp_c{0.0F};
    std::atomic<std::uint16_t> raw{0};
    std::atomic<bool>          valid{false};
};

Snapshot g_snapshot;

struct SensorTaskCtx {
    espid::tsic306::Tsic306Driver*  driver;
    espid::tsic306::Tsic306Decoder* decoder;
};

void sensor_task(void* arg) {
    auto* ctx = static_cast<SensorTaskCtx*>(arg);
    ESP_LOGI(kTag, "sensor_task: running on core %d", xPortGetCoreID());

    for (;;) {
        const auto fr = ctx->driver->next_frame(kFrameTimeoutMs);
        if (!fr.ok()) {
            ESP_LOGW(kTag, "capture error: %s",
                     espid::hal_cpp::to_string(fr.error()));
            ctx->decoder->reset();
            continue;
        }
        const auto& frame = fr.value();
        const auto  dec   = ctx->decoder->decode(frame);
        if (!dec.ok()) {
            ESP_LOGW(kTag, "decode error: %s  raw=0x%04X parity=0x%X",
                     espid::hal_cpp::to_string(dec.error()),
                     frame.raw, frame.parity_mask);
            continue;
        }
        const float t = dec.value();
        g_snapshot.temp_c.store(t, std::memory_order_relaxed);
        g_snapshot.raw.store(frame.raw, std::memory_order_relaxed);
        g_snapshot.valid.store(true, std::memory_order_release);

        ESP_LOGI(kTag, "T=%.2f C  raw=0x%04X  ts=%" PRIu64 " us",
                 static_cast<double>(t), frame.raw, frame.timestamp_us);
    }
}

void display_task(void* arg) {
    auto* display = static_cast<espid::ssd1306::Ssd1306Display*>(arg);
    ESP_LOGI(kTag, "display_task: running on core %d", xPortGetCoreID());

    char big[16];
    char raw[16];

    for (;;) {
        const bool          valid = g_snapshot.valid.load(std::memory_order_acquire);
        const float         t     = g_snapshot.temp_c.load(std::memory_order_relaxed);
        const std::uint16_t r     = g_snapshot.raw.load(std::memory_order_relaxed);

        if (valid && std::isfinite(t)) {
            std::snprintf(big, sizeof(big), "%5.2f", static_cast<double>(t));
        } else {
            std::snprintf(big, sizeof(big), "--.--");
        }
        std::snprintf(raw, sizeof(raw), "raw=0x%04X", static_cast<unsigned>(r));

        display->clear();
        display->draw_text(0,   8, espid::hal_cpp::Font::kSmall, "ESPID-Coffee");
        display->draw_text(8,  40, espid::hal_cpp::Font::kLarge, big);
        display->draw_text(110, 22, espid::hal_cpp::Font::kSmall, "C");
        display->draw_text(0,  62, espid::hal_cpp::Font::kSmall, raw);
        (void)display->flush();

        vTaskDelay(pdMS_TO_TICKS(kDisplayPeriodMs));
    }
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(kTag, "boot OK - ESPID-Coffee v0.0.3");
    ESP_LOGI(kTag, "target=esp32s3 flash=16MB psram=octal/80MHz");

    static espid::tsic306::Tsic306Driver  driver;
    static espid::tsic306::Tsic306Decoder decoder;
    static espid::ssd1306::Ssd1306Display display;

    const espid::tsic306::Tsic306Driver::Config drv_cfg{kTsicGpio, 0};
    const auto                                  s = driver.init(drv_cfg);
    if (!s.ok()) {
        ESP_LOGE(kTag, "TSIC driver init failed: %s - halting",
                 espid::hal_cpp::to_string(s.error()));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const espid::ssd1306::Ssd1306Display::Config dsp_cfg{
        /* i2c_port      */ 0,
        /* sda           */ kOledSda,
        /* scl           */ kOledScl,
        /* clk_hz        */ 400'000U,
        /* dev_addr_7bit */ kOledAddr7bit,
        /* reset_pin     */ GPIO_NUM_NC,
    };
    const auto ds = display.init(dsp_cfg);
    if (!ds.ok()) {
        // OLED is non-critical: continue without it.
        ESP_LOGW(kTag, "OLED init failed: %s - continuing headless",
                 espid::hal_cpp::to_string(ds.error()));
    }

    static SensorTaskCtx ctx{&driver, &decoder};
    BaseType_t           ok = xTaskCreatePinnedToCore(
        &sensor_task, "sensor", 4096, &ctx, 5, nullptr, 1);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "sensor_task create failed - halting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (ds.ok()) {
        ok = xTaskCreatePinnedToCore(
            &display_task, "display", 4096, &display, 4, nullptr, 1);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "display_task create failed - continuing without UI");
        }
    }

    // Idle heartbeat from app_main so we can see the device is alive even if
    // no frames arrive (e.g. sensor disconnected).
    std::uint32_t tick = 0;
    while (true) {
        ESP_LOGI(kTag, "heartbeat tick=%" PRIu32, tick++);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
