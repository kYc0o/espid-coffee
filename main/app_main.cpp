// ESPID-Coffee — application entry point.
//
// Phase 0..2b: scaffold + HAL contracts + TSIC 306 decoder + RMT driver.
// This revision spawns a sensor task that drives the RMT capture and feeds
// the decoder. Display, brew button, Wi-Fi, and web UI come in later phases.

#include <cinttypes>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal_cpp/result.hpp"
#include "tsic306/tsic306_decoder.hpp"
#include "tsic306/tsic306_driver.hpp"

namespace {

constexpr const char* kTag = "espid";

// ZACwire data line. Verify against your wiring; GPIO 1 is the project
// default on the ESP32-S3 N16R8.
constexpr gpio_num_t     kTsicGpio       = GPIO_NUM_1;
constexpr std::uint32_t  kFrameTimeoutMs = 500;

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
        ESP_LOGI(kTag, "T=%.2f C  raw=0x%04X  ts=%" PRIu64 " us",
                 static_cast<double>(dec.value()), frame.raw, frame.timestamp_us);
    }
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(kTag, "boot OK - ESPID-Coffee v0.0.2");
    ESP_LOGI(kTag, "target=esp32s3 flash=16MB psram=octal/80MHz");

    static espid::tsic306::Tsic306Driver  driver;
    static espid::tsic306::Tsic306Decoder decoder;

    const espid::tsic306::Tsic306Driver::Config drv_cfg{kTsicGpio, 0};
    const auto                                  s = driver.init(drv_cfg);
    if (!s.ok()) {
        ESP_LOGE(kTag, "TSIC driver init failed: %s - halting",
                 espid::hal_cpp::to_string(s.error()));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    static SensorTaskCtx ctx{&driver, &decoder};
    const BaseType_t     ok = xTaskCreatePinnedToCore(
        &sensor_task, "sensor", 4096, &ctx, 5, nullptr, 1);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "sensor_task create failed - halting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
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
