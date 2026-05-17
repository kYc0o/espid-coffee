// ESPID-Coffee — application entry point.
//
// Phase 0: prove the toolchain, partition table, and boot path work on the
// ESP32-S3 N16R8 target. Subsequent phases add HAL interfaces (P1), TSIC 306
// + OLED (P2), brew button + timer (P3), Wi-Fi provisioning (P4), web UI (P5),
// and safety/OTA scaffolding (P6).

#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char* kTag = "espid";
}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(kTag, "boot OK — ESPID-Coffee v%d.%d.%d",
             0, 0, 1);
    ESP_LOGI(kTag, "target=esp32s3 flash=16MB psram=octal/80MHz");

    // Heartbeat so we can see the device is alive on the monitor.
    uint32_t tick = 0;
    while (true) {
        ESP_LOGI(kTag, "tick=%lu", static_cast<unsigned long>(tick++));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
