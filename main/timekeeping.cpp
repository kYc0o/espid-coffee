// Timekeeping -- see timekeeping.hpp.

#include "timekeeping.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"

#include "config.hpp"
#include "power/daily_wake.hpp"

namespace espid::app {
namespace {

namespace cfg = espid::config;

constexpr const char* kTag = "time";

bool g_sntp_started = false;

void on_time_sync(struct timeval* /*tv*/) {
    char now[24];
    local_time_text(now, sizeof(now));
    ESP_LOGI(kTag, "SNTP sync: %s local", now);
}

}  // namespace

void init_timezone() noexcept {
    setenv("TZ", cfg::timekeeping::kTimezone, 1);
    tzset();
}

bool start_sntp() noexcept {
    if (g_sntp_started) {
        return true;
    }
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(cfg::timekeeping::kNtpServer);
    sntp_cfg.sync_cb = &on_time_sync;
    const esp_err_t err = esp_netif_sntp_init(&sntp_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "SNTP init failed: %s", esp_err_to_name(err));
        return false;
    }
    g_sntp_started = true;
    return true;
}

bool wait_for_sntp(std::uint32_t timeout_ms) noexcept {
    return g_sntp_started &&
           esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

bool clock_is_set() noexcept {
    return espid::power::clock_is_set(std::time(nullptr));
}

void local_time_text(char* out, std::size_t len) noexcept {
    if (out == nullptr || len == 0) {
        return;
    }
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    if (!espid::power::clock_is_set(now) || localtime_r(&now, &local) == nullptr ||
        std::strftime(out, len, "%Y-%m-%d %H:%M", &local) == 0) {
        std::snprintf(out, len, "unset");
    }
}

}  // namespace espid::app
