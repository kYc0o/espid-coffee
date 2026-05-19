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

#include "brew/brew_button.hpp"
#include "brew/shot_timer.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal_cpp/result.hpp"
#include "ssd1306/ssd1306_display.hpp"
#include "tsic306/tsic306_decoder.hpp"
#include "tsic306/tsic306_driver.hpp"
#include "wifi/esp_wifi_network.hpp"

namespace {

namespace cfg = espid::config;

constexpr const char* kTag = cfg::kLogTag;

const char* wifi_label(espid::hal_cpp::WifiState s) noexcept {
    switch (s) {
        case espid::hal_cpp::WifiState::kConnected:  return "W:OK";
        case espid::hal_cpp::WifiState::kConnecting: return "W:..";
        case espid::hal_cpp::WifiState::kApFallback: return "W:AP";
        case espid::hal_cpp::WifiState::kError:      return "W:ER";
        case espid::hal_cpp::WifiState::kIdle:
        default:                                      return "W:--";
    }
}

const char* configured_ap_ssid() noexcept {
    return (cfg::net::kApSsid != nullptr && cfg::net::kApSsid[0] != '\0')
               ? cfg::net::kApSsid
               : "ESPID-Coffee";
}

bool init_nvs_for_wifi() noexcept {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_OK) {
        return true;
    }

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(kTag, "NVS init returned %s, erasing NVS partition",
                 esp_err_to_name(err));
        if (nvs_flash_erase() != ESP_OK) {
            return false;
        }
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(kTag, "NVS init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool wifi_ready(espid::hal_cpp::WifiState s) noexcept {
    return s == espid::hal_cpp::WifiState::kConnected ||
           s == espid::hal_cpp::WifiState::kApFallback;
}

void draw_wifi_wait_screen(espid::ssd1306::Ssd1306Display& display,
                           espid::hal_cpp::WifiState state,
                           const char* ip) noexcept {
    display.clear();

    switch (state) {
        case espid::hal_cpp::WifiState::kConnected:
            display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                              "Wi-Fi connected");
            break;

        case espid::hal_cpp::WifiState::kApFallback: {
            char ssid_line[24];
            char pw_line[24];
            std::snprintf(ssid_line, sizeof(ssid_line), "SSID:%s",
                          configured_ap_ssid());
            if (cfg::net::kApPassword != nullptr && cfg::net::kApPassword[0] != '\0') {
                std::snprintf(pw_line, sizeof(pw_line), "PW:%s",
                              cfg::net::kApPassword);
            } else {
                std::snprintf(pw_line, sizeof(pw_line), "PW:(open)");
            }
            display.draw_text(0, 4, espid::hal_cpp::Font::kSmall,
                              "AP setup active");
            display.draw_text(0, 22, espid::hal_cpp::Font::kSmall, ssid_line);
            display.draw_text(0, 40, espid::hal_cpp::Font::kSmall, pw_line);
            break;
        }

        case espid::hal_cpp::WifiState::kError:
            display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                              "Wi-Fi error");
            display.draw_text(0, 30, espid::hal_cpp::Font::kSmall,
                              "Check logs");
            break;

        case espid::hal_cpp::WifiState::kConnecting:
        case espid::hal_cpp::WifiState::kIdle:
        default:
            display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                              "Connecting Wi-Fi...");
            break;
    }

    char ip_line[24];
    std::snprintf(ip_line, sizeof(ip_line), "IP:%s", ip != nullptr ? ip : "0.0.0.0");
    display.draw_text(0, 58, espid::hal_cpp::Font::kSmall, ip_line);
    (void)display.flush();
}

espid::ssd1306::Ssd1306Display::Rotation oled_rotation() {
    using R = espid::ssd1306::Ssd1306Display::Rotation;
    switch (cfg::oled::kRotationQuadrants & 0x3) {
        case 1:  return R::k90;
        case 2:  return R::k180;
        case 3:  return R::k270;
        default: return R::k0;
    }
}

// Atomic snapshot published by sensor_task and consumed by display_task.
// 32-bit values on ESP32-S3 are naturally aligned/atomic, so no spinlock
// is needed; `valid` acts as the publication barrier.
struct Snapshot {
    std::atomic<float>         temp_c{0.0F};
    std::atomic<std::uint16_t> raw{0};
    std::atomic<bool>          valid{false};
};

Snapshot g_snapshot;

// Shot-timer snapshot, also published atomically.  display_task reads
// (running, elapsed_ms) to render the bottom line.
struct ShotSnapshot {
    std::atomic<std::uint32_t> elapsed_ms{0};
    std::atomic<bool>          running{false};
};

ShotSnapshot g_shot;
espid::hal_cpp::INetwork* g_network = nullptr;

struct SensorTaskCtx {
    espid::tsic306::Tsic306Driver*  driver;
    espid::tsic306::Tsic306Decoder* decoder;
};

void sensor_task(void* arg) {
    auto* ctx = static_cast<SensorTaskCtx*>(arg);
    ESP_LOGI(kTag, "sensor_task: running on core %d", xPortGetCoreID());

    for (;;) {
        const auto fr = ctx->driver->next_frame(cfg::tsic::kFrameTimeoutMs);
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

    // Hold the boot splash painted by app_main for the configured budget
    // so the user gets to see firmware identity at power-on.
    vTaskDelay(pdMS_TO_TICKS(cfg::oled::kSplashDurationMs));

    char big[16];
    char small[24];

    for (;;) {
        const bool  valid = g_snapshot.valid.load(std::memory_order_acquire);
        const float t     = g_snapshot.temp_c.load(std::memory_order_relaxed);
        const std::uint32_t shot_ms =
            g_shot.elapsed_ms.load(std::memory_order_relaxed);
        const bool shot_active =
            g_shot.running.load(std::memory_order_relaxed);
        espid::hal_cpp::WifiState wifi_state =
            espid::hal_cpp::WifiState::kIdle;
        espid::hal_cpp::INetwork::IpString wifi_ip{};
        std::snprintf(wifi_ip.data(), wifi_ip.size(), "0.0.0.0");
        if (g_network != nullptr) {
            wifi_state = g_network->state();
            wifi_ip = g_network->ip_string();
        }

        char temp_str[16];
        if (valid && std::isfinite(t)) {
            std::snprintf(temp_str, sizeof(temp_str), "%5.2f",
                          static_cast<double>(t));
        } else {
            std::snprintf(temp_str, sizeof(temp_str), "--.--");
        }

        display->clear();
        if (wifi_state == espid::hal_cpp::WifiState::kApFallback) {
            char ap_hdr[20];
            std::snprintf(ap_hdr, sizeof(ap_hdr), "AP:%s", configured_ap_ssid());
            display->draw_text(0, 8, espid::hal_cpp::Font::kSmall, ap_hdr);
            display->draw_text(86, 8, espid::hal_cpp::Font::kSmall,
                               wifi_ip.data());
        } else {
            display->draw_text(0, 8, espid::hal_cpp::Font::kSmall,
                               "ESPID-Coffee");
            display->draw_text(96, 8, espid::hal_cpp::Font::kSmall,
                               wifi_label(wifi_state));
        }

        if (shot_active) {
            // Shot in progress (or just finished): timer takes the big slot,
            // temperature drops to the small footer.
            std::snprintf(big, sizeof(big), "%4.1fs",
                          static_cast<double>(shot_ms) / 1000.0);
            std::snprintf(small, sizeof(small), "T=%s C", temp_str);
            display->draw_text(8,  40, espid::hal_cpp::Font::kLarge, big);
            display->draw_text(0,  62, espid::hal_cpp::Font::kSmall, small);
        } else {
            // Idle: temperature is the headline; timer line shows 0.0 s.
            std::snprintf(small, sizeof(small), "shot %4.1f s",
                          static_cast<double>(shot_ms) / 1000.0);
            display->draw_text(8,  40, espid::hal_cpp::Font::kLarge, temp_str);
            display->draw_text(110, 22, espid::hal_cpp::Font::kSmall, "C");
            display->draw_text(0,  62, espid::hal_cpp::Font::kSmall, small);
        }
        if (wifi_state == espid::hal_cpp::WifiState::kApFallback) {
            char ap_pw[24];
            if (cfg::net::kApPassword != nullptr &&
                cfg::net::kApPassword[0] != '\0') {
                std::snprintf(ap_pw, sizeof(ap_pw), "PW:%s", cfg::net::kApPassword);
            } else {
                std::snprintf(ap_pw, sizeof(ap_pw), "PW:(open)");
            }
            display->draw_text(0, 62, espid::hal_cpp::Font::kSmall, ap_pw);
        }

        (void)display->flush();

        vTaskDelay(pdMS_TO_TICKS(cfg::oled::kRefreshPeriodMs));
    }
}

// Polled brew-button task.  Reads the raw GPIO at kButtonPeriodMs,
// runs it through the BrewButton debounce filter, and toggles the
// ShotTimer on each debounced press (release is ignored: the timer is
// edge-triggered so a quick tap starts it and another tap stops it).
void button_task(void* /*arg*/) {
    ESP_LOGI(kTag, "button_task: running on core %d (gpio=%d)",
             xPortGetCoreID(), static_cast<int>(cfg::brew::kButtonGpio));

    espid::brew::BrewButton::Config bcfg{};
    bcfg.active_low  = cfg::brew::kButtonActiveLow;
    bcfg.debounce_ms = cfg::brew::kDebounceMs;
    espid::brew::BrewButton btn(bcfg);
    espid::brew::ShotTimer  timer;

    for (;;) {
        const int           raw    = gpio_get_level(cfg::brew::kButtonGpio);
        const std::uint64_t now_ms = static_cast<std::uint64_t>(
            esp_timer_get_time() / 1000);

        const auto ev = btn.poll(raw != 0, now_ms);
        if (ev == espid::brew::BrewButton::Event::kPressed) {
            if (timer.running()) {
                timer.stop(now_ms);
                ESP_LOGI(kTag, "brew: stop  elapsed=%u ms",
                         static_cast<unsigned>(timer.elapsed_ms(now_ms)));
            } else {
                timer.reset();
                timer.start(now_ms);
                ESP_LOGI(kTag, "brew: start");
            }
        }

        g_shot.elapsed_ms.store(timer.elapsed_ms(now_ms),
                                std::memory_order_relaxed);
        g_shot.running.store(timer.running(), std::memory_order_relaxed);

        vTaskDelay(pdMS_TO_TICKS(cfg::brew::kPollPeriodMs));
    }
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(kTag, "boot OK - %s v%u.%u.%u (%s)",
             cfg::kFirmwareName,
             static_cast<unsigned>(cfg::kFwVersionMajor),
             static_cast<unsigned>(cfg::kFwVersionMinor),
             static_cast<unsigned>(cfg::kFwVersionPatch),
             cfg::kFwBranch);
    ESP_LOGI(kTag, "target=esp32s3 flash=16MB psram=octal/80MHz");

    static espid::tsic306::Tsic306Driver  driver;
    static espid::tsic306::Tsic306Decoder decoder;
    static espid::ssd1306::Ssd1306Display display;
    static espid::wifi::EspWifiNetwork     wifi;

    const espid::tsic306::Tsic306Driver::Config drv_cfg{cfg::tsic::kDataGpio, 0};
    const auto                                  s = driver.init(drv_cfg);
    if (!s.ok()) {
        ESP_LOGE(kTag, "TSIC driver init failed: %s - halting",
                 espid::hal_cpp::to_string(s.error()));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const espid::ssd1306::Ssd1306Display::Config dsp_cfg{
        /* i2c_port      */ cfg::oled::kI2cBusNum,
        /* sda           */ cfg::oled::kSdaGpio,
        /* scl           */ cfg::oled::kSclGpio,
        /* clk_hz        */ cfg::oled::kI2cFreqHz,
        /* dev_addr_7bit */ cfg::oled::kI2cAddr7bit,
        /* reset_pin     */ GPIO_NUM_NC,
        /* rotation      */ oled_rotation(),
    };
    const auto ds = display.init(dsp_cfg);
    if (!ds.ok()) {
        // OLED is non-critical: continue without it.
        ESP_LOGW(kTag, "OLED init failed: %s - continuing headless",
                 espid::hal_cpp::to_string(ds.error()));
    } else {
        // Boot splash: shown until sensor_task publishes the first sample
        // or kSplashDurationMs elapses, whichever comes first.  Renders
        // banner + firmware version centred on the panel.
        char ver[16];
        std::snprintf(ver, sizeof(ver), "v%u.%u.%u",
                      static_cast<unsigned>(cfg::kFwVersionMajor),
                      static_cast<unsigned>(cfg::kFwVersionMinor),
                      static_cast<unsigned>(cfg::kFwVersionPatch));
        display.clear();
        display.draw_text(8,  28, espid::hal_cpp::Font::kLarge,
                          cfg::kFirmwareName);
        display.draw_text(32, 50, espid::hal_cpp::Font::kSmall, ver);
        (void)display.flush();
    }

    if (cfg::net::kWifiEnabled) {
        if (!init_nvs_for_wifi()) {
            ESP_LOGE(kTag, "Wi-Fi init blocked: NVS is not available");
            if (ds.ok()) {
                display.clear();
                display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                                  "Wi-Fi unavailable");
                display.draw_text(0, 30, espid::hal_cpp::Font::kSmall,
                                  "NVS init failed");
                (void)display.flush();
            }
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else {
            espid::hal_cpp::INetwork::Config wifi_cfg{};
            wifi_cfg.sta_ssid = cfg::net::kStaSsid;
            wifi_cfg.sta_password = cfg::net::kStaPassword;
            wifi_cfg.ap_ssid = cfg::net::kApSsid;
            wifi_cfg.ap_password = cfg::net::kApPassword;
            wifi_cfg.hostname = cfg::net::kHostname;
            wifi_cfg.max_sta_retries = cfg::net::kMaxReconnects;
            wifi_cfg.reconnect_delay_ms = cfg::net::kReconnectDelayMs;

            const auto ws = wifi.init(wifi_cfg);
            if (!ws.ok()) {
                ESP_LOGE(kTag, "Wi-Fi init failed: %s",
                         espid::hal_cpp::to_string(ws.error()));
                if (ds.ok()) {
                    display.clear();
                    display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                                      "Wi-Fi init failed");
                    display.draw_text(0, 30, espid::hal_cpp::Font::kSmall,
                                      "Check logs");
                    (void)display.flush();
                }
                for (;;) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }

            g_network = &wifi;

            // Gate downstream tasks on network readiness so the AP/STA
            // onboarding state is presented first on OLED.
            while (true) {
                const auto wifi_state = wifi.state();
                const auto wifi_ip = wifi.ip_string();
                if (ds.ok()) {
                    draw_wifi_wait_screen(display, wifi_state, wifi_ip.data());
                }
                if (wifi_ready(wifi_state)) {
                    ESP_LOGI(kTag, "Wi-Fi ready (%s, ip=%s)",
                             espid::hal_cpp::to_string(wifi_state),
                             wifi_ip.data());
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
    }

    static SensorTaskCtx ctx{&driver, &decoder};
    BaseType_t           ok = xTaskCreatePinnedToCore(
        &sensor_task, "sensor", cfg::tsic::kTaskStackBytes, &ctx,
        cfg::tsic::kTaskPriority, nullptr, cfg::tsic::kTaskCore);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "sensor_task create failed - halting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (ds.ok()) {
        ok = xTaskCreatePinnedToCore(
            &display_task, "display", cfg::oled::kTaskStackBytes, &display,
            cfg::oled::kTaskPriority, nullptr, cfg::oled::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "display_task create failed - continuing without UI");
        }
    }

    // Brew button.  Hardware debounce + internal pull-up fallback;
    // software debounce per cfg::brew::kDebounceMs.
    if (cfg::brew::kEnabled) {
        gpio_config_t bcfg{};
        bcfg.pin_bit_mask = 1ULL << static_cast<int>(cfg::brew::kButtonGpio);
        bcfg.mode         = GPIO_MODE_INPUT;
        bcfg.pull_up_en   = cfg::brew::kButtonPullupEnable
                                ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        bcfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        bcfg.intr_type    = GPIO_INTR_DISABLE;
        const esp_err_t err = gpio_config(&bcfg);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "brew button gpio_config failed: %s - skipping",
                     esp_err_to_name(err));
        } else {
            ok = xTaskCreatePinnedToCore(
                &button_task, "button", cfg::brew::kTaskStackBytes, nullptr,
                cfg::brew::kTaskPriority, nullptr, cfg::brew::kTaskCore);
            if (ok != pdPASS) {
                ESP_LOGW(kTag, "button_task create failed - no brew control");
            }
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
