// ESPID-Coffee — application entry point.
//
// Phase 0..2c: scaffold + HAL contracts + TSIC 306 decoder + RMT driver
// + SSD1306 OLED rendering.  sensor_task publishes the latest decoded
// temperature into an atomic snapshot; display_task renders that
// snapshot to the OLED at ~5 Hz.

#include <atomic>
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include "abp2/abp2_driver.hpp"
#include "brew/brew_button.hpp"
#include "brew/shot_timer.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "hal_cpp/result.hpp"
#include "heater/slow_pwm_ssr_heater.hpp"
#include "pid/controller.hpp"
#include "pid/duty_accumulator.hpp"
#include "pid/heater_phase.hpp"
#include "pid/over_temp_guard.hpp"
#include "pid/telemetry_policy.hpp"
#include "pump/refill_sequencer.hpp"
#include "ui/machine_status.hpp"
#include "pid/shot_thermal_stats.hpp"
#include "pump/psm_pump.hpp"
#include "scale/hx711_scale.hpp"
#include "scale/scale_core.hpp"
#include "ssd1306/ssd1306_display.hpp"
#include "tsic306/tsic306_decoder.hpp"
#include "tsic306/tsic306_driver.hpp"
#include "valve/solenoid_valve.hpp"
#include "wifi/esp_wifi_network.hpp"
#include "esp_wifi.h"
#include "hawkbit/hawkbit_client.hpp"
#include "esp_system.h"

namespace {

namespace cfg = espid::config;

constexpr const char* kTag = cfg::kLogTag;

extern const std::uint8_t mqtt_client_crt_start[] asm("_binary_mqtt_client_crt_start");
extern const std::uint8_t mqtt_client_key_start[] asm("_binary_mqtt_client_key_start");

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
    // AP fallback means onboarding is available, but boot should stay
    // on the setup screen until an actual network connection is present.
    return s == espid::hal_cpp::WifiState::kConnected;
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
            display.draw_text(0, 12, espid::hal_cpp::Font::kSmall,
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
    display.draw_text(0, 60, espid::hal_cpp::Font::kSmall, ip_line);
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
    std::atomic<std::uint64_t> timestamp_ms{0};
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

// Pressure snapshot published by pressure_task.
struct PressureSnapshot {
    std::atomic<float>         pressure_bar{0.0F};
    std::atomic<float>         pressure_psi{0.0F};
    std::atomic<float>         sensor_temp_c{0.0F};
    std::atomic<std::uint32_t> pressure_counts{0};
    std::atomic<bool>          stale{false};
    std::atomic<bool>          valid{false};
};

PressureSnapshot g_pressure;
espid::hal_cpp::INetwork* g_network = nullptr;
httpd_handle_t g_http_server = nullptr;
char g_saved_sta_ssid[cfg::net::kStaSsidMaxLen + 1]{};
char g_saved_sta_password[cfg::net::kStaPasswordMaxLen + 1]{};

constexpr const char* kCfgNvsNamespace = "app_cfg";
constexpr const char* kCfgPidKp = "pid_kp";
constexpr const char* kCfgPidTn = "pid_tn";
constexpr const char* kCfgPidTv = "pid_tv";
constexpr const char* kCfgPumpKp = "pump_kp";
constexpr const char* kCfgPumpKi = "pump_ki";
constexpr const char* kCfgPumpKd = "pump_kd";
constexpr const char* kCfgBrewKp = "brew_kp";
constexpr const char* kCfgBrewTn = "brew_tn";
constexpr const char* kCfgBrewTv = "brew_tv";
constexpr const char* kCfgBrewTrailing = "brew_trail";
constexpr const char* kCfgBrewPulse = "brew_pulse";
constexpr const char* kCfgHawkbitServer = "hb_server";
constexpr const char* kCfgHawkbitToken  = "hb_token";
constexpr const char* kCfgBrewSetpoint = "brew_sp";
constexpr const char* kCfgSteamSetpoint = "steam_sp";
constexpr const char* kCfgBrewTime = "brew_t";
constexpr const char* kCfgPreinfusion = "preinf_t";
constexpr const char* kCfgPreinfusionPause = "preinf_pause_t";
constexpr const char* kCfgMqttHost = "mqtt_host";
constexpr const char* kCfgMqttPort = "mqtt_port";
constexpr const char* kCfgMqttUsername = "mqtt_user";
constexpr const char* kCfgMqttPassword = "mqtt_pass";
constexpr const char* kCfgMqttClientId = "mqtt_client";

bool load_setting_from_nvs(const char* key, const char* fallback,
                           char* out, std::size_t out_len) noexcept;

float default_pid_tn_seconds() noexcept {
    return (cfg::pid::kKi > 0.0F) ? (cfg::pid::kKp / cfg::pid::kKi) : 0.0F;
}

float default_pid_tv_seconds() noexcept {
    return (cfg::pid::kKp > 0.0F) ? (cfg::pid::kKd / cfg::pid::kKp) : 0.0F;
}

struct RuntimeControlSettings {
    std::atomic<float> pid_kp{cfg::pid::kKp};
    std::atomic<float> pid_tn_s{default_pid_tn_seconds()};
    std::atomic<float> pid_tv_s{default_pid_tv_seconds()};
    std::atomic<float> brew_setpoint_c{cfg::pid::kSetpointC};
    std::atomic<float> steam_setpoint_c{cfg::pid::kSteamSetpointC};
    std::atomic<float> pump_kp{cfg::pump::kPidKp};
    std::atomic<float> pump_ki{cfg::pump::kPidKi};
    std::atomic<float> pump_kd{cfg::pump::kPidKd};
    std::atomic<float> brew_kp{cfg::pid::kBrewKp};
    std::atomic<float> brew_tn_s{cfg::pid::kBrewTnS};
    std::atomic<float> brew_tv_s{cfg::pid::kBrewTvS};
    std::atomic<float> brew_trailing_s{cfg::pid::kBrewTrailingS};
    std::atomic<float> brew_pulse_s{cfg::pid::kBrewPulseS};
};

// Plain-value mirror of RuntimeControlSettings for load/save paths.
struct ControlSettingsValues {
    float pid_kp          = cfg::pid::kKp;
    float pid_tn_s        = default_pid_tn_seconds();
    float pid_tv_s        = default_pid_tv_seconds();
    float brew_setpoint_c = cfg::pid::kSetpointC;
    float steam_setpoint_c = cfg::pid::kSteamSetpointC;
    float pump_kp         = cfg::pump::kPidKp;
    float pump_ki         = cfg::pump::kPidKi;
    float pump_kd         = cfg::pump::kPidKd;
    float brew_kp         = cfg::pid::kBrewKp;
    float brew_tn_s       = cfg::pid::kBrewTnS;
    float brew_tv_s       = cfg::pid::kBrewTvS;
    float brew_trailing_s = cfg::pid::kBrewTrailingS;
    float brew_pulse_s    = cfg::pid::kBrewPulseS;
};

RuntimeControlSettings g_runtime_control;
std::atomic<float> g_control_duty_pct{0.0F};
// Every applied heater duty, drained into each telemetry record so short
// pulses between publishes still count toward the delivered energy.
espid::pid::DutyAccumulator g_duty_accum;
std::atomic<float> g_pump_power_pct{0.0F};

// Heater-loop state published every control step for telemetry. Scalars
// only; a torn read costs one slightly inconsistent telemetry record.
struct ControlSnapshot {
    std::atomic<float>         setpoint_c{0.0F};
    std::atomic<float>         error_c{0.0F};
    std::atomic<float>         slope_cps{0.0F};
    std::atomic<float>         p_term{0.0F};
    std::atomic<float>         i_term{0.0F};
    std::atomic<float>         d_term{0.0F};
    std::atomic<float>         unclamped{0.0F};
    std::atomic<float>         kp{0.0F};
    std::atomic<float>         ki{0.0F};
    std::atomic<float>         kd{0.0F};
    std::atomic<std::uint8_t>  phase{0};
    std::atomic<bool>          feed_forward{false};
    std::atomic<bool>          valid{false};
};

ControlSnapshot g_control;

// Per-shot thermal summary handed to mqtt_task. `seq` is bumped after the
// fields are written; the reader re-checks it to detect a torn record.
struct ShotSummarySnapshot {
    std::atomic<std::uint32_t> seq{0};
    std::atomic<float>         setpoint_c{0.0F};
    std::atomic<float>         start_temp_c{0.0F};
    std::atomic<float>         min_temp_c{0.0F};
    std::atomic<float>         max_after_min_c{0.0F};
    std::atomic<std::uint32_t> min_at_ms{0};
    std::atomic<std::uint32_t> brew_ms{0};
    std::atomic<std::uint32_t> window_ms{0};
    std::atomic<std::uint32_t> settled_at_ms{0};
    std::atomic<bool>          settled{false};
    std::atomic<float>         avg_duty_brew{0.0F};
    std::atomic<float>         avg_duty_trail{0.0F};
};

ShotSummarySnapshot g_shot_summary;

// Post-steam refill: button_task raises g_refill_switch for a brew press made
// while control_task reports the steam cool-down (g_steam_cooldown); pump_task
// runs the refill and fills RefillSnapshot, bumping start_seq / stop_seq after
// the fields so mqtt_task can publish each edge once.
std::atomic<bool> g_steam_cooldown{false};
std::atomic<bool> g_refill_switch{false};

struct RefillSnapshot {
    std::atomic<std::uint32_t> start_seq{0};
    std::atomic<std::uint32_t> stop_seq{0};
    std::atomic<float>         start_temp_c{0.0F};
    std::atomic<float>         start_pressure_bar{0.0F};
    std::atomic<float>         end_temp_c{0.0F};
    std::atomic<float>         end_pressure_bar{0.0F};
    std::atomic<std::uint32_t> duration_ms{0};
    std::atomic<std::uint32_t> fired_half_cycles{0};
    std::atomic<std::uint8_t>  reason{0};
    std::atomic<std::uint8_t>  state{0};        // RefillSequencer::State, every loop
    std::atomic<std::uint64_t> started_ms{0};
};

RefillSnapshot g_refill;

// Incremented by app_main's heartbeat loop; mqtt_task publishes a status
// record whenever it changes.
std::atomic<std::uint32_t> g_heartbeat_tick{0};

std::atomic<float> g_weight_g{0.0F};
std::atomic<std::uint32_t> g_sensor_consecutive_errors{0};
std::atomic<std::uint32_t> g_pressure_consecutive_errors{0};
std::atomic<bool> g_mqtt_connected{false};
std::atomic<bool> g_emergency_stop{false};
std::atomic<bool> g_steam_active{false};
std::atomic<bool> g_fill_complete{false};
std::atomic<bool> g_fill_ok{false};
SemaphoreHandle_t g_i2c_bus_mutex = nullptr;

// Raw machine signals for ui::MachineStatusTracker. Both the OLED and the
// MQTT status record interpret them, each with its own tracker, so the
// dashboard keeps working when the display failed to initialise.
espid::ui::RawStatusInputs gather_status_inputs() noexcept {
    using RefillState = espid::pump::RefillSequencer::State;
    using StopReason  = espid::pump::RefillSequencer::StopReason;
    const auto refill_state =
        static_cast<RefillState>(g_refill.state.load(std::memory_order_acquire));
    const auto refill_reason =
        static_cast<StopReason>(g_refill.reason.load(std::memory_order_relaxed));
    const bool temp_valid = g_snapshot.valid.load(std::memory_order_acquire);

    espid::ui::RawStatusInputs r{};
    r.fill_complete =
        cfg::pump::kTestMode || g_fill_complete.load(std::memory_order_acquire);
    r.fill_ok          = cfg::pump::kTestMode || g_fill_ok.load(std::memory_order_acquire);
    r.estop            = g_emergency_stop.load(std::memory_order_relaxed);
    r.shot_running     = g_shot.running.load(std::memory_order_relaxed);
    r.steam_active     = g_steam_active.load(std::memory_order_relaxed);
    r.temp_c           = temp_valid ? g_snapshot.temp_c.load(std::memory_order_relaxed) : NAN;
    r.steam_setpoint_c = g_runtime_control.steam_setpoint_c.load(std::memory_order_relaxed);
    r.steam_cooldown   = g_steam_cooldown.load(std::memory_order_acquire);
    r.refill_running   = refill_state == RefillState::kRunning;
    r.refill_finished  = refill_state == RefillState::kFinished;
    r.refill_full      = refill_reason == StopReason::kFull;
    r.refill_timed_out = refill_reason == StopReason::kTimeout;
    return r;
}

TickType_t ticks_from_ms(std::uint32_t ms) noexcept {
    const TickType_t t = pdMS_TO_TICKS(ms);
    return (t > 0) ? t : 1;
}

int hex_value(char c) noexcept {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

bool url_decode_segment(const char* src, std::size_t src_len,
                        char* dst, std::size_t dst_len) noexcept {
    if (src == nullptr || dst == nullptr || dst_len == 0) {
        return false;
    }

    std::size_t di = 0;
    for (std::size_t si = 0; si < src_len; ++si) {
        char out = src[si];
        if (out == '+') {
            out = ' ';
        } else if (out == '%') {
            if (si + 2 >= src_len) {
                return false;
            }
            const int hi = hex_value(src[si + 1]);
            const int lo = hex_value(src[si + 2]);
            if (hi < 0 || lo < 0) {
                return false;
            }
            out = static_cast<char>((hi << 4) | lo);
            si += 2;
        }

        if (di + 1 >= dst_len) {
            return false;
        }
        dst[di++] = out;
    }

    dst[di] = '\0';
    return true;
}

bool form_field(const char* body, const char* key,
                char* out, std::size_t out_len) noexcept {
    if (body == nullptr || key == nullptr || out == nullptr || out_len == 0) {
        return false;
    }

    char key_eq[24]{};
    const int key_n = std::snprintf(key_eq, sizeof(key_eq), "%s=", key);
    if (key_n <= 0 || static_cast<std::size_t>(key_n) >= sizeof(key_eq)) {
        return false;
    }

    const char* start = std::strstr(body, key_eq);
    if (start == nullptr) {
        return false;
    }
    start += std::strlen(key_eq);

    const char* end = std::strchr(start, '&');
    if (end == nullptr) {
        end = start + std::strlen(start);
    }

    const std::size_t encoded_len = static_cast<std::size_t>(end - start);
    return url_decode_segment(start, encoded_len, out, out_len);
}

bool parse_float_in_range(const char* text,
                          float min_v,
                          float max_v,
                          float* out) noexcept {
    if (text == nullptr || out == nullptr || text[0] == '\0') {
        return false;
    }

    char* end = nullptr;
    const float v = std::strtof(text, &end);
    if (end == text || (end != nullptr && *end != '\0') || !std::isfinite(v)) {
        return false;
    }
    if (v < min_v || v > max_v) {
        return false;
    }

    *out = v;
    return true;
}

bool parse_u16_in_range(const char* text,
                        std::uint16_t min_v,
                        std::uint16_t max_v,
                        std::uint16_t* out) noexcept {
    if (text == nullptr || out == nullptr || text[0] == '\0') {
        return false;
    }

    char* end = nullptr;
    const unsigned long v = std::strtoul(text, &end, 10);
    if (end == text || (end != nullptr && *end != '\0')) {
        return false;
    }
    if (v < static_cast<unsigned long>(min_v) ||
        v > static_cast<unsigned long>(max_v)) {
        return false;
    }

    *out = static_cast<std::uint16_t>(v);
    return true;
}

bool valid_mqtt_host(const char* host) noexcept {
    if (host == nullptr) {
        return false;
    }

    for (const char* p = host; *p != '\0'; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        const bool allowed =
            std::isalnum(c) != 0 || c == '.' || c == '-' || c == '_' || c == ':';
        if (!allowed) {
            return false;
        }
    }
    return true;
}

bool mqtt_endpoint_configured(const char* host) noexcept {
    if (host == nullptr || host[0] == '\0') {
        return false;
    }
    return std::strcmp(host, "0.0.0.0") != 0;
}

struct MqttRuntimeSettings {
    char host[cfg::mqtt::kHostMaxLen]{};
    std::uint16_t port{cfg::mqtt::kServerPort};
    char username[cfg::mqtt::kUsernameMaxLen]{};
    char password[cfg::mqtt::kPasswordMaxLen]{};
    char client_id[cfg::mqtt::kClientIdMaxLen]{};
};

bool same_mqtt_runtime_settings(const MqttRuntimeSettings& a,
                                const MqttRuntimeSettings& b) noexcept {
    return a.port == b.port &&
           std::strcmp(a.host, b.host) == 0 &&
           std::strcmp(a.username, b.username) == 0 &&
           std::strcmp(a.password, b.password) == 0 &&
           std::strcmp(a.client_id, b.client_id) == 0;
}

bool load_mqtt_runtime_from_nvs(MqttRuntimeSettings* out) noexcept {
    if (out == nullptr) {
        return false;
    }

    char port_default[8];
    char port_txt[8];
    std::snprintf(port_default, sizeof(port_default), "%u",
                  static_cast<unsigned>(cfg::mqtt::kServerPort));

    (void)load_setting_from_nvs(kCfgMqttHost, cfg::mqtt::kServerIp,
                                out->host, sizeof(out->host));
    (void)load_setting_from_nvs(kCfgMqttPort, port_default,
                                port_txt, sizeof(port_txt));
    (void)load_setting_from_nvs(kCfgMqttUsername, cfg::mqtt::kUsername,
                                out->username, sizeof(out->username));
    (void)load_setting_from_nvs(kCfgMqttPassword, cfg::mqtt::kPassword,
                                out->password, sizeof(out->password));
    (void)load_setting_from_nvs(kCfgMqttClientId, cfg::mqtt::kClientId,
                                out->client_id, sizeof(out->client_id));

    std::uint16_t parsed_port = cfg::mqtt::kServerPort;
    if (!parse_u16_in_range(port_txt,
                            cfg::mqtt::kServerPortMin,
                            cfg::mqtt::kServerPortMax,
                            &parsed_port)) {
        parsed_port = cfg::mqtt::kServerPort;
    }

    if (!valid_mqtt_host(out->host) || !mqtt_endpoint_configured(out->host)) {
        return false;
    }

    out->port = parsed_port;
    return true;
}

void build_mqtt_topic(char* out,
                      std::size_t out_len,
                      const char* prefix,
                      const char* suffix) noexcept {
    if (out == nullptr || out_len == 0) {
        return;
    }

    const char* pfx = (prefix != nullptr) ? prefix : "";
    if (pfx[0] == '\0') {
        std::snprintf(out, out_len, "%s", suffix);
        return;
    }
    const std::size_t pfx_len = std::strlen(pfx);

    if (pfx_len > 0 && pfx[pfx_len - 1] == '/') {
        std::snprintf(out, out_len, "%s%s", pfx, suffix);
    } else {
        std::snprintf(out, out_len, "%s/%s", pfx, suffix);
    }
}

void mqtt_event_handler(void* /*handler_args*/,
                        esp_event_base_t /*base*/,
                        int32_t event_id,
                        void* event_data) {
    const auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);
    switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
        case MQTT_EVENT_CONNECTED:
            g_mqtt_connected.store(true, std::memory_order_relaxed);
            ESP_LOGI(kTag, "mqtt: connected");
            break;

        case MQTT_EVENT_DISCONNECTED:
            g_mqtt_connected.store(false, std::memory_order_relaxed);
            ESP_LOGW(kTag, "mqtt: disconnected");
            break;

        case MQTT_EVENT_ERROR:
            g_mqtt_connected.store(false, std::memory_order_relaxed);
            if (event != nullptr && event->error_handle != nullptr) {
                ESP_LOGW(kTag, "mqtt: error type=%d tls=0x%x sock_errno=%d",
                         static_cast<int>(event->error_handle->error_type),
                         static_cast<unsigned>(event->error_handle->esp_tls_last_esp_err),
                         event->error_handle->esp_transport_sock_errno);
            } else {
                ESP_LOGW(kTag, "mqtt: error");
            }
            break;

        default:
            break;
    }
}

void publish_runtime_control_settings(const ControlSettingsValues& v) noexcept {
    g_runtime_control.pid_kp.store(v.pid_kp, std::memory_order_relaxed);
    g_runtime_control.pid_tn_s.store(v.pid_tn_s, std::memory_order_relaxed);
    g_runtime_control.pid_tv_s.store(v.pid_tv_s, std::memory_order_relaxed);
    g_runtime_control.brew_setpoint_c.store(v.brew_setpoint_c, std::memory_order_relaxed);
    g_runtime_control.steam_setpoint_c.store(v.steam_setpoint_c, std::memory_order_relaxed);
    g_runtime_control.pump_kp.store(v.pump_kp, std::memory_order_relaxed);
    g_runtime_control.pump_ki.store(v.pump_ki, std::memory_order_relaxed);
    g_runtime_control.pump_kd.store(v.pump_kd, std::memory_order_relaxed);
    g_runtime_control.brew_kp.store(v.brew_kp, std::memory_order_relaxed);
    g_runtime_control.brew_tn_s.store(v.brew_tn_s, std::memory_order_relaxed);
    g_runtime_control.brew_tv_s.store(v.brew_tv_s, std::memory_order_relaxed);
    g_runtime_control.brew_trailing_s.store(v.brew_trailing_s, std::memory_order_relaxed);
    g_runtime_control.brew_pulse_s.store(v.brew_pulse_s, std::memory_order_relaxed);
}

bool save_sta_credentials_to_nvs(const char* ssid, const char* password) noexcept {
    if (ssid == nullptr || ssid[0] == '\0' || password == nullptr) {
        return false;
    }

    nvs_handle_t nvs{};
    esp_err_t err = nvs_open(cfg::net::kProvisionNvsNamespace,
                             NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "NVS open for provisioning failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_str(nvs, cfg::net::kProvisionNvsStaSsidKey, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, cfg::net::kProvisionNvsStaPasswordKey, password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGE(kTag, "NVS save provisioning failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool load_sta_credentials_from_nvs(char* ssid, std::size_t ssid_len,
                                   char* password, std::size_t password_len) noexcept {
    if (ssid == nullptr || password == nullptr || ssid_len == 0 || password_len == 0) {
        return false;
    }

    ssid[0] = '\0';
    password[0] = '\0';

    nvs_handle_t nvs{};
    const esp_err_t open_err = nvs_open(cfg::net::kProvisionNvsNamespace,
                                        NVS_READONLY, &nvs);
    if (open_err != ESP_OK) {
        return false;
    }

    std::size_t ssid_out_len = ssid_len;
    esp_err_t err = nvs_get_str(nvs, cfg::net::kProvisionNvsStaSsidKey,
                                ssid, &ssid_out_len);
    if (err != ESP_OK || ssid[0] == '\0') {
        nvs_close(nvs);
        return false;
    }

    std::size_t pass_out_len = password_len;
    err = nvs_get_str(nvs, cfg::net::kProvisionNvsStaPasswordKey,
                      password, &pass_out_len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        password[0] = '\0';
    } else if (err != ESP_OK) {
        nvs_close(nvs);
        return false;
    }

    nvs_close(nvs);
    return true;
}

void write_fallback(const char* fallback, char* out, std::size_t out_len) noexcept {
    if (out == nullptr || out_len == 0) {
        return;
    }
    if (fallback == nullptr) {
        out[0] = '\0';
        return;
    }

    std::snprintf(out, out_len, "%s", fallback);
}

bool load_setting_from_nvs(const char* key, const char* fallback,
                           char* out, std::size_t out_len) noexcept {
    if (key == nullptr || out == nullptr || out_len == 0) {
        return false;
    }

    write_fallback(fallback, out, out_len);

    nvs_handle_t nvs{};
    const esp_err_t open_err = nvs_open(kCfgNvsNamespace, NVS_READONLY, &nvs);
    if (open_err != ESP_OK) {
        return false;
    }

    std::size_t len = out_len;
    const esp_err_t err = nvs_get_str(nvs, key, out, &len);
    nvs_close(nvs);
    return err == ESP_OK;
}

void load_runtime_control_settings_from_nvs() noexcept {
    ControlSettingsValues v{};

    const auto load = [](const char* key, float min, float max, float* out) {
        char value[32];
        if (load_setting_from_nvs(key, nullptr, value, sizeof(value))) {
            (void)parse_float_in_range(value, min, max, out);
        }
    };
    load(kCfgPidKp, cfg::pid::kKpMin, cfg::pid::kKpMax, &v.pid_kp);
    load(kCfgPidTn, cfg::pid::kTnMinS, cfg::pid::kTnMaxS, &v.pid_tn_s);
    load(kCfgPidTv, cfg::pid::kTvMinS, cfg::pid::kTvMaxS, &v.pid_tv_s);
    load(kCfgBrewSetpoint, cfg::pid::kSetpointMinC, cfg::pid::kSetpointMaxC,
         &v.brew_setpoint_c);
    load(kCfgSteamSetpoint, cfg::pid::kSteamSetpointMinC, cfg::pid::kSteamSetpointMaxC,
         &v.steam_setpoint_c);
    load(kCfgPumpKp, cfg::pump::kPidGainMin, cfg::pump::kPidKpMax, &v.pump_kp);
    load(kCfgPumpKi, cfg::pump::kPidGainMin, cfg::pump::kPidKiMax, &v.pump_ki);
    load(kCfgPumpKd, cfg::pump::kPidGainMin, cfg::pump::kPidKdMax, &v.pump_kd);
    load(kCfgBrewKp, cfg::pid::kKpMin, cfg::pid::kKpMax, &v.brew_kp);
    load(kCfgBrewTn, cfg::pid::kBrewTnMinS, cfg::pid::kTnMaxS, &v.brew_tn_s);
    load(kCfgBrewTv, cfg::pid::kTvMinS, cfg::pid::kTvMaxS, &v.brew_tv_s);
    load(kCfgBrewTrailing, cfg::pid::kBrewTrailingMinS, cfg::pid::kBrewTrailingMaxS,
         &v.brew_trailing_s);
    load(kCfgBrewPulse, cfg::pid::kBrewPulseMinS, cfg::pid::kBrewPulseMaxS,
         &v.brew_pulse_s);

    publish_runtime_control_settings(v);
}

void restart_task(void* /*arg*/) {
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

bool mount_littlefs() noexcept {
    esp_vfs_littlefs_conf_t lfs_cfg{};
    lfs_cfg.base_path             = "/www";
    lfs_cfg.partition_label       = "storage";
    lfs_cfg.format_if_mount_failed = false;
    const esp_err_t err = esp_vfs_littlefs_register(&lfs_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "LittleFS mount failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(kTag, "LittleFS mounted at /www");
    return true;
}

// Serve a static file from the LittleFS VFS as a chunked HTTP response.
esp_err_t serve_static_file(httpd_req_t* req, const char* path,
                             const char* content_type) noexcept {
    FILE* f = fopen(path, "r");
    if (f == nullptr) {
        ESP_LOGE(kTag, "serve_static_file: not found: %s", path);
        return httpd_resp_send_404(req);
    }
    httpd_resp_set_type(req, content_type);
    char buf[512];
    std::size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, static_cast<ssize_t>(n)) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, nullptr, 0);
}

// GET /api/settings — JSON of all persisted settings for form pre-population.
esp_err_t api_settings_get_handler(httpd_req_t* req) {
    char pid_kp[16];
    char pid_tn[16];
    char pid_tv[16];
    char brew_sp[16];
    char steam_sp[16];
    char pump_kp[16];
    char pump_ki[16];
    char pump_kd[16];
    char brew_kp[16];
    char brew_tn[16];
    char brew_tv[16];
    char brew_trail[16];
    char brew_pulse[16];
    char brew_t[16];
    char preinf_t[16];
    char preinf_pause_t[16];
    char mqtt_host[64];
    char mqtt_port[8];
    char mqtt_user[cfg::mqtt::kUsernameMaxLen];
    char mqtt_pass[cfg::mqtt::kPasswordMaxLen];
    char mqtt_client[cfg::mqtt::kClientIdMaxLen];

    const float pid_tn_default =
        (cfg::pid::kKi > 0.0F) ? (cfg::pid::kKp / cfg::pid::kKi) : 0.0F;
    const float pid_tv_default =
        (cfg::pid::kKp > 0.0F) ? (cfg::pid::kKd / cfg::pid::kKp) : 0.0F;

    char pid_kp_default[16];
    char pid_tn_default_str[16];
    char pid_tv_default_str[16];
    char pump_kp_default[16];
    char pump_ki_default[16];
    char pump_kd_default[16];
    char brew_kp_default[16];
    char brew_tn_default[16];
    char brew_tv_default[16];
    char brew_trail_default[16];
    char brew_pulse_default[16];
    char brew_sp_default[16];
    char steam_sp_default[16];
    char brew_t_default[16];
    char preinf_t_default[16];
    char preinf_pause_t_default[16];
    char mqtt_port_default[8];

    std::snprintf(pid_kp_default, sizeof(pid_kp_default), "%.2f",
                  static_cast<double>(cfg::pid::kKp));
    std::snprintf(pid_tn_default_str, sizeof(pid_tn_default_str), "%.2f",
                  static_cast<double>(pid_tn_default));
    std::snprintf(pid_tv_default_str, sizeof(pid_tv_default_str), "%.2f",
                  static_cast<double>(pid_tv_default));
    std::snprintf(pump_kp_default, sizeof(pump_kp_default), "%.3f",
                  static_cast<double>(cfg::pump::kPidKp));
    std::snprintf(pump_ki_default, sizeof(pump_ki_default), "%.3f",
                  static_cast<double>(cfg::pump::kPidKi));
    std::snprintf(pump_kd_default, sizeof(pump_kd_default), "%.3f",
                  static_cast<double>(cfg::pump::kPidKd));
    std::snprintf(brew_kp_default, sizeof(brew_kp_default), "%.2f",
                  static_cast<double>(cfg::pid::kBrewKp));
    std::snprintf(brew_tn_default, sizeof(brew_tn_default), "%.2f",
                  static_cast<double>(cfg::pid::kBrewTnS));
    std::snprintf(brew_tv_default, sizeof(brew_tv_default), "%.2f",
                  static_cast<double>(cfg::pid::kBrewTvS));
    std::snprintf(brew_trail_default, sizeof(brew_trail_default), "%.1f",
                  static_cast<double>(cfg::pid::kBrewTrailingS));
    std::snprintf(brew_pulse_default, sizeof(brew_pulse_default), "%.1f",
                  static_cast<double>(cfg::pid::kBrewPulseS));
    std::snprintf(brew_sp_default, sizeof(brew_sp_default), "%.2f",
                  static_cast<double>(cfg::pid::kSetpointC));
    std::snprintf(steam_sp_default, sizeof(steam_sp_default), "%.2f",
                  static_cast<double>(cfg::pid::kSteamSetpointC));
    std::snprintf(brew_t_default, sizeof(brew_t_default), "%.2f",
                  static_cast<double>(cfg::brew::kBrewTimeS));
    std::snprintf(preinf_t_default, sizeof(preinf_t_default), "%.2f",
                  static_cast<double>(cfg::brew::kPreinfusionS));
    std::snprintf(preinf_pause_t_default, sizeof(preinf_pause_t_default), "%.2f",
                  static_cast<double>(cfg::brew::kPreinfusionPauseS));
    std::snprintf(mqtt_port_default, sizeof(mqtt_port_default), "%u",
                  static_cast<unsigned>(cfg::mqtt::kServerPort));

    (void)load_setting_from_nvs(kCfgPidKp, pid_kp_default, pid_kp, sizeof(pid_kp));
    (void)load_setting_from_nvs(kCfgPidTn, pid_tn_default_str, pid_tn, sizeof(pid_tn));
    (void)load_setting_from_nvs(kCfgPidTv, pid_tv_default_str, pid_tv, sizeof(pid_tv));
    (void)load_setting_from_nvs(kCfgPumpKp, pump_kp_default, pump_kp, sizeof(pump_kp));
    (void)load_setting_from_nvs(kCfgPumpKi, pump_ki_default, pump_ki, sizeof(pump_ki));
    (void)load_setting_from_nvs(kCfgPumpKd, pump_kd_default, pump_kd, sizeof(pump_kd));
    (void)load_setting_from_nvs(kCfgBrewKp, brew_kp_default, brew_kp, sizeof(brew_kp));
    (void)load_setting_from_nvs(kCfgBrewTn, brew_tn_default, brew_tn, sizeof(brew_tn));
    (void)load_setting_from_nvs(kCfgBrewTv, brew_tv_default, brew_tv, sizeof(brew_tv));
    (void)load_setting_from_nvs(kCfgBrewTrailing, brew_trail_default,
                                brew_trail, sizeof(brew_trail));
    (void)load_setting_from_nvs(kCfgBrewPulse, brew_pulse_default,
                                brew_pulse, sizeof(brew_pulse));
    (void)load_setting_from_nvs(kCfgBrewSetpoint, brew_sp_default, brew_sp, sizeof(brew_sp));
    (void)load_setting_from_nvs(kCfgSteamSetpoint, steam_sp_default, steam_sp, sizeof(steam_sp));
    (void)load_setting_from_nvs(kCfgBrewTime, brew_t_default, brew_t, sizeof(brew_t));
    (void)load_setting_from_nvs(kCfgPreinfusion, preinf_t_default, preinf_t, sizeof(preinf_t));
    (void)load_setting_from_nvs(kCfgPreinfusionPause, preinf_pause_t_default,
                                preinf_pause_t, sizeof(preinf_pause_t));
    (void)load_setting_from_nvs(kCfgMqttHost, cfg::mqtt::kServerIp,
                                mqtt_host, sizeof(mqtt_host));
    (void)load_setting_from_nvs(kCfgMqttPort, mqtt_port_default,
                                mqtt_port, sizeof(mqtt_port));
    (void)load_setting_from_nvs(kCfgMqttUsername, cfg::mqtt::kUsername,
                                mqtt_user, sizeof(mqtt_user));
    (void)load_setting_from_nvs(kCfgMqttPassword, cfg::mqtt::kPassword,
                                mqtt_pass, sizeof(mqtt_pass));
    (void)load_setting_from_nvs(kCfgMqttClientId, cfg::mqtt::kClientId,
                                mqtt_client, sizeof(mqtt_client));

    char hb_server[cfg::hawkbit::kServerUrlMaxLen + 1]{};
    char hb_token[cfg::hawkbit::kTokenMaxLen + 1]{};
    (void)load_setting_from_nvs(kCfgHawkbitServer, "", hb_server, sizeof(hb_server));
    (void)load_setting_from_nvs(kCfgHawkbitToken,  "", hb_token,  sizeof(hb_token));

    char json[1536];
    const int n = std::snprintf(json, sizeof(json),
        "{\"ap_ssid\":\"%s\",\"wifi_ssid\":\"%s\","
        "\"pid_kp\":\"%s\",\"pid_tn\":\"%s\",\"pid_tv\":\"%s\","
        "\"pump_kp\":\"%s\",\"pump_ki\":\"%s\",\"pump_kd\":\"%s\","
        "\"brew_kp\":\"%s\",\"brew_tn\":\"%s\",\"brew_tv\":\"%s\","
        "\"brew_trail\":\"%s\",\"brew_pulse\":\"%s\","
        "\"brew_sp\":\"%s\",\"steam_sp\":\"%s\","
        "\"brew_t\":\"%s\",\"preinf_t\":\"%s\",\"preinf_pause_t\":\"%s\","
        "\"mqtt_host\":\"%s\",\"mqtt_port\":\"%s\","
        "\"mqtt_user\":\"%s\",\"mqtt_pass\":\"%s\",\"mqtt_client\":\"%s\","
        "\"hb_server\":\"%s\",\"hb_token\":\"%s\"}",
        configured_ap_ssid(),
        g_saved_sta_ssid,
        pid_kp, pid_tn, pid_tv,
        pump_kp, pump_ki, pump_kd,
        brew_kp, brew_tn, brew_tv, brew_trail, brew_pulse,
        brew_sp, steam_sp,
        brew_t, preinf_t, preinf_pause_t,
        mqtt_host, mqtt_port, mqtt_user, mqtt_pass, mqtt_client,
        hb_server, hb_token);

    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(json)) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t root_get_handler(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return serve_static_file(req, "/www/index.html", "text/html");
}

esp_err_t style_css_handler(httpd_req_t* req) {
    return serve_static_file(req, "/www/style.css", "text/css");
}

esp_err_t app_js_handler(httpd_req_t* req) {
    return serve_static_file(req, "/www/app.js", "application/javascript");
}

esp_err_t wifi_post_handler(httpd_req_t* req) {
    if (req->content_len <= 0 ||
        req->content_len > static_cast<int>(cfg::net::kProvisionFormMaxLen)) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        return httpd_resp_send(req, "Invalid payload", HTTPD_RESP_USE_STRLEN);
    }

    char body[cfg::net::kProvisionFormMaxLen + 1]{};
    int read_total = 0;
    while (read_total < req->content_len) {
        const int chunk = httpd_req_recv(
            req,
            body + read_total,
            req->content_len - read_total);
        if (chunk <= 0) {
            return httpd_resp_send_500(req);
        }
        read_total += chunk;
    }
    body[read_total] = '\0';

    char ssid[cfg::net::kStaSsidMaxLen + 1]{};
    char password[cfg::net::kStaPasswordMaxLen + 1]{};
    if (!form_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Missing SSID", HTTPD_RESP_USE_STRLEN);
    }
    if (!form_field(body, "password", password, sizeof(password))) {
        password[0] = '\0';
    }

    const std::size_t ssid_len = std::strlen(ssid);
    const std::size_t pass_len = std::strlen(password);
    if (ssid_len == 0 || ssid_len > cfg::net::kStaSsidMaxLen) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Invalid SSID length", HTTPD_RESP_USE_STRLEN);
    }
    if (pass_len > cfg::net::kStaPasswordMaxLen) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Invalid password length", HTTPD_RESP_USE_STRLEN);
    }
    if (pass_len > 0 && pass_len < cfg::net::kWpaMinPasswordLen) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Password too short", HTTPD_RESP_USE_STRLEN);
    }

    if (!save_sta_credentials_to_nvs(ssid, password)) {
        return httpd_resp_send_500(req);
    }

    std::snprintf(g_saved_sta_ssid, sizeof(g_saved_sta_ssid), "%s", ssid);
    std::snprintf(g_saved_sta_password, sizeof(g_saved_sta_password), "%s", password);

    ESP_LOGI(kTag, "Wi-Fi credentials saved (ssid=%s), rebooting", ssid);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req,
                    "<html><body><h3>Saved. Rebooting...</h3></body></html>",
                    HTTPD_RESP_USE_STRLEN);

    BaseType_t ok = xTaskCreatePinnedToCore(&restart_task, "reboot",
                                            2048, nullptr, 2, nullptr,
                                            tskNO_AFFINITY);
    if (ok != pdPASS) {
        esp_restart();
    }
    return ESP_OK;
}

esp_err_t settings_post_handler(httpd_req_t* req) {
    if (req->content_len <= 0 ||
        req->content_len > static_cast<int>(cfg::net::kProvisionFormMaxLen)) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        return httpd_resp_send(req, "Invalid payload", HTTPD_RESP_USE_STRLEN);
    }

    char body[cfg::net::kProvisionFormMaxLen + 1]{};
    int read_total = 0;
    while (read_total < req->content_len) {
        const int chunk = httpd_req_recv(
            req,
            body + read_total,
            req->content_len - read_total);
        if (chunk <= 0) {
            return httpd_resp_send_500(req);
        }
        read_total += chunk;
    }
    body[read_total] = '\0';

    const auto reject_bad_request = [req](const char* msg) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
    };

    char pid_kp_txt[16]{};
    char pid_tn_txt[16]{};
    char pid_tv_txt[16]{};
    char pump_kp_txt[16]{};
    char pump_ki_txt[16]{};
    char pump_kd_txt[16]{};
    char brew_kp_txt[16]{};
    char brew_tn_txt[16]{};
    char brew_tv_txt[16]{};
    char brew_trail_txt[16]{};
    char brew_pulse_txt[16]{};
    char brew_sp_txt[16]{};
    char steam_sp_txt[16]{};
    char brew_t_txt[16]{};
    char preinf_t_txt[16]{};
    char preinf_pause_txt[16]{};
    char mqtt_host_txt[64]{};
    char mqtt_port_txt[8]{};
    char mqtt_user_txt[cfg::mqtt::kUsernameMaxLen]{};
    char mqtt_pass_txt[cfg::mqtt::kPasswordMaxLen]{};
    char mqtt_client_txt[cfg::mqtt::kClientIdMaxLen]{};

    if (!form_field(body, "pid_kp", pid_kp_txt, sizeof(pid_kp_txt))) {
        return reject_bad_request("Missing pid_kp");
    }
    if (!form_field(body, "pid_tn", pid_tn_txt, sizeof(pid_tn_txt))) {
        return reject_bad_request("Missing pid_tn");
    }
    if (!form_field(body, "pid_tv", pid_tv_txt, sizeof(pid_tv_txt))) {
        return reject_bad_request("Missing pid_tv");
    }
    if (!form_field(body, "pump_kp", pump_kp_txt, sizeof(pump_kp_txt))) {
        return reject_bad_request("Missing pump_kp");
    }
    if (!form_field(body, "pump_ki", pump_ki_txt, sizeof(pump_ki_txt))) {
        return reject_bad_request("Missing pump_ki");
    }
    if (!form_field(body, "pump_kd", pump_kd_txt, sizeof(pump_kd_txt))) {
        return reject_bad_request("Missing pump_kd");
    }
    if (!form_field(body, "brew_kp", brew_kp_txt, sizeof(brew_kp_txt))) {
        return reject_bad_request("Missing brew_kp");
    }
    if (!form_field(body, "brew_tn", brew_tn_txt, sizeof(brew_tn_txt))) {
        return reject_bad_request("Missing brew_tn");
    }
    if (!form_field(body, "brew_tv", brew_tv_txt, sizeof(brew_tv_txt))) {
        return reject_bad_request("Missing brew_tv");
    }
    if (!form_field(body, "brew_trail", brew_trail_txt, sizeof(brew_trail_txt))) {
        return reject_bad_request("Missing brew_trail");
    }
    if (!form_field(body, "brew_pulse", brew_pulse_txt, sizeof(brew_pulse_txt))) {
        return reject_bad_request("Missing brew_pulse");
    }
    if (!form_field(body, "brew_sp", brew_sp_txt, sizeof(brew_sp_txt))) {
        return reject_bad_request("Missing brew_sp");
    }
    if (!form_field(body, "steam_sp", steam_sp_txt, sizeof(steam_sp_txt))) {
        return reject_bad_request("Missing steam_sp");
    }
    if (!form_field(body, "brew_t", brew_t_txt, sizeof(brew_t_txt))) {
        return reject_bad_request("Missing brew_t");
    }
    if (!form_field(body, "preinf_t", preinf_t_txt, sizeof(preinf_t_txt))) {
        return reject_bad_request("Missing preinf_t");
    }
    if (!form_field(body, "preinf_pause_t", preinf_pause_txt, sizeof(preinf_pause_txt))) {
        return reject_bad_request("Missing preinf_pause_t");
    }
    if (!form_field(body, "mqtt_host", mqtt_host_txt, sizeof(mqtt_host_txt))) {
        return reject_bad_request("Missing mqtt_host");
    }
    if (!form_field(body, "mqtt_port", mqtt_port_txt, sizeof(mqtt_port_txt))) {
        return reject_bad_request("Missing mqtt_port");
    }

    if (!form_field(body, "mqtt_user", mqtt_user_txt, sizeof(mqtt_user_txt))) {
        (void)load_setting_from_nvs(kCfgMqttUsername, cfg::mqtt::kUsername,
                                    mqtt_user_txt, sizeof(mqtt_user_txt));
    }
    if (!form_field(body, "mqtt_pass", mqtt_pass_txt, sizeof(mqtt_pass_txt))) {
        (void)load_setting_from_nvs(kCfgMqttPassword, cfg::mqtt::kPassword,
                                    mqtt_pass_txt, sizeof(mqtt_pass_txt));
    }
    if (!form_field(body, "mqtt_client", mqtt_client_txt, sizeof(mqtt_client_txt))) {
        (void)load_setting_from_nvs(kCfgMqttClientId, cfg::mqtt::kClientId,
                                    mqtt_client_txt, sizeof(mqtt_client_txt));
    }

    char hb_server_txt[cfg::hawkbit::kServerUrlMaxLen + 1]{};
    char hb_token_txt[cfg::hawkbit::kTokenMaxLen + 1]{};
    if (!form_field(body, "hb_server", hb_server_txt, sizeof(hb_server_txt))) {
        (void)load_setting_from_nvs(kCfgHawkbitServer, "",
                                    hb_server_txt, sizeof(hb_server_txt));
    }
    if (!form_field(body, "hb_token", hb_token_txt, sizeof(hb_token_txt))) {
        (void)load_setting_from_nvs(kCfgHawkbitToken, "",
                                    hb_token_txt, sizeof(hb_token_txt));
    }

    ControlSettingsValues v{};
    float brew_t = 0.0F;
    float preinf_t = 0.0F;
    float preinf_pause_t = 0.0F;
    std::uint16_t mqtt_port = 0;

    if (!parse_float_in_range(pid_kp_txt, cfg::pid::kKpMin, cfg::pid::kKpMax, &v.pid_kp)) {
        return reject_bad_request("Invalid pid_kp");
    }
    if (!parse_float_in_range(pid_tn_txt, cfg::pid::kTnMinS, cfg::pid::kTnMaxS, &v.pid_tn_s)) {
        return reject_bad_request("Invalid pid_tn");
    }
    if (!parse_float_in_range(pid_tv_txt, cfg::pid::kTvMinS, cfg::pid::kTvMaxS, &v.pid_tv_s)) {
        return reject_bad_request("Invalid pid_tv");
    }
    if (!parse_float_in_range(pump_kp_txt, cfg::pump::kPidGainMin,
                              cfg::pump::kPidKpMax, &v.pump_kp)) {
        return reject_bad_request("Invalid pump_kp");
    }
    if (!parse_float_in_range(pump_ki_txt, cfg::pump::kPidGainMin,
                              cfg::pump::kPidKiMax, &v.pump_ki)) {
        return reject_bad_request("Invalid pump_ki");
    }
    if (!parse_float_in_range(pump_kd_txt, cfg::pump::kPidGainMin,
                              cfg::pump::kPidKdMax, &v.pump_kd)) {
        return reject_bad_request("Invalid pump_kd");
    }
    if (!parse_float_in_range(brew_kp_txt, cfg::pid::kKpMin, cfg::pid::kKpMax, &v.brew_kp)) {
        return reject_bad_request("Invalid brew_kp");
    }
    if (!parse_float_in_range(brew_tn_txt, cfg::pid::kBrewTnMinS, cfg::pid::kTnMaxS,
                              &v.brew_tn_s)) {
        return reject_bad_request("Invalid brew_tn");
    }
    if (!parse_float_in_range(brew_tv_txt, cfg::pid::kTvMinS, cfg::pid::kTvMaxS,
                              &v.brew_tv_s)) {
        return reject_bad_request("Invalid brew_tv");
    }
    if (!parse_float_in_range(brew_trail_txt,
                              cfg::pid::kBrewTrailingMinS,
                              cfg::pid::kBrewTrailingMaxS,
                              &v.brew_trailing_s)) {
        return reject_bad_request("Invalid brew_trail");
    }
    if (!parse_float_in_range(brew_pulse_txt,
                              cfg::pid::kBrewPulseMinS,
                              cfg::pid::kBrewPulseMaxS,
                              &v.brew_pulse_s)) {
        return reject_bad_request("Invalid brew_pulse");
    }
    if (!parse_float_in_range(brew_sp_txt,
                              cfg::pid::kSetpointMinC,
                              cfg::pid::kSetpointMaxC,
                              &v.brew_setpoint_c)) {
        return reject_bad_request("Invalid brew_sp");
    }
    if (!parse_float_in_range(steam_sp_txt,
                              cfg::pid::kSteamSetpointMinC,
                              cfg::pid::kSteamSetpointMaxC,
                              &v.steam_setpoint_c)) {
        return reject_bad_request("Invalid steam_sp");
    }
    if (!parse_float_in_range(brew_t_txt,
                              cfg::brew::kBrewTimeMinS,
                              cfg::brew::kBrewTimeMaxS,
                              &brew_t)) {
        return reject_bad_request("Invalid brew_t");
    }
    if (!parse_float_in_range(preinf_t_txt,
                              cfg::brew::kPreinfusionMinS,
                              cfg::brew::kPreinfusionMaxS,
                              &preinf_t)) {
        return reject_bad_request("Invalid preinf_t");
    }
    if (!parse_float_in_range(preinf_pause_txt,
                              cfg::brew::kPreinfusionPauseMinS,
                              cfg::brew::kPreinfusionPauseMaxS,
                              &preinf_pause_t)) {
        return reject_bad_request("Invalid preinf_pause_t");
    }
    if (!parse_u16_in_range(mqtt_port_txt,
                            cfg::mqtt::kServerPortMin,
                            cfg::mqtt::kServerPortMax,
                            &mqtt_port)) {
        return reject_bad_request("Invalid mqtt_port");
    }
    if (!valid_mqtt_host(mqtt_host_txt)) {
        return reject_bad_request("Invalid mqtt_host");
    }

    // Parsed and range-checked now; runtime wiring is phased for these fields.
    (void)brew_t;
    (void)preinf_t;
    (void)preinf_pause_t;
    (void)mqtt_port;

    nvs_handle_t nvs{};
    esp_err_t err = nvs_open(kCfgNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return httpd_resp_send_500(req);
    }

    const struct Entry {
        const char* key;
        const char* value;
    } entries[] = {
        {kCfgPidKp, pid_kp_txt},
        {kCfgPidTn, pid_tn_txt},
        {kCfgPidTv, pid_tv_txt},
        {kCfgPumpKp, pump_kp_txt},
        {kCfgPumpKi, pump_ki_txt},
        {kCfgPumpKd, pump_kd_txt},
        {kCfgBrewKp, brew_kp_txt},
        {kCfgBrewTn, brew_tn_txt},
        {kCfgBrewTv, brew_tv_txt},
        {kCfgBrewTrailing, brew_trail_txt},
        {kCfgBrewPulse, brew_pulse_txt},
        {kCfgBrewSetpoint, brew_sp_txt},
        {kCfgSteamSetpoint, steam_sp_txt},
        {kCfgBrewTime, brew_t_txt},
        {kCfgPreinfusion, preinf_t_txt},
        {kCfgPreinfusionPause, preinf_pause_txt},
        {kCfgMqttHost, mqtt_host_txt},
        {kCfgMqttPort, mqtt_port_txt},
        {kCfgMqttUsername, mqtt_user_txt},
        {kCfgMqttPassword, mqtt_pass_txt},
        {kCfgMqttClientId,   mqtt_client_txt},
        {kCfgHawkbitServer,  hb_server_txt},
        {kCfgHawkbitToken,   hb_token_txt},
    };

    for (const auto& e : entries) {
        err = nvs_set_str(nvs, e.key, e.value);
        if (err != ESP_OK) {
            nvs_close(nvs);
            return httpd_resp_send_500(req);
        }
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        return httpd_resp_send_500(req);
    }

    publish_runtime_control_settings(v);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req,
                    "<html><body><h3>Settings saved.</h3><a href='/'>Back</a></body></html>",
                    HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

bool start_http_server() noexcept {
    if (g_http_server != nullptr) {
        return true;
    }

    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.server_port    = static_cast<int>(cfg::net::kHttpPort);
    http_cfg.lru_purge_enable = true;
    http_cfg.stack_size     = 8192;  // default 4096 is too small for settings handler

    esp_err_t err = httpd_start(&g_http_server, &http_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "HTTP server start failed: %s", esp_err_to_name(err));
        g_http_server = nullptr;
        return false;
    }

    httpd_uri_t root_uri{};
    root_uri.uri = "/";
    root_uri.method = HTTP_GET;
    root_uri.handler = &root_get_handler;
    root_uri.user_ctx = nullptr;

    httpd_uri_t wifi_uri{};
    wifi_uri.uri = "/wifi";
    wifi_uri.method = HTTP_POST;
    wifi_uri.handler = &wifi_post_handler;
    wifi_uri.user_ctx = nullptr;

    httpd_uri_t settings_uri{};
    settings_uri.uri = "/settings";
    settings_uri.method = HTTP_POST;
    settings_uri.handler = &settings_post_handler;
    settings_uri.user_ctx = nullptr;

    err = httpd_register_uri_handler(g_http_server, &root_uri);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "HTTP route registration failed: %s", esp_err_to_name(err));
        (void)httpd_stop(g_http_server);
        g_http_server = nullptr;
        return false;
    }

    err = httpd_register_uri_handler(g_http_server, &wifi_uri);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "HTTP POST registration failed: %s", esp_err_to_name(err));
        (void)httpd_stop(g_http_server);
        g_http_server = nullptr;
        return false;
    }

    err = httpd_register_uri_handler(g_http_server, &settings_uri);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "HTTP settings registration failed: %s", esp_err_to_name(err));
        (void)httpd_stop(g_http_server);
        g_http_server = nullptr;
        return false;
    }

    httpd_uri_t css_uri{};
    css_uri.uri     = "/style.css";
    css_uri.method  = HTTP_GET;
    css_uri.handler = &style_css_handler;

    httpd_uri_t js_uri{};
    js_uri.uri     = "/app.js";
    js_uri.method  = HTTP_GET;
    js_uri.handler = &app_js_handler;

    httpd_uri_t api_uri{};
    api_uri.uri     = "/api/settings";
    api_uri.method  = HTTP_GET;
    api_uri.handler = &api_settings_get_handler;

    const httpd_uri_t* static_uris[] = {&css_uri, &js_uri, &api_uri};
    for (const httpd_uri_t* u : static_uris) {
        err = httpd_register_uri_handler(g_http_server, u);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "HTTP route '%s' registration failed: %s",
                     u->uri, esp_err_to_name(err));
            (void)httpd_stop(g_http_server);
            g_http_server = nullptr;
            return false;
        }
    }

    ESP_LOGI(kTag, "HTTP server listening on port %u",
             static_cast<unsigned>(cfg::net::kHttpPort));
    return true;
}

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
            const std::uint32_t errors =
                g_sensor_consecutive_errors.fetch_add(1, std::memory_order_relaxed) + 1U;
            if (errors >= cfg::tsic::kMaxConsecutiveSensorErrors) {
                g_snapshot.valid.store(false, std::memory_order_release);
            }
            ESP_LOGW(kTag, "capture error: %s",
                     espid::hal_cpp::to_string(fr.error()));
            ctx->decoder->reset();
            continue;
        }
        const auto& frame = fr.value();
        const auto  dec   = ctx->decoder->decode(frame);
        if (!dec.ok()) {
            const std::uint32_t errors =
                g_sensor_consecutive_errors.fetch_add(1, std::memory_order_relaxed) + 1U;
            if (errors >= cfg::tsic::kMaxConsecutiveSensorErrors) {
                g_snapshot.valid.store(false, std::memory_order_release);
            }
            ESP_LOGW(kTag, "decode error: %s  raw=0x%04X parity=0x%X",
                     espid::hal_cpp::to_string(dec.error()),
                     frame.raw, frame.parity_mask);
            continue;
        }
        const float t = dec.value();
        g_sensor_consecutive_errors.store(0U, std::memory_order_relaxed);
        g_snapshot.temp_c.store(t, std::memory_order_relaxed);
        g_snapshot.raw.store(frame.raw, std::memory_order_relaxed);
        g_snapshot.timestamp_ms.store(frame.timestamp_us / 1000U,
                                      std::memory_order_relaxed);
        g_snapshot.valid.store(true, std::memory_order_release);
    }
}

struct PressureTaskCtx {
    espid::abp2::Abp2Driver* driver;
};

void pressure_task(void* arg) {
    auto* ctx = static_cast<PressureTaskCtx*>(arg);
    ESP_LOGI(kTag, "pressure_task: running on core %d", xPortGetCoreID());

    std::uint32_t log_div = 0;
    std::uint32_t lock_div = 0;
    for (;;) {
        bool bus_locked = false;
        if (g_i2c_bus_mutex != nullptr) {
            bus_locked = (xSemaphoreTake(
                g_i2c_bus_mutex,
                ticks_from_ms(cfg::pressure::kI2cTimeoutMs)) == pdTRUE);
            if (!bus_locked) {
                if ((++lock_div % 10U) == 0U) {
                    ESP_LOGW(kTag, "pressure bus lock timeout");
                }
                vTaskDelay(pdMS_TO_TICKS(cfg::pressure::kReadPeriodMs));
                continue;
            }
        }

        espid::abp2::DecodedSample sample_value{};
        bool sample_ok = false;
        espid::hal_cpp::Error sample_error = espid::hal_cpp::Error::kOk;

        for (std::uint32_t attempt = 0;
             attempt <= cfg::pressure::kReadRetryCount;
             ++attempt) {
            const auto sample = ctx->driver->read_sample();
            if (sample.ok()) {
                sample_value = sample.value();
                sample_ok = true;
                break;
            }

            sample_error = sample.error();
            if (sample_error != espid::hal_cpp::Error::kBusError &&
                sample_error != espid::hal_cpp::Error::kTimeout) {
                break;
            }

            if (attempt < cfg::pressure::kReadRetryCount) {
                vTaskDelay(pdMS_TO_TICKS(cfg::pressure::kReadRetryDelayMs));
            }
        }

        if (bus_locked) {
            xSemaphoreGive(g_i2c_bus_mutex);
        }

        if (!sample_ok) {
            const std::uint32_t errors =
                g_pressure_consecutive_errors.fetch_add(1, std::memory_order_relaxed) + 1U;
            if (errors >= cfg::pressure::kMaxConsecutiveErrors) {
                g_pressure.valid.store(false, std::memory_order_release);
            }
            if (errors == 1U || (errors % 10U) == 0U) {
                ESP_LOGW(kTag, "pressure read error: %s (n=%" PRIu32 ")",
                         espid::hal_cpp::to_string(sample_error), errors);
            }

            if ((sample_error == espid::hal_cpp::Error::kRangeError ||
                 sample_error == espid::hal_cpp::Error::kHardwareFault) &&
                (errors <= 5U || (errors % 50U) == 0U)) {
                std::uint8_t raw[7]{};
                if (ctx->driver->last_raw_frame(raw)) {
                    const std::uint8_t status_bits =
                        static_cast<std::uint8_t>((raw[0] >> 6) & 0x03U);
                    const std::uint32_t pressure_counts =
                        static_cast<std::uint32_t>((static_cast<std::uint32_t>(raw[1]) << 16) |
                                                   (static_cast<std::uint32_t>(raw[2]) << 8) |
                                                   static_cast<std::uint32_t>(raw[3]));
                    const std::uint32_t temperature_counts =
                        static_cast<std::uint32_t>((static_cast<std::uint32_t>(raw[4]) << 16) |
                                                   (static_cast<std::uint32_t>(raw[5]) << 8) |
                                                   static_cast<std::uint32_t>(raw[6]));

                    const char* status_label = "unknown";
                    switch (status_bits) {
                        case 0U: status_label = "unpowered"; break;
                        case 1U: status_label = "powered"; break;
                        case 2U: status_label = "reserved"; break;
                        case 3U: status_label = "diagnostic"; break;
                        default: break;
                    }

                    ESP_LOGW(kTag,
                             "pressure raw=[%02X %02X %02X %02X %02X %02X %02X] status=%s(%u) p_counts=%u t_counts=%u tf=[%u..%u]",
                             static_cast<unsigned>(raw[0]),
                             static_cast<unsigned>(raw[1]),
                             static_cast<unsigned>(raw[2]),
                             static_cast<unsigned>(raw[3]),
                             static_cast<unsigned>(raw[4]),
                             static_cast<unsigned>(raw[5]),
                             static_cast<unsigned>(raw[6]),
                             status_label,
                             static_cast<unsigned>(status_bits),
                             static_cast<unsigned>(pressure_counts),
                             static_cast<unsigned>(temperature_counts),
                             static_cast<unsigned>(cfg::pressure::kOutputMinCounts),
                             static_cast<unsigned>(cfg::pressure::kOutputMaxCounts));
                }
            }
        } else {
            const auto& s = sample_value;
            const bool stale_sample = s.busy;
            g_pressure_consecutive_errors.store(0U, std::memory_order_relaxed);
            g_pressure.pressure_bar.store(s.pressure_bar, std::memory_order_relaxed);
            g_pressure.pressure_psi.store(s.pressure_psi, std::memory_order_relaxed);
            g_pressure.sensor_temp_c.store(s.temperature_c, std::memory_order_relaxed);
            g_pressure.pressure_counts.store(s.pressure_counts, std::memory_order_relaxed);
            g_pressure.stale.store(stale_sample, std::memory_order_relaxed);
            g_pressure.valid.store(true, std::memory_order_release);

            if ((++log_div % 20U) == 0U) {
                ESP_LOGI(kTag,
                         "pressure: p=%.3fbar psi=%.2f status=%u stale=%d perr=%" PRIu32,
                         static_cast<double>(s.pressure_bar),
                         static_cast<double>(s.pressure_psi),
                         static_cast<unsigned>(s.status),
                         stale_sample ? 1 : 0,
                         g_pressure_consecutive_errors.load(std::memory_order_relaxed));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(cfg::pressure::kReadPeriodMs));
    }
}

void mqtt_task(void* /*arg*/) {
    ESP_LOGI(kTag, "mqtt_task: running on core %d", xPortGetCoreID());

    char uri[cfg::mqtt::kUriMaxLen]{};
    char topic[cfg::mqtt::kTopicMaxLen]{};
    char payload[cfg::mqtt::kPayloadMaxLen]{};
    std::uint32_t publish_errors = 0;

    for (;;) {
        MqttRuntimeSettings runtime_cfg{};
        if (!load_mqtt_runtime_from_nvs(&runtime_cfg)) {
            ESP_LOGW(kTag, "mqtt: endpoint not configured; set mqtt_host/mqtt_port");
            vTaskDelay(pdMS_TO_TICKS(cfg::mqtt::kReconnectDelayMs));
            continue;
        }

        std::snprintf(uri, sizeof(uri), "mqtt://%s:%u",
                      runtime_cfg.host, static_cast<unsigned>(runtime_cfg.port));
        if (cfg::mqtt::kUseTls) {
            std::snprintf(uri, sizeof(uri), "mqtts://%s:%u",
                          runtime_cfg.host, static_cast<unsigned>(runtime_cfg.port));
        }
        build_mqtt_topic(topic, sizeof(topic),
                         cfg::mqtt::kTopicPrefix,
                         cfg::mqtt::kTelemetryTopic);

        esp_mqtt_client_config_t mqtt_cfg{};
        mqtt_cfg.broker.address.uri = uri;
        mqtt_cfg.credentials.client_id =
            (runtime_cfg.client_id[0] != '\0') ? runtime_cfg.client_id : nullptr;
        mqtt_cfg.credentials.username =
            (runtime_cfg.username[0] != '\0') ? runtime_cfg.username : nullptr;
        mqtt_cfg.credentials.authentication.password =
            (runtime_cfg.password[0] != '\0') ? runtime_cfg.password : nullptr;
        if (cfg::mqtt::kUseTls) {
            mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
            mqtt_cfg.credentials.authentication.certificate =
                reinterpret_cast<const char*>(mqtt_client_crt_start);
            mqtt_cfg.credentials.authentication.key =
                reinterpret_cast<const char*>(mqtt_client_key_start);
        }
        mqtt_cfg.network.reconnect_timeout_ms =
            static_cast<int>(cfg::mqtt::kReconnectDelayMs);
        mqtt_cfg.network.timeout_ms =
            static_cast<int>(cfg::mqtt::kNetworkTimeoutMs);

        esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
        if (client == nullptr) {
            ESP_LOGE(kTag, "mqtt: init failed");
            vTaskDelay(pdMS_TO_TICKS(cfg::mqtt::kReconnectDelayMs));
            continue;
        }

        g_mqtt_connected.store(false, std::memory_order_relaxed);
        (void)esp_mqtt_client_register_event(
            client, MQTT_EVENT_ANY, mqtt_event_handler, nullptr);

        const esp_err_t start_err = esp_mqtt_client_start(client);
        if (start_err != ESP_OK) {
            ESP_LOGE(kTag, "mqtt: start failed: %s", esp_err_to_name(start_err));
            (void)esp_mqtt_client_destroy(client);
            vTaskDelay(pdMS_TO_TICKS(cfg::mqtt::kReconnectDelayMs));
            continue;
        }

        std::uint64_t next_temp_publish_ms = 0;
        std::uint64_t next_pressure_publish_ms = 0;
        std::uint64_t next_runtime_check_ms = 0;
        bool prev_shot_running = false;
        bool prev_steam_active = g_steam_active.load(std::memory_order_relaxed);
        std::uint64_t steam_started_ms = 0;
        espid::ui::MachineStatusTracker status_tracker(cfg::oled::kSteamReadyOnBelowC,
                                                       cfg::oled::kSteamReadyOffBelowC);
        std::uint32_t last_heartbeat_tick =
            g_heartbeat_tick.load(std::memory_order_acquire);
        std::uint32_t last_refill_start_seq =
            g_refill.start_seq.load(std::memory_order_acquire);
        std::uint32_t last_refill_stop_seq =
            g_refill.stop_seq.load(std::memory_order_acquire);
        std::uint32_t last_summary_seq =
            g_shot_summary.seq.load(std::memory_order_acquire);

        espid::pid::TelemetryPolicy::Config tcfg{};
        tcfg.warmup_ms        = cfg::mqtt::kWarmupPublishWindowMs;
        tcfg.warmup_period_ms = cfg::mqtt::kWarmupPublishPeriodMs;
        tcfg.brew_period_ms   = cfg::mqtt::kBrewPublishPeriodMs;
        tcfg.active_period_ms = cfg::mqtt::kActivePublishPeriodMs;
        tcfg.idle_period_ms   = cfg::mqtt::kIdlePublishPeriodMs;
        tcfg.settle_band_c    = cfg::mqtt::kActivePublishBandC;
        tcfg.active_slope_cps = cfg::mqtt::kActivePublishSlopeCps;

        const auto publish_payload = [&](const char* msg) {
            const int msg_id = esp_mqtt_client_publish(
                client, topic, msg, 0, 0, 0);
            if (msg_id >= 0) {
                return;
            }

            ++publish_errors;
            if ((publish_errors % 10U) == 0U) {
                ESP_LOGW(kTag, "mqtt: publish failures=%" PRIu32,
                         publish_errors);
            }
        };

        while (true) {
            const std::uint64_t now_ms =
                static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
            const bool mqtt_connected =
                g_mqtt_connected.load(std::memory_order_relaxed);
            const bool shot_running =
                g_shot.running.load(std::memory_order_relaxed);

            if (!prev_shot_running && shot_running) {
                next_pressure_publish_ms = now_ms;
                next_temp_publish_ms = now_ms;  // immediate reading on brew start
                // Brew-start event with initial conditions.
                const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
                const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
                const int en = std::snprintf(payload, sizeof(payload),
                    "{\"type\":\"brew_start\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                    ",\"temp_c\":%.2f,\"pressure_bar\":%.3f}",
                    static_cast<unsigned>(cfg::mqtt::RecordType::kBrewStart),
                    now_ms,
                    static_cast<double>(temp_c),
                    static_cast<double>(p));
                if (en > 0 && static_cast<std::size_t>(en) < sizeof(payload) && mqtt_connected) {
                    publish_payload(payload);
                }
            } else if (prev_shot_running && !shot_running) {
                // Brew-stop event with final conditions.
                const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
                const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
                const std::uint32_t shot_ms = g_shot.elapsed_ms.load(std::memory_order_relaxed);
                const int en = std::snprintf(payload, sizeof(payload),
                    "{\"type\":\"brew_stop\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                    ",\"duration_s\":%.1f,\"temp_c\":%.2f,\"pressure_bar\":%.3f}",
                    static_cast<unsigned>(cfg::mqtt::RecordType::kBrewStop),
                    now_ms,
                    static_cast<double>(shot_ms) / 1000.0,
                    static_cast<double>(temp_c),
                    static_cast<double>(p));
                if (en > 0 && static_cast<std::size_t>(en) < sizeof(payload) && mqtt_connected) {
                    publish_payload(payload);
                }
            }
            prev_shot_running = shot_running;

            const espid::ui::MachineStatus machine_status =
                status_tracker.update(gather_status_inputs());

            // Steam switch edges: the heater runs to a different setpoint with
            // its own over-temp guard, so analysis has to segment on these.
            const bool steam_active =
                g_steam_active.load(std::memory_order_relaxed);
            if (steam_active != prev_steam_active) {
                next_temp_publish_ms = now_ms;  // sample the transition, like brew start
                const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
                const float setpoint_c =
                    g_control.setpoint_c.load(std::memory_order_relaxed);
                const float duty_pct =
                    g_control_duty_pct.load(std::memory_order_relaxed);
                int en = 0;
                if (steam_active) {
                    steam_started_ms = now_ms;
                    en = std::snprintf(payload, sizeof(payload),
                        "{\"type\":\"steam_on\",\"rec\":%u,\"steam\":1,\"uptime_ms\":%" PRIu64
                        ",\"temp_c\":%.2f,\"setpoint_c\":%.2f,\"heater_duty_pct\":%.1f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kSteamOn),
                        now_ms,
                        static_cast<double>(temp_c),
                        static_cast<double>(setpoint_c),
                        static_cast<double>(duty_pct));
                } else {
                    const double steam_s =
                        (steam_started_ms > 0 && now_ms >= steam_started_ms)
                            ? static_cast<double>(now_ms - steam_started_ms) / 1000.0
                            : 0.0;
                    en = std::snprintf(payload, sizeof(payload),
                        "{\"type\":\"steam_off\",\"rec\":%u,\"steam\":0,\"uptime_ms\":%" PRIu64
                        ",\"duration_s\":%.1f,\"temp_c\":%.2f"
                        ",\"setpoint_c\":%.2f,\"heater_duty_pct\":%.1f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kSteamOff),
                        now_ms, steam_s,
                        static_cast<double>(temp_c),
                        static_cast<double>(setpoint_c),
                        static_cast<double>(duty_pct));
                }
                if (en > 0 && static_cast<std::size_t>(en) < sizeof(payload) && mqtt_connected) {
                    publish_payload(payload);
                }
                prev_steam_active = steam_active;
            }

            if (mqtt_connected) {
                if (now_ms >= next_temp_publish_ms) {
                    const bool temp_valid =
                        g_snapshot.valid.load(std::memory_order_acquire);
                    const float temp_c =
                        g_snapshot.temp_c.load(std::memory_order_relaxed);
                    const float duty_pct =
                        g_control_duty_pct.load(std::memory_order_relaxed);
                    float duty_avg_pct = duty_pct;
                    (void)g_duty_accum.take(&duty_avg_pct);
                    const auto phase = static_cast<espid::pid::HeaterPhase>(
                        g_control.phase.load(std::memory_order_relaxed));
                    const float setpoint_c =
                        g_control.setpoint_c.load(std::memory_order_relaxed);
                    const float error_c =
                        g_control.error_c.load(std::memory_order_relaxed);
                    const float slope_cps =
                        g_control.slope_cps.load(std::memory_order_relaxed);
                    const float pressure_bar =
                        g_pressure.pressure_bar.load(std::memory_order_relaxed);
                    const std::uint32_t shot_ms =
                        g_shot.elapsed_ms.load(std::memory_order_relaxed);

                    // Everything needed to replay the loop offline: inputs,
                    // the P/I/D split, the gains in force, and the phase they
                    // belong to (gains switch between idle and brew sets).
                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"temperature\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"phase\":\"%s\",\"phase_id\":%u"
                        ",\"temp_c\":%.2f,\"temp_valid\":%u"
                        ",\"setpoint_c\":%.2f,\"error_c\":%.2f,\"slope_cps\":%.3f"
                        ",\"heater_duty_pct\":%.1f,\"heater_duty_avg_pct\":%.2f"
                        ",\"p\":%.1f,\"i\":%.1f,\"d\":%.1f"
                        ",\"u\":%.1f,\"kp\":%.2f,\"ki\":%.4f,\"kd\":%.1f"
                        ",\"ff\":%u,\"steam\":%u,\"estop\":%u,\"brewing\":%u"
                        ",\"shot_s\":%.1f,\"pressure_bar\":%.3f,\"pump_pct\":%.1f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kTemperature),
                        now_ms,
                        espid::pid::to_string(phase),
                        static_cast<unsigned>(phase),
                        static_cast<double>(temp_c),
                        temp_valid ? 1U : 0U,
                        static_cast<double>(setpoint_c),
                        static_cast<double>(error_c),
                        static_cast<double>(slope_cps),
                        static_cast<double>(duty_pct),
                        static_cast<double>(duty_avg_pct),
                        static_cast<double>(g_control.p_term.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.i_term.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.d_term.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.unclamped.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.kp.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.ki.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control.kd.load(std::memory_order_relaxed)),
                        g_control.feed_forward.load(std::memory_order_relaxed) ? 1U : 0U,
                        g_steam_active.load(std::memory_order_relaxed) ? 1U : 0U,
                        g_emergency_stop.load(std::memory_order_relaxed) ? 1U : 0U,
                        shot_running ? 1U : 0U,
                        static_cast<double>(shot_ms) / 1000.0,
                        static_cast<double>(pressure_bar),
                        static_cast<double>(g_pump_power_pct.load(std::memory_order_relaxed)));

                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload)) {
                        publish_payload(payload);
                    }

                    next_temp_publish_ms =
                        now_ms + espid::pid::TelemetryPolicy::period_ms(
                                     tcfg, now_ms, phase, error_c, slope_cps,
                                     shot_running,
                                     g_steam_active.load(std::memory_order_relaxed));
                }

                // Per-shot thermal summary, emitted once the trailing phase
                // closes the window in control_task.
                const std::uint32_t summary_seq =
                    g_shot_summary.seq.load(std::memory_order_acquire);
                if (summary_seq != last_summary_seq) {
                    const float sp = g_shot_summary.setpoint_c.load(std::memory_order_relaxed);
                    const float tmin = g_shot_summary.min_temp_c.load(std::memory_order_relaxed);
                    const float tmax = g_shot_summary.max_after_min_c.load(std::memory_order_relaxed);
                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"shot_summary\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"setpoint_c\":%.2f,\"start_temp_c\":%.2f"
                        ",\"min_temp_c\":%.2f,\"drop_c\":%.2f,\"min_at_s\":%.1f"
                        ",\"max_after_min_c\":%.2f,\"overshoot_c\":%.2f"
                        ",\"brew_s\":%.1f,\"window_s\":%.1f,\"settled\":%u"
                        ",\"settled_at_s\":%.1f,\"duty_brew_pct\":%.1f"
                        ",\"duty_trail_pct\":%.1f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kShotSummary),
                        now_ms,
                        static_cast<double>(sp),
                        static_cast<double>(g_shot_summary.start_temp_c.load(std::memory_order_relaxed)),
                        static_cast<double>(tmin),
                        static_cast<double>(sp - tmin),
                        static_cast<double>(g_shot_summary.min_at_ms.load(std::memory_order_relaxed)) / 1000.0,
                        static_cast<double>(tmax),
                        static_cast<double>(tmax - sp),
                        static_cast<double>(g_shot_summary.brew_ms.load(std::memory_order_relaxed)) / 1000.0,
                        static_cast<double>(g_shot_summary.window_ms.load(std::memory_order_relaxed)) / 1000.0,
                        g_shot_summary.settled.load(std::memory_order_relaxed) ? 1U : 0U,
                        static_cast<double>(g_shot_summary.settled_at_ms.load(std::memory_order_relaxed)) / 1000.0,
                        static_cast<double>(g_shot_summary.avg_duty_brew.load(std::memory_order_relaxed)),
                        static_cast<double>(g_shot_summary.avg_duty_trail.load(std::memory_order_relaxed)));
                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload) &&
                        summary_seq == g_shot_summary.seq.load(std::memory_order_acquire)) {
                        publish_payload(payload);
                        last_summary_seq = summary_seq;
                    }
                }

                // Post-steam refill edges. Each record carries enough to
                // estimate the volume put back: pump half-cycles delivered
                // and the boiler temperature drop the cold water caused.
                const std::uint32_t rs = g_refill.start_seq.load(std::memory_order_acquire);
                if (rs != last_refill_start_seq) {
                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"refill_start\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"temp_c\":%.2f,\"pressure_bar\":%.3f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kRefillStart),
                        now_ms,
                        static_cast<double>(g_refill.start_temp_c.load(std::memory_order_relaxed)),
                        static_cast<double>(g_refill.start_pressure_bar.load(std::memory_order_relaxed)));
                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload) &&
                        rs == g_refill.start_seq.load(std::memory_order_acquire)) {
                        publish_payload(payload);
                        last_refill_start_seq = rs;
                    }
                }
                const std::uint32_t re = g_refill.stop_seq.load(std::memory_order_acquire);
                if (re != last_refill_stop_seq) {
                    const float t0 = g_refill.start_temp_c.load(std::memory_order_relaxed);
                    const float t1 = g_refill.end_temp_c.load(std::memory_order_relaxed);
                    const auto reason = static_cast<espid::pump::RefillSequencer::StopReason>(
                        g_refill.reason.load(std::memory_order_relaxed));
                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"refill_stop\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"reason\":\"%s\",\"reason_id\":%u,\"duration_s\":%.1f"
                        ",\"fired_half_cycles\":%" PRIu32
                        ",\"start_temp_c\":%.2f,\"temp_c\":%.2f,\"temp_drop_c\":%.2f"
                        ",\"start_pressure_bar\":%.3f,\"pressure_bar\":%.3f}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kRefillStop),
                        now_ms,
                        espid::pump::to_string(reason),
                        static_cast<unsigned>(reason),
                        static_cast<double>(g_refill.duration_ms.load(std::memory_order_relaxed)) / 1000.0,
                        g_refill.fired_half_cycles.load(std::memory_order_relaxed),
                        static_cast<double>(t0),
                        static_cast<double>(t1),
                        static_cast<double>(t0 - t1),
                        static_cast<double>(g_refill.start_pressure_bar.load(std::memory_order_relaxed)),
                        static_cast<double>(g_refill.end_pressure_bar.load(std::memory_order_relaxed)));
                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload) &&
                        re == g_refill.stop_seq.load(std::memory_order_acquire)) {
                        publish_payload(payload);
                        last_refill_stop_seq = re;
                    }
                }

                if (shot_running && now_ms >= next_pressure_publish_ms) {
                    const bool pressure_valid =
                        g_pressure.valid.load(std::memory_order_acquire);
                    const bool pressure_stale =
                        g_pressure.stale.load(std::memory_order_relaxed);
                    const float pressure_bar =
                        g_pressure.pressure_bar.load(std::memory_order_relaxed);
                    const std::uint32_t shot_ms =
                        g_shot.elapsed_ms.load(std::memory_order_relaxed);

                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"pressure\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"shot_s\":%.1f,\"pressure_bar\":%.3f"
                        ",\"pressure_valid\":%u,\"pressure_stale\":%u}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kPressure),
                        now_ms,
                        static_cast<double>(shot_ms) / 1000.0,
                        static_cast<double>(pressure_bar),
                        pressure_valid ? 1U : 0U,
                        pressure_stale ? 1U : 0U);

                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload)) {
                        publish_payload(payload);
                    }

                    next_pressure_publish_ms =
                        now_ms + static_cast<std::uint64_t>(cfg::mqtt::kPressurePublishPeriodMs);
                }

                // Fixed-rate status record, one per heartbeat: liveness plus the
                // machine state the OLED shows, for Grafana / live dashboards.
                const std::uint32_t hb = g_heartbeat_tick.load(std::memory_order_acquire);
                if (hb != last_heartbeat_tick) {
                    last_heartbeat_tick = hb;
                    wifi_ap_record_t ap{};
                    const int rssi =
                        (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
                    const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"status\",\"rec\":%u,\"uptime_ms\":%" PRIu64
                        ",\"tick\":%" PRIu32 ",\"machine_state\":\"%s\""
                        ",\"machine_state_id\":%u,\"fw_major\":%u,\"fw_minor\":%u"
                        ",\"fw_patch\":%u,\"temp_c\":%.2f,\"temp_valid\":%u"
                        ",\"setpoint_c\":%.2f,\"heater_duty_pct\":%.1f"
                        ",\"pressure_bar\":%.3f,\"pump_pct\":%.1f"
                        ",\"steam\":%u,\"brewing\":%u,\"estop\":%u"
                        ",\"free_heap\":%" PRIu32 ",\"min_free_heap\":%" PRIu32
                        ",\"wifi_rssi\":%d}",
                        static_cast<unsigned>(cfg::mqtt::RecordType::kStatus),
                        now_ms, hb,
                        espid::ui::to_string(machine_status),
                        static_cast<unsigned>(machine_status),
                        static_cast<unsigned>(cfg::kFwVersionMajor),
                        static_cast<unsigned>(cfg::kFwVersionMinor),
                        static_cast<unsigned>(cfg::kFwVersionPatch),
                        static_cast<double>(temp_c),
                        g_snapshot.valid.load(std::memory_order_acquire) ? 1U : 0U,
                        static_cast<double>(g_control.setpoint_c.load(std::memory_order_relaxed)),
                        static_cast<double>(g_control_duty_pct.load(std::memory_order_relaxed)),
                        static_cast<double>(g_pressure.pressure_bar.load(std::memory_order_relaxed)),
                        static_cast<double>(g_pump_power_pct.load(std::memory_order_relaxed)),
                        g_steam_active.load(std::memory_order_relaxed) ? 1U : 0U,
                        shot_running ? 1U : 0U,
                        g_emergency_stop.load(std::memory_order_relaxed) ? 1U : 0U,
                        static_cast<std::uint32_t>(esp_get_free_heap_size()),
                        static_cast<std::uint32_t>(esp_get_minimum_free_heap_size()),
                        rssi);
                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload)) {
                        publish_payload(payload);
                    }
                }
            } else {
                next_temp_publish_ms = now_ms;
                next_pressure_publish_ms = now_ms;
                // Nothing is being published: keep the averaging window short so
                // the first record after a reconnect covers only recent control.
                float discard = 0.0F;
                (void)g_duty_accum.take(&discard);
            }

            if (now_ms >= next_runtime_check_ms) {
                MqttRuntimeSettings new_cfg{};
                if (load_mqtt_runtime_from_nvs(&new_cfg)) {
                    if (!same_mqtt_runtime_settings(new_cfg, runtime_cfg)) {
                        ESP_LOGI(kTag, "mqtt: settings changed, reconnecting");
                        break;
                    }
                }

                next_runtime_check_ms =
                    now_ms + static_cast<std::uint64_t>(cfg::mqtt::kRuntimeSettingsCheckPeriodMs);
            }

            vTaskDelay(ticks_from_ms(cfg::mqtt::kSchedulerPeriodMs));
        }

        (void)esp_mqtt_client_stop(client);
        (void)esp_mqtt_client_destroy(client);
        g_mqtt_connected.store(false, std::memory_order_relaxed);
        vTaskDelay(pdMS_TO_TICKS(cfg::mqtt::kReconnectDelayMs));
    }
}

void control_task(void* arg) {
    auto* heater = static_cast<espid::heater::SlowPwmSsrHeater*>(arg);
    ESP_LOGI(kTag, "control_task: running on core %d", xPortGetCoreID());

    espid::pid::Controller controller;
    espid::pid::HeaterPhaseTracker phase_tracker;
    espid::pid::OverTempGuard over_temp(espid::pid::OverTempGuard::Config{
        {cfg::pid::kEmergencyStopTempC, cfg::pid::kEmergencyResetTempC},
        {cfg::pid::kSteamEmergencyStopTempC, cfg::pid::kSteamEmergencyResetTempC},
        cfg::pid::kSteamCooldownMarginC});
    espid::pid::ShotThermalStats shot_stats;
    bool heater_ready = false;

    if (cfg::heater::kEnabled && heater != nullptr) {
        // Wait for the startup fill to complete before heating (empty-boiler guard).
        ESP_LOGI(kTag, "control: waiting for startup fill...");
        while (!g_fill_complete.load(std::memory_order_acquire)) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!g_fill_ok.load(std::memory_order_acquire)) {
            ESP_LOGE(kTag,
                     "control: boiler not filled to %.1f bar - heater stays OFF "
                     "(dry-boiler guard); refill and reboot",
                     static_cast<double>(cfg::pump::kStartupFillThresholdBar));
        } else {
            espid::hal_cpp::IHeater::Config hcfg{};
            hcfg.ssr_gpio = static_cast<int>(cfg::heater::kSsrGpio);
            hcfg.active_high = cfg::heater::kSsrActiveHigh;
            hcfg.pwm_period_ms = cfg::heater::kPwmPeriodMs;
            const auto hs = heater->init(hcfg);
            if (hs.ok()) {
                heater_ready = true;
                ESP_LOGI(kTag, "heater initialised on gpio=%d", hcfg.ssr_gpio);
            } else {
                ESP_LOGE(kTag, "heater init failed: %s",
                         espid::hal_cpp::to_string(hs.error()));
            }
        }
    }

    std::uint32_t period_ms = static_cast<std::uint32_t>(
        cfg::pid::kSamplePeriodS * 1000.0F);
    if (period_ms == 0) {
        period_ms = 1;
    }
    const TickType_t period_ticks = pdMS_TO_TICKS(period_ms);

    std::uint32_t log_div = 0;
    for (;;) {
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
        const bool valid = g_snapshot.valid.load(std::memory_order_acquire);
        const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
        const std::uint64_t sample_ms =
            g_snapshot.timestamp_ms.load(std::memory_order_relaxed);

        const std::uint64_t freshness_budget_ms =
            static_cast<std::uint64_t>(cfg::pid::kSamplePeriodS * 3000.0F);
        const bool sample_fresh =
            valid && sample_ms > 0U && now_ms >= sample_ms &&
            (now_ms - sample_ms) <= freshness_budget_ms;

        // Steam mode heats the boiler far past the brew over-temp ceiling, so
        // the stop thresholds track the mode, with a cool-down after steam.
        const bool steam_active =
            g_steam_active.load(std::memory_order_relaxed);
        {
            const auto tr = over_temp.update(steam_active,
                                             sample_fresh ? temp_c : NAN);
            const auto& lim = over_temp.limits();
            if (tr.cooldown_started) {
                ESP_LOGI(kTag, "over-temp: steam off at %.2fC - steam limits held "
                         "until T < %.1fC",
                         static_cast<double>(temp_c),
                         static_cast<double>(over_temp.cooldown_exit_c()));
            }
            if (tr.cooldown_ended) {
                ESP_LOGI(kTag, "over-temp: cool-down done at %.2fC - brew limits re-armed",
                         static_cast<double>(temp_c));
            }
            if (tr.tripped) {
                ESP_LOGE(kTag, "EMERGENCY STOP: temp %.2fC >= %.2fC",
                         static_cast<double>(temp_c),
                         static_cast<double>(lim.stop_c));
            }
            if (tr.released) {
                ESP_LOGW(kTag, "Emergency stop reset: temp %.2fC <= %.2fC",
                         static_cast<double>(temp_c),
                         static_cast<double>(lim.reset_c));
            }
        }
        g_emergency_stop.store(over_temp.tripped(), std::memory_order_relaxed);
        g_steam_cooldown.store(over_temp.cooling_down(), std::memory_order_release);
        const bool emergency_stop = g_emergency_stop.load(std::memory_order_relaxed);

        const bool shot_running = g_shot.running.load(std::memory_order_relaxed);
        const std::uint32_t trailing_ms = static_cast<std::uint32_t>(
            g_runtime_control.brew_trailing_s.load(std::memory_order_relaxed) * 1000.0F);
        const espid::pid::HeaterPhase phase =
            phase_tracker.update(steam_active, shot_running, now_ms, trailing_ms);
        const bool brew_gains = espid::pid::HeaterPhaseTracker::uses_brew_gains(phase);

        const float kp = brew_gains
            ? g_runtime_control.brew_kp.load(std::memory_order_relaxed)
            : g_runtime_control.pid_kp.load(std::memory_order_relaxed);
        const float tn_s = brew_gains
            ? g_runtime_control.brew_tn_s.load(std::memory_order_relaxed)
            : g_runtime_control.pid_tn_s.load(std::memory_order_relaxed);
        const float tv_s = brew_gains
            ? g_runtime_control.brew_tv_s.load(std::memory_order_relaxed)
            : g_runtime_control.pid_tv_s.load(std::memory_order_relaxed);
        const float active_setpoint_c = steam_active
            ? g_runtime_control.steam_setpoint_c.load(std::memory_order_relaxed)
            : g_runtime_control.brew_setpoint_c.load(std::memory_order_relaxed);

        // Gains swap in place: the integrator stores the ki-scaled sum, so
        // the transition is bumpless and Tn = 0 simply freezes it.
        espid::pid::Controller::Config pcfg{};
        pcfg.kp = kp;
        pcfg.ki = (tn_s > 0.0F) ? (kp / tn_s) : 0.0F;
        pcfg.kd = kp * tv_s;
        pcfg.output_min = cfg::pid::kOutputMin;
        pcfg.output_max = cfg::pid::kOutputMax;
        pcfg.integral_min = cfg::pid::kIntegralMin;
        pcfg.integral_max = cfg::pid::kIntegralMax;
        pcfg.sample_period_s = cfg::pid::kSamplePeriodS;
        pcfg.d_filter_tau_s = cfg::pid::kDerivativeFilterTauS;
        pcfg.conditional_integration = true;
        pcfg.integral_band = cfg::pid::kIntegralBandC;
        controller.set_config(pcfg);

        const std::uint64_t brew_pulse_ms = static_cast<std::uint64_t>(
            g_runtime_control.brew_pulse_s.load(std::memory_order_relaxed) * 1000.0F);
        const bool brew_pulse = phase == espid::pid::HeaterPhase::kBrew &&
                                (now_ms - phase_tracker.entered_ms()) < brew_pulse_ms;

        float duty_pct = 0.0F;
        if (sample_fresh && std::isfinite(temp_c) && !emergency_stop) {
            duty_pct = controller.update(active_setpoint_c, temp_c);
            // Feed-forward: the PID still runs (state stays current), only the
            // applied duty is raised while the pulse window is open.
            if (brew_pulse && duty_pct < cfg::pid::kBrewPulseDutyPct) {
                duty_pct = cfg::pid::kBrewPulseDutyPct;
            }
        } else {
            controller.reset();
            duty_pct = 0.0F;
        }

        if (heater_ready) {
            auto hs = heater->set_duty_pct(duty_pct);
            if (hs.ok()) {
                hs = heater->step(now_ms);
            }
            if (!hs.ok()) {
                heater_ready = false;
                ESP_LOGE(kTag, "heater drive failed: %s",
                         espid::hal_cpp::to_string(hs.error()));
                duty_pct = 0.0F;
            }
        } else {
            duty_pct = 0.0F;
        }

        g_control_duty_pct.store(duty_pct, std::memory_order_relaxed);
        g_duty_accum.add(duty_pct);

        {
            const auto& t = controller.last_terms();
            g_control.setpoint_c.store(active_setpoint_c, std::memory_order_relaxed);
            g_control.error_c.store(active_setpoint_c - temp_c, std::memory_order_relaxed);
            g_control.slope_cps.store(t.slope, std::memory_order_relaxed);
            g_control.p_term.store(t.p, std::memory_order_relaxed);
            g_control.i_term.store(t.i, std::memory_order_relaxed);
            g_control.d_term.store(t.d, std::memory_order_relaxed);
            g_control.unclamped.store(t.unclamped, std::memory_order_relaxed);
            g_control.kp.store(pcfg.kp, std::memory_order_relaxed);
            g_control.ki.store(pcfg.ki, std::memory_order_relaxed);
            g_control.kd.store(pcfg.kd, std::memory_order_relaxed);
            g_control.phase.store(static_cast<std::uint8_t>(phase), std::memory_order_relaxed);
            g_control.feed_forward.store(brew_pulse, std::memory_order_relaxed);
            g_control.valid.store(sample_fresh, std::memory_order_release);
        }

        if (phase_tracker.changed()) {
            ESP_LOGI(kTag,
                     "heat phase %s->%s: Kp=%.2f Tn=%.1fs Tv=%.2fs "
                     "(ki=%.4f kd=%.2f) sp=%.2fC T=%.2fC I=%.2f",
                     espid::pid::to_string(phase_tracker.previous()),
                     espid::pid::to_string(phase),
                     static_cast<double>(kp), static_cast<double>(tn_s),
                     static_cast<double>(tv_s), static_cast<double>(pcfg.ki),
                     static_cast<double>(pcfg.kd),
                     static_cast<double>(active_setpoint_c),
                     static_cast<double>(temp_c),
                     static_cast<double>(controller.integral_state()));

            const bool was_brew_window =
                espid::pid::HeaterPhaseTracker::uses_brew_gains(phase_tracker.previous());
            if (phase == espid::pid::HeaterPhase::kBrew && !shot_stats.active()) {
                shot_stats.begin(now_ms, active_setpoint_c, temp_c,
                                 cfg::pid::kShotSettleBandC);
            } else if (was_brew_window && !brew_gains && shot_stats.active()) {
                const auto r = shot_stats.finish();
                ESP_LOGI(kTag,
                         "heat shot: brew=%.1fs window=%.1fs sp=%.2fC T0=%.2fC "
                         "Tmin=%.2fC@%.1fs drop=%.2fC Tmax=%.2fC over=%+.2fC "
                         "settle(+-%.1fC)=%.1fs%s duty_brew=%.1f%% duty_trail=%.1f%%",
                         static_cast<double>(r.brew_ms) / 1000.0,
                         static_cast<double>(r.window_ms) / 1000.0,
                         static_cast<double>(r.setpoint_c),
                         static_cast<double>(r.start_temp_c),
                         static_cast<double>(r.min_temp_c),
                         static_cast<double>(r.min_at_ms) / 1000.0,
                         static_cast<double>(r.drop_c()),
                         static_cast<double>(r.max_after_min_c),
                         static_cast<double>(r.overshoot_c()),
                         static_cast<double>(cfg::pid::kShotSettleBandC),
                         static_cast<double>(r.settled_at_ms) / 1000.0,
                         r.settled ? "" : " (NOT settled: raise trailing time)",
                         static_cast<double>(r.avg_duty_brew),
                         static_cast<double>(r.avg_duty_trail));

                g_shot_summary.setpoint_c.store(r.setpoint_c, std::memory_order_relaxed);
                g_shot_summary.start_temp_c.store(r.start_temp_c, std::memory_order_relaxed);
                g_shot_summary.min_temp_c.store(r.min_temp_c, std::memory_order_relaxed);
                g_shot_summary.max_after_min_c.store(r.max_after_min_c, std::memory_order_relaxed);
                g_shot_summary.min_at_ms.store(r.min_at_ms, std::memory_order_relaxed);
                g_shot_summary.brew_ms.store(r.brew_ms, std::memory_order_relaxed);
                g_shot_summary.window_ms.store(r.window_ms, std::memory_order_relaxed);
                g_shot_summary.settled_at_ms.store(r.settled_at_ms, std::memory_order_relaxed);
                g_shot_summary.settled.store(r.settled, std::memory_order_relaxed);
                g_shot_summary.avg_duty_brew.store(r.avg_duty_brew, std::memory_order_relaxed);
                g_shot_summary.avg_duty_trail.store(r.avg_duty_trail, std::memory_order_relaxed);
                g_shot_summary.seq.fetch_add(1, std::memory_order_release);
            }
        }
        if (shot_stats.active() && sample_fresh) {
            shot_stats.sample(now_ms, temp_c, duty_pct, shot_running);
        }

        const std::uint32_t log_every = brew_gains ? cfg::pid::kLogEveryStepsBrew
                                                   : cfg::pid::kLogEveryStepsIdle;
        if (phase_tracker.changed() || (++log_div >= log_every)) {
            log_div = 0;
            const auto& t = controller.last_terms();
            ESP_LOGI(kTag,
                     "heat ph=%s t=%.1f T=%.2f sp=%.2f e=%+.2f dTdt=%+.3f "
                     "P=%+.1f I=%+.1f D=%+.1f u=%+.1f duty=%.1f "
                     "ff=%d fresh=%d estop=%d serr=%" PRIu32,
                     espid::pid::to_string(phase),
                     static_cast<double>(now_ms - phase_tracker.entered_ms()) / 1000.0,
                     static_cast<double>(temp_c),
                     static_cast<double>(active_setpoint_c),
                     static_cast<double>(active_setpoint_c - temp_c),
                     static_cast<double>(t.slope),
                     static_cast<double>(t.p),
                     static_cast<double>(t.i),
                     static_cast<double>(t.d),
                     static_cast<double>(t.unclamped),
                     static_cast<double>(duty_pct),
                     brew_pulse ? 1 : 0,
                     sample_fresh ? 1 : 0,
                     emergency_stop ? 1 : 0,
                     g_sensor_consecutive_errors.load(std::memory_order_relaxed));
        }

        vTaskDelay(period_ticks);
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
    char footer[48];

    bool          was_shot_active = false;
    std::uint64_t post_shot_until_ms = 0;
    std::uint32_t lock_div = 0;
    espid::ui::MachineStatusTracker status_tracker(cfg::oled::kSteamReadyOnBelowC,
                                                   cfg::oled::kSteamReadyOffBelowC);

    for (;;) {
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
        const bool  valid = g_snapshot.valid.load(std::memory_order_acquire);
        const float t     = g_snapshot.temp_c.load(std::memory_order_relaxed);
        const std::uint32_t shot_ms =
            g_shot.elapsed_ms.load(std::memory_order_relaxed);
        const bool shot_active =
            g_shot.running.load(std::memory_order_relaxed);
        const bool pressure_valid =
            g_pressure.valid.load(std::memory_order_acquire);
        const float pressure_bar =
            g_pressure.pressure_bar.load(std::memory_order_relaxed);
        const bool pressure_stale =
            g_pressure.stale.load(std::memory_order_relaxed);
        const float brew_setpoint_c =
            g_runtime_control.brew_setpoint_c.load(std::memory_order_relaxed);
        const bool steam_active =
            g_steam_active.load(std::memory_order_relaxed);
        const float steam_setpoint_c =
            g_runtime_control.steam_setpoint_c.load(std::memory_order_relaxed);
        espid::hal_cpp::WifiState wifi_state =
            espid::hal_cpp::WifiState::kIdle;
        espid::hal_cpp::INetwork::IpString wifi_ip{};
        std::snprintf(wifi_ip.data(), wifi_ip.size(), "0.0.0.0");
        if (g_network != nullptr) {
            wifi_state = g_network->state();
            wifi_ip = g_network->ip_string();
        }

        if (shot_active) {
            post_shot_until_ms = now_ms + static_cast<std::uint64_t>(cfg::brew::kPostShotHoldMs);
        } else if (was_shot_active && shot_ms > 0U) {
            post_shot_until_ms = now_ms + static_cast<std::uint64_t>(cfg::brew::kPostShotHoldMs);
        }
        was_shot_active = shot_active;

        const bool show_shot_screen =
            shot_active || (shot_ms > 0U && now_ms < post_shot_until_ms);

        char temp_str[16];
        if (valid && std::isfinite(t)) {
            std::snprintf(temp_str, sizeof(temp_str), "%5.2f",
                          static_cast<double>(t));
        } else {
            std::snprintf(temp_str, sizeof(temp_str), "--.--");
        }

        // What the machine is doing, and what the operator should do next.
        const espid::ui::RawStatusInputs status_raw = gather_status_inputs();
        const espid::ui::MachineStatus status = status_tracker.update(status_raw);
        const std::uint64_t refill_started_ms =
            g_refill.started_ms.load(std::memory_order_relaxed);
        const std::uint32_t refill_elapsed_s =
            (status_raw.refill_running && now_ms >= refill_started_ms)
                ? static_cast<std::uint32_t>((now_ms - refill_started_ms) / 1000U)
                : 0U;
        char status_str[24];
        (void)espid::ui::format_header(status, refill_elapsed_s,
                                       status_str, sizeof(status_str));
        const bool status_visible =
            !espid::ui::needs_attention(status) ||
            (now_ms % cfg::oled::kPromptBlinkPeriodMs) < cfg::oled::kPromptBlinkOnMs;

        char setpoint_str[16];
        const float shown_setpoint_c = steam_active ? steam_setpoint_c : brew_setpoint_c;
        const char* setpoint_prefix   = steam_active ? "ST" : "SP";
        if (std::isfinite(shown_setpoint_c)) {
            std::snprintf(setpoint_str, sizeof(setpoint_str), "%s:%4.1f",
                          setpoint_prefix, static_cast<double>(shown_setpoint_c));
        } else {
            std::snprintf(setpoint_str, sizeof(setpoint_str), "%s:--.-", setpoint_prefix);
        }

        display->clear();
        if (wifi_state == espid::hal_cpp::WifiState::kApFallback) {
            char ap_hdr[20];
            std::snprintf(ap_hdr, sizeof(ap_hdr), "AP:%s", configured_ap_ssid());
            display->draw_text(0, 10, espid::hal_cpp::Font::kSmall, ap_hdr);
            display->draw_text(86, 10, espid::hal_cpp::Font::kSmall,
                               wifi_ip.data());
        } else {
            if (status_visible) {
                display->draw_text(0, 10, espid::hal_cpp::Font::kSmall, status_str);
            }
            display->draw_text(96, 10, espid::hal_cpp::Font::kSmall,
                               wifi_label(wifi_state));
        }

        if (show_shot_screen) {
            char pressure_str[16];
            if (pressure_valid && std::isfinite(pressure_bar)) {
                std::snprintf(pressure_str, sizeof(pressure_str), "%4.1f%sbar",
                              static_cast<double>(pressure_bar),
                              pressure_stale ? "*" : "");
            } else {
                std::snprintf(pressure_str, sizeof(pressure_str), "--.-bar");
            }

            std::snprintf(big, sizeof(big), "%4.1fs",
                          static_cast<double>(shot_ms) / 1000.0);
            std::snprintf(footer, sizeof(footer), "P:%s T:%sC",
                          pressure_str, temp_str);
            display->draw_text(8,  44, espid::hal_cpp::Font::kLarge, big);
            display->draw_text(0,  60, espid::hal_cpp::Font::kSmall, footer);
        } else {
            // Idle: big temperature, footer with shot + setpoint.
            if (steam_active) {
                std::snprintf(small, sizeof(small), "STEAM");
            } else {
                std::snprintf(small, sizeof(small), "shot %4.1fs",
                              static_cast<double>(shot_ms) / 1000.0);
            }
            std::snprintf(footer, sizeof(footer), "%s %sC", small, setpoint_str);
            // During and right after a refill the footer shows its progress
            // and, when it stopped, why.
            if (status == espid::ui::MachineStatus::kRefilling) {
                if (pressure_valid && std::isfinite(pressure_bar)) {
                    std::snprintf(footer, sizeof(footer), "P:%4.1fbar full@%.1f",
                                  static_cast<double>(pressure_bar),
                                  static_cast<double>(cfg::pump::kRefillFullPressureBar));
                } else {
                    std::snprintf(footer, sizeof(footer), "P:--.-bar full@%.1f",
                                  static_cast<double>(cfg::pump::kRefillFullPressureBar));
                }
            } else if (status == espid::ui::MachineStatus::kRefillFullRelease) {
                std::snprintf(footer, sizeof(footer), "boiler refilled");
            } else if (status == espid::ui::MachineStatus::kRefillTimeoutRelease) {
                std::snprintf(footer, sizeof(footer), "steam knob open?");
            }
            display->draw_text(8,  44, espid::hal_cpp::Font::kLarge, temp_str);
            display->draw_text(110, 28, espid::hal_cpp::Font::kSmall, "C");
            display->draw_text(0,  60, espid::hal_cpp::Font::kSmall, footer);
        }
        if (wifi_state == espid::hal_cpp::WifiState::kApFallback) {
            char ap_pw[24];
            if (cfg::net::kApPassword != nullptr &&
                cfg::net::kApPassword[0] != '\0') {
                std::snprintf(ap_pw, sizeof(ap_pw), "PW:%s", cfg::net::kApPassword);
            } else {
                std::snprintf(ap_pw, sizeof(ap_pw), "PW:(open)");
            }
            display->draw_text(0, 60, espid::hal_cpp::Font::kSmall, ap_pw);
        }

        if (g_i2c_bus_mutex == nullptr) {
            (void)display->flush();
        } else {
            const bool got_lock = (xSemaphoreTake(
                g_i2c_bus_mutex,
                ticks_from_ms(cfg::oled::kI2cLockTimeoutMs)) == pdTRUE);
            if (got_lock) {
                (void)display->flush();
                xSemaphoreGive(g_i2c_bus_mutex);
            } else if ((++lock_div % 10U) == 0U) {
                ESP_LOGW(kTag, "display bus lock timeout");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(cfg::oled::kRefreshPeriodMs));
    }
}

// Pump task: startup fill then pressure-PID-controlled pump power during brew.
struct PumpTaskCtx {
    espid::pump::PsmPump*       psm;
    espid::valve::SolenoidValve* valve;  // stays closed during fill and refill
};

void pump_task(void* arg) {
    auto* ctx = static_cast<PumpTaskCtx*>(arg);
    ESP_LOGI(kTag, "pump_task: running on core %d", xPortGetCoreID());

    // Phase 1: startup fill — run pump until pressure >= threshold or timeout.
    // This ensures the boiler has water before the heater is allowed to start.
    ESP_LOGI(kTag, "pump: waiting for pressure sensor...");
    while (!g_pressure.valid.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(kTag, "pump: startup fill (target %.1f bar, timeout %u s)",
             static_cast<double>(cfg::pump::kStartupFillThresholdBar),
             static_cast<unsigned>(cfg::pump::kStartupFillTimeoutMs / 1000U));

    (void)ctx->psm->set_power_pct(cfg::pump::kStartupFillPowerPct);
    g_pump_power_pct.store(cfg::pump::kStartupFillPowerPct, std::memory_order_relaxed);
    // Valve stays CLOSED during fill: pump pushes into boiler via check valve.
    // Opening the solenoid routes water to group head and prevents pressure build.
    const std::uint64_t fill_deadline_ms =
        static_cast<std::uint64_t>(esp_timer_get_time() / 1000) +
        cfg::pump::kStartupFillTimeoutMs;
    const std::uint32_t zc_at_fill_start = ctx->psm->zc_count();
    bool fill_ok = false;
    std::uint32_t confirm_count = 0;
    for (;;) {
        const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
        const bool usable = g_pressure.valid.load(std::memory_order_acquire) &&
                            !g_pressure.stale.load(std::memory_order_relaxed);
        // One bad frame must not satisfy the dry-boiler guard: the threshold
        // has to hold across consecutive good samples before we trust it.
        if (usable && p >= cfg::pump::kStartupFillThresholdBar) {
            ++confirm_count;
        } else {
            if (confirm_count > 0) {
                ESP_LOGW(kTag,
                         "pump: fill reading not confirmed (p=%.2f bar, usable=%d) — restarting count",
                         static_cast<double>(p), usable ? 1 : 0);
            }
            confirm_count = 0;
        }
        if (confirm_count >= cfg::pump::kStartupFillConfirmSamples) {
            fill_ok = true;
            ESP_LOGI(kTag, "pump: fill done (p=%.2f bar, %u consecutive samples)",
                     static_cast<double>(p),
                     static_cast<unsigned>(confirm_count));
            break;
        }
        if (static_cast<std::uint64_t>(esp_timer_get_time() / 1000) >= fill_deadline_ms) {
            // A boiler that never pressurises (sensor fault, or a leak) should
            // not block the heater forever if the pump demonstrably ran, so
            // accept the fill when the ZC count confirms it.
            const std::uint32_t zc_delta = ctx->psm->zc_count() - zc_at_fill_start;
            if (zc_delta >= cfg::pump::kStartupFillMinZcCount) {
                fill_ok = true;
                ESP_LOGW(kTag, "pump: fill timeout but pump ran (zc_delta=%u) — accepting",
                         static_cast<unsigned>(zc_delta));
            } else {
                ESP_LOGW(kTag, "pump: fill timeout and pump barely ran (zc_delta=%u) — check water",
                         static_cast<unsigned>(zc_delta));
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)ctx->psm->set_power_pct(0.0F);
    g_pump_power_pct.store(0.0F, std::memory_order_relaxed);
    ESP_LOGI(kTag, "pump: fill end — zc_count=%u", ctx->psm->zc_count());
    g_fill_ok.store(fill_ok, std::memory_order_release);
    g_fill_complete.store(true, std::memory_order_release);
    ESP_LOGI(kTag, "pump: %s", fill_ok ? "ready for brewing" : "fill FAILED");

    // Phase 2: pressure PID — drives pump power during brew to maintain setpoint.
    espid::pid::Controller pid;
    espid::pid::Controller::Config pcfg{};
    pcfg.kp             = g_runtime_control.pump_kp.load(std::memory_order_relaxed);
    pcfg.ki             = g_runtime_control.pump_ki.load(std::memory_order_relaxed);
    pcfg.kd             = g_runtime_control.pump_kd.load(std::memory_order_relaxed);
    pcfg.output_min     = cfg::pump::kPidOutputMin;
    pcfg.output_max     = cfg::pump::kPidOutputMax;
    pcfg.integral_min   = cfg::pump::kPidIntegralMin;
    pcfg.integral_max   = cfg::pump::kPidIntegralMax;
    pcfg.sample_period_s = cfg::pump::kPidSamplePeriodS;
    // The ramp from 0 bar to setpoint saturates the output for several
    // seconds; hold the integral there so it does not wind up.
    pcfg.conditional_integration = true;
    pid.set_config(pcfg);

    const TickType_t period = ticks_from_ms(
        static_cast<std::uint32_t>(cfg::pump::kPidSamplePeriodS * 1000.0F));

    std::uint32_t log_div = 0;
    bool was_brewing = false;
    std::uint32_t last_zc = 0;

    espid::pump::RefillSequencer refill(espid::pump::RefillSequencer::Config{
        cfg::pump::kRefillPowerPct, cfg::pump::kRefillFullPressureBar,
        cfg::pump::kRefillConfirmMs, cfg::pump::kRefillMaxDurationMs});
    bool refill_switch_was = false;
    std::uint32_t refill_fired_start = 0;

    for (;;) {
        const bool brewing = g_shot.running.load(std::memory_order_relaxed);
        const bool refill_switch = g_refill_switch.load(std::memory_order_acquire);
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
        const float p_now = g_pressure.pressure_bar.load(std::memory_order_relaxed);
        const float temp_now = g_snapshot.temp_c.load(std::memory_order_relaxed);

        if (refill_switch && !refill_switch_was && refill.state() ==
                                espid::pump::RefillSequencer::State::kIdle) {
            refill.start(now_ms);
            g_refill.started_ms.store(now_ms, std::memory_order_relaxed);
            refill_fired_start = ctx->psm->zc_stats().fired;
            g_refill.start_temp_c.store(temp_now, std::memory_order_relaxed);
            g_refill.start_pressure_bar.store(p_now, std::memory_order_relaxed);
            g_refill.start_seq.fetch_add(1, std::memory_order_release);
            ESP_LOGI(kTag, "refill: start T=%.2fC p=%.2fbar (power %.0f%%, full at %.1fbar)",
                     static_cast<double>(temp_now), static_cast<double>(p_now),
                     static_cast<double>(cfg::pump::kRefillPowerPct),
                     static_cast<double>(cfg::pump::kRefillFullPressureBar));
        }
        refill_switch_was = refill_switch;

        if (brewing) {
            if (!was_brewing) {
                // Brew just started: reload gains from NVS/web-UI, reset state.
                pcfg.kp = g_runtime_control.pump_kp.load(std::memory_order_relaxed);
                pcfg.ki = g_runtime_control.pump_ki.load(std::memory_order_relaxed);
                pcfg.kd = g_runtime_control.pump_kd.load(std::memory_order_relaxed);
                pid.set_config(pcfg);
                last_zc = ctx->psm->zc_count();
                // Record the gains actually in force: these come from NVS, so
                // a stale web-UI value silently overrides the config default.
                ESP_LOGI(kTag,
                         "pump: brew start — sp=%.2fbar Kp=%.2f Ki=%.3f Kd=%.3f "
                         "I=[%.0f..%.0f] out=[%.0f..%.0f]",
                         static_cast<double>(cfg::pump::kPressureSetpointBar),
                         static_cast<double>(pcfg.kp),
                         static_cast<double>(pcfg.ki),
                         static_cast<double>(pcfg.kd),
                         static_cast<double>(pcfg.integral_min),
                         static_cast<double>(pcfg.integral_max),
                         static_cast<double>(pcfg.output_min),
                         static_cast<double>(pcfg.output_max));
            }
            was_brewing = true;
            if (g_pressure.valid.load(std::memory_order_acquire)) {
                const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
                const float power = pid.update(cfg::pump::kPressureSetpointBar, p);
                (void)ctx->psm->set_power_pct(power);
                g_pump_power_pct.store(power, std::memory_order_relaxed);
                if ((++log_div % 10U) == 0U) {
                    const std::uint32_t zc_now = ctx->psm->zc_count();
                    const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
                    // ZC delta over 10 samples (1 s): expect ~100 for healthy 50 Hz.
                    // P/I/D split tells a starved loop (power < 100 while short
                    // of setpoint) from a hydraulic limit (power pinned at 100).
                    const auto& t = pid.last_terms();
                    ESP_LOGI(kTag,
                             "pump: p=%.2fbar e=%+.2f power=%.1f%% P=%+.1f I=%+.1f "
                             "D=%+.1f u=%+.1f temp=%.1fC zc/s=%u",
                             static_cast<double>(p),
                             static_cast<double>(cfg::pump::kPressureSetpointBar - p),
                             static_cast<double>(power),
                             static_cast<double>(t.p),
                             static_cast<double>(t.i),
                             static_cast<double>(t.d),
                             static_cast<double>(t.unclamped),
                             static_cast<double>(temp_c),
                             static_cast<unsigned>(zc_now - last_zc));
                    last_zc = zc_now;
                }
            } else {
                // No pressure reading yet — run at full power as fallback.
                (void)ctx->psm->set_power_pct(cfg::pump::kDefaultPowerPct);
                g_pump_power_pct.store(cfg::pump::kDefaultPowerPct, std::memory_order_relaxed);
            }
        } else if (refill.state() != espid::pump::RefillSequencer::State::kIdle) {
            was_brewing = false;
            pid.reset();
            // Steam back on cancels: refilling would only kill the steam.
            const bool keep_going =
                refill_switch && !g_steam_active.load(std::memory_order_relaxed);
            const bool usable = g_pressure.valid.load(std::memory_order_acquire) &&
                                !g_pressure.stale.load(std::memory_order_relaxed);
            const float power = refill.step(now_ms, keep_going, usable, p_now);
            (void)ctx->psm->set_power_pct(power);
            g_pump_power_pct.store(power, std::memory_order_relaxed);

            if (refill.running() && (++log_div % 10U) == 0U) {
                ESP_LOGI(kTag, "refill: p=%.2fbar T=%.2fC power=%.0f%% t=%.1fs",
                         static_cast<double>(p_now), static_cast<double>(temp_now),
                         static_cast<double>(power),
                         static_cast<double>(now_ms) / 1000.0);
            }
            if (refill.finished_now()) {
                const std::uint32_t fired =
                    ctx->psm->zc_stats().fired - refill_fired_start;
                g_refill.end_temp_c.store(temp_now, std::memory_order_relaxed);
                g_refill.end_pressure_bar.store(p_now, std::memory_order_relaxed);
                g_refill.duration_ms.store(refill.duration_ms(), std::memory_order_relaxed);
                g_refill.fired_half_cycles.store(fired, std::memory_order_relaxed);
                g_refill.reason.store(static_cast<std::uint8_t>(refill.reason()),
                                      std::memory_order_relaxed);
                g_refill.stop_seq.fetch_add(1, std::memory_order_release);
                ESP_LOGI(kTag,
                         "refill: stop (%s) after %.1fs, %u pump half-cycles, "
                         "T %.2f -> %.2fC, p=%.2fbar",
                         espid::pump::to_string(refill.reason()),
                         static_cast<double>(refill.duration_ms()) / 1000.0,
                         static_cast<unsigned>(fired),
                         static_cast<double>(g_refill.start_temp_c.load(std::memory_order_relaxed)),
                         static_cast<double>(temp_now),
                         static_cast<double>(p_now));
            }
        } else {
            was_brewing = false;
            pid.reset();
            (void)ctx->psm->set_power_pct(0.0F);
            g_pump_power_pct.store(0.0F, std::memory_order_relaxed);
        }
        g_refill.state.store(static_cast<std::uint8_t>(refill.state()),
                             std::memory_order_release);
        vTaskDelay(period);
    }
}

// Pump bench/bring-up test: measure the zero-cross signal, then sweep the
// duty cycle. Heater is kept OFF by app_main when cfg::pump::kTestMode is set,
// so the boiler never heats; the pump does run DRY during the sweep, so keep
// it short. Idles forever once the sweep completes.
void pump_test_task(void* arg) {
    auto* ctx = static_cast<PumpTaskCtx*>(arg);
    ESP_LOGW(kTag,
             "PUMP TEST MODE — heater OFF, pump will run DRY. Stay present.");
    ESP_LOGI(kTag, "pump_test: running on core %d", xPortGetCoreID());

    const float    expected_hz = 2.0F * static_cast<float>(cfg::pump::kMainsFrequencyHz);
    const unsigned expected_edges =
        static_cast<unsigned>(2U * cfg::pump::kMainsFrequencyHz);

    // Phase A: measure the zero-cross frequency with the pump OFF (no dry run
    // while we characterise the ZC signal).
    (void)ctx->psm->set_power_pct(0.0F);
    ctx->psm->reset_zc_stats();
    ESP_LOGI(kTag, "pump_test: measuring ZC for %u ms (pump OFF)...",
             static_cast<unsigned>(cfg::pump::kTestZcMeasureMs));
    vTaskDelay(pdMS_TO_TICKS(cfg::pump::kTestZcMeasureMs));

    const auto  zc   = ctx->psm->zc_stats();
    const float secs = static_cast<float>(cfg::pump::kTestZcMeasureMs) / 1000.0F;
    const float hz   = static_cast<float>(zc.accepted) / secs;
    ESP_LOGI(kTag,
             "pump_test: ZC accepted=%u rejected=%u -> %.1f Hz (expect ~%u)",
             static_cast<unsigned>(zc.accepted),
             static_cast<unsigned>(zc.rejected),
             static_cast<double>(hz), expected_edges);
    ESP_LOGI(kTag,
             "pump_test: ZC interval last/min/max = %u/%u/%u us (jitter %u us)",
             static_cast<unsigned>(zc.last_interval_us),
             static_cast<unsigned>(zc.min_interval_us),
             static_cast<unsigned>(zc.max_interval_us),
             static_cast<unsigned>(zc.max_interval_us >= zc.min_interval_us
                                       ? zc.max_interval_us - zc.min_interval_us
                                       : 0U));
    if (hz < expected_hz * 0.9F || hz > expected_hz * 1.1F) {
        ESP_LOGW(kTag,
                 "pump_test: ZC frequency off by >10%% — check mains freq, "
                 "wiring, or a dead opto");
    }
    if (zc.rejected > (zc.accepted / 4U)) {
        ESP_LOGW(kTag,
                 "pump_test: NOISY ZC (rejected %u vs accepted %u) — add an RC "
                 "filter / Schmitt trigger, or enable a GPIO glitch filter",
                 static_cast<unsigned>(zc.rejected),
                 static_cast<unsigned>(zc.accepted));
    }

    // Phase B: duty sweep. Open the valve so the pump is not dead-heading.
    // Each step starts from OFF (kTestGapMs) so the pump-start is audible.
    if (ctx->valve != nullptr) {
        (void)ctx->valve->open();
    }
    for (const float duty : cfg::pump::kTestSweepPct) {
        (void)ctx->psm->set_power_pct(0.0F);
        vTaskDelay(pdMS_TO_TICKS(cfg::pump::kTestGapMs));
        ctx->psm->reset_zc_stats();
        (void)ctx->psm->set_power_pct(duty);
        ESP_LOGI(kTag, "pump_test: duty=%.0f%% for %u ms",
                 static_cast<double>(duty),
                 static_cast<unsigned>(cfg::pump::kTestStepMs));
        vTaskDelay(pdMS_TO_TICKS(cfg::pump::kTestStepMs));
        const auto s = ctx->psm->zc_stats();
        const float fire_pct = (s.accepted > 0U)
            ? (100.0F * static_cast<float>(s.fired) / static_cast<float>(s.accepted))
            : 0.0F;
        ESP_LOGI(kTag,
                 "pump_test:   zc=%u fired=%u (%.1f%% of cycles, target %.0f%%) "
                 "rejected=%u  zc-int min/max=%u/%u us",
                 static_cast<unsigned>(s.accepted),
                 static_cast<unsigned>(s.fired),
                 static_cast<double>(fire_pct), static_cast<double>(duty),
                 static_cast<unsigned>(s.rejected),
                 static_cast<unsigned>(s.min_interval_us),
                 static_cast<unsigned>(s.max_interval_us));
    }

    // Done: pump off, valve closed, idle so nothing else drives the pump.
    (void)ctx->psm->set_power_pct(0.0F);
    if (ctx->valve != nullptr) {
        (void)ctx->valve->close();
    }
    ESP_LOGI(kTag, "pump_test: sweep complete — pump OFF, idling.");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// OTA task: polls HawkBit every kPollIntervalMs for firmware updates.
void hawkbit_task(void* /*arg*/) {
    // Wait for Wi-Fi STA connection before first poll.
    while (g_network == nullptr ||
           g_network->state() != espid::hal_cpp::WifiState::kConnected) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(kTag, "hawkbit_task: running (poll every %u s)",
             cfg::hawkbit::kPollIntervalMs / 1000U);

    for (;;) {
        char server[cfg::hawkbit::kServerUrlMaxLen + 1]{};
        char token[cfg::hawkbit::kTokenMaxLen + 1]{};
        (void)load_setting_from_nvs(kCfgHawkbitServer, "", server, sizeof(server));
        (void)load_setting_from_nvs(kCfgHawkbitToken,  "", token,  sizeof(token));

        if (server[0] != '\0') {
            espid::hawkbit::Config hb_cfg{};
            hb_cfg.server_url     = server;
            hb_cfg.tenant         = cfg::hawkbit::kDefaultTenant;
            hb_cfg.device_id      = cfg::mqtt::kClientId;
            hb_cfg.security_token = token;

            if (espid::hawkbit::poll_and_apply(hb_cfg)) {
                ESP_LOGI(kTag, "hawkbit: new firmware installed — rebooting");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(cfg::hawkbit::kPollIntervalMs));
    }
}

// Polled brew-button task.  Reads the raw GPIO at kButtonPeriodMs,
// runs it through the BrewButton debounce filter, and toggles the
// ShotTimer on each debounced press (release is ignored: the timer is
// edge-triggered so a quick tap starts it and another tap stops it).
void button_task(void* arg) {
    auto* valve = static_cast<espid::valve::SolenoidValve*>(arg);
    ESP_LOGI(kTag, "button_task: running on core %d (gpio=%d)",
             xPortGetCoreID(), static_cast<int>(cfg::brew::kButtonGpio));

    espid::brew::BrewButton::Config bcfg{};
    bcfg.active_low  = cfg::brew::kButtonActiveLow;
    bcfg.debounce_ms = cfg::brew::kDebounceMs;
    espid::brew::BrewButton btn(bcfg);
    espid::brew::ShotTimer  timer;
    // True while the current press is a refill rather than a shot.
    bool refill_press = false;

    for (;;) {
        const int           raw    = gpio_get_level(cfg::brew::kButtonGpio);
        const std::uint64_t now_ms = static_cast<std::uint64_t>(
            esp_timer_get_time() / 1000);

        const auto ev = btn.poll(raw != 0, now_ms);
        if (ev == espid::brew::BrewButton::Event::kPressed) {
            if (g_steam_cooldown.load(std::memory_order_acquire)) {
                // No shot is wanted from a 115 C+ boiler; after steaming the
                // brew switch tops the boiler up instead. The valve stays
                // closed so the pump can only push water into the boiler.
                refill_press = true;
                g_refill_switch.store(true, std::memory_order_release);
                ESP_LOGI(kTag, "brew switch during steam cool-down: boiler refill");
            } else {
                timer.reset();
                timer.start(now_ms);
                if (valve != nullptr) {
                    (void)valve->open();
                }
                ESP_LOGI(kTag, "brew: start");
            }
        } else if (ev == espid::brew::BrewButton::Event::kReleased) {
            if (refill_press) {
                refill_press = false;
                g_refill_switch.store(false, std::memory_order_release);
                ESP_LOGI(kTag, "brew switch released: refill session closed");
            } else {
                timer.stop(now_ms);
                if (valve != nullptr) {
                    (void)valve->close();
                }
                ESP_LOGI(kTag, "brew: stop  elapsed=%u ms",
                         static_cast<unsigned>(timer.elapsed_ms(now_ms)));
            }
        }

        g_shot.elapsed_ms.store(timer.elapsed_ms(now_ms),
                                std::memory_order_relaxed);
        g_shot.running.store(timer.running(), std::memory_order_relaxed);

        vTaskDelay(pdMS_TO_TICKS(cfg::brew::kPollPeriodMs));
    }
}

// Polled steam-switch task.  Debounced latching input; publishes the
// steam-mode flag consumed by control_task to select the setpoint.
void steam_task(void* /*arg*/) {
    ESP_LOGI(kTag, "steam_task: running on core %d (gpio=%d)",
             xPortGetCoreID(), static_cast<int>(cfg::steam::kSwitchGpio));

    espid::brew::BrewButton::Config scfg{};
    scfg.active_low  = !cfg::steam::kSwitchActiveHigh;
    scfg.debounce_ms = cfg::steam::kDebounceMs;
    espid::brew::BrewButton sw(scfg);

    bool last = false;
    for (;;) {
        const int           raw    = gpio_get_level(cfg::steam::kSwitchGpio);
        const std::uint64_t now_ms = static_cast<std::uint64_t>(
            esp_timer_get_time() / 1000);

        (void)sw.poll(raw != 0, now_ms);
        const bool active = sw.is_pressed();
        if (active != last) {
            g_steam_active.store(active, std::memory_order_relaxed);
            const float temp_c = g_snapshot.temp_c.load(std::memory_order_relaxed);
            const float setpoint_c =
                active ? g_runtime_control.steam_setpoint_c.load(std::memory_order_relaxed)
                       : g_runtime_control.brew_setpoint_c.load(std::memory_order_relaxed);
            ESP_LOGI(kTag, "steam: %s T=%.2fC sp=%.2fC",
                     active ? "ON" : "OFF",
                     static_cast<double>(temp_c),
                     static_cast<double>(setpoint_c));
            last = active;
        }

        vTaskDelay(pdMS_TO_TICKS(cfg::steam::kPollPeriodMs));
    }
}

// Polled dual-HX711 scale task.  Reads both cells, feeds ScaleCore, and
// publishes the summed weight.  Tares once on the first successful read.
void scale_task(void* arg) {
    auto* dev = static_cast<espid::scale::Hx711Scale*>(arg);
    ESP_LOGI(kTag, "scale_task: running on core %d", xPortGetCoreID());

    espid::scale::ScaleCore        core;
    espid::scale::ScaleCore::Config ccfg{};
    ccfg.calibration_factor1 = cfg::scale::kCalibrationFactor1;
    ccfg.calibration_factor2 = cfg::scale::kCalibrationFactor2;
    (void)core.set_config(ccfg);

    bool          tared   = false;
    std::uint32_t log_div = 0;
    for (;;) {
        if (dev->ready()) {
            std::int32_t r1 = 0;
            std::int32_t r2 = 0;
            if (dev->read(r1, r2).ok()) {
                core.update(r1, r2);
                if (!tared) {
                    core.tare();
                    tared = true;
                    ESP_LOGI(kTag, "scale: tared");
                }
                const float g = core.weight_g();
                g_weight_g.store(g, std::memory_order_relaxed);
                if ((++log_div % 20U) == 0U) {
                    ESP_LOGI(kTag, "scale: %.1f g (r1=%ld r2=%ld)",
                             static_cast<double>(g),
                             static_cast<long>(r1), static_cast<long>(r2));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(cfg::scale::kReadPeriodMs));
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
    static espid::heater::SlowPwmSsrHeater heater;
    static espid::abp2::Abp2Driver         pressure;
    static espid::pump::PsmPump            pump;
    static espid::valve::SolenoidValve     valve;
    static espid::scale::Hx711Scale        scale_hw;

    const espid::tsic306::Tsic306Driver::Config drv_cfg{cfg::tsic::kDataGpio, 0};

    // Power the TSIC306 from GPIO2 (draws <1 mA, well within GPIO source limit).
    gpio_config_t tsic_pwr_cfg{};
    tsic_pwr_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg::tsic::kPowerGpio);
    tsic_pwr_cfg.mode         = GPIO_MODE_OUTPUT;
    tsic_pwr_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    tsic_pwr_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    tsic_pwr_cfg.intr_type    = GPIO_INTR_DISABLE;
    (void)gpio_config(&tsic_pwr_cfg);
    gpio_set_level(cfg::tsic::kPowerGpio, 1);
    vTaskDelay(pdMS_TO_TICKS(cfg::tsic::kPowerOnDelayMs));

    // Safety: force the heater SSR to its inactive level immediately at boot,
    // before control_task takes over after the fill. Prevents a floating gate
    // from energising the element during the (possibly minutes-long) fill.
    if (cfg::heater::kEnabled) {
        gpio_config_t htr_cfg{};
        htr_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg::heater::kSsrGpio);
        htr_cfg.mode         = GPIO_MODE_OUTPUT;
        htr_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        htr_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        htr_cfg.intr_type    = GPIO_INTR_DISABLE;
        (void)gpio_config(&htr_cfg);
        gpio_set_level(cfg::heater::kSsrGpio, cfg::heater::kSsrActiveHigh ? 0 : 1);
    }

    const auto s = driver.init(drv_cfg);
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
    const espid::hal_cpp::Status ds =
        cfg::oled::kEnabled
            ? display.init(dsp_cfg)
            : espid::hal_cpp::Status(espid::hal_cpp::Error::kNotInitialised);
    if (!cfg::oled::kEnabled) {
        ESP_LOGI(kTag, "OLED disabled in config - running headless");
    } else if (!ds.ok()) {
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

    bool pressure_ready = false;
    if (cfg::pressure::kEnabled) {
        espid::abp2::Abp2Driver::Config pcfg{};
        pcfg.i2c_port = static_cast<int>(cfg::pressure::kI2cBusNum);
        pcfg.sda = cfg::pressure::kSdaGpio;
        pcfg.scl = cfg::pressure::kSclGpio;
        pcfg.i2c_freq_hz = cfg::pressure::kI2cFreqHz;
        pcfg.i2c_addr_7bit = cfg::pressure::kI2cAddr7bit;
        pcfg.timeout_ms = cfg::pressure::kI2cTimeoutMs;
        pcfg.command_delay_ms = cfg::pressure::kCommandDelayMs;
        pcfg.transfer.output_min_counts = cfg::pressure::kOutputMinCounts;
        pcfg.transfer.output_max_counts = cfg::pressure::kOutputMaxCounts;
        pcfg.transfer.pressure_min_psi = cfg::pressure::kPressureMinPsi;
        pcfg.transfer.pressure_max_psi = cfg::pressure::kPressureMaxPsi;

        const auto ps = pressure.init(pcfg);
        if (!ps.ok()) {
            ESP_LOGW(kTag, "ABP2 init failed: %s - pressure disabled",
                     espid::hal_cpp::to_string(ps.error()));
        } else {
            pressure_ready = true;
        }
    }

    // PSM pump dimmer: ISR-driven, no periodic task. Starts OFF (0 %).
    if (cfg::pump::kEnabled) {
        // Pump neutral SSR (optional): only driven if still wired in. With the
        // neutral connected directly to mains, kSsrEnabled is false.
        if (cfg::pump::kSsrEnabled) {
            espid::valve::SolenoidValve::Config ssr_cfg{};
            ssr_cfg.gpio        = static_cast<int>(cfg::pump::kSsrGpio);
            ssr_cfg.active_high = cfg::pump::kSsrActiveHigh;
            static espid::valve::SolenoidValve pump_ssr;
            if (pump_ssr.init(ssr_cfg).ok()) {
                (void)pump_ssr.open();
                // Hold the pin HIGH across any sleep; PSM triac controls power.
                (void)gpio_hold_en(static_cast<gpio_num_t>(ssr_cfg.gpio));
                const int readback = gpio_get_level(static_cast<gpio_num_t>(ssr_cfg.gpio));
                ESP_LOGI(kTag, "pump SSR gpio=%d written=1 readback=%d%s",
                         ssr_cfg.gpio, readback,
                         readback == 1 ? "" : " ← GPIO FAULT - pin may be damaged");
            }
        }

        espid::pump::PsmPump::Config pump_cfg{};
        pump_cfg.zero_cross_gpio   = static_cast<int>(cfg::pump::kZeroCrossGpio);
        pump_cfg.gate_gpio         = static_cast<int>(cfg::pump::kPsmGpio);
        pump_cfg.gate_active_high  = cfg::pump::kPsmActiveHigh;
        pump_cfg.mains_frequency_hz = cfg::pump::kMainsFrequencyHz;
        pump_cfg.gate_pulse_us     = cfg::pump::kGatePulseUs;
        pump_cfg.glitch_filter_ns  =
            cfg::pump::kZcGlitchFilterEnable ? cfg::pump::kZcGlitchFilterNs : 0U;

        const auto pmps = pump.init(pump_cfg);
        if (!pmps.ok()) {
            ESP_LOGW(kTag, "pump PSM init failed: %s - pump disabled",
                     espid::hal_cpp::to_string(pmps.error()));
        } else {
            (void)pump.set_power_pct(0.0F);
            ESP_LOGI(kTag, "pump PSM ready (zc=%d gate=%d)",
                     pump_cfg.zero_cross_gpio, pump_cfg.gate_gpio);
        }
    }

    // 3-way solenoid valve output; starts closed, opened during brew.
    bool valve_ready = false;
    if (cfg::valve::kEnabled) {
        espid::valve::SolenoidValve::Config vcfg{};
        vcfg.gpio        = static_cast<int>(cfg::valve::kRelayGpio);
        vcfg.active_high = cfg::valve::kActiveHigh;

        const auto vs = valve.init(vcfg);
        if (!vs.ok()) {
            ESP_LOGW(kTag, "valve init failed: %s - valve disabled",
                     espid::hal_cpp::to_string(vs.error()));
        } else {
            valve_ready = true;
            // Valve starts closed (LOW). Readback confirms GPIO is responding.
            const int readback = gpio_get_level(static_cast<gpio_num_t>(vcfg.gpio));
            ESP_LOGI(kTag, "valve gpio=%d written=0 readback=%d%s",
                     vcfg.gpio, readback,
                     readback == 0 ? " (closed, OK)" : " ← UNEXPECTED HIGH");
        }
    }

    // Dual-HX711 scale (disabled by default; bench-test before enabling).
    bool scale_ready = false;
    if (cfg::scale::kEnabled) {
        espid::scale::Hx711Scale::Config sc{};
        sc.data1_gpio = static_cast<int>(cfg::scale::kData1Gpio);
        sc.data2_gpio = static_cast<int>(cfg::scale::kData2Gpio);
        sc.clock_gpio = static_cast<int>(cfg::scale::kClockGpio);

        const auto ss = scale_hw.init(sc);
        if (!ss.ok()) {
            ESP_LOGW(kTag, "scale init failed: %s - scale disabled",
                     espid::hal_cpp::to_string(ss.error()));
        } else {
            scale_ready = true;
            ESP_LOGI(kTag, "scale ready (d1=%d d2=%d clk=%d)",
                     sc.data1_gpio, sc.data2_gpio, sc.clock_gpio);
        }
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
            load_runtime_control_settings_from_nvs();

            espid::hal_cpp::INetwork::Config wifi_cfg{};
            const bool has_saved_sta = load_sta_credentials_from_nvs(
                g_saved_sta_ssid, sizeof(g_saved_sta_ssid),
                g_saved_sta_password, sizeof(g_saved_sta_password));
            if (has_saved_sta) {
                ESP_LOGI(kTag, "Using saved STA credentials from NVS");
            }

            wifi_cfg.sta_ssid = has_saved_sta ? g_saved_sta_ssid : cfg::net::kStaSsid;
            wifi_cfg.sta_password = has_saved_sta
                                        ? g_saved_sta_password
                                        : cfg::net::kStaPassword;
            wifi_cfg.ap_ssid = cfg::net::kApSsid;
            wifi_cfg.ap_password = cfg::net::kApPassword;
            wifi_cfg.hostname = cfg::net::kHostname;
            wifi_cfg.max_sta_retries = cfg::net::kMaxReconnects;
            wifi_cfg.reconnect_delay_ms = cfg::net::kReconnectDelayMs;

            const auto ws = wifi.init(wifi_cfg);
            if (!ws.ok()) {
                // Non-fatal: the driver often returns ESP_ERR_WIFI_CONN on a
                // first-attempt transient and self-heals. Log and continue.
                ESP_LOGW(kTag, "Wi-Fi init error: %s - proceeding, driver will retry",
                         espid::hal_cpp::to_string(ws.error()));
            }

            g_network = &wifi;

            if (cfg::net::kHttpServerEnabled) {
                (void)mount_littlefs();
                (void)start_http_server();
            }

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
                    esp_wifi_set_ps(WIFI_PS_NONE);
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

    // Heater PID control loop. Skipped in pump test mode so the boiler
    // never heats while the pump is being characterised dry.
    if (!cfg::pump::kTestMode) {
        ok = xTaskCreatePinnedToCore(
            &control_task, "control", cfg::pid::kTaskStackBytes, &heater,
            cfg::pid::kTaskPriority, nullptr, cfg::pid::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGE(kTag, "control_task create failed - halting");
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    } else {
        ESP_LOGW(kTag, "pump test mode: heater control loop DISABLED");
    }
    if (pressure_ready) {
        static PressureTaskCtx pressure_ctx{&pressure};
        ok = xTaskCreatePinnedToCore(
            &pressure_task, "pressure", cfg::pressure::kTaskStackBytes,
            &pressure_ctx, cfg::pressure::kTaskPriority, nullptr,
            cfg::pressure::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "pressure_task create failed - continuing without pressure");
        }
    }

    if (cfg::pump::kEnabled) {
        static PumpTaskCtx pump_ctx{&pump,
                                    valve_ready ? &valve : nullptr};
        ok = xTaskCreatePinnedToCore(
            cfg::pump::kTestMode ? &pump_test_task : &pump_task,
            cfg::pump::kTestMode ? "pump_test" : "pump",
            cfg::pump::kTaskStackBytes, &pump_ctx,
            cfg::pump::kTaskPriority, nullptr, cfg::pump::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "pump task create failed - fill skipped");
            g_fill_complete.store(true, std::memory_order_relaxed);
        }
    } else {
        g_fill_complete.store(true, std::memory_order_relaxed);
    }

    if (ds.ok() && pressure_ready &&
        cfg::oled::kI2cBusNum == cfg::pressure::kI2cBusNum &&
        cfg::oled::kSdaGpio == cfg::pressure::kSdaGpio &&
        cfg::oled::kSclGpio == cfg::pressure::kSclGpio) {
        g_i2c_bus_mutex = xSemaphoreCreateMutex();
        if (g_i2c_bus_mutex == nullptr) {
            ESP_LOGW(kTag, "I2C bus mutex create failed; display/pressure remain unsynchronised");
        } else {
            ESP_LOGI(kTag, "I2C bus mutex enabled for shared OLED/ABP2 bus");
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
    if (cfg::brew::kEnabled && !cfg::pump::kTestMode) {
        gpio_config_t bcfg{};
        bcfg.pin_bit_mask = 1ULL << static_cast<int>(cfg::brew::kButtonGpio);
        bcfg.mode         = GPIO_MODE_INPUT;
        bcfg.pull_up_en   = cfg::brew::kButtonPullupEnable
                                ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        bcfg.pull_down_en = cfg::brew::kButtonPulldownEnable
                                ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        bcfg.intr_type    = GPIO_INTR_DISABLE;
        const esp_err_t err = gpio_config(&bcfg);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "brew button gpio_config failed: %s - skipping",
                     esp_err_to_name(err));
        } else {
            ok = xTaskCreatePinnedToCore(
                &button_task, "button", cfg::brew::kTaskStackBytes,
                valve_ready ? &valve : nullptr,
                cfg::brew::kTaskPriority, nullptr, cfg::brew::kTaskCore);
            if (ok != pdPASS) {
                ESP_LOGW(kTag, "button_task create failed - no brew control");
            }
        }
    }

    // Steam switch: latching input that selects the steam setpoint.
    if (cfg::steam::kEnabled) {
        gpio_config_t scfg{};
        scfg.pin_bit_mask = 1ULL << static_cast<int>(cfg::steam::kSwitchGpio);
        scfg.mode         = GPIO_MODE_INPUT;
        scfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        scfg.pull_down_en = cfg::steam::kPulldownEnable
                                ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        scfg.intr_type    = GPIO_INTR_DISABLE;
        const esp_err_t err = gpio_config(&scfg);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "steam switch gpio_config failed: %s - skipping",
                     esp_err_to_name(err));
        } else {
            ok = xTaskCreatePinnedToCore(
                &steam_task, "steam", cfg::brew::kTaskStackBytes, nullptr,
                cfg::brew::kTaskPriority, nullptr, cfg::brew::kTaskCore);
            if (ok != pdPASS) {
                ESP_LOGW(kTag, "steam_task create failed - no steam switch");
            }
        }
    }

    if (cfg::scale::kEnabled && scale_ready) {
        ok = xTaskCreatePinnedToCore(
            &scale_task, "scale", cfg::scale::kTaskStackBytes, &scale_hw,
            cfg::scale::kTaskPriority, nullptr, cfg::scale::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "scale_task create failed - no scale");
        }
    }

    if (cfg::mqtt::kEnabled) {
        if (!cfg::net::kWifiEnabled) {
            ESP_LOGW(kTag, "mqtt: disabled at runtime because wifi is disabled");
        } else {
            ok = xTaskCreatePinnedToCore(
                &mqtt_task, "mqtt", cfg::mqtt::kTaskStackBytes, nullptr,
                cfg::mqtt::kTaskPriority, nullptr, cfg::mqtt::kTaskCore);
            if (ok != pdPASS) {
                ESP_LOGW(kTag, "mqtt_task create failed - telemetry disabled");
            }
        }
    }

    if (cfg::hawkbit::kEnabled) {
        ok = xTaskCreatePinnedToCore(
            &hawkbit_task, "hawkbit", cfg::hawkbit::kTaskStackBytes, nullptr,
            cfg::hawkbit::kTaskPriority, nullptr, cfg::hawkbit::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "hawkbit_task create failed - OTA disabled");
        }
    }

    // Idle heartbeat from app_main so we can see the device is alive even if
    // no frames arrive (e.g. sensor disconnected).
    std::uint32_t tick = 0;
    while (true) {
        ESP_LOGI(kTag, "heartbeat tick=%" PRIu32, tick);
        g_heartbeat_tick.store(tick, std::memory_order_release);
        ++tick;
        vTaskDelay(pdMS_TO_TICKS(cfg::kHeartbeatPeriodMs));
    }
}
