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
#include "pump/psm_pump.hpp"
#include "scale/hx711_scale.hpp"
#include "scale/scale_core.hpp"
#include "ssd1306/ssd1306_display.hpp"
#include "tsic306/tsic306_decoder.hpp"
#include "tsic306/tsic306_driver.hpp"
#include "valve/solenoid_valve.hpp"
#include "wifi/esp_wifi_network.hpp"
#include "esp_wifi.h"

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
};

RuntimeControlSettings g_runtime_control;
std::atomic<float> g_control_duty_pct{0.0F};
std::atomic<float> g_weight_g{0.0F};
std::atomic<std::uint32_t> g_sensor_consecutive_errors{0};
std::atomic<std::uint32_t> g_pressure_consecutive_errors{0};
std::atomic<bool> g_mqtt_connected{false};
std::atomic<bool> g_emergency_stop{false};
std::atomic<bool> g_steam_active{false};
std::atomic<bool> g_fill_complete{false};
std::atomic<bool> g_fill_ok{false};
SemaphoreHandle_t g_i2c_bus_mutex = nullptr;

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

void publish_runtime_control_settings(float pid_kp,
                                      float pid_tn_s,
                                      float pid_tv_s,
                                      float brew_setpoint_c,
                                      float steam_setpoint_c) noexcept {
    g_runtime_control.pid_kp.store(pid_kp, std::memory_order_relaxed);
    g_runtime_control.pid_tn_s.store(pid_tn_s, std::memory_order_relaxed);
    g_runtime_control.pid_tv_s.store(pid_tv_s, std::memory_order_relaxed);
    g_runtime_control.brew_setpoint_c.store(brew_setpoint_c, std::memory_order_relaxed);
    g_runtime_control.steam_setpoint_c.store(steam_setpoint_c, std::memory_order_relaxed);
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
    float pid_kp = cfg::pid::kKp;
    float pid_tn = default_pid_tn_seconds();
    float pid_tv = default_pid_tv_seconds();
    float brew_sp = cfg::pid::kSetpointC;
    float steam_sp = cfg::pid::kSteamSetpointC;

    char value[32];
    if (load_setting_from_nvs(kCfgPidKp, nullptr, value, sizeof(value))) {
        (void)parse_float_in_range(value, cfg::pid::kKpMin, cfg::pid::kKpMax, &pid_kp);
    }
    if (load_setting_from_nvs(kCfgPidTn, nullptr, value, sizeof(value))) {
        (void)parse_float_in_range(value, cfg::pid::kTnMinS, cfg::pid::kTnMaxS, &pid_tn);
    }
    if (load_setting_from_nvs(kCfgPidTv, nullptr, value, sizeof(value))) {
        (void)parse_float_in_range(value, cfg::pid::kTvMinS, cfg::pid::kTvMaxS, &pid_tv);
    }
    if (load_setting_from_nvs(kCfgBrewSetpoint, nullptr, value, sizeof(value))) {
        (void)parse_float_in_range(value,
                                   cfg::pid::kSetpointMinC,
                                   cfg::pid::kSetpointMaxC,
                                   &brew_sp);
    }
    if (load_setting_from_nvs(kCfgSteamSetpoint, nullptr, value, sizeof(value))) {
        (void)parse_float_in_range(value,
                                   cfg::pid::kSteamSetpointMinC,
                                   cfg::pid::kSteamSetpointMaxC,
                                   &steam_sp);
    }

    publish_runtime_control_settings(pid_kp, pid_tn, pid_tv, brew_sp, steam_sp);
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

    char json[768];
    const int n = std::snprintf(json, sizeof(json),
        "{\"ap_ssid\":\"%s\",\"wifi_ssid\":\"%s\","
        "\"pid_kp\":\"%s\",\"pid_tn\":\"%s\",\"pid_tv\":\"%s\","
        "\"brew_sp\":\"%s\",\"steam_sp\":\"%s\","
        "\"brew_t\":\"%s\",\"preinf_t\":\"%s\",\"preinf_pause_t\":\"%s\","
        "\"mqtt_host\":\"%s\",\"mqtt_port\":\"%s\","
        "\"mqtt_user\":\"%s\",\"mqtt_pass\":\"%s\",\"mqtt_client\":\"%s\"}",
        configured_ap_ssid(),
        g_saved_sta_ssid,
        pid_kp, pid_tn, pid_tv,
        brew_sp, steam_sp,
        brew_t, preinf_t, preinf_pause_t,
        mqtt_host, mqtt_port, mqtt_user, mqtt_pass, mqtt_client);

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

    float pid_kp = 0.0F;
    float pid_tn = 0.0F;
    float pid_tv = 0.0F;
    float brew_sp = 0.0F;
    float steam_sp = 0.0F;
    float brew_t = 0.0F;
    float preinf_t = 0.0F;
    float preinf_pause_t = 0.0F;
    std::uint16_t mqtt_port = 0;

    if (!parse_float_in_range(pid_kp_txt, cfg::pid::kKpMin, cfg::pid::kKpMax, &pid_kp)) {
        return reject_bad_request("Invalid pid_kp");
    }
    if (!parse_float_in_range(pid_tn_txt, cfg::pid::kTnMinS, cfg::pid::kTnMaxS, &pid_tn)) {
        return reject_bad_request("Invalid pid_tn");
    }
    if (!parse_float_in_range(pid_tv_txt, cfg::pid::kTvMinS, cfg::pid::kTvMaxS, &pid_tv)) {
        return reject_bad_request("Invalid pid_tv");
    }
    if (!parse_float_in_range(brew_sp_txt,
                              cfg::pid::kSetpointMinC,
                              cfg::pid::kSetpointMaxC,
                              &brew_sp)) {
        return reject_bad_request("Invalid brew_sp");
    }
    if (!parse_float_in_range(steam_sp_txt,
                              cfg::pid::kSteamSetpointMinC,
                              cfg::pid::kSteamSetpointMaxC,
                              &steam_sp)) {
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
        {kCfgBrewSetpoint, brew_sp_txt},
        {kCfgSteamSetpoint, steam_sp_txt},
        {kCfgBrewTime, brew_t_txt},
        {kCfgPreinfusion, preinf_t_txt},
        {kCfgPreinfusionPause, preinf_pause_txt},
        {kCfgMqttHost, mqtt_host_txt},
        {kCfgMqttPort, mqtt_port_txt},
        {kCfgMqttUsername, mqtt_user_txt},
        {kCfgMqttPassword, mqtt_pass_txt},
        {kCfgMqttClientId, mqtt_client_txt},
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

    publish_runtime_control_settings(pid_kp, pid_tn, pid_tv, brew_sp, steam_sp);

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
    http_cfg.server_port = static_cast<int>(cfg::net::kHttpPort);
    http_cfg.lru_purge_enable = true;

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
                        case 0U: status_label = "normal"; break;
                        case 1U: status_label = "command"; break;
                        case 2U: status_label = "stale"; break;
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
            const bool stale_sample = (s.status == espid::abp2::FrameStatus::kStaleData);
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
            }
            prev_shot_running = shot_running;

            if (mqtt_connected) {
                if (now_ms >= next_temp_publish_ms) {
                    const bool temp_valid =
                        g_snapshot.valid.load(std::memory_order_acquire);
                    const float temp_c =
                        g_snapshot.temp_c.load(std::memory_order_relaxed);

                    const int n = std::snprintf(
                        payload, sizeof(payload),
                        "{\"type\":\"temperature\",\"uptime_ms\":%" PRIu64
                        ",\"temp_c\":%.2f,\"temp_valid\":%u}",
                        now_ms,
                        static_cast<double>(temp_c),
                        temp_valid ? 1U : 0U);

                    if (n > 0 && static_cast<std::size_t>(n) < sizeof(payload)) {
                        publish_payload(payload);
                    }

                    next_temp_publish_ms =
                        now_ms + static_cast<std::uint64_t>(cfg::mqtt::kTemperaturePublishPeriodMs);
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
                        "{\"type\":\"pressure\",\"uptime_ms\":%" PRIu64
                        ",\"shot_s\":%.1f,\"pressure_bar\":%.3f"
                        ",\"pressure_valid\":%u,\"pressure_stale\":%u}",
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
            } else {
                next_temp_publish_ms = now_ms;
                next_pressure_publish_ms = now_ms;
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

        if (sample_fresh && temp_c >= cfg::pid::kEmergencyStopTempC) {
            const bool was_stopped = g_emergency_stop.exchange(true, std::memory_order_relaxed);
            if (!was_stopped) {
                ESP_LOGE(kTag, "EMERGENCY STOP: temp %.2fC >= %.2fC",
                         static_cast<double>(temp_c),
                         static_cast<double>(cfg::pid::kEmergencyStopTempC));
            }
        } else if (g_emergency_stop.load(std::memory_order_relaxed) &&
                   sample_fresh &&
                   temp_c <= cfg::pid::kEmergencyResetTempC) {
            g_emergency_stop.store(false, std::memory_order_relaxed);
            ESP_LOGW(kTag, "Emergency stop reset: temp %.2fC <= %.2fC",
                     static_cast<double>(temp_c),
                     static_cast<double>(cfg::pid::kEmergencyResetTempC));
        }
        const bool emergency_stop = g_emergency_stop.load(std::memory_order_relaxed);

        const float pid_kp = g_runtime_control.pid_kp.load(std::memory_order_relaxed);
        const float pid_tn_s = g_runtime_control.pid_tn_s.load(std::memory_order_relaxed);
        const float pid_tv_s = g_runtime_control.pid_tv_s.load(std::memory_order_relaxed);
        const float brew_setpoint_c =
            g_runtime_control.brew_setpoint_c.load(std::memory_order_relaxed);
        const bool steam_active =
            g_steam_active.load(std::memory_order_relaxed);
        const float steam_setpoint_c =
            g_runtime_control.steam_setpoint_c.load(std::memory_order_relaxed);
        const float active_setpoint_c =
            steam_active ? steam_setpoint_c : brew_setpoint_c;

        espid::pid::Controller::Config pcfg{};
        pcfg.kp = pid_kp;
        pcfg.ki = (pid_tn_s > 0.0F) ? (pid_kp / pid_tn_s) : 0.0F;
        pcfg.kd = pid_kp * pid_tv_s;
        pcfg.output_min = cfg::pid::kOutputMin;
        pcfg.output_max = cfg::pid::kOutputMax;
        pcfg.integral_min = cfg::pid::kIntegralMin;
        pcfg.integral_max = cfg::pid::kIntegralMax;
        pcfg.sample_period_s = cfg::pid::kSamplePeriodS;
        controller.set_config(pcfg);

        float duty_pct = 0.0F;
        if (sample_fresh && std::isfinite(temp_c) && !emergency_stop) {
            duty_pct = controller.update(active_setpoint_c, temp_c);
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

        if ((++log_div % 10U) == 0U) {
            ESP_LOGI(kTag, "control: duty=%.1f%% fresh=%d estop=%d serr=%" PRIu32,
                     static_cast<double>(duty_pct),
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

        char setpoint_str[16];
        if (std::isfinite(brew_setpoint_c)) {
            std::snprintf(setpoint_str, sizeof(setpoint_str), "SP:%4.1f",
                          static_cast<double>(brew_setpoint_c));
        } else {
            std::snprintf(setpoint_str, sizeof(setpoint_str), "SP:--.-");
        }

        display->clear();
        if (wifi_state == espid::hal_cpp::WifiState::kApFallback) {
            char ap_hdr[20];
            std::snprintf(ap_hdr, sizeof(ap_hdr), "AP:%s", configured_ap_ssid());
            display->draw_text(0, 10, espid::hal_cpp::Font::kSmall, ap_hdr);
            display->draw_text(86, 10, espid::hal_cpp::Font::kSmall,
                               wifi_ip.data());
        } else {
            display->draw_text(0, 10, espid::hal_cpp::Font::kSmall,
                               "ESPID-Coffee");
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
            std::snprintf(small, sizeof(small), "shot %4.1fs",
                          static_cast<double>(shot_ms) / 1000.0);
            std::snprintf(footer, sizeof(footer), "%s %sC", small, setpoint_str);
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
    espid::valve::SolenoidValve* valve;  // must open during fill to complete pump AC circuit
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
    // Open the solenoid/neutral relay so the pump AC circuit is complete.
    if (ctx->valve != nullptr) {
        (void)ctx->valve->open();
    }
    const std::uint64_t fill_deadline_ms =
        static_cast<std::uint64_t>(esp_timer_get_time() / 1000) +
        cfg::pump::kStartupFillTimeoutMs;
    bool fill_ok = false;
    for (;;) {
        const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
        if (p >= cfg::pump::kStartupFillThresholdBar) {
            fill_ok = true;
            ESP_LOGI(kTag, "pump: fill done (p=%.2f bar)", static_cast<double>(p));
            break;
        }
        if (static_cast<std::uint64_t>(esp_timer_get_time() / 1000) >= fill_deadline_ms) {
            ESP_LOGW(kTag, "pump: fill timeout - boiler may be empty; heater stays OFF");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)ctx->psm->set_power_pct(0.0F);
    // Close the solenoid after fill so the group head doesn't drip.
    if (ctx->valve != nullptr) {
        (void)ctx->valve->close();
    }
    g_fill_ok.store(fill_ok, std::memory_order_release);
    g_fill_complete.store(true, std::memory_order_release);
    ESP_LOGI(kTag, "pump: %s", fill_ok ? "ready for brewing" : "fill FAILED");

    // Phase 2: pressure PID — drives pump power during brew to maintain setpoint.
    espid::pid::Controller pid;
    espid::pid::Controller::Config pcfg{};
    pcfg.kp             = cfg::pump::kPidKp;
    pcfg.ki             = cfg::pump::kPidKi;
    pcfg.kd             = cfg::pump::kPidKd;
    pcfg.output_min     = cfg::pump::kPidOutputMin;
    pcfg.output_max     = cfg::pump::kPidOutputMax;
    pcfg.integral_min   = cfg::pump::kPidIntegralMin;
    pcfg.integral_max   = cfg::pump::kPidIntegralMax;
    pcfg.sample_period_s = cfg::pump::kPidSamplePeriodS;
    pid.set_config(pcfg);

    const TickType_t period = ticks_from_ms(
        static_cast<std::uint32_t>(cfg::pump::kPidSamplePeriodS * 1000.0F));

    std::uint32_t log_div = 0;
    for (;;) {
        const bool brewing = g_shot.running.load(std::memory_order_relaxed);
        if (brewing) {
            if (g_pressure.valid.load(std::memory_order_acquire)) {
                const float p = g_pressure.pressure_bar.load(std::memory_order_relaxed);
                const float power = pid.update(cfg::pump::kPressureSetpointBar, p);
                (void)ctx->psm->set_power_pct(power);
                if ((++log_div % 10U) == 0U) {
                    ESP_LOGI(kTag, "pump: p=%.2fbar power=%.1f%%",
                             static_cast<double>(p), static_cast<double>(power));
                }
            } else {
                // No pressure reading yet — run at full power as fallback.
                (void)ctx->psm->set_power_pct(cfg::pump::kDefaultPowerPct);
            }
        } else {
            pid.reset();
            (void)ctx->psm->set_power_pct(0.0F);
        }
        vTaskDelay(period);
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

    for (;;) {
        const int           raw    = gpio_get_level(cfg::brew::kButtonGpio);
        const std::uint64_t now_ms = static_cast<std::uint64_t>(
            esp_timer_get_time() / 1000);

        const auto ev = btn.poll(raw != 0, now_ms);
        if (ev == espid::brew::BrewButton::Event::kPressed) {
            timer.reset();
            timer.start(now_ms);
            if (valve != nullptr) {
                (void)valve->open();
            }
            ESP_LOGI(kTag, "brew: start");
        } else if (ev == espid::brew::BrewButton::Event::kReleased) {
            timer.stop(now_ms);
            if (valve != nullptr) {
                (void)valve->close();
            }
            ESP_LOGI(kTag, "brew: stop  elapsed=%u ms",
                     static_cast<unsigned>(timer.elapsed_ms(now_ms)));
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
            ESP_LOGI(kTag, "steam: %s", active ? "ON" : "OFF");
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
        // Drive the pump SSR always-on; PSM triac controls actual power.
        espid::valve::SolenoidValve::Config ssr_cfg{};
        ssr_cfg.gpio        = static_cast<int>(cfg::pump::kSsrGpio);
        ssr_cfg.active_high = cfg::pump::kSsrActiveHigh;
        static espid::valve::SolenoidValve pump_ssr;
        if (pump_ssr.init(ssr_cfg).ok()) {
            (void)pump_ssr.open();
            // Hold the pin HIGH across any sleep; PSM triac controls actual power.
            (void)gpio_hold_en(static_cast<gpio_num_t>(ssr_cfg.gpio));
            const int readback = gpio_get_level(static_cast<gpio_num_t>(ssr_cfg.gpio));
            ESP_LOGI(kTag, "pump SSR gpio=%d written=1 readback=%d%s",
                     ssr_cfg.gpio, readback,
                     readback == 1 ? "" : " ← GPIO FAULT - pin may be damaged");
        }

        espid::pump::PsmPump::Config pump_cfg{};
        pump_cfg.zero_cross_gpio   = static_cast<int>(cfg::pump::kZeroCrossGpio);
        pump_cfg.gate_gpio         = static_cast<int>(cfg::pump::kPsmGpio);
        pump_cfg.gate_active_high  = cfg::pump::kPsmActiveHigh;
        pump_cfg.mains_frequency_hz = cfg::pump::kMainsFrequencyHz;
        pump_cfg.gate_pulse_us     = cfg::pump::kGatePulseUs;

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

    ok = xTaskCreatePinnedToCore(
        &control_task, "control", cfg::pid::kTaskStackBytes, &heater,
        cfg::pid::kTaskPriority, nullptr, cfg::pid::kTaskCore);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "control_task create failed - halting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
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
            &pump_task, "pump", cfg::pump::kTaskStackBytes, &pump_ctx,
            cfg::pump::kTaskPriority, nullptr, cfg::pump::kTaskCore);
        if (ok != pdPASS) {
            ESP_LOGW(kTag, "pump_task create failed - fill skipped");
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
    if (cfg::brew::kEnabled) {
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

    // Idle heartbeat from app_main so we can see the device is alive even if
    // no frames arrive (e.g. sensor disconnected).
    std::uint32_t tick = 0;
    while (true) {
        ESP_LOGI(kTag, "heartbeat tick=%" PRIu32, tick++);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
