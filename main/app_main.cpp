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
#include <cstring>
#include <cstdio>

#include "brew/brew_button.hpp"
#include "brew/shot_timer.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
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

    char json[512];
    const int n = std::snprintf(json, sizeof(json),
        "{\"ap_ssid\":\"%s\",\"wifi_ssid\":\"%s\","
        "\"pid_kp\":\"%s\",\"pid_tn\":\"%s\",\"pid_tv\":\"%s\","
        "\"brew_sp\":\"%s\",\"steam_sp\":\"%s\","
        "\"brew_t\":\"%s\",\"preinf_t\":\"%s\",\"preinf_pause_t\":\"%s\","
        "\"mqtt_host\":\"%s\",\"mqtt_port\":\"%s\"}",
        configured_ap_ssid(),
        g_saved_sta_ssid,
        pid_kp, pid_tn, pid_tv,
        brew_sp, steam_sp,
        brew_t, preinf_t, preinf_pause_t,
        mqtt_host, mqtt_port);

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

    if (!save_sta_credentials_to_nvs(ssid, password)) {
        return httpd_resp_send_500(req);
    }

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

    nvs_handle_t nvs{};
    esp_err_t err = nvs_open(kCfgNvsNamespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return httpd_resp_send_500(req);
    }

    char value[64]{};
    const struct Field {
        const char* form_name;
        const char* nvs_key;
    } fields[] = {
        {"pid_kp", kCfgPidKp},
        {"pid_tn", kCfgPidTn},
        {"pid_tv", kCfgPidTv},
        {"brew_sp", kCfgBrewSetpoint},
        {"steam_sp", kCfgSteamSetpoint},
        {"brew_t", kCfgBrewTime},
        {"preinf_t", kCfgPreinfusion},
        {"preinf_pause_t", kCfgPreinfusionPause},
        {"mqtt_host", kCfgMqttHost},
        {"mqtt_port", kCfgMqttPort},
    };

    for (const auto& f : fields) {
        if (form_field(body, f.form_name, value, sizeof(value))) {
            err = nvs_set_str(nvs, f.nvs_key, value);
            if (err != ESP_OK) {
                nvs_close(nvs);
                return httpd_resp_send_500(req);
            }
        }
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        return httpd_resp_send_500(req);
    }

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
            display->draw_text(0, 12, espid::hal_cpp::Font::kSmall, ap_hdr);
            display->draw_text(86, 12, espid::hal_cpp::Font::kSmall,
                               wifi_ip.data());
        } else {
            display->draw_text(0, 12, espid::hal_cpp::Font::kSmall,
                               "ESPID-Coffee");
            display->draw_text(96, 12, espid::hal_cpp::Font::kSmall,
                               wifi_label(wifi_state));
        }

        if (shot_active) {
            // Shot in progress (or just finished): timer takes the big slot,
            // temperature drops to the small footer.
            std::snprintf(big, sizeof(big), "%4.1fs",
                          static_cast<double>(shot_ms) / 1000.0);
            std::snprintf(small, sizeof(small), "T=%s C", temp_str);
            display->draw_text(8,  40, espid::hal_cpp::Font::kLarge, big);
            display->draw_text(0,  60, espid::hal_cpp::Font::kSmall, small);
        } else {
            // Idle: temperature is the headline; timer line shows 0.0 s.
            std::snprintf(small, sizeof(small), "shot %4.1f s",
                          static_cast<double>(shot_ms) / 1000.0);
            display->draw_text(8,  40, espid::hal_cpp::Font::kLarge, temp_str);
            display->draw_text(110, 22, espid::hal_cpp::Font::kSmall, "C");
            display->draw_text(0,  60, espid::hal_cpp::Font::kSmall, small);
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
