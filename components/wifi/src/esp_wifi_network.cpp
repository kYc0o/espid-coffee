// EspWifiNetwork — STA-first Wi-Fi with AP fallback.

#include "wifi/esp_wifi_network.hpp"

#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "lwip/ip4_addr.h"

namespace espid::wifi {

namespace {

constexpr const char* kTag = "wifi_net";

constexpr std::uint8_t kDefaultApChannel = 1;
constexpr std::uint8_t kDefaultApMaxSta = 4;
constexpr std::uint16_t kDefaultApBeaconMs = 100;

bool is_empty(const char* s) noexcept {
    return s == nullptr || s[0] == '\0';
}

std::size_t bounded_len(const char* s, std::size_t max_len) noexcept {
    if (s == nullptr) {
        return 0;
    }
    std::size_t n = 0;
    while (n < max_len && s[n] != '\0') {
        ++n;
    }
    return n;
}

hal_cpp::Status hw_fault_status() noexcept {
    return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
}

}  // namespace

EspWifiNetwork::~EspWifiNetwork() {
    // App lifetime object; explicit teardown is not required for this firmware.
    // Keep destructor conservative and best-effort.
    if (ip_evt_inst_ != nullptr) {
        (void)esp_event_handler_instance_unregister(IP_EVENT,
                                                    IP_EVENT_STA_GOT_IP,
                                                    ip_evt_inst_);
        ip_evt_inst_ = nullptr;
    }
    if (wifi_evt_inst_ != nullptr) {
        (void)esp_event_handler_instance_unregister(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    wifi_evt_inst_);
        wifi_evt_inst_ = nullptr;
    }
}

hal_cpp::Status EspWifiNetwork::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    if (cfg.max_sta_retries == 0) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    cfg_ = cfg;
    sta_retries_.store(0, std::memory_order_relaxed);
    ip_addr_.store(0, std::memory_order_relaxed);
    state_.store(hal_cpp::WifiState::kIdle, std::memory_order_release);

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "esp_netif_init failed: %s", esp_err_to_name(err));
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "esp_event_loop_create_default failed: %s",
                 esp_err_to_name(err));
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    sta_netif_ = esp_netif_create_default_wifi_sta();
    if (sta_netif_ == nullptr) {
        ESP_LOGE(kTag, "esp_netif_create_default_wifi_sta failed");
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    ap_netif_ = esp_netif_create_default_wifi_ap();
    if (ap_netif_ == nullptr) {
        ESP_LOGE(kTag, "esp_netif_create_default_wifi_ap failed");
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    if (!is_empty(cfg_.hostname)) {
        const esp_err_t h = esp_netif_set_hostname(sta_netif_, cfg_.hostname);
        if (h != ESP_OK) {
            ESP_LOGW(kTag, "esp_netif_set_hostname failed: %s",
                     esp_err_to_name(h));
        }
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(err));
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &EspWifiNetwork::on_wifi_event_,
                                              this, &wifi_evt_inst_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "register WIFI_EVENT handler failed: %s",
                 esp_err_to_name(err));
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &EspWifiNetwork::on_ip_event_,
                                              this, &ip_evt_inst_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "register IP_EVENT handler failed: %s",
                 esp_err_to_name(err));
        state_.store(hal_cpp::WifiState::kError, std::memory_order_release);
        return hw_fault_status();
    }

    initialised_ = true;

    if (has_sta_credentials_()) {
        return start_sta_();
    }

    ESP_LOGW(kTag, "no STA credentials configured, entering AP fallback");
    return start_ap_fallback_();
}

hal_cpp::WifiState EspWifiNetwork::state() const noexcept {
    return state_.load(std::memory_order_acquire);
}

EspWifiNetwork::IpString EspWifiNetwork::ip_string() const noexcept {
    IpString out{};

    const std::uint32_t raw = ip_addr_.load(std::memory_order_acquire);
    if (raw == 0) {
        std::snprintf(out.data(), out.size(), "0.0.0.0");
        return out;
    }

    ip4_addr_t ip{};
    ip.addr = raw;
    if (ip4addr_ntoa_r(&ip, out.data(), out.size()) == nullptr) {
        std::snprintf(out.data(), out.size(), "0.0.0.0");
    }
    return out;
}

void EspWifiNetwork::on_wifi_event_(void* arg, esp_event_base_t /*event_base*/,
                                    std::int32_t event_id,
                                    void* event_data) {
    auto* self = static_cast<EspWifiNetwork*>(arg);
    if (self != nullptr) {
        self->handle_wifi_event_(event_id, event_data);
    }
}

void EspWifiNetwork::on_ip_event_(void* arg, esp_event_base_t /*event_base*/,
                                  std::int32_t event_id,
                                  void* event_data) {
    auto* self = static_cast<EspWifiNetwork*>(arg);
    if (self != nullptr) {
        self->handle_ip_event_(event_id, event_data);
    }
}

void EspWifiNetwork::handle_wifi_event_(std::int32_t event_id,
                                        void* /*event_data*/) noexcept {
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            state_.store(hal_cpp::WifiState::kConnecting,
                         std::memory_order_release);
            (void)esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            if (state_.load(std::memory_order_acquire) ==
                hal_cpp::WifiState::kApFallback) {
                return;
            }

            ip_addr_.store(0, std::memory_order_release);
            const std::uint8_t tries = static_cast<std::uint8_t>(
                sta_retries_.fetch_add(1, std::memory_order_acq_rel) + 1);

            if (tries <= cfg_.max_sta_retries) {
                state_.store(hal_cpp::WifiState::kConnecting,
                             std::memory_order_release);
                ESP_LOGW(kTag, "STA disconnect, retry %u/%u",
                         static_cast<unsigned>(tries),
                         static_cast<unsigned>(cfg_.max_sta_retries));
                (void)esp_wifi_connect();
            } else {
                ESP_LOGW(kTag,
                         "STA retries exhausted (%u), switching to AP fallback",
                         static_cast<unsigned>(cfg_.max_sta_retries));
                const auto s = start_ap_fallback_();
                if (!s.ok()) {
                    state_.store(hal_cpp::WifiState::kError,
                                 std::memory_order_release);
                }
            }
            break;
        }

        default:
            break;
    }
}

void EspWifiNetwork::handle_ip_event_(std::int32_t event_id,
                                      void* event_data) noexcept {
    if (event_id != IP_EVENT_STA_GOT_IP || event_data == nullptr) {
        return;
    }

    const auto* ev = static_cast<const ip_event_got_ip_t*>(event_data);
    ip_addr_.store(ev->ip_info.ip.addr, std::memory_order_release);
    sta_retries_.store(0, std::memory_order_relaxed);
    state_.store(hal_cpp::WifiState::kConnected, std::memory_order_release);

    ESP_LOGI(kTag, "STA connected, IP=" IPSTR, IP2STR(&ev->ip_info.ip));
}

hal_cpp::Status EspWifiNetwork::start_sta_() noexcept {
    wifi_config_t sta_cfg{};

    const int nw = std::snprintf(reinterpret_cast<char*>(sta_cfg.sta.ssid),
                                 sizeof(sta_cfg.sta.ssid), "%s",
                                 cfg_.sta_ssid);
    if (nw < 0 || static_cast<std::size_t>(nw) >= sizeof(sta_cfg.sta.ssid)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    const char* pass = is_empty(cfg_.sta_password) ? "" : cfg_.sta_password;
    const int pw = std::snprintf(
        reinterpret_cast<char*>(sta_cfg.sta.password),
        sizeof(sta_cfg.sta.password), "%s", pass);
    if (pw < 0 || static_cast<std::size_t>(pw) >= sizeof(sta_cfg.sta.password)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
    sta_cfg.sta.failure_retry_cnt = cfg_.max_sta_retries;
    sta_cfg.sta.pmf_cfg.capable = true;
    sta_cfg.sta.pmf_cfg.required = false;

    const esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (mode_err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_mode(STA) failed: %s",
                 esp_err_to_name(mode_err));
        return hw_fault_status();
    }

    const esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_config(STA) failed: %s",
                 esp_err_to_name(cfg_err));
        return hw_fault_status();
    }

    if (!wifi_started_) {
        const esp_err_t start_err = esp_wifi_start();
        if (start_err != ESP_OK) {
            ESP_LOGE(kTag, "esp_wifi_start failed: %s", esp_err_to_name(start_err));
            return hw_fault_status();
        }
        wifi_started_ = true;
    }

    state_.store(hal_cpp::WifiState::kConnecting, std::memory_order_release);
    sta_retries_.store(0, std::memory_order_relaxed);
    ip_addr_.store(0, std::memory_order_release);

    const esp_err_t conn_err = esp_wifi_connect();
    if (conn_err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_connect failed: %s", esp_err_to_name(conn_err));
        return hw_fault_status();
    }

    ESP_LOGI(kTag, "STA connect started (ssid=%s)", cfg_.sta_ssid);
    return hal_cpp::Status{};
}

hal_cpp::Status EspWifiNetwork::start_ap_fallback_() noexcept {
    wifi_config_t ap_cfg{};

    const char* ssid = ap_ssid_();
    const std::size_t ssid_len = bounded_len(ssid, sizeof(ap_cfg.ap.ssid));
    if (ssid_len == 0 || ssid_len > 32) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    std::memcpy(ap_cfg.ap.ssid, ssid, ssid_len);
    ap_cfg.ap.ssid_len = static_cast<std::uint8_t>(ssid_len);

    const char* password = is_empty(cfg_.ap_password) ? "" : cfg_.ap_password;
    const std::size_t pass_len = bounded_len(password, sizeof(ap_cfg.ap.password));

    if (pass_len >= 8) {
        std::memcpy(ap_cfg.ap.password, password, pass_len);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        std::memset(ap_cfg.ap.password, 0, sizeof(ap_cfg.ap.password));
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ap_cfg.ap.channel = kDefaultApChannel;
    ap_cfg.ap.max_connection = kDefaultApMaxSta;
    ap_cfg.ap.beacon_interval = kDefaultApBeaconMs;

    (void)esp_wifi_disconnect();

    const esp_err_t mode_err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (mode_err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_mode(AP) failed: %s",
                 esp_err_to_name(mode_err));
        return hw_fault_status();
    }

    const esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_config(AP) failed: %s",
                 esp_err_to_name(cfg_err));
        return hw_fault_status();
    }

    if (!wifi_started_) {
        const esp_err_t start_err = esp_wifi_start();
        if (start_err != ESP_OK) {
            ESP_LOGE(kTag, "esp_wifi_start (AP) failed: %s",
                     esp_err_to_name(start_err));
            return hw_fault_status();
        }
        wifi_started_ = true;
    }

    if (ap_netif_ != nullptr) {
        esp_netif_ip_info_t info{};
        if (esp_netif_get_ip_info(ap_netif_, &info) == ESP_OK) {
            ip_addr_.store(info.ip.addr, std::memory_order_release);
        } else {
            ip_addr_.store(0, std::memory_order_release);
        }
    }

    state_.store(hal_cpp::WifiState::kApFallback, std::memory_order_release);
    sta_retries_.store(0, std::memory_order_relaxed);

    ESP_LOGW(kTag, "AP fallback active (ssid=%s)", ssid);
    return hal_cpp::Status{};
}

bool EspWifiNetwork::has_sta_credentials_() const noexcept {
    return !is_empty(cfg_.sta_ssid);
}

const char* EspWifiNetwork::ap_ssid_() noexcept {
    if (!is_empty(cfg_.ap_ssid)) {
        return cfg_.ap_ssid;
    }

    std::uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) {
        std::snprintf(generated_ap_ssid_, sizeof(generated_ap_ssid_), "ESPID-Coffee");
        return generated_ap_ssid_;
    }

    std::snprintf(generated_ap_ssid_, sizeof(generated_ap_ssid_),
                  "ESPID-%02X%02X%02X", mac[3], mac[4], mac[5]);
    return generated_ap_ssid_;
}

}  // namespace espid::wifi
