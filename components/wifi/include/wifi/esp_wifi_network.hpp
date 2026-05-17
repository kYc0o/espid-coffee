// EspWifiNetwork — esp_wifi-backed INetwork implementation.
//
// Brings up Wi-Fi in station mode first; after configurable reconnect
// failures (or when no STA credentials are provided), switches to a
// soft-AP fallback.  Exposes coarse connection state + active IPv4 address
// via the HAL-level INetwork interface.
//
// Threading: init() is single-shot and must be called once from app start.
// state()/ip_string() are lock-free and safe from any task.

#pragma once

#include <atomic>
#include <cstdint>

#include "esp_event.h"
#include "esp_netif.h"

#include "hal_cpp/i_network.hpp"

namespace espid::wifi {

class EspWifiNetwork final : public hal_cpp::INetwork {
   public:
    EspWifiNetwork() = default;
    ~EspWifiNetwork() override;

    EspWifiNetwork(const EspWifiNetwork&) = delete;
    EspWifiNetwork& operator=(const EspWifiNetwork&) = delete;

    // INetwork
    hal_cpp::Status init(const Config& cfg) noexcept override;
    hal_cpp::WifiState state() const noexcept override;
    IpString ip_string() const noexcept override;

   private:
    static void on_wifi_event_(void* arg, esp_event_base_t event_base,
                               std::int32_t event_id, void* event_data);
    static void on_ip_event_(void* arg, esp_event_base_t event_base,
                             std::int32_t event_id, void* event_data);

    void handle_wifi_event_(std::int32_t event_id, void* event_data) noexcept;
    void handle_ip_event_(std::int32_t event_id, void* event_data) noexcept;

    hal_cpp::Status start_sta_() noexcept;
    hal_cpp::Status start_ap_fallback_() noexcept;
    bool has_sta_credentials_() const noexcept;
    const char* ap_ssid_() noexcept;

    static constexpr std::size_t kGeneratedApSsidLen = 32;

    Config cfg_{};

    esp_netif_t* sta_netif_{nullptr};
    esp_netif_t* ap_netif_{nullptr};
    esp_event_handler_instance_t wifi_evt_inst_{nullptr};
    esp_event_handler_instance_t ip_evt_inst_{nullptr};

    std::atomic<hal_cpp::WifiState> state_{hal_cpp::WifiState::kIdle};
    std::atomic<std::uint32_t> ip_addr_{0};
    std::atomic<std::uint8_t> sta_retries_{0};

    char generated_ap_ssid_[kGeneratedApSsidLen]{};

    bool initialised_{false};
    bool wifi_started_{false};
};

}  // namespace espid::wifi
