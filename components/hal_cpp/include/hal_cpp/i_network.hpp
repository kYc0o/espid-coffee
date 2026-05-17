// INetwork — minimal Wi-Fi / IP network contract.
//
// Concrete drivers (e.g. an esp_wifi-backed STA + AP-fallback
// implementation) bring up a network interface, expose a coarse-grained
// state machine, and publish whatever IPv4 address is currently active.
// The interface is intentionally narrow: bring-up + observability only.
// Higher-level concerns (mDNS, HTTP server, OTA, captive portal) sit on
// top of an already-running INetwork and are not part of this contract.
//
// Lifecycle: init(Config) once; then call state()/ip_string() at any
// time from any task.  Implementations must guarantee both observers
// are safe to invoke concurrently with internal state changes.

#pragma once

#include <array>
#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

// Coarse-grained connection state.  Concrete drivers map their internal
// FSMs onto this enum so application code (and the OLED status glyph)
// can render a single value without knowing about esp_wifi events.
enum class WifiState : std::uint8_t {
    kIdle       = 0,  // init() not yet called
    kConnecting = 1,  // STA attempt in progress
    kConnected  = 2,  // STA associated + IP acquired
    kApFallback = 3,  // running as soft-AP after STA failures
    kError      = 4,  // unrecoverable failure (e.g. driver init)
};

constexpr const char* to_string(WifiState s) noexcept {
    switch (s) {
        case WifiState::kIdle:       return "idle";
        case WifiState::kConnecting: return "connecting";
        case WifiState::kConnected:  return "connected";
        case WifiState::kApFallback: return "ap-fallback";
        case WifiState::kError:      return "error";
    }
    return "unknown";
}

class INetwork {
   public:
    // Wi-Fi parameters passed in by the application.  `sta_ssid` /
    // `sta_password` are the preferred credentials; if association fails
    // (or they are empty), the implementation falls back to an open or
    // password-protected soft-AP whose SSID/password are taken from
    // `ap_ssid` / `ap_password`.
    struct Config {
        const char*    sta_ssid       = nullptr;
        const char*    sta_password   = nullptr;
        const char*    ap_ssid        = nullptr;
        const char*    ap_password    = nullptr;
        const char*    hostname       = nullptr;
        std::uint8_t   max_sta_retries = 5;
        std::uint32_t  reconnect_delay_ms = 10'000;
    };

    // IPv4 address rendered as a null-terminated dotted-quad
    // ("192.168.4.1\0", up to 16 chars including the terminator).
    using IpString = std::array<char, 16>;

    virtual ~INetwork() = default;

    // Bring the radio + IP stack up.  Returns:
    //   kOk             — driver/event-loop installed; FSM has moved out
    //                     of kIdle.  Connection may still be in progress.
    //   kAlreadyInit    — init() was already called successfully.
    //   kInvalidArg     — Config fields are missing or malformed.
    //   kHardwareFault  — esp_wifi / netif / event-loop init failure.
    virtual Status init(const Config& cfg) noexcept = 0;

    // Current coarse state.  Safe to call from any task.
    virtual WifiState state() const noexcept = 0;

    // Active IPv4 address (STA when connected, AP gateway when in
    // fallback).  Returns "0.0.0.0" when no address is bound.
    virtual IpString ip_string() const noexcept = 0;
};

}  // namespace espid::hal_cpp
