// ESPID-Coffee — compile-time configuration.
//
// Mirrors the role of the legacy ranciliopid `userConfig.h`, but in
// modern C++: namespaced `constexpr` values instead of preprocessor
// macros, grouped by subsystem.  These are the boot-time defaults;
// once the embedded web UI lands, runtime-mutable parameters will be
// persisted to NVS and override the values declared here.
//
// Sections marked "stub" describe hardware that is not yet wired in
// firmware; the values are placeholders so the migration to a single
// source of truth is incremental.

#pragma once

#include <cstdint>

#include "driver/gpio.h"

namespace espid::config {

// ---------------------------------------------------------------------------
// Firmware identity
// ---------------------------------------------------------------------------
constexpr const char* kFirmwareName    = "ESPID-Coffee";
constexpr std::uint8_t kFwVersionMajor = 0;
constexpr std::uint8_t kFwVersionMinor = 0;
constexpr std::uint8_t kFwVersionPatch = 4;
constexpr const char*  kFwBranch       = "main";

// ---------------------------------------------------------------------------
// Machine selection
// ---------------------------------------------------------------------------
enum class Machine : std::uint8_t {
    kRancilioSilvia  = 0,
    kRancilioSilviaE = 1,
    kGaggia          = 2,
    kQuickMill       = 3,
};

constexpr Machine kMachine = Machine::kRancilioSilvia;

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
constexpr const char* kLogTag = "espid";

// ---------------------------------------------------------------------------
// Temperature sensor (TSIC 306 via RMT / ZACwire)
// ---------------------------------------------------------------------------
namespace tsic {
constexpr gpio_num_t    kDataGpio       = GPIO_NUM_1;
constexpr std::uint32_t kFrameTimeoutMs = 500;
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::uint32_t kTaskPriority   = 5;
constexpr std::int32_t  kTaskCore       = 1;
}  // namespace tsic

// ---------------------------------------------------------------------------
// OLED display (SSD1306, I2C)
// ---------------------------------------------------------------------------
namespace oled {
constexpr bool          kEnabled        = true;
constexpr gpio_num_t    kSdaGpio        = GPIO_NUM_8;
constexpr gpio_num_t    kSclGpio        = GPIO_NUM_9;
constexpr std::uint32_t kI2cFreqHz      = 400'000;
constexpr std::uint8_t  kI2cAddr7bit    = 0x3C;
constexpr std::uint8_t  kI2cBusNum      = 0;
constexpr std::uint16_t kWidthPx        = 128;
constexpr std::uint16_t kHeightPx       = 64;
// 0 = normal, 1 = 90° CW, 2 = 180°, 3 = 270° CW.
constexpr std::uint8_t  kRotationQuadrants = 2;
constexpr std::uint32_t kRefreshPeriodMs   = 200;
constexpr std::uint32_t kSplashDurationMs  = 2000;
constexpr std::uint32_t kTaskStackBytes    = 4096;
constexpr std::uint32_t kTaskPriority      = 4;
constexpr std::int32_t  kTaskCore          = 1;
}  // namespace oled

// ---------------------------------------------------------------------------
// Brew button + shot timer
// ---------------------------------------------------------------------------
namespace brew {
constexpr bool          kEnabled        = true;
constexpr gpio_num_t    kButtonGpio     = GPIO_NUM_5;
constexpr bool          kButtonActiveLow = true;
constexpr bool          kButtonPullupEnable = true;  // internal fallback
constexpr std::uint32_t kDebounceMs     = 10;
constexpr std::uint32_t kPollPeriodMs   = 10;
constexpr std::uint32_t kTaskStackBytes = 3072;
constexpr std::uint32_t kTaskPriority   = 3;
constexpr std::int32_t  kTaskCore       = 1;
// Time in ms the shot result remains highlighted after the shot ends.
constexpr std::uint32_t kPostShotHoldMs = 5000;
}  // namespace brew

// ---------------------------------------------------------------------------
// PID controller (stub — not yet wired to a heater)
// ---------------------------------------------------------------------------
namespace pid {
constexpr float         kSetpointC      = 95.0F;
constexpr float         kKp             = 30.0F;
constexpr float         kKi             = 0.4F;
constexpr float         kKd             = 60.0F;
constexpr float         kOutputMin      = 0.0F;
constexpr float         kOutputMax      = 100.0F;
constexpr float         kIntegralMin    = -50.0F;
constexpr float         kIntegralMax    = 50.0F;
constexpr float         kSamplePeriodS  = 0.2F;   // 5 Hz loop
constexpr float         kEmaFactor      = 0.6F;   // input smoothing for D term
}  // namespace pid

// ---------------------------------------------------------------------------
// Heater (stub — slow-PWM SSR not yet implemented)
// ---------------------------------------------------------------------------
namespace heater {
constexpr bool          kEnabled        = false;
constexpr gpio_num_t    kSsrGpio        = GPIO_NUM_15;
constexpr bool          kSsrActiveHigh  = true;
constexpr std::uint32_t kPwmPeriodMs    = 1000;
}  // namespace heater

// ---------------------------------------------------------------------------
// Networking (stub — Wi-Fi / web UI not yet implemented)
// ---------------------------------------------------------------------------
namespace net {
constexpr bool          kWifiEnabled    = false;
constexpr const char*   kHostname       = "espid-coffee";
constexpr const char*   kApPassword     = "EspidCoffee";  // WiFiManager AP fallback
constexpr std::uint8_t  kMaxReconnects  = 5;
constexpr std::uint32_t kReconnectDelayMs = 10'000;
constexpr bool          kOtaEnabled     = false;
}  // namespace net

// ---------------------------------------------------------------------------
// MQTT (stub)
// ---------------------------------------------------------------------------
namespace mqtt {
constexpr bool          kEnabled        = false;
constexpr const char*   kServerIp       = "0.0.0.0";
constexpr std::uint16_t kServerPort     = 1883;
constexpr const char*   kUsername       = "";
constexpr const char*   kPassword       = "";
constexpr const char*   kTopicPrefix    = "espid/";
constexpr bool          kHassioDiscovery = false;
constexpr const char*   kHassioPrefix   = "homeassistant";
}  // namespace mqtt

}  // namespace espid::config
