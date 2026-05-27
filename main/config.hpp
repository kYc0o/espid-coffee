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

#include <cstddef>
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
constexpr std::uint32_t kMaxConsecutiveSensorErrors = 3;
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
constexpr float         kBrewTimeS      = 25.0F;
constexpr float         kPreinfusionS   = 5.0F;
constexpr float         kPreinfusionPauseS = 3.0F;
constexpr float         kBrewTimeMinS      = 1.0F;
constexpr float         kBrewTimeMaxS      = 120.0F;
constexpr float         kPreinfusionMinS   = 0.0F;
constexpr float         kPreinfusionMaxS   = 30.0F;
constexpr float         kPreinfusionPauseMinS = 0.0F;
constexpr float         kPreinfusionPauseMaxS = 30.0F;
}  // namespace brew

// ---------------------------------------------------------------------------
// PID controller (stub — not yet wired to a heater)
// ---------------------------------------------------------------------------
namespace pid {
constexpr float         kSetpointC      = 95.0F;
constexpr float         kSteamSetpointC = 140.0F;
constexpr float         kEmergencyStopTempC = 120.0F;
constexpr float         kEmergencyResetTempC = 100.0F;
constexpr float         kKp             = 30.0F;
constexpr float         kKi             = 0.4F;
constexpr float         kKd             = 60.0F;
constexpr float         kOutputMin      = 0.0F;
constexpr float         kOutputMax      = 100.0F;
constexpr float         kIntegralMin    = -50.0F;
constexpr float         kIntegralMax    = 50.0F;
constexpr float         kSamplePeriodS  = 0.2F;   // 5 Hz loop
constexpr float         kEmaFactor      = 0.6F;   // input smoothing for D term
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::uint32_t kTaskPriority   = 5;
constexpr std::int32_t  kTaskCore       = 1;
constexpr float         kKpMin          = 0.0F;
constexpr float         kKpMax          = 200.0F;
constexpr float         kTnMinS         = 0.1F;
constexpr float         kTnMaxS         = 600.0F;
constexpr float         kTvMinS         = 0.0F;
constexpr float         kTvMaxS         = 120.0F;
constexpr float         kSetpointMinC   = 70.0F;
constexpr float         kSetpointMaxC   = 110.0F;
constexpr float         kSteamSetpointMinC = 110.0F;
constexpr float         kSteamSetpointMaxC = 170.0F;
}  // namespace pid

// ---------------------------------------------------------------------------
// Heater (slow-PWM SSR)
// ---------------------------------------------------------------------------
namespace heater {
constexpr bool          kEnabled        = false;
constexpr gpio_num_t    kSsrGpio        = GPIO_NUM_15;
constexpr bool          kSsrActiveHigh  = true;
constexpr std::uint32_t kPwmPeriodMs    = 1000;
}  // namespace heater

// ---------------------------------------------------------------------------
// Pressure sensor (Honeywell ABP2, I2C)
// ---------------------------------------------------------------------------
namespace pressure {
constexpr bool          kEnabled              = true;
constexpr std::uint8_t  kI2cAddr7bit          = 0x28;
constexpr std::uint8_t  kI2cBusNum            = oled::kI2cBusNum;
constexpr gpio_num_t    kSdaGpio              = oled::kSdaGpio;
constexpr gpio_num_t    kSclGpio              = oled::kSclGpio;
constexpr std::uint32_t kI2cFreqHz            = 100'000;
constexpr std::uint32_t kI2cTimeoutMs         = 50;
constexpr std::uint32_t kCommandDelayMs       = 10;
constexpr std::uint32_t kReadPeriodMs         = 200;
constexpr std::uint32_t kTaskStackBytes       = 4096;
constexpr std::uint32_t kTaskPriority         = 4;
constexpr std::int32_t  kTaskCore             = 1;
constexpr std::uint32_t kMaxConsecutiveErrors = 5;

// ABP2LANT010BG2A3XX transfer counts (10%-90% of 24-bit full scale).
constexpr std::uint32_t kOutputMinCounts = 1'677'722;
constexpr std::uint32_t kOutputMaxCounts = 15'099'494;

// Engineering span for ABP2LANT010BG2A3XX (0..10 bar).
constexpr float         kPressureMinPsi = 0.0F;
constexpr float         kPressureMaxPsi = 145.038F;
}  // namespace pressure

// ---------------------------------------------------------------------------
// Networking (stub — Wi-Fi / web UI not yet implemented)
// ---------------------------------------------------------------------------
namespace net {
constexpr bool          kWifiEnabled    = true;
constexpr const char*   kHostname       = "espid-coffee";
constexpr const char*   kStaSsid        = "";
constexpr const char*   kStaPassword    = "";
constexpr const char*   kApSsid         = "ESPID-Coffee";
constexpr const char*   kApPassword     = "EspidCoffee";
constexpr bool          kHttpServerEnabled = true;
constexpr std::uint16_t kHttpPort       = 80;
constexpr std::size_t   kStaSsidMaxLen  = 32;
constexpr std::size_t   kStaPasswordMaxLen = 64;
constexpr std::size_t   kProvisionFormMaxLen = 256;
constexpr std::size_t   kWpaMinPasswordLen = 8;
constexpr const char*   kProvisionNvsNamespace = "wifi_cfg";
constexpr const char*   kProvisionNvsStaSsidKey = "sta_ssid";
constexpr const char*   kProvisionNvsStaPasswordKey = "sta_pass";
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
constexpr std::uint16_t kServerPortMin  = 1;
constexpr std::uint16_t kServerPortMax  = 65535;
constexpr const char*   kUsername       = "";
constexpr const char*   kPassword       = "";
constexpr const char*   kTopicPrefix    = "espid/";
constexpr bool          kHassioDiscovery = false;
constexpr const char*   kHassioPrefix   = "homeassistant";
}  // namespace mqtt

}  // namespace espid::config
