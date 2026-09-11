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
constexpr gpio_num_t    kPowerGpio      = GPIO_NUM_2;  // GPIO-sourced VDD; saves a 3V3 rail output
constexpr std::uint32_t kPowerOnDelayMs = 5;           // TSIC VDD ramp + startup
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
constexpr std::uint32_t kI2cLockTimeoutMs  = 50;
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
constexpr bool          kButtonActiveLow    = false;
constexpr bool          kButtonPullupEnable  = false;
constexpr bool          kButtonPulldownEnable = true;
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
// Steam switch (mode-select input)
// ---------------------------------------------------------------------------
// Wired to 3V3 through the machine switch; idle state held low by the S3
// internal pull-down, so no external resistor is required.
namespace steam {
constexpr bool          kEnabled        = true;
constexpr gpio_num_t    kSwitchGpio     = GPIO_NUM_4;
constexpr bool          kSwitchActiveHigh = true;
constexpr bool          kPulldownEnable   = true;
constexpr std::uint32_t kDebounceMs       = 10;
constexpr std::uint32_t kPollPeriodMs     = 10;
}  // namespace steam

// ---------------------------------------------------------------------------
// PID controller (stub — not yet wired to a heater)
// ---------------------------------------------------------------------------
namespace pid {
constexpr float         kSetpointC      = 92.0F;
constexpr float         kSteamSetpointC = 130.0F;
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
constexpr bool          kEnabled        = true;
constexpr gpio_num_t    kSsrGpio        = GPIO_NUM_47;  // GPIO19/20 are USB D-/D+
constexpr bool          kSsrActiveHigh  = true;
constexpr std::uint32_t kPwmPeriodMs    = 1000;
}  // namespace heater

// ---------------------------------------------------------------------------
// Solenoid valve (3-way, external relay/SSR module)
// ---------------------------------------------------------------------------
namespace valve {
constexpr bool          kEnabled        = true;
constexpr gpio_num_t    kRelayGpio      = GPIO_NUM_16;
constexpr bool          kActiveHigh     = true;
}  // namespace valve

// ---------------------------------------------------------------------------
// Pump (PSM phase-control dimmer: triac gate + mains zero-cross sync)
// ---------------------------------------------------------------------------
// The zero-cross opto pulses twice per mains cycle; the gate is retriggered
// each half-cycle at the computed phase delay.
namespace pump {
constexpr bool          kEnabled             = true;
constexpr gpio_num_t    kSsrGpio             = GPIO_NUM_15;  // always-on pump neutral SSR
constexpr bool          kSsrActiveHigh       = true;
constexpr gpio_num_t    kPsmGpio             = GPIO_NUM_17;  // triac gate out
constexpr bool          kPsmActiveHigh       = true;
constexpr gpio_num_t    kZeroCrossGpio       = GPIO_NUM_18;  // opto ZC input
constexpr bool          kZeroCrossActiveHigh = true;
constexpr std::uint32_t kMainsFrequencyHz    = 50;           // 100 Hz half-cycles
constexpr std::uint32_t kGatePulseUs         = 50;           // RobotDyn-style gate pulse
constexpr float         kDefaultPowerPct     = 100.0F;
constexpr float         kPowerMinPct         = 0.0F;
constexpr float         kPowerMaxPct         = 100.0F;

// Startup fill: pump runs at full power until the boiler has water.
constexpr float          kStartupFillThresholdBar = 1.0F;
constexpr std::uint32_t  kStartupFillTimeoutMs    = 120'000;  // empty boiler takes a while
constexpr float          kStartupFillPowerPct     = 100.0F;

// Pressure PID: output is pump PSM power %; setpoint is target group pressure.
// PID gains are conservative starting points — tune on the machine.
constexpr float          kPressureSetpointBar = 9.0F;
constexpr float          kPidKp               = 10.0F;
constexpr float          kPidKi               = 0.5F;
constexpr float          kPidKd               = 0.2F;
constexpr float          kPidOutputMin        = 0.0F;
constexpr float          kPidOutputMax        = 100.0F;
constexpr float          kPidIntegralMin      = -20.0F;
constexpr float          kPidIntegralMax      = 20.0F;
constexpr float          kPidSamplePeriodS    = 0.2F;
constexpr std::uint32_t  kTaskStackBytes      = 4096;
constexpr std::uint32_t  kTaskPriority        = 5;
constexpr std::int32_t   kTaskCore            = 1;
}  // namespace pump

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
constexpr std::uint32_t kReadRetryCount       = 1;
constexpr std::uint32_t kReadRetryDelayMs     = 5;
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
// Brew scale (dual HX711 — one load cell per side of the drip tray)
// ---------------------------------------------------------------------------
// Both HX711 share the clock line; each exposes its own data line. Total
// weight is the sum of the two calibrated cell readings.
namespace scale {
constexpr bool          kEnabled        = false;  // bench-test before enabling
constexpr std::uint8_t  kHx711Count     = 2;
constexpr gpio_num_t    kData1Gpio      = GPIO_NUM_7;
constexpr gpio_num_t    kData2Gpio      = GPIO_NUM_10;
constexpr gpio_num_t    kClockGpio      = GPIO_NUM_21;  // shared SCK
constexpr std::uint8_t  kSampleCount    = 2;            // averaged reads per sample
constexpr float         kCalibrationFactor1 = 1.0F;     // raw counts per gram, cell 1
constexpr float         kCalibrationFactor2 = 1.0F;     // raw counts per gram, cell 2
constexpr std::uint32_t kReadPeriodMs   = 100;
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::uint32_t kTaskPriority   = 3;
constexpr std::int32_t  kTaskCore       = 1;
}  // namespace scale

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
constexpr bool          kEnabled        = true;
constexpr bool          kUseTls         = true;
constexpr const char*   kServerIp       = "hono.lizher.eu";
constexpr std::uint16_t kServerPort     = 8883;
constexpr std::uint16_t kServerPortMin  = 1;
constexpr std::uint16_t kServerPortMax  = 65535;
constexpr const char*   kClientId       = "ESPID-Coffee-Rancilio";
constexpr const char*   kUsername       = "ESPID-Coffee-Rancilio@home-lab";
constexpr const char*   kPassword       = "";
constexpr const char*   kTopicPrefix    = "";
constexpr const char*   kTelemetryTopic = "telemetry";
constexpr bool          kHassioDiscovery = false;
constexpr const char*   kHassioPrefix   = "homeassistant";
constexpr std::size_t   kHostMaxLen     = 64;
constexpr std::size_t   kUsernameMaxLen = 64;
constexpr std::size_t   kPasswordMaxLen = 64;
constexpr std::size_t   kClientIdMaxLen = 64;
constexpr std::size_t   kUriMaxLen      = 96;
constexpr std::size_t   kTopicMaxLen    = 96;
constexpr std::size_t   kPayloadMaxLen  = 256;
constexpr std::uint32_t kTemperaturePublishPeriodMs = 30'000;
constexpr std::uint32_t kPressurePublishPeriodMs = 500;
constexpr std::uint32_t kRuntimeSettingsCheckPeriodMs = 1000;
constexpr std::uint32_t kSchedulerPeriodMs = 100;
constexpr std::uint32_t kReconnectDelayMs = 5000;
constexpr std::uint32_t kNetworkTimeoutMs = 10000;
constexpr std::uint32_t kTaskStackBytes = 6144;
constexpr std::uint32_t kTaskPriority   = 2;
constexpr std::int32_t  kTaskCore       = 1;
}  // namespace mqtt

}  // namespace espid::config
