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
constexpr std::uint8_t kFwVersionPatch = 9;
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
// Logging and heartbeat
// ---------------------------------------------------------------------------
constexpr const char* kLogTag = "espid";
// app_main logs a heartbeat on this period; mqtt_task publishes a "status"
// record on every heartbeat tick, so dashboards get a fixed-rate liveness
// and machine-state feed independent of the adaptive telemetry cadence.
constexpr std::uint32_t kHeartbeatPeriodMs = 5000;

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
// Header "STEAM ready" once within kSteamReadyOnBelowC of the steam setpoint;
// back to "STEAM heating" only after dropping kSteamReadyOffBelowC below it
// (a 125 g pitcher pulled the boiler down ~6.6 C on 2026-09-24).
constexpr float         kSteamReadyOnBelowC  = 2.0F;
constexpr float         kSteamReadyOffBelowC = 5.0F;
// Operator prompts (press / release BREW) blink: visible kPromptBlinkOnMs of
// every kPromptBlinkPeriodMs.
constexpr std::uint32_t kPromptBlinkPeriodMs = 1000;
constexpr std::uint32_t kPromptBlinkOnMs     = 700;
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
// Heater PID controller
// ---------------------------------------------------------------------------
namespace pid {
constexpr float         kSetpointC      = 92.0F;
constexpr float         kSteamSetpointC = 130.0F;
constexpr float         kEmergencyStopTempC = 120.0F;
constexpr float         kEmergencyResetTempC = 100.0F;
// Steam mode targets a much higher boiler temperature, so the brew-mode
// over-temp guard would trip before the steam setpoint is reached. These
// thresholds apply while the steam switch is engaged; keep steam setpoints
// comfortably below kSteamEmergencyStopTempC for overshoot headroom.
constexpr float         kSteamEmergencyStopTempC  = 165.0F;
constexpr float         kSteamEmergencyResetTempC = 145.0F;
// After the steam switch goes off the boiler is still far above the brew stop
// threshold, so the steam limits stay in force (a "cool-down") until it has
// fallen this far below kEmergencyStopTempC. Without it every normal steam
// session tripped the brew-mode stop on the way back down.
constexpr float         kSteamCooldownMarginC     = 5.0F;
// Identified from a cold-start log (2026-09-16): at 100 % the boiler climbs
// 0.48 °C/s after ~4 s dead time and two lags of ~12 s and ~10 s (≈27 s of
// heat "in flight"); idle loss needs ~5.6 % duty at 94.5 °C. Tv ≈ that lag
// lets D cut the heater before the in-flight heat overshoots. Simulated cold
// start: ~2 °C overshoot vs the measured ~10 °C with Kp 31.5 / Tn 75 / Tv 1.8.
// Equivalent Tn/Tv: Kp 18, Tn 90 s, Tv 22 s.
constexpr float         kKp             = 18.0F;
constexpr float         kKi             = 0.2F;    // Kp / Tn
constexpr float         kKd             = 396.0F;  // Kp * Tv
constexpr float         kOutputMin      = 0.0F;
constexpr float         kOutputMax      = 100.0F;
// The integral only ever has to supply the holding duty: measured loss needs
// ~5.6 % at 94.5 °C and ~8.4 % at a 130 °C steam setpoint. The old ±50 clamp
// let it wind to +50 % during the final approach of a cold start, which the
// 2026-09-22 log showed delaying heater cut-off from ~83 °C to ~88 °C and
// adding ~2.2 °C of the 4.3 °C overshoot. Keep it just above what holding
// actually needs, so P and D decide the approach.
constexpr float         kIntegralMin    = -10.0F;
constexpr float         kIntegralMax    = 15.0F;
// The integral only runs within this band around setpoint. Further out the
// error reflects a transient (cold start, flush, shot), not the holding
// duty. Idle hunting stays inside it (±0.4 °C measured 2026-09-27).
constexpr float         kIntegralBandC  = 2.0F;
constexpr float         kSamplePeriodS  = 0.2F;   // 5 Hz loop
// First-order low-pass on dT/dt feeding the D term. The TSIC LSB is
// ~0.098 °C, so at 5 Hz one LSB flicker reads as ±0.49 °C/s; unfiltered,
// Kd = Kp*Tv turns that into a large duty spike.
constexpr float         kDerivativeFilterTauS = 4.0F;
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
// Brew-phase gains: active while the shot runs and for kBrewTrailingS
// afterwards. The heater lag (~27 s) is as long as a shot, so heat added in
// response to the dip mostly lands after the pump stops: in simulation a
// more aggressive set (Kp 50 / Tv 10) trimmed the dip by ~0.4 °C but nearly
// doubled the post-shot overshoot. Default is the idle set with Tn = 0 to
// freeze the integral (no windup from the dip, bumpless return).
// Tune from the "heat shot:" console summaries.
constexpr float         kBrewKp            = 18.0F;
constexpr float         kBrewTnS           = 0.0F;   // 0 = integral frozen
constexpr float         kBrewTvS           = 22.0F;
// Feed-forward pulse: heater forced to kBrewPulseDutyPct for the first
// kBrewPulseS of a shot, so that heat arrives while cold water is still
// flowing. Simulated at ~1 ml/s: 8 s halves the dip (2.0 → 0.9 °C) and cuts
// overshoot (2.9 → 1.6 °C). 0 = off.
constexpr float         kBrewPulseS        = 0.0F;
constexpr float         kBrewPulseMinS     = 0.0F;
constexpr float         kBrewPulseMaxS     = 30.0F;
constexpr float         kBrewPulseDutyPct  = 100.0F;
constexpr float         kBrewTnMinS        = 0.0F;
constexpr float         kBrewTrailingS     = 20.0F;
constexpr float         kBrewTrailingMinS  = 0.0F;
constexpr float         kBrewTrailingMaxS  = 120.0F;

// Console diagnostics. One "heat" line every N control steps; brew/trailing
// log every step so the dip and recovery are fully resolved.
constexpr std::uint32_t kLogEveryStepsIdle  = 5;    // 1 Hz at 5 Hz loop
constexpr std::uint32_t kLogEveryStepsBrew  = 1;    // 5 Hz
// |T - setpoint| band used for the per-shot settling time.
constexpr float         kShotSettleBandC    = 0.5F;
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
// Pump neutral SSR: removed from the circuit — neutral now wired directly
// between pump and mains. Left in config (disabled) for reference/reuse.
constexpr bool          kSsrEnabled          = false;
constexpr gpio_num_t    kSsrGpio             = GPIO_NUM_15;  // (unused; SSR removed)
constexpr bool          kSsrActiveHigh       = true;
constexpr gpio_num_t    kPsmGpio             = GPIO_NUM_17;  // triac gate out
constexpr bool          kPsmActiveHigh       = true;
constexpr gpio_num_t    kZeroCrossGpio       = GPIO_NUM_18;  // opto ZC input
constexpr bool          kZeroCrossActiveHigh = true;
// ESP32-S3 hardware glitch filter on the ZC pin. The S3 only offers the fixed
// pin glitch filter (~2 IO-MUX cycles, tens of ns) — a token backstop for fast
// spikes. The RC filter on the opto output is the primary noise fix. 0 = off.
constexpr bool          kZcGlitchFilterEnable = true;
constexpr std::uint32_t kZcGlitchFilterNs     = 500;  // advisory; S3 filter is fixed
constexpr std::uint32_t kMainsFrequencyHz    = 50;           // 100 Hz half-cycles
// Gate ON-time (hold) per fired half-cycle, held via a one-shot timer. ~2 ms
// lets the triac latch the inductive pump as the mains voltage rises past the
// zero-cross; a short blip often fails to latch and the pump runs erratically.
constexpr std::uint32_t kGatePulseUs         = 2000;
constexpr float         kDefaultPowerPct     = 100.0F;
constexpr float         kPowerMinPct         = 0.0F;
constexpr float         kPowerMaxPct         = 100.0F;

// Startup fill: pump runs at full power until the boiler has water.
constexpr float          kStartupFillThresholdBar = 1.0F;
constexpr std::uint32_t  kStartupFillTimeoutMs    = 120'000;  // empty boiler takes a while
constexpr float          kStartupFillPowerPct     = 100.0F;
// Minimum ZC edges during fill that confirm the pump was actually running.
// At 100/s (50 Hz both-edge), 60 s of pumping = 6000 edges.
constexpr std::uint32_t  kStartupFillMinZcCount   = 3000;  // 30 s minimum
// Consecutive above-threshold, non-stale samples needed to accept the fill.
// A single bogus ABP2 frame (7.33 bar on 2026-09-22, with the very next read
// back at 0.17 bar) otherwise satisfies the dry-boiler guard in ~3 s.
constexpr std::uint32_t  kStartupFillConfirmSamples = 3;

// Post-steam boiler refill (brew switch pressed during the steam cool-down).
// The valve stays closed, so water can only enter the boiler; while a steam
// space remains, pressure sits near the steam pressure (~2-3.5 bar measured
// at 120-130 C), and once the boiler is hydraulically full it climbs quickly
// toward the pump dead-head. 5 bar sits clear of both. Low power keeps the
// cold water from hitting the element as a slug. The timeout is the backstop
// for the steam knob left open (boiler never pressurises); the refill_stop
// telemetry reports the stop reason, so it can be widened if it trips on a
// closed knob. 60 s did trip on one: after a 9 min steam session with two
// pulls (2026-09-30) the boiler was still filling at 0.9 bar, 20 °C cooler
// from the water already pumped in. 49 s had been enough after a 6 min
// session. An open knob shows as water at the wand well before 120 s.
constexpr float          kRefillPowerPct        = 30.0F;
constexpr float          kRefillFullPressureBar = 5.0F;
constexpr std::uint32_t  kRefillConfirmMs       = 600;
constexpr std::uint32_t  kRefillMaxDurationMs   = 120'000;

// Pressure PID: output is pump PSM power %; setpoint is target group pressure.
// The original gains were validated with a blind portafilter, which passes no
// water: holding 9 bar then takes ~3 % pump and the integral never has to
// work. A real puck needs most of the pump's range, and telemetry from
// 2026-09-17 showed the loop starved — pressure sagged to ~5 bar while the
// reconstructed command sat at ~56-64 %, because the ±5 % integral clamp caps
// the answer to a steady error at 15*e + 5, and Tn = Kp/Ki = 50 s is longer
// than the shot itself. The integral now spans the output range (Tn = 5 s)
// and pump_task runs conditional integration so the saturated ramp-up to
// pressure does not wind it up.
//
// Tn = 5 s was still too slow: as the puck erodes during a shot the pump
// needs steadily more power (~3.7 %/s on 2026-09-25), and a PI loop tracks
// such a ramp with a steady error of rate / Ki -- 1.2 bar at Ki = 3, which is
// the plateau at 8.4 bar and sag to 7.6 bar that shot showed. The pressure
// responds to the pump within about a second, so Tn = 1.5 s is still
// conservative; a model fitted to that shot puts Ki = 10 at 9.0-9.1 bar
// through mid-shot with ~0.1 bar overshoot and no ringing. Late in a shot
// the pump saturates (97 % at 8.1 bar) and no gain can help.
constexpr float          kPressureSetpointBar = 9.0F;
constexpr float          kPidKp               = 15.0F;
constexpr float          kPidKi               = 10.0F;  // Kp / Tn, Tn = 1.5 s
constexpr float          kPidKd               = 0.8F;
constexpr float          kPidOutputMin        = 0.0F;
constexpr float          kPidOutputMax        = 100.0F;
constexpr float          kPidIntegralMin      = -20.0F;
constexpr float          kPidIntegralMax      = 100.0F;
// Accepted range for the web-UI / NVS pump gains.
constexpr float          kPidGainMin          = 0.0F;
constexpr float          kPidKpMax            = 100.0F;
constexpr float          kPidKiMax            = 30.0F;  // was 10: leave room above the default
constexpr float          kPidKdMax            = 50.0F;
constexpr float          kPidSamplePeriodS    = 0.1F;  // 10 Hz
constexpr std::uint32_t  kTaskStackBytes      = 4096;
constexpr std::uint32_t  kTaskPriority        = 5;
constexpr std::int32_t   kTaskCore            = 1;

// ---------------------------------------------------------------------------
// Bench / bring-up test mode (pump only).
// ---------------------------------------------------------------------------
// When kTestMode is true, app_main runs pump_test_task instead of the normal
// fill+PID pump_task, and does NOT start the heater control loop or the brew
// button task. The test first measures the zero-cross frequency with the pump
// OFF, then sweeps the duty cycle high→low so gate/phase timing can be checked
// on a scope or by ear.
//
// SAFETY: the heater stays OFF the whole time, but the pump runs DRY during
// the sweep. Keep steps short and stay at the machine. Leave kTestMode = false
// for production firmware.
constexpr bool           kTestMode            = false;
constexpr std::uint32_t  kTestZcMeasureMs     = 3'000;  // ZC measure (pump off)
constexpr std::uint32_t  kTestStepMs          = 2'500;  // dwell per duty step
constexpr std::uint32_t  kTestGapMs           = 1'000;  // pump OFF between steps
// Duty-cycle sweep, high→low. Each step starts from OFF (see kTestGapMs) so the
// pump-start is clearly audible. Floored well above the inaudible/no-flow zone.
constexpr float          kTestSweepPct[]      = {100.0F, 75.0F, 50.0F};
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
constexpr std::size_t   kProvisionFormMaxLen = 512;
constexpr std::size_t   kWpaMinPasswordLen = 8;
constexpr const char*   kProvisionNvsNamespace = "wifi_cfg";
constexpr const char*   kProvisionNvsStaSsidKey = "sta_ssid";
constexpr const char*   kProvisionNvsStaPasswordKey = "sta_pass";
constexpr std::uint8_t  kMaxReconnects  = 5;
constexpr std::uint32_t kReconnectDelayMs = 10'000;
constexpr bool          kOtaEnabled     = false;
}  // namespace net

// ---------------------------------------------------------------------------
// Wall-clock time (SNTP) for the daily wake schedule
// ---------------------------------------------------------------------------
// The ESP keeps time across deep sleep on its RTC, but a power cut resets it
// to 1970: until SNTP syncs, nothing time-of-day based runs.
namespace timekeeping {
// POSIX TZ for Europe/Paris: CET, CEST from the last Sunday of March 02:00
// to the last Sunday of October 03:00.
constexpr const char*   kTimezone       = "CET-1CEST,M3.5.0,M10.5.0/3";
constexpr const char*   kNtpServer      = "pool.ntp.org";
}  // namespace timekeeping

// ---------------------------------------------------------------------------
// Standby: deep sleep after inactivity, daily scheduled wake
// ---------------------------------------------------------------------------
// The machine stays mains-powered; standby is ESP32-S3 deep sleep with every
// output latched off and both switches armed as wake sources. Holding
// 94.5 C costs ~45 W, so the inactivity timeout is what saves energy; the
// MCU sleep itself trims the remaining fraction of a watt.
namespace power {
constexpr bool          kStandbyEnabledDefault  = true;
constexpr std::uint32_t kStandbyTimeoutMinDefault = 90;
constexpr std::uint32_t kStandbyTimeoutMinMin   = 5;
constexpr std::uint32_t kStandbyTimeoutMinMax   = 720;
constexpr bool          kWakeEnabledDefault     = true;
constexpr int           kWakeMinuteDefault      = 9 * 60;   // 09:00 local
constexpr std::uint32_t kTaskPeriodMs           = 1000;
constexpr std::uint32_t kTaskStackBytes         = 4096;
constexpr std::uint32_t kTaskPriority           = 2;
constexpr std::int32_t  kTaskCore               = 0;
// Before sleeping, tasks get this long to park: heater off, last MQTT
// record out, OLED showing the goodbye screen and then asleep.
constexpr std::uint32_t kParkTimeoutMs          = 3000;
constexpr std::uint32_t kGoodbyeScreenMs        = 1500;
// The heater is slow PWM with a 1 s period: one full period at 0 % before
// the SSR pin is latched.
constexpr std::uint32_t kHeaterSettleMs         = 1100;
// Remaining-time console log while the countdown runs.
constexpr std::uint32_t kLogPeriodMs            = 60'000;
}  // namespace power

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
constexpr std::size_t   kPayloadMaxLen  = 512;  // full control record; worst case 458
// Numeric mirror of each record's "type" string. The Hono->Kafka->Telegraf
// pipeline's JSON parser keeps numeric fields and drops strings that are not
// declared as tags, so the strings stay for human/MQTT consumers while these
// codes are what actually lands in TimescaleDB. `phase` is mirrored the same
// way by publishing pid::HeaterPhase's value as "phase_id".
enum class RecordType : std::uint8_t {
    kTemperature = 1,
    kPressure    = 2,
    kBrewStart   = 3,
    kBrewStop    = 4,
    kSteamOn     = 5,
    kSteamOff    = 6,
    kShotSummary = 7,
    kRefillStart = 8,
    kRefillStop  = 9,
    kStatus      = 10,
    kSleep       = 11,  // last record before deep sleep
};
// Telemetry cadence (see pid::TelemetryPolicy). Transients are what PID
// tuning needs, so the rate follows the heater phase rather than a fixed
// period: fast while warming up, brewing, steaming or off setpoint, and a
// slow heartbeat once the boiler is parked at setpoint.
constexpr std::uint32_t kWarmupPublishWindowMs   = 600'000;  // 10 min after boot
constexpr std::uint32_t kWarmupPublishPeriodMs   = 1'000;
constexpr std::uint32_t kBrewPublishPeriodMs     = 500;   // brew + trailing
constexpr std::uint32_t kActivePublishPeriodMs   = 1'000;  // steam / off-setpoint
constexpr std::uint32_t kIdlePublishPeriodMs     = 10'000;  // settled heartbeat
constexpr float         kActivePublishBandC      = 0.5F;
constexpr float         kActivePublishSlopeCps   = 0.05F;
constexpr std::uint32_t kPressurePublishPeriodMs = 500;
constexpr std::uint32_t kRuntimeSettingsCheckPeriodMs = 1000;
constexpr std::uint32_t kSchedulerPeriodMs = 100;
constexpr std::uint32_t kReconnectDelayMs = 5000;
constexpr std::uint32_t kNetworkTimeoutMs = 10000;
constexpr std::uint32_t kTaskStackBytes = 6144;
constexpr std::uint32_t kTaskPriority   = 2;
constexpr std::int32_t  kTaskCore       = 1;
}  // namespace mqtt

// ---------------------------------------------------------------------------
// HawkBit OTA (Direct Device Integration)
// ---------------------------------------------------------------------------
namespace hawkbit {
constexpr bool          kEnabled         = true;
constexpr std::size_t   kServerUrlMaxLen = 96;
constexpr std::size_t   kTokenMaxLen     = 64;
constexpr std::uint32_t kPollIntervalMs  = 300'000;  // 5 min
constexpr std::uint32_t kTaskStackBytes  = 8192;     // JSON + OTA stack
constexpr std::uint32_t kTaskPriority    = 1;        // lowest — background
constexpr std::int32_t  kTaskCore        = 0;        // CPU0, frees CPU1 for RT tasks
constexpr const char*   kDefaultTenant   = "DEFAULT";
}  // namespace hawkbit

}  // namespace espid::config
