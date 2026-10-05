// Deep sleep -- see deep_sleep.hpp.

#include "deep_sleep.hpp"

#include <cinttypes>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.hpp"
#include "power/daily_wake.hpp"
#include "power/sleep_planner.hpp"
#include "power/wake_pins.hpp"
#include "power/wake_reason.hpp"

namespace espid::app {
namespace {

namespace cfg = espid::config;

constexpr const char* kTag = "sleep";

// Both switches must read high when on: wake_pins arms the ones that are
// off on "any high". A polarity change here needs a different arming.
static_assert(!cfg::brew::kButtonActiveLow, "brew switch must be active-high");
static_assert(cfg::steam::kSwitchActiveHigh, "steam switch must be active-high");

struct HeldOutput {
    gpio_num_t gpio;
    int        inactive_level;
};

// Every output that drives mains hardware or powers a sensor, at the level
// that keeps it off. Latched through deep sleep, re-asserted at boot before
// the latch is dropped so the pad never glitches.
constexpr HeldOutput kHeldOutputs[] = {
    {cfg::heater::kSsrGpio, cfg::heater::kSsrActiveHigh ? 0 : 1},
    {cfg::valve::kRelayGpio, cfg::valve::kActiveHigh ? 0 : 1},
    {cfg::pump::kPsmGpio, cfg::pump::kPsmActiveHigh ? 0 : 1},
    {cfg::tsic::kPowerGpio, 0},
};

constexpr std::uint32_t kMagic = 0x51EE9C0F;
constexpr std::uint64_t kUsPerS = 1'000'000ULL;
// A wake time already reached (or missed while arming) still goes through a
// short timer sleep, so the wake is reported as a scheduled one.
constexpr std::uint64_t kMinTimerSleepUs = kUsPerS;
// Lets the last log lines leave the USB/UART before the CPU stops.
constexpr std::uint32_t kLogDrainMs = 50;

// Survives deep sleep; reset on power-on.
struct SleepContext {
    std::uint32_t magic;
    std::int64_t  wake_at;
    std::int8_t   ext0_gpio;
    bool          resync;    // the timer is set short of wake_at
    bool          brew_on;   // switch levels when the chip went to sleep
    bool          steam_on;
    std::uint16_t spurious;  // glitch wakes since the last real one
};

RTC_DATA_ATTR SleepContext g_ctx{};

WakeReason    g_reason        = WakeReason::kPowerOn;
std::uint32_t g_wake_causes   = 0;  // raw esp_sleep_get_wakeup_causes() bitmap
std::uint64_t g_wake_ext1     = 0;  // GPIOs that triggered an ext1 wake
std::time_t   g_sleep_target  = -1;
std::uint16_t g_spurious_seen = 0;  // reported by this (real) boot

bool switch_on(gpio_num_t gpio) noexcept {
    return gpio_get_level(gpio) != 0;
}

// Same input setup as app_main's switch tasks; needed before the first
// read on a resync wake, where those tasks never start.
void configure_switch_inputs() noexcept {
    gpio_config_t c{};
    c.pin_bit_mask = (1ULL << static_cast<int>(cfg::brew::kButtonGpio)) |
                     (1ULL << static_cast<int>(cfg::steam::kSwitchGpio));
    c.mode         = GPIO_MODE_INPUT;
    c.pull_up_en   = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_ENABLE;
    c.intr_type    = GPIO_INTR_DISABLE;
    (void)gpio_config(&c);
}

WakeReason classify(std::uint32_t causes) noexcept {
    espid::power::WakeInputs in{};
    in.from_deep_sleep = (causes & BIT(ESP_SLEEP_WAKEUP_UNDEFINED)) == 0U;
    in.context_valid   = g_ctx.magic == kMagic;
    in.by_switch_line  = (causes & (BIT(ESP_SLEEP_WAKEUP_EXT0) |
                                    BIT(ESP_SLEEP_WAKEUP_EXT1))) != 0U;
    in.by_timer        = (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) != 0U;
    in.resync_leg      = g_ctx.resync;
    if (in.from_deep_sleep && in.context_valid) {
        configure_switch_inputs();
        in.brew_changed  = switch_on(cfg::brew::kButtonGpio) != g_ctx.brew_on;
        in.steam_changed = switch_on(cfg::steam::kSwitchGpio) != g_ctx.steam_on;
    }
    return espid::power::classify_wake(in);
}

}  // namespace

WakeReason begin_boot() noexcept {
    for (const auto& o : kHeldOutputs) {
        (void)gpio_set_direction(o.gpio, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(o.gpio, static_cast<std::uint32_t>(o.inactive_level));
        (void)gpio_hold_dis(o.gpio);
    }
    gpio_deep_sleep_hold_dis();

    g_wake_causes = esp_sleep_get_wakeup_causes();
    g_wake_ext1   = esp_sleep_get_ext1_wakeup_status();
    g_reason      = classify(g_wake_causes);
    g_ctx.magic   = 0;  // consumed: a later reset must not look like a wake
    if (g_reason == WakeReason::kClockResync || g_reason == WakeReason::kSpurious) {
        g_sleep_target = static_cast<std::time_t>(g_ctx.wake_at);
        if (g_reason == WakeReason::kSpurious && g_ctx.spurious < UINT16_MAX) {
            ++g_ctx.spurious;
        }
    } else {
        // A real boot reports the glitch wakes since the last one and
        // starts counting afresh (RTC memory is zeroed on power-on).
        g_spurious_seen = g_reason == WakeReason::kPowerOn ? 0 : g_ctx.spurious;
        g_ctx.spurious  = 0;
    }
    ESP_LOGI(kTag, "boot: %s (causes=0x%" PRIx32 " ext1=0x%" PRIx64 " spurious=%u)",
             to_string(g_reason), g_wake_causes, g_wake_ext1,
             static_cast<unsigned>(g_ctx.spurious));
    return g_reason;
}

WakeReason wake_reason() noexcept {
    return g_reason;
}

std::uint32_t wake_causes() noexcept {
    return g_wake_causes;
}

std::uint64_t wake_ext1_mask() noexcept {
    return g_wake_ext1;
}

bool is_clock_resync() noexcept {
    return g_reason == WakeReason::kClockResync;
}

bool is_spurious_wake() noexcept {
    return g_reason == WakeReason::kSpurious;
}

std::time_t sleep_target() noexcept {
    return g_sleep_target;
}

std::uint16_t spurious_wakes() noexcept {
    return g_spurious_seen;
}

bool switches_changed_since_sleep() noexcept {
    configure_switch_inputs();
    return switch_on(cfg::brew::kButtonGpio) != g_ctx.brew_on ||
           switch_on(cfg::steam::kSwitchGpio) != g_ctx.steam_on;
}

bool enter_deep_sleep(std::time_t wake_at) noexcept {
    const gpio_num_t brew  = cfg::brew::kButtonGpio;
    const gpio_num_t steam = cfg::steam::kSwitchGpio;
    configure_switch_inputs();
    const bool brew_on  = switch_on(brew);
    const bool steam_on = switch_on(steam);
    const espid::power::WakeArming arm = espid::power::select_wake_pins(
        {static_cast<int>(brew), static_cast<int>(steam)}, brew_on, steam_on);
    if (!arm.ok) {
        ESP_LOGW(kTag, "both switches on: cannot arm a wake source, staying awake");
        return false;
    }

    (void)esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (arm.ext1_high_mask != 0U) {
        (void)esp_sleep_enable_ext1_wakeup_io(arm.ext1_high_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    if (arm.ext0_gpio >= 0) {
        (void)esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(arm.ext0_gpio),
                                           arm.ext0_level);
    }
    // The switches rely on internal pull-downs, which only exist in sleep
    // while the RTC peripheral domain stays powered (a few uA).
    (void)esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    const gpio_num_t switches[] = {brew, steam};
    for (const gpio_num_t sw : switches) {
        (void)rtc_gpio_pullup_dis(sw);
        (void)rtc_gpio_pulldown_en(sw);
    }

    espid::power::SleepPlan plan{};
    const std::time_t now = std::time(nullptr);
    if (wake_at >= 0 && espid::power::clock_is_set(now)) {
        espid::power::SleepPlanConfig pc{};
        pc.drift_fraction   = cfg::power::kRtcDriftFraction;
        pc.min_margin_s     = cfg::power::kResyncMinMarginS;
        pc.precise_window_s = cfg::power::kPreciseWindowS;
        plan = espid::power::plan_sleep(now, wake_at, pc);
        if (plan.sleep_us == 0) {
            plan.sleep_us = kMinTimerSleepUs;
        }
        (void)esp_sleep_enable_timer_wakeup(plan.sleep_us);
    }

    g_ctx.magic     = kMagic;
    g_ctx.wake_at   = static_cast<std::int64_t>(wake_at);
    g_ctx.ext0_gpio = static_cast<std::int8_t>(arm.ext0_gpio);
    g_ctx.resync    = plan.resync_first;
    g_ctx.brew_on   = brew_on;
    g_ctx.steam_on  = steam_on;

    for (const auto& o : kHeldOutputs) {
        (void)gpio_set_direction(o.gpio, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(o.gpio, static_cast<std::uint32_t>(o.inactive_level));
        (void)gpio_hold_en(o.gpio);
    }
    // GPIO47 (heater SSR) is a digital pad: its latch only holds through
    // deep sleep with the global digital hold enabled.
    gpio_deep_sleep_hold_en();

    (void)esp_wifi_stop();
    ESP_LOGI(kTag, "deep sleep: ext1=0x%" PRIx64 " ext0=%d timer=%" PRIu64 " s%s",
             arm.ext1_high_mask, arm.ext0_gpio, plan.sleep_us / kUsPerS,
             plan.resync_first ? " (then clock resync)" : "");
    std::fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(kLogDrainMs));
    esp_deep_sleep_start();
}

}  // namespace espid::app
