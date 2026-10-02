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
#include "power/wake_pins.hpp"

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
};

RTC_DATA_ATTR SleepContext g_ctx{};

WakeReason g_reason = WakeReason::kPowerOn;

WakeReason switch_reason(int gpio) noexcept {
    if (gpio == static_cast<int>(cfg::brew::kButtonGpio)) {
        return WakeReason::kBrewSwitch;
    }
    if (gpio == static_cast<int>(cfg::steam::kSwitchGpio)) {
        return WakeReason::kSteamSwitch;
    }
    return WakeReason::kOther;
}

WakeReason classify(std::uint32_t causes) noexcept {
    if ((causes & BIT(ESP_SLEEP_WAKEUP_UNDEFINED)) != 0U || g_ctx.magic != kMagic) {
        return WakeReason::kPowerOn;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) != 0U) {
        const std::uint64_t mask = esp_sleep_get_ext1_wakeup_status();
        if ((mask & (1ULL << static_cast<int>(cfg::brew::kButtonGpio))) != 0U) {
            return WakeReason::kBrewSwitch;
        }
        if ((mask & (1ULL << static_cast<int>(cfg::steam::kSwitchGpio))) != 0U) {
            return WakeReason::kSteamSwitch;
        }
        return WakeReason::kOther;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_EXT0)) != 0U) {
        return switch_reason(g_ctx.ext0_gpio);
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) != 0U) {
        return WakeReason::kSchedule;
    }
    return WakeReason::kOther;
}

bool switch_on(gpio_num_t gpio) noexcept {
    return gpio_get_level(gpio) != 0;
}

}  // namespace

const char* to_string(WakeReason r) noexcept {
    switch (r) {
        case WakeReason::kPowerOn:     return "power_on";
        case WakeReason::kSchedule:    return "schedule";
        case WakeReason::kBrewSwitch:  return "brew_switch";
        case WakeReason::kSteamSwitch: return "steam_switch";
        case WakeReason::kOther:       return "other";
    }
    return "?";
}

WakeReason begin_boot() noexcept {
    for (const auto& o : kHeldOutputs) {
        (void)gpio_set_direction(o.gpio, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(o.gpio, static_cast<std::uint32_t>(o.inactive_level));
        (void)gpio_hold_dis(o.gpio);
    }
    gpio_deep_sleep_hold_dis();

    g_reason    = classify(esp_sleep_get_wakeup_causes());
    g_ctx.magic = 0;  // consumed: a later reset must not look like a wake
    ESP_LOGI(kTag, "boot: %s", to_string(g_reason));
    return g_reason;
}

WakeReason wake_reason() noexcept {
    return g_reason;
}

bool enter_deep_sleep(std::time_t wake_at) noexcept {
    const gpio_num_t brew  = cfg::brew::kButtonGpio;
    const gpio_num_t steam = cfg::steam::kSwitchGpio;
    const espid::power::WakeArming arm = espid::power::select_wake_pins(
        {static_cast<int>(brew), static_cast<int>(steam)}, switch_on(brew),
        switch_on(steam));
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

    std::uint64_t sleep_us = 0;
    const std::time_t now = std::time(nullptr);
    if (wake_at >= 0 && espid::power::clock_is_set(now)) {
        sleep_us = wake_at > now ? static_cast<std::uint64_t>(wake_at - now) * kUsPerS
                                 : kMinTimerSleepUs;
        (void)esp_sleep_enable_timer_wakeup(sleep_us);
    }

    g_ctx.magic     = kMagic;
    g_ctx.wake_at   = static_cast<std::int64_t>(wake_at);
    g_ctx.ext0_gpio = static_cast<std::int8_t>(arm.ext0_gpio);

    for (const auto& o : kHeldOutputs) {
        (void)gpio_set_direction(o.gpio, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(o.gpio, static_cast<std::uint32_t>(o.inactive_level));
        (void)gpio_hold_en(o.gpio);
    }
    // GPIO47 (heater SSR) is a digital pad: its latch only holds through
    // deep sleep with the global digital hold enabled.
    gpio_deep_sleep_hold_en();

    (void)esp_wifi_stop();
    ESP_LOGI(kTag, "deep sleep: ext1=0x%" PRIx64 " ext0=%d timer=%" PRIu64 " s",
             arm.ext1_high_mask, arm.ext0_gpio, sleep_us / kUsPerS);
    std::fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(kLogDrainMs));
    esp_deep_sleep_start();
}

}  // namespace espid::app
