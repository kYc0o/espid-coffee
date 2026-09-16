// PsmPump -- ESP-IDF hardware glue for the pulse-skip pump dimmer.

#include "pump/psm_pump.hpp"

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"

namespace espid::pump {

namespace {

hal_cpp::Status hw_fault() noexcept {
    return hal_cpp::Status(hal_cpp::Error::kHardwareFault);
}

}  // namespace

PsmPump::~PsmPump() {
    if (initialised_ && cfg_.zero_cross_gpio >= 0) {
        gpio_isr_handler_remove(static_cast<gpio_num_t>(cfg_.zero_cross_gpio));
    }
    if (gate_timer_ != nullptr) {
        gptimer_stop(gate_timer_);
        gptimer_disable(gate_timer_);
        gptimer_del_timer(gate_timer_);
        gate_timer_ = nullptr;
    }
    if (glitch_filter_ != nullptr) {
        gpio_glitch_filter_disable(glitch_filter_);
        gpio_del_glitch_filter(glitch_filter_);
        glitch_filter_ = nullptr;
    }
}

hal_cpp::Status PsmPump::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kAlreadyInit);
    }
    if (cfg.zero_cross_gpio < 0 || cfg.gate_gpio < 0 || cfg.gate_pulse_us == 0) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }

    cfg_ = cfg;
    gate_active_level_ = cfg_.gate_active_high ? 1 : 0;

    if (!mod_.configure(cfg_.mains_frequency_hz)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    mod_.reset();
    // Debounce window: 3/4 of a mains half-cycle. Wide enough to swallow
    // opto bounce/noise after a real edge, but comfortably below the true
    // half-period so the next real zero-cross is never rejected.
    min_interval_us_ = mod_.half_period_us() * 3U / 4U;
    target_step_.store(0, std::memory_order_release);
    power_pct_snap_.store(0.0F, std::memory_order_relaxed);

    // Gate output, driven to the inactive level.
    gpio_config_t gate_cfg{};
    gate_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg_.gate_gpio);
    gate_cfg.mode = GPIO_MODE_OUTPUT;
    gate_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gate_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gate_cfg.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&gate_cfg) != ESP_OK) {
        return hw_fault();
    }
    gpio_set_level(static_cast<gpio_num_t>(cfg_.gate_gpio),
                   gate_active_level_ != 0 ? 0 : 1);

    // Zero-cross input: interrupt on both edges (every half-cycle).
    gpio_config_t zc_cfg{};
    zc_cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(cfg_.zero_cross_gpio);
    zc_cfg.mode = GPIO_MODE_INPUT;
    zc_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    zc_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    zc_cfg.intr_type = GPIO_INTR_ANYEDGE;
    if (gpio_config(&zc_cfg) != ESP_OK) {
        return hw_fault();
    }

    // Optional hardware glitch filter on the ZC pin. The ESP32-S3 only has the
    // fixed pin glitch filter (~2 IO-MUX cycles, tens of ns) -- a token backstop
    // for very fast spikes; the RC filter + software debounce do the real work.
    // Non-fatal if unavailable.
#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
    if (cfg_.glitch_filter_ns > 0) {
        gpio_pin_glitch_filter_config_t fcfg{};
        fcfg.clk_src  = GLITCH_FILTER_CLK_SRC_DEFAULT;
        fcfg.gpio_num = static_cast<gpio_num_t>(cfg_.zero_cross_gpio);
        if (gpio_new_pin_glitch_filter(&fcfg, &glitch_filter_) == ESP_OK) {
            (void)gpio_glitch_filter_enable(glitch_filter_);
        } else {
            glitch_filter_ = nullptr;
        }
    }
#endif

    // One-shot high-res timer (1 tick = 1 us) that turns the gate OFF after the
    // hold window. Firing arms it; the alarm callback drops the gate. Holding
    // the gate for ~2 ms lets the triac latch the inductive pump as the mains
    // voltage rises past the zero-cross.
    gptimer_config_t tcfg{};
    tcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    tcfg.direction = GPTIMER_COUNT_UP;
    tcfg.resolution_hz = 1'000'000;
    if (gptimer_new_timer(&tcfg, &gate_timer_) != ESP_OK) {
        return hw_fault();
    }
    gptimer_event_callbacks_t cbs{};
    cbs.on_alarm = &PsmPump::gate_off_cb_;
    if (gptimer_register_event_callbacks(gate_timer_, &cbs, this) != ESP_OK) {
        return hw_fault();
    }
    if (gptimer_enable(gate_timer_) != ESP_OK) {
        return hw_fault();
    }

    // ISR service may already be installed by another component.
    const esp_err_t is = gpio_install_isr_service(0);
    if (is != ESP_OK && is != ESP_ERR_INVALID_STATE) {
        return hw_fault();
    }
    if (gpio_isr_handler_add(static_cast<gpio_num_t>(cfg_.zero_cross_gpio),
                             &PsmPump::zero_cross_isr_, this) != ESP_OK) {
        return hw_fault();
    }

    initialised_ = true;
    return hal_cpp::Status{};
}

hal_cpp::Status PsmPump::set_power_pct(float power_pct) noexcept {
    if (!initialised_) {
        return hal_cpp::Status(hal_cpp::Error::kNotInitialised);
    }
    if (!std::isfinite(power_pct) || power_pct < 0.0F || power_pct > 100.0F) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    const auto step = static_cast<std::uint32_t>(power_pct + 0.5F);  // 0..100
    // Publish the ISR-visible snapshot (release; ISR reads with acquire).
    power_pct_snap_.store(power_pct, std::memory_order_relaxed);
    target_step_.store(step, std::memory_order_release);
    return hal_cpp::Status{};
}

PsmPump::ZcStats PsmPump::zc_stats() const noexcept {
    ZcStats s{};
    s.accepted         = zc_count_.load(std::memory_order_relaxed);
    s.rejected         = zc_rejected_.load(std::memory_order_relaxed);
    s.fired            = fired_count_.load(std::memory_order_relaxed);
    s.last_interval_us = zc_last_interval_us_.load(std::memory_order_relaxed);
    const std::uint32_t mn = zc_min_interval_us_.load(std::memory_order_relaxed);
    s.min_interval_us  = (mn == 0xFFFFFFFFU) ? 0U : mn;
    s.max_interval_us  = zc_max_interval_us_.load(std::memory_order_relaxed);
    return s;
}

void PsmPump::reset_zc_stats() noexcept {
    // Order matters only loosely: clear have_last first so any in-flight ISR
    // re-seeds cleanly instead of computing an interval across the reset.
    zc_have_last_.store(false, std::memory_order_relaxed);
    zc_count_.store(0, std::memory_order_relaxed);
    zc_rejected_.store(0, std::memory_order_relaxed);
    fired_count_.store(0, std::memory_order_relaxed);
    zc_last_interval_us_.store(0, std::memory_order_relaxed);
    zc_min_interval_us_.store(0xFFFFFFFFU, std::memory_order_relaxed);
    zc_max_interval_us_.store(0, std::memory_order_relaxed);
}

void IRAM_ATTR PsmPump::zero_cross_isr_(void* arg) {
    auto* self = static_cast<PsmPump*>(arg);

    // Debounce against opto noise/bounce. A clean ZC opto toggles once per
    // mains half-cycle (~10 ms at 50 Hz); noise can add spurious edges in the
    // gap between real crossings. Any edge closer than min_interval_us_ to the
    // last accepted one is treated as noise and dropped. Using absolute us
    // from esp_timer (IRAM-safe, monotonic) avoids the CPU-frequency guesswork
    // of a cycle-count debounce.
    const std::uint32_t now =
        static_cast<std::uint32_t>(esp_timer_get_time());
    const std::uint32_t last =
        self->last_accepted_us_.load(std::memory_order_relaxed);
    const std::uint32_t interval = now - last;  // wraparound-safe

    if (self->zc_have_last_.load(std::memory_order_relaxed) &&
        interval < self->min_interval_us_) {
        self->zc_rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Accepted edge: publish interval stats (single writer, so plain
    // load/compare/store on the min/max atomics is race-free here).
    if (self->zc_have_last_.load(std::memory_order_relaxed)) {
        self->zc_last_interval_us_.store(interval, std::memory_order_relaxed);
        if (interval < self->zc_min_interval_us_.load(std::memory_order_relaxed)) {
            self->zc_min_interval_us_.store(interval, std::memory_order_relaxed);
        }
        if (interval > self->zc_max_interval_us_.load(std::memory_order_relaxed)) {
            self->zc_max_interval_us_.store(interval, std::memory_order_relaxed);
        }
    }
    self->last_accepted_us_.store(now, std::memory_order_relaxed);
    self->zc_have_last_.store(true, std::memory_order_relaxed);
    self->zc_count_.fetch_add(1, std::memory_order_relaxed);

    // Pulse-skip modulation: advance the modulator one half-cycle and drive the
    // gate ON if this half-cycle is selected. Whole cycles are fired together
    // (DC-balanced); a one-shot timer drops the gate after the hold window.
    self->mod_.set_power_step(self->target_step_.load(std::memory_order_acquire));
    if (self->mod_.on_half_cycle()) {
        self->gate_on_();
    }
}

bool IRAM_ATTR PsmPump::gate_off_cb_(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t* /*edata*/,
                                     void* arg) {
    auto* self = static_cast<PsmPump*>(arg);
    gptimer_stop(timer);
    self->gate_set_(false);
    return false;
}

void IRAM_ATTR PsmPump::gate_set_(bool active) noexcept {
    const auto pin = static_cast<gpio_num_t>(cfg_.gate_gpio);
    const int level = active ? gate_active_level_ : (gate_active_level_ != 0 ? 0 : 1);
    gpio_set_level(pin, static_cast<std::uint32_t>(level));
}

void IRAM_ATTR PsmPump::gate_on_() noexcept {
    fired_count_.fetch_add(1, std::memory_order_relaxed);
    gate_set_(true);
    // Arm the one-shot timer to release the gate after the hold window.
    gptimer_stop(gate_timer_);
    gptimer_set_raw_count(gate_timer_, 0);
    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = cfg_.gate_pulse_us;
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = false;
    gptimer_set_alarm_action(gate_timer_, &alarm);
    gptimer_start(gate_timer_);
}

}  // namespace espid::pump
