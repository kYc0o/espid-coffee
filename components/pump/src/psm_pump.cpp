// PsmPump -- ESP-IDF hardware glue for the phase-control pump dimmer.

#include "pump/psm_pump.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

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

    if (!core_.configure(cfg_.mains_frequency_hz)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    (void)core_.set_power_pct(0.0F);
    firing_enabled_.store(false, std::memory_order_release);
    firing_delay_us_.store(core_.half_period_us(), std::memory_order_release);

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

    // One-shot high-res timer, 1 tick = 1 us.
    gptimer_config_t tcfg{};
    tcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    tcfg.direction = GPTIMER_COUNT_UP;
    tcfg.resolution_hz = 1'000'000;
    if (gptimer_new_timer(&tcfg, &gate_timer_) != ESP_OK) {
        return hw_fault();
    }

    gptimer_event_callbacks_t cbs{};
    cbs.on_alarm = &PsmPump::gate_alarm_cb_;
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
    if (!core_.set_power_pct(power_pct)) {
        return hal_cpp::Status(hal_cpp::Error::kInvalidArg);
    }
    // Publish the ISR-visible snapshot (release; ISR reads with acquire).
    firing_delay_us_.store(core_.firing_delay_us(), std::memory_order_release);
    firing_enabled_.store(core_.firing_enabled(), std::memory_order_release);
    return hal_cpp::Status{};
}

void IRAM_ATTR PsmPump::zero_cross_isr_(void* arg) {
    auto* self = static_cast<PsmPump*>(arg);
    self->zc_count_.fetch_add(1, std::memory_order_relaxed);
    if (!self->firing_enabled_.load(std::memory_order_acquire)) {
        return;
    }

    const std::uint32_t delay =
        self->firing_delay_us_.load(std::memory_order_acquire);
    if (delay == 0) {
        self->fire_gate_pulse_();
        return;
    }

    gptimer_stop(self->gate_timer_);
    gptimer_set_raw_count(self->gate_timer_, 0);
    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = delay;
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = false;
    gptimer_set_alarm_action(self->gate_timer_, &alarm);
    gptimer_start(self->gate_timer_);
}

bool IRAM_ATTR PsmPump::gate_alarm_cb_(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t* /*edata*/,
    void* arg) {
    auto* self = static_cast<PsmPump*>(arg);
    gptimer_stop(timer);
    self->fire_gate_pulse_();
    return false;
}

void IRAM_ATTR PsmPump::fire_gate_pulse_() noexcept {
    const auto pin = static_cast<gpio_num_t>(cfg_.gate_gpio);
    gpio_set_level(pin, static_cast<std::uint32_t>(gate_active_level_));
    esp_rom_delay_us(cfg_.gate_pulse_us);
    gpio_set_level(pin, gate_active_level_ != 0 ? 0 : 1);
}

}  // namespace espid::pump
