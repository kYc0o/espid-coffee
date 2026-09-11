// PsmPump -- phase-control pump dimmer driver (RobotDyn-style).
//
// Wires PsmCore to hardware: a zero-cross opto input GPIO raises an
// interrupt on every mains half-cycle (both edges); a one-shot gptimer
// is armed for the computed firing delay, and its ISR emits a short
// gate pulse to the triac. Power percentage is set from a task; the
// ISR path only reads pre-computed atomics (snapshot pattern), so no
// lock is shared with interrupt context.
//
// Threading: init()/set_power_pct() run in task context. The zero-cross
// GPIO ISR and the gptimer alarm ISR run in interrupt context and touch
// only atomics + ISR-safe GPIO/timer calls.

#pragma once

#include <atomic>
#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "hal_cpp/result.hpp"
#include "pump/psm_core.hpp"

namespace espid::pump {

class PsmPump {
   public:
    struct Config {
        int           zero_cross_gpio      = -1;
        int           gate_gpio            = -1;
        bool          gate_active_high     = true;
        std::uint32_t mains_frequency_hz   = 50;
        std::uint32_t gate_pulse_us        = 50;
    };

    PsmPump() = default;
    ~PsmPump();

    PsmPump(const PsmPump&)            = delete;
    PsmPump& operator=(const PsmPump&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept;

    // Set requested pump power (0..100 %). 0 disables gate firing.
    hal_cpp::Status set_power_pct(float power_pct) noexcept;

    float power_pct() const noexcept { return core_.power_pct(); }

    // Diagnostic: total zero-cross edges received since init.
    std::uint32_t zc_count() const noexcept {
        return zc_count_.load(std::memory_order_relaxed);
    }

   private:
    static void zero_cross_isr_(void* arg);
    static bool gate_alarm_cb_(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t* edata,
                               void* arg);
    void fire_gate_pulse_() noexcept;

    Config                     cfg_{};
    PsmCore                    core_{};
    gptimer_handle_t           gate_timer_{nullptr};
    std::atomic<std::uint32_t> zc_count_{0};
    // Cycle count of the last accepted ZC edge; used to debounce opto noise.
    std::atomic<std::uint32_t> last_valid_zc_cycles_{0};
    std::atomic<std::uint32_t> firing_delay_us_{0};
    std::atomic<bool>          firing_enabled_{false};
    int                        gate_active_level_{1};
    bool                       initialised_{false};
};

}  // namespace espid::pump
