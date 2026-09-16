// PsmPump -- pulse-skip pump dimmer driver (RobotDyn-style triac + opto ZC).
//
// Wires PsmCore to hardware: a zero-cross opto input GPIO raises an interrupt
// on every mains half-cycle (both edges). On each accepted (debounced) edge the
// modulator decides fire-or-skip; selected half-cycles emit a short gate pulse
// AT the zero-cross so a whole half-cycle passes (integral-cycle / PSM control,
// not phase-angle chopping). The requested power is published from a task as a
// pre-validated integer step read by the ISR (snapshot pattern), so no lock is
// shared with interrupt context.
//
// Threading: init()/set_power_pct() run in task context. The zero-cross GPIO
// ISR runs in interrupt context and touches only atomics, the ISR-owned
// modulator, and ISR-safe GPIO calls.

#pragma once

#include <atomic>
#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "hal_cpp/result.hpp"
#include "pump/psm_core.hpp"

struct gpio_glitch_filter_t;  // opaque; from driver/gpio_filter.h

namespace espid::pump {

class PsmPump {
   public:
    struct Config {
        int           zero_cross_gpio       = -1;
        int           gate_gpio             = -1;
        bool          gate_active_high      = true;
        std::uint32_t mains_frequency_hz    = 50;
        // Gate ON-time (hold) per fired half-cycle. Held via a one-shot timer
        // (non-blocking) so the triac reliably latches an inductive pump as
        // the mains voltage rises past the zero-cross. ~2 ms is a safe hold.
        std::uint32_t gate_pulse_us         = 2000;
        // Optional ZC-input glitch filter. >0 enables the ESP32-S3 pin glitch
        // filter (fixed ~2 IO-MUX cycles); the value is advisory only there.
        std::uint32_t glitch_filter_ns      = 0;
    };

    // Zero-cross signal-quality snapshot for bring-up diagnostics.
    struct ZcStats {
        std::uint32_t accepted;          // edges that passed the debounce
        std::uint32_t rejected;          // edges dropped as too-soon (noise/bounce)
        std::uint32_t fired;             // half-cycles the gate was actually fired
        std::uint32_t last_interval_us;  // most recent accepted-edge spacing
        std::uint32_t min_interval_us;   // shortest accepted spacing seen
        std::uint32_t max_interval_us;   // longest accepted spacing seen
    };

    PsmPump() = default;
    ~PsmPump();

    PsmPump(const PsmPump&)            = delete;
    PsmPump& operator=(const PsmPump&) = delete;

    hal_cpp::Status init(const Config& cfg) noexcept;

    // Set requested pump power (0..100 %). 0 disables gate firing.
    hal_cpp::Status set_power_pct(float power_pct) noexcept;

    float power_pct() const noexcept {
        return power_pct_snap_.load(std::memory_order_relaxed);
    }

    // Diagnostic: total accepted zero-cross edges since init/last reset.
    std::uint32_t zc_count() const noexcept {
        return zc_count_.load(std::memory_order_relaxed);
    }

    // Snapshot of zero-cross timing/quality counters.
    ZcStats zc_stats() const noexcept;

    // Clear the ZC counters/interval stats to start a fresh measurement.
    void reset_zc_stats() noexcept;

   private:
    static void zero_cross_isr_(void* arg);
    static bool gate_off_cb_(gptimer_handle_t timer,
                             const gptimer_alarm_event_data_t* edata,
                             void* arg);
    void gate_on_() noexcept;
    void gate_set_(bool active) noexcept;

    Config                     cfg_{};
    PsmCore                    mod_{};  // modulation state; driven only by the ISR
    gpio_glitch_filter_t*      glitch_filter_{nullptr};
    gptimer_handle_t           gate_timer_{nullptr};
    std::atomic<std::uint32_t> zc_count_{0};
    // Half-cycles the gate was actually fired since init/last reset.
    std::atomic<std::uint32_t> fired_count_{0};
    // Zero-cross debounce/diagnostics. Written only by the ZC ISR (single
    // writer, pinned to one core), read by task context. Timestamps are the
    // low 32 bits of esp_timer_get_time() in us; unsigned subtraction is
    // wraparound-safe over the ~71 min rollover.
    std::atomic<std::uint32_t> zc_rejected_{0};
    std::atomic<std::uint32_t> last_accepted_us_{0};
    std::atomic<std::uint32_t> zc_last_interval_us_{0};
    std::atomic<std::uint32_t> zc_min_interval_us_{0xFFFFFFFFU};
    std::atomic<std::uint32_t> zc_max_interval_us_{0};
    std::atomic<bool>          zc_have_last_{false};
    // Reject edges closer than this to the previous accepted one; derived
    // from the mains half-period at init (3/4 of a half-cycle).
    std::uint32_t              min_interval_us_{0};
    // Requested power step (0..100), published task -> ISR.
    std::atomic<std::uint32_t> target_step_{0};
    std::atomic<float>         power_pct_snap_{0.0F};
    int                        gate_active_level_{1};
    bool                       initialised_{false};
};

}  // namespace espid::pump
