// Discrete PID controller for ESPID-Coffee.
//
// Pure C++17, no ESP-IDF or FreeRTOS includes, so the same translation
// unit compiles on host for fast unit-testing.
//
// Design notes:
//   - Fixed sample period passed in `Config::sample_period_s`. Callers
//     are expected to invoke `update()` at that rate; the integrator
//     and derivative both rely on it.
//   - Derivative on **measurement** (not error). This eliminates the
//     derivative "kick" that a setpoint step would otherwise produce
//     and which can be brutal on a heater.
//   - Optional first-order low-pass on the measured slope
//     (`d_filter_tau_s`). Quantised sensors (TSIC: ~0.1 °C LSB) turn a
//     single-LSB flicker into a large dPV/dt at high loop rates; the
//     filter keeps the D term from amplifying that noise. 0 = unfiltered.
//   - Integral anti-windup via fixed clamps (`integral_min/max`). The
//     output is independently clamped to `output_min/max`.
//   - Optional conditional integration (`conditional_integration`): the
//     integrator holds while the output is saturated in the direction the
//     error is pushing. Without it a long saturated run (e.g. boiler cold
//     start at 100 %) parks the integral at its clamp and it has to unwind
//     through an overshoot. It also holds while the D term drives the
//     output into saturation against the error, e.g. a boiler coasting up
//     to setpoint on stored heat with D braking the heater to 0 %:
//     integrating that error would park the integral above the holding
//     value. Saturation caused by P or I still lets the integral unwind.
//   - The integrator stores the already-`ki`-scaled sum, so swapping
//     gains with `set_config()` mid-run is bumpless. `ki == 0` freezes
//     the integral at its current contribution.
//   - Stateless w.r.t. the caller; copy/move freely. Not thread-safe;
//     drive from a single task.

#pragma once

namespace espid::pid {

class Controller {
   public:
    struct Config {
        float kp              = 0.0F;
        float ki              = 0.0F;
        float kd              = 0.0F;
        float output_min      = 0.0F;
        float output_max      = 1.0F;
        float integral_min    = -1.0F;
        float integral_max    =  1.0F;
        float sample_period_s = 0.1F;  // 10 Hz default
        float d_filter_tau_s  = 0.0F;  // slope low-pass time constant; 0 = off
        bool  conditional_integration = false;
    };

    // Per-term breakdown of the most recent `update()`, for diagnostics.
    struct Terms {
        float p          = 0.0F;
        float i          = 0.0F;
        float d          = 0.0F;
        float slope      = 0.0F;  // filtered d(measurement)/dt, units/s
        float unclamped  = 0.0F;  // p + i + d before output clamp
        float output     = 0.0F;
    };

    Controller() = default;
    explicit Controller(const Config& cfg) noexcept : cfg_(cfg) {}

    void          set_config(const Config& cfg) noexcept { cfg_ = cfg; }
    const Config& config() const noexcept                { return cfg_; }

    // Reset accumulated state. The next `update()` call will behave as
    // if it were the very first one (no derivative term, zero integral).
    void reset() noexcept {
        integral_         = 0.0F;
        last_measurement_ = 0.0F;
        slope_            = 0.0F;
        has_last_         = false;
        terms_            = Terms{};
    }

    // Step the controller. `setpoint` and `measurement` must share
    // units. Returns the control output in [output_min, output_max].
    float update(float setpoint, float measurement) noexcept {
        const float error = setpoint - measurement;

        // Integral with clamp anti-windup.
        float integral = integral_ + cfg_.ki * error * cfg_.sample_period_s;
        if (integral > cfg_.integral_max) integral = cfg_.integral_max;
        if (integral < cfg_.integral_min) integral = cfg_.integral_min;

        // Derivative on measurement: rejects setpoint-step kicks.
        float d_term = 0.0F;
        if (has_last_ && cfg_.sample_period_s > 0.0F) {
            const float raw_slope =
                (measurement - last_measurement_) / cfg_.sample_period_s;
            if (cfg_.d_filter_tau_s > 0.0F) {
                const float alpha = cfg_.sample_period_s /
                                    (cfg_.d_filter_tau_s + cfg_.sample_period_s);
                slope_ += alpha * (raw_slope - slope_);
            } else {
                slope_ = raw_slope;
            }
            d_term = -cfg_.kd * slope_;
        }
        last_measurement_ = measurement;
        has_last_         = true;

        const float p_term = cfg_.kp * error;
        float out = p_term + integral + d_term;
        const bool pushes_into_sat = (out > cfg_.output_max && error > 0.0F) ||
                                     (out < cfg_.output_min && error < 0.0F);
        const bool d_brakes_into_sat =
            (out < cfg_.output_min && error > 0.0F && d_term < 0.0F) ||
            (out > cfg_.output_max && error < 0.0F && d_term > 0.0F);
        if (cfg_.conditional_integration && (pushes_into_sat || d_brakes_into_sat)) {
            integral = integral_;
            out      = p_term + integral + d_term;
        }
        integral_ = integral;
        terms_.p         = p_term;
        terms_.i         = integral_;
        terms_.d         = d_term;
        terms_.slope     = slope_;
        terms_.unclamped = out;
        if (out > cfg_.output_max) out = cfg_.output_max;
        if (out < cfg_.output_min) out = cfg_.output_min;
        terms_.output = out;
        return out;
    }

    // Test/diagnostic accessors. Const-only; do not mutate state from
    // outside `update()` or `reset()`.
    float        integral_state() const noexcept    { return integral_; }
    float        last_measurement() const noexcept  { return last_measurement_; }
    const Terms& last_terms() const noexcept        { return terms_; }

   private:
    Config cfg_{};
    float  integral_         = 0.0F;
    float  last_measurement_ = 0.0F;
    float  slope_            = 0.0F;
    bool   has_last_         = false;
    Terms  terms_{};
};

}  // namespace espid::pid
