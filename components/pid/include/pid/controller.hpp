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
//   - Integral anti-windup via fixed clamps (`integral_min/max`). The
//     output is independently clamped to `output_min/max`.
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
        has_last_         = false;
    }

    // Step the controller. `setpoint` and `measurement` must share
    // units. Returns the control output in [output_min, output_max].
    float update(float setpoint, float measurement) noexcept {
        const float error = setpoint - measurement;

        // Integral with clamp anti-windup.
        integral_ += cfg_.ki * error * cfg_.sample_period_s;
        if (integral_ > cfg_.integral_max) integral_ = cfg_.integral_max;
        if (integral_ < cfg_.integral_min) integral_ = cfg_.integral_min;

        // Derivative on measurement: rejects setpoint-step kicks.
        float d_term = 0.0F;
        if (has_last_ && cfg_.sample_period_s > 0.0F) {
            const float d_meas =
                (measurement - last_measurement_) / cfg_.sample_period_s;
            d_term = -cfg_.kd * d_meas;
        }
        last_measurement_ = measurement;
        has_last_         = true;

        float out = cfg_.kp * error + integral_ + d_term;
        if (out > cfg_.output_max) out = cfg_.output_max;
        if (out < cfg_.output_min) out = cfg_.output_min;
        return out;
    }

    // Test/diagnostic accessors. Const-only; do not mutate state from
    // outside `update()` or `reset()`.
    float integral_state() const noexcept    { return integral_; }
    float last_measurement() const noexcept  { return last_measurement_; }

   private:
    Config cfg_{};
    float  integral_         = 0.0F;
    float  last_measurement_ = 0.0F;
    bool   has_last_         = false;
};

}  // namespace espid::pid
