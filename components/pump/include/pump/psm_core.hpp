// PsmCore -- pulse-skip (integral-cycle) modulation logic for the pump.
//
// Power is expressed 0..100 % and interpreted as the fraction of mains cycles
// the triac is allowed to fire. Selected cycles fire AT the zero-cross so a
// full half-cycle passes (no phase chopping); unselected cycles are skipped.
// This is the classic "PSM"/burst-firing scheme, which is the right fit for a
// vibratory pump: the triac always latches at the zero-cross and there is far
// less EMI than phase-angle control.
//
// Modulation is done on WHOLE mains cycles (both half-cycles fired together)
// so the inductive pump coil stays DC-balanced. A Bresenham accumulator spreads
// the fired cycles evenly across the skip pattern (e.g. 70 % -> 7 of every 10
// cycles, interleaved rather than 7 on then 3 off).
//
// on_half_cycle() is called once per mains zero-cross (every half-cycle) and
// returns whether the gate should fire on that half-cycle. All state lives in
// this object; the driver keeps a single instance driven solely from the ZC
// ISR (single writer). This file has no ESP-IDF dependency and is host-testable.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::pump {

class PsmCore {
   public:
    explicit PsmCore(std::uint32_t mains_frequency_hz = 50) noexcept {
        configure(mains_frequency_hz);
    }

    bool configure(std::uint32_t mains_frequency_hz) noexcept {
        if (mains_frequency_hz == 0) {
            return false;
        }
        half_period_us_ = 1'000'000U / (2U * mains_frequency_hz);
        return true;
    }

    std::uint32_t half_period_us() const noexcept { return half_period_us_; }

    // Requested power 0..100 %. Rounded to whole-percent modulation steps.
    bool set_power_pct(float power_pct) noexcept {
        if (!std::isfinite(power_pct) || power_pct < 0.0F || power_pct > 100.0F) {
            return false;
        }
        power_pct_  = power_pct;
        power_step_ = static_cast<std::uint32_t>(power_pct + 0.5F);  // 0..100
        return true;
    }

    // Fast path for the ISR: set the pre-validated integer step (0..100).
    void set_power_step(std::uint32_t step) noexcept {
        power_step_ = (step > kFullScale) ? kFullScale : step;
    }

    float power_pct() const noexcept { return power_pct_; }

    std::uint32_t power_step() const noexcept { return power_step_; }

    // True when any cycle is allowed to fire.
    bool firing_enabled() const noexcept { return power_step_ > 0U; }

    // Advance one half-cycle (call once per zero-cross). Returns true if the
    // triac gate should be fired now. The fire/skip decision is taken once per
    // whole cycle and held across both of its half-cycles, so fired cycles are
    // polarity-balanced.
    bool on_half_cycle() noexcept {
        // parity_ toggles each half-cycle; a whole-cycle boundary is when it
        // returns to 0. Decide fire/skip there, then hold across both halves.
        parity_ ^= 1U;
        if (parity_ == 0U) {
            accumulator_ += power_step_;
            if (accumulator_ >= kFullScale) {
                accumulator_ -= kFullScale;
                fire_cycle_ = true;
            } else {
                fire_cycle_ = false;
            }
        }
        return fire_cycle_ && (power_step_ > 0U);
    }

    // Reset modulation state; call when (re)enabling the pump.
    void reset() noexcept {
        accumulator_ = 0U;
        parity_      = 1U;  // first edge -> whole-cycle boundary
        fire_cycle_  = false;
    }

   private:
    static constexpr std::uint32_t kFullScale = 100U;

    std::uint32_t half_period_us_{10'000};  // 50 Hz default
    float         power_pct_{0.0F};
    std::uint32_t power_step_{0U};          // rounded 0..100
    std::uint32_t accumulator_{0U};
    std::uint32_t parity_{1U};
    bool          fire_cycle_{false};
};

}  // namespace espid::pump
