// ESPID-Coffee — pid::Controller unit tests.
//
// Pure host tests. Cover the contract described in
// `components/pid/include/pid/controller.hpp`:
//   - P-only, fresh controller: output = clamp(kp * error)
//   - Output saturation
//   - Integrator accumulates at ki * error * dt and clamps
//   - Derivative on measurement: no kick on setpoint step
//   - Derivative reacts to a measurement change
//   - reset() clears integrator and derivative history
//   - First update has zero D contribution (no previous measurement)

#include <gtest/gtest.h>

#include "pid/controller.hpp"

namespace {

using espid::pid::Controller;

constexpr float kEps = 1e-5F;

Controller::Config p_only(float kp, float out_min = -10.0F, float out_max = 10.0F) {
    Controller::Config c{};
    c.kp              = kp;
    c.output_min      = out_min;
    c.output_max      = out_max;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 0.1F;
    return c;
}

TEST(PidController, ProportionalOnlyMatchesKpTimesError) {
    Controller pid(p_only(2.0F));
    EXPECT_NEAR(pid.update(/*setpoint=*/10.0F, /*meas=*/8.0F), 4.0F, kEps);
}

TEST(PidController, OutputClampsAtMax) {
    auto cfg       = p_only(100.0F);
    cfg.output_max = 1.0F;
    Controller pid(cfg);
    EXPECT_FLOAT_EQ(pid.update(50.0F, 0.0F), 1.0F);
}

TEST(PidController, OutputClampsAtMin) {
    auto cfg       = p_only(100.0F);
    cfg.output_min = -1.0F;
    Controller pid(cfg);
    EXPECT_FLOAT_EQ(pid.update(0.0F, 50.0F), -1.0F);
}

TEST(PidController, IntegralAccumulatesAtKiErrorDt) {
    Controller::Config c{};
    c.ki              = 1.0F;
    c.sample_period_s = 0.1F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    Controller pid(c);

    // error=2, dt=0.1, ki=1 → +0.2 per step.
    EXPECT_NEAR(pid.update(2.0F, 0.0F), 0.2F, kEps);
    EXPECT_NEAR(pid.update(2.0F, 0.0F), 0.4F, kEps);
    EXPECT_NEAR(pid.update(2.0F, 0.0F), 0.6F, kEps);
}

TEST(PidController, IntegralAntiWindupClamps) {
    Controller::Config c{};
    c.ki              = 1.0F;
    c.sample_period_s = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -0.5F;
    c.integral_max    =  0.5F;
    Controller pid(c);

    // Drive integrator far past max; it must clamp.
    for (int i = 0; i < 100; ++i) {
        (void)pid.update(1.0F, 0.0F);
    }
    EXPECT_FLOAT_EQ(pid.integral_state(), 0.5F);

    // Now drive it negative; must clamp the other way.
    for (int i = 0; i < 100; ++i) {
        (void)pid.update(0.0F, 1.0F);
    }
    EXPECT_FLOAT_EQ(pid.integral_state(), -0.5F);
}

TEST(PidController, SetpointStepProducesNoDerivativeKick) {
    Controller::Config c{};
    c.kp              = 0.0F;
    c.ki              = 0.0F;
    c.kd              = 10.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 0.1F;
    Controller pid(c);

    // Settle: measurement stays at 20 across two calls.
    (void)pid.update(20.0F, 20.0F);
    const float out_settled = pid.update(20.0F, 20.0F);
    EXPECT_NEAR(out_settled, 0.0F, kEps);

    // Setpoint jumps; measurement does NOT. D-on-measurement → 0 kick.
    const float out_after_step = pid.update(100.0F, 20.0F);
    EXPECT_NEAR(out_after_step, 0.0F, kEps);
}

TEST(PidController, DerivativeReactsToMeasurementChange) {
    Controller::Config c{};
    c.kp              = 0.0F;
    c.ki              = 0.0F;
    c.kd              = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 0.1F;
    Controller pid(c);

    (void)pid.update(50.0F, 10.0F);  // seed last_measurement
    // Measurement rises by 5 over dt=0.1 → d_meas = +50; out = -kd * 50 = -50
    EXPECT_NEAR(pid.update(50.0F, 15.0F), -50.0F, kEps);
}

TEST(PidController, FirstUpdateHasNoDerivativeTerm) {
    Controller::Config c{};
    c.kp              = 0.0F;
    c.kd              = 1000.0F;
    c.output_min      = -1e6F;
    c.output_max      =  1e6F;
    c.integral_min    = -1e6F;
    c.integral_max    =  1e6F;
    c.sample_period_s = 0.1F;
    Controller pid(c);

    // Despite huge kd, the very first call has no prior sample → 0.
    EXPECT_NEAR(pid.update(0.0F, 1234.0F), 0.0F, kEps);
}

TEST(PidController, ResetClearsIntegralAndDerivativeHistory) {
    Controller::Config c{};
    c.kp              = 0.0F;
    c.ki              = 1.0F;
    c.kd              = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 0.1F;
    Controller pid(c);

    (void)pid.update(10.0F, 0.0F);
    (void)pid.update(10.0F, 5.0F);
    EXPECT_GT(pid.integral_state(), 0.0F);

    pid.reset();
    EXPECT_FLOAT_EQ(pid.integral_state(), 0.0F);

    // After reset, first call must not produce a derivative term.
    Controller::Config kd_only = c;
    kd_only.ki                 = 0.0F;
    pid.set_config(kd_only);
    pid.reset();
    EXPECT_NEAR(pid.update(0.0F, 9999.0F), 0.0F, kEps);
}

TEST(PidController, DerivativeFilterAttenuatesSingleStep) {
    Controller::Config c{};
    c.kd              = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 0.2F;
    c.d_filter_tau_s  = 0.8F;  // alpha = 0.2 / (0.8 + 0.2) = 0.2
    Controller pid(c);

    (void)pid.update(0.0F, 90.0F);
    // Raw slope = 0.1 / 0.2 = 0.5/s; filtered = 0.2 * 0.5 = 0.1/s.
    EXPECT_NEAR(pid.update(0.0F, 90.1F), -0.1F, 1e-4F);
    EXPECT_NEAR(pid.last_terms().slope, 0.1F, 1e-4F);
    // Flat afterwards: filtered slope decays by (1 - alpha) = 0.8 per step.
    EXPECT_NEAR(pid.update(0.0F, 90.1F), -0.08F, 1e-4F);
}

TEST(PidController, DerivativeFilterConvergesToTrueRamp) {
    Controller::Config c{};
    c.kd              = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.sample_period_s = 0.2F;
    c.d_filter_tau_s  = 1.0F;
    Controller pid(c);

    float meas = 90.0F;
    for (int i = 0; i < 200; ++i) {
        (void)pid.update(0.0F, meas);
        meas -= 0.1F;  // -0.5 °C/s
    }
    EXPECT_NEAR(pid.last_terms().slope, -0.5F, 1e-3F);
}

TEST(PidController, LastTermsReportBreakdownAndClamp) {
    Controller::Config c{};
    c.kp              = 10.0F;
    c.ki              = 1.0F;
    c.output_min      = 0.0F;
    c.output_max      = 15.0F;
    c.integral_min    = -100.0F;
    c.integral_max    =  100.0F;
    c.sample_period_s = 1.0F;
    Controller pid(c);

    // error = 2 → P = 20, I = 2, D = 0 (first call); clamps to 15.
    EXPECT_FLOAT_EQ(pid.update(2.0F, 0.0F), 15.0F);
    const auto& t = pid.last_terms();
    EXPECT_FLOAT_EQ(t.p, 20.0F);
    EXPECT_FLOAT_EQ(t.i, 2.0F);
    EXPECT_FLOAT_EQ(t.d, 0.0F);
    EXPECT_FLOAT_EQ(t.unclamped, 22.0F);
    EXPECT_FLOAT_EQ(t.output, 15.0F);
}

TEST(PidController, GainSwapIsBumplessAndZeroKiFreezesIntegral) {
    Controller::Config c{};
    c.kp              = 0.0F;
    c.ki              = 1.0F;
    c.output_min      = -1000.0F;
    c.output_max      =  1000.0F;
    c.integral_min    = -1000.0F;
    c.integral_max    =  1000.0F;
    c.sample_period_s = 1.0F;
    Controller pid(c);

    (void)pid.update(3.0F, 0.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 3.0F);

    // Swap to "brew" gains with ki = 0: integral contribution is held.
    Controller::Config brew = c;
    brew.ki = 0.0F;
    pid.set_config(brew);
    EXPECT_FLOAT_EQ(pid.update(0.0F, 5.0F), 3.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 3.0F);
}

TEST(PidController, ConditionalIntegrationHoldsWhileSaturated) {
    Controller::Config c{};
    c.kp                      = 10.0F;
    c.ki                      = 1.0F;
    c.output_min              = 0.0F;
    c.output_max              = 100.0F;
    c.integral_min            = -50.0F;
    c.integral_max            =  50.0F;
    c.sample_period_s         = 1.0F;
    c.conditional_integration = true;
    Controller pid(c);

    // Cold start: error 70 saturates the output high; integral must not grow.
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(pid.update(94.0F, 24.0F), 100.0F);
    }
    EXPECT_FLOAT_EQ(pid.integral_state(), 0.0F);

    // Unsaturated: integrates normally.
    EXPECT_FLOAT_EQ(pid.update(94.0F, 93.0F), 11.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 1.0F);

    // Saturated low with negative error: holds as well.
    EXPECT_FLOAT_EQ(pid.update(94.0F, 104.0F), 0.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 1.0F);
}

TEST(PidController, ConditionalIntegrationStillUnwindsAgainstSaturation) {
    Controller::Config c{};
    c.kp                      = 10.0F;
    c.ki                      = 1.0F;
    c.output_min              = 0.0F;
    c.output_max              = 100.0F;
    c.integral_min            = -50.0F;
    c.integral_max            =  50.0F;
    c.sample_period_s         = 1.0F;
    c.conditional_integration = true;
    Controller pid(c);

    for (int i = 0; i < 20; ++i) {
        (void)pid.update(94.0F, 93.0F);  // build I = 20 while unsaturated
    }
    ASSERT_FLOAT_EQ(pid.integral_state(), 20.0F);
    // Output saturated high but error negative: integration must proceed.
    c.kp = -200.0F;  // force high saturation with negative error
    pid.set_config(c);
    (void)pid.update(94.0F, 94.5F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 19.5F);
}

// Boiler coasting up on stored heat: below setpoint (error > 0) but D brakes
// the output to 0. Integrating there only winds the integral up.
TEST(PidController, ConditionalIntegrationHoldsWhileDerivativeBrakes) {
    Controller::Config c{};
    c.kp                      = 10.0F;
    c.ki                      = 1.0F;
    c.kd                      = 100.0F;
    c.output_min              = 0.0F;
    c.output_max              = 100.0F;
    c.integral_min            = -50.0F;
    c.integral_max            =  50.0F;
    c.sample_period_s         = 1.0F;
    c.conditional_integration = true;
    Controller pid(c);

    (void)pid.update(94.0F, 90.0F);                   // first sample: no D yet
    const float i0 = pid.integral_state();
    EXPECT_FLOAT_EQ(pid.update(94.0F, 91.0F), 0.0F);  // +1 C/s: D = -100
    EXPECT_FLOAT_EQ(pid.update(94.0F, 92.0F), 0.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), i0);
}

// Mirror case: above setpoint but falling fast, D drives the output high.
TEST(PidController, ConditionalIntegrationHoldsWhileDerivativeKicksUp) {
    Controller::Config c{};
    c.kp                      = 10.0F;
    c.ki                      = 1.0F;
    c.kd                      = 100.0F;
    c.output_min              = 0.0F;
    c.output_max              = 100.0F;
    c.integral_min            = -50.0F;
    c.integral_max            =  50.0F;
    c.sample_period_s         = 1.0F;
    c.conditional_integration = true;
    Controller pid(c);

    (void)pid.update(94.0F, 98.0F);
    const float i0 = pid.integral_state();
    EXPECT_FLOAT_EQ(pid.update(94.0F, 96.0F), 100.0F);  // -2 C/s: D = +200
    EXPECT_FLOAT_EQ(pid.integral_state(), i0);
}

Controller::Config banded(float band) {
    Controller::Config c{};
    c.kp              = 10.0F;
    c.ki              = 1.0F;
    c.output_min      = 0.0F;
    c.output_max      = 100.0F;
    c.integral_min    = -50.0F;
    c.integral_max    =  50.0F;
    c.sample_period_s = 1.0F;
    c.integral_band   = band;
    return c;
}

TEST(PidController, IntegralBandZeroIntegratesAtAnyError) {
    Controller pid(banded(0.0F));
    (void)pid.update(94.0F, 87.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 7.0F);
}

// Post-flush recovery: the boiler coasts up from 7 C below setpoint on
// stored heat with D holding the output at 0. Integrating that error
// winds the integral up and parks the boiler above setpoint afterwards.
TEST(PidController, IntegralHoldsOutsideTheBand) {
    Controller pid(banded(2.0F));
    for (int i = 0; i < 10; ++i) {
        (void)pid.update(94.0F, 87.0F);
        (void)pid.update(94.0F, 101.0F);
    }
    EXPECT_FLOAT_EQ(pid.integral_state(), 0.0F);
    EXPECT_FLOAT_EQ(pid.last_terms().i, 0.0F);
}

TEST(PidController, IntegralRunsInsideTheBandIncludingItsEdge) {
    Controller pid(banded(2.0F));
    (void)pid.update(94.0F, 93.0F);   // error  1
    (void)pid.update(94.0F, 92.0F);   // error  2: on the edge, still inside
    (void)pid.update(94.0F, 94.5F);   // error -0.5
    EXPECT_FLOAT_EQ(pid.integral_state(), 2.5F);
}

TEST(PidController, IntegralBandKeepsTheHeldValueInTheOutput) {
    Controller pid(banded(2.0F));
    (void)pid.update(94.0F, 93.0F);   // I = 1
    // Outside the band: I is held, not dropped, so the output stays bumpless.
    EXPECT_FLOAT_EQ(pid.update(94.0F, 90.0F), 10.0F * 4.0F + 1.0F);
    EXPECT_FLOAT_EQ(pid.integral_state(), 1.0F);
}

// ---------------- free integration band ----------------

Controller::Config holding_plant(float free_band) {
    Controller::Config c{};
    c.kp                      = 18.0F;
    c.ki                      = 0.2F;
    c.output_min              = 0.0F;
    c.output_max              = 100.0F;
    c.integral_min            = -10.0F;
    c.integral_max            = 15.0F;
    c.sample_period_s         = 1.0F;
    c.conditional_integration = true;
    c.integral_band           = 2.0F;
    c.free_integration_band   = free_band;
    return c;
}

// Idle hunting of +-0.3 C around setpoint with a 2 % holding integral: the
// output hits the 0 % floor on every excursion above setpoint.
float integral_after_symmetric_hunting(float free_band) {
    Controller pid(holding_plant(free_band));
    // Seed the integral at the holding value.
    for (int i = 0; i < 10; ++i) {
        (void)pid.update(94.5F, 94.4F);  // +0.1 error: 18*0.1 + I, unsaturated
    }
    const float seeded = pid.integral_state();
    for (int cycle = 0; cycle < 50; ++cycle) {
        for (int i = 0; i < 30; ++i) (void)pid.update(94.5F, 94.2F);  // below: heats
        for (int i = 0; i < 30; ++i) (void)pid.update(94.5F, 94.8F);  // above: floor
    }
    return pid.integral_state() - seeded;
}

TEST(PidController, ConditionalHoldRatchetsTheIntegralUpOnSymmetricHunting) {
    // The defect being fixed: same time above and below, integral climbs.
    EXPECT_GT(integral_after_symmetric_hunting(0.0F), 5.0F);
}

TEST(PidController, FreeIntegrationBandKeepsSymmetricHuntingCentred) {
    EXPECT_NEAR(integral_after_symmetric_hunting(0.5F), 0.0F, 0.5F);
}

TEST(PidController, FreeIntegrationBandStillHoldsFurtherOut) {
    Controller pid(holding_plant(0.5F));
    (void)pid.update(94.5F, 94.4F);
    const float i0 = pid.integral_state();
    // 1.5 C above (post-steam cool-down through the band): output at the
    // floor, error beyond the free band -> held as before.
    for (int i = 0; i < 60; ++i) {
        EXPECT_FLOAT_EQ(pid.update(94.5F, 96.0F), 0.0F);
    }
    EXPECT_FLOAT_EQ(pid.integral_state(), i0);
}

}  // namespace
