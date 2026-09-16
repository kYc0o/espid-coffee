// ESPID-Coffee -- pump::PsmCore unit tests (pulse-skip / integral-cycle mode).

#include <gtest/gtest.h>

#include "pump/psm_core.hpp"

namespace {

using espid::pump::PsmCore;

// Count how many half-cycles fire over n_half half-cycles at the current power.
int count_fired(PsmCore& core, int n_half) {
    int fired = 0;
    for (int i = 0; i < n_half; ++i) {
        if (core.on_half_cycle()) {
            ++fired;
        }
    }
    return fired;
}

TEST(PsmCore, RejectsZeroFrequency) {
    PsmCore core;
    EXPECT_FALSE(core.configure(0));
}

TEST(PsmCore, HalfPeriodFromMainsFrequency) {
    PsmCore core(50);
    EXPECT_EQ(core.half_period_us(), 10'000U);  // 50 Hz -> 10 ms half-cycle

    ASSERT_TRUE(core.configure(60));
    EXPECT_EQ(core.half_period_us(), 8'333U);   // 60 Hz -> ~8.33 ms
}

TEST(PsmCore, RejectsInvalidPowerRange) {
    PsmCore core(50);

    EXPECT_FALSE(core.set_power_pct(-1.0F));
    EXPECT_FALSE(core.set_power_pct(101.0F));
    EXPECT_TRUE(core.set_power_pct(42.0F));
    EXPECT_FLOAT_EQ(core.power_pct(), 42.0F);
    EXPECT_EQ(core.power_step(), 42U);
}

TEST(PsmCore, ZeroPowerNeverFires) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(0.0F));
    core.reset();

    EXPECT_FALSE(core.firing_enabled());
    EXPECT_EQ(count_fired(core, 200), 0);
}

TEST(PsmCore, FullPowerFiresEveryHalfCycle) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(100.0F));
    core.reset();

    EXPECT_TRUE(core.firing_enabled());
    EXPECT_EQ(count_fired(core, 200), 200);
}

TEST(PsmCore, HalfPowerFiresHalfTheCycles) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(50.0F));
    core.reset();

    // Over 200 half-cycles (100 whole cycles), ~50 % should fire.
    EXPECT_EQ(count_fired(core, 200), 100);
}

TEST(PsmCore, FiredFractionTracksPowerAcrossRange) {
    for (int pct = 0; pct <= 100; pct += 10) {
        PsmCore core(50);
        ASSERT_TRUE(core.set_power_pct(static_cast<float>(pct)));
        core.reset();

        // 100 whole cycles = 200 half-cycles; fired halves ~= 2 * pct.
        const int fired = count_fired(core, 200);
        EXPECT_NEAR(fired, 2 * pct, 2) << "at power " << pct << "%";
    }
}

TEST(PsmCore, FiredCyclesArePolarityBalanced) {
    // Whole-cycle modulation: fired half-cycles must come in adjacent pairs
    // (both halves of a selected cycle), never as isolated single halves.
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(50.0F));
    core.reset();

    int run = 0;  // length of the current run of fired half-cycles
    for (int i = 0; i < 400; ++i) {
        if (core.on_half_cycle()) {
            ++run;
        } else {
            EXPECT_EQ(run % 2, 0) << "odd fired-run ended at half-cycle " << i;
            run = 0;
        }
    }
    EXPECT_EQ(run % 2, 0);
}

TEST(PsmCore, ResetRestartsPattern) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(25.0F));
    core.reset();
    const int first = count_fired(core, 200);

    core.reset();
    const int second = count_fired(core, 200);
    EXPECT_EQ(first, second);
}

TEST(PsmCore, PowerChangeTakesEffectWithoutReset) {
    PsmCore core(50);
    core.reset();

    core.set_power_step(0U);
    EXPECT_EQ(count_fired(core, 100), 0);

    core.set_power_step(100U);
    EXPECT_EQ(count_fired(core, 100), 100);
}

}  // namespace
