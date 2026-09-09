// ESPID-Coffee -- pump::PsmCore unit tests.

#include <gtest/gtest.h>

#include "pump/psm_core.hpp"

namespace {

using espid::pump::PsmCore;

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

TEST(PsmCore, FullPowerFiresImmediately) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(100.0F));

    EXPECT_TRUE(core.firing_enabled());
    EXPECT_EQ(core.firing_delay_us(), 0U);
}

TEST(PsmCore, ZeroPowerDisablesFiring) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(0.0F));

    EXPECT_FALSE(core.firing_enabled());
    EXPECT_EQ(core.firing_delay_us(), core.half_period_us());
}

TEST(PsmCore, HalfPowerFiresAtMidHalfCycle) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(50.0F));

    EXPECT_TRUE(core.firing_enabled());
    EXPECT_EQ(core.firing_delay_us(), 5'000U);  // half of 10 ms
}

TEST(PsmCore, DelayScalesLinearlyWithPower) {
    PsmCore core(50);

    ASSERT_TRUE(core.set_power_pct(25.0F));
    EXPECT_EQ(core.firing_delay_us(), 7'500U);

    ASSERT_TRUE(core.set_power_pct(75.0F));
    EXPECT_EQ(core.firing_delay_us(), 2'500U);
}

TEST(PsmCore, RejectsInvalidPowerRange) {
    PsmCore core(50);

    EXPECT_FALSE(core.set_power_pct(-1.0F));
    EXPECT_FALSE(core.set_power_pct(101.0F));
    EXPECT_TRUE(core.set_power_pct(42.5F));
    EXPECT_FLOAT_EQ(core.power_pct(), 42.5F);
}

TEST(PsmCore, DelayNeverExceedsHalfPeriod) {
    PsmCore core(50);
    ASSERT_TRUE(core.set_power_pct(0.1F));
    EXPECT_LE(core.firing_delay_us(), core.half_period_us());
}

}  // namespace
