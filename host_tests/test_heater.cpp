// ESPID-Coffee -- heater::SlowPwmCore unit tests.

#include <gtest/gtest.h>

#include "heater/slow_pwm_core.hpp"

namespace {

using espid::heater::SlowPwmCore;

TEST(SlowPwmCore, RejectsZeroPeriod) {
    SlowPwmCore core;
    EXPECT_FALSE(core.configure(0));
}

TEST(SlowPwmCore, ZeroDutyIsAlwaysOff) {
    SlowPwmCore core(1000);
    ASSERT_TRUE(core.set_duty_pct(0.0F));

    EXPECT_FALSE(core.output_on(0));
    EXPECT_FALSE(core.output_on(250));
    EXPECT_FALSE(core.output_on(999));
    EXPECT_FALSE(core.output_on(2000));
}

TEST(SlowPwmCore, FullDutyIsAlwaysOn) {
    SlowPwmCore core(1000);
    ASSERT_TRUE(core.set_duty_pct(100.0F));

    EXPECT_TRUE(core.output_on(0));
    EXPECT_TRUE(core.output_on(250));
    EXPECT_TRUE(core.output_on(999));
    EXPECT_TRUE(core.output_on(2000));
}

TEST(SlowPwmCore, QuarterDutyUsesFirstQuarterOfEachPeriod) {
    SlowPwmCore core(1000);
    ASSERT_TRUE(core.set_duty_pct(25.0F));

    EXPECT_TRUE(core.output_on(0));
    EXPECT_TRUE(core.output_on(249));
    EXPECT_FALSE(core.output_on(250));
    EXPECT_FALSE(core.output_on(999));
    EXPECT_TRUE(core.output_on(1000));
}

TEST(SlowPwmCore, DutyChangeTakesEffectOnNextEvaluation) {
    SlowPwmCore core(1000);
    ASSERT_TRUE(core.set_duty_pct(50.0F));

    EXPECT_TRUE(core.output_on(400));

    ASSERT_TRUE(core.set_duty_pct(10.0F));
    EXPECT_FALSE(core.output_on(600));
}

TEST(SlowPwmCore, RejectsInvalidDutyRange) {
    SlowPwmCore core(1000);

    EXPECT_FALSE(core.set_duty_pct(-1.0F));
    EXPECT_FALSE(core.set_duty_pct(101.0F));
    EXPECT_TRUE(core.set_duty_pct(42.5F));
    EXPECT_FLOAT_EQ(core.duty_pct(), 42.5F);
}

}  // namespace
