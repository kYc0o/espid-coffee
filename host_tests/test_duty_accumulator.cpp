// ESPID-Coffee — interval-average duty accumulator tests.

#include <gtest/gtest.h>

#include <cmath>

#include "pid/duty_accumulator.hpp"

namespace {

using espid::pid::DutyAccumulator;

TEST(DutyAccumulator, EmptyTakeReportsNothing) {
    DutyAccumulator a;
    float m = -1.0F;
    EXPECT_FALSE(a.take(&m));
    EXPECT_FLOAT_EQ(m, -1.0F);
}

// A short pulse between two records must still show up in the mean --
// the instantaneous value at publish time would read 0.
TEST(DutyAccumulator, ShortPulseSurvivesIntoTheMean) {
    DutyAccumulator a;
    a.add(0.0F);
    a.add(50.0F);
    a.add(0.0F);
    a.add(0.0F);
    a.add(0.0F);
    float m = 0.0F;
    ASSERT_TRUE(a.take(&m));
    EXPECT_NEAR(m, 10.0F, 1e-4F);
}

TEST(DutyAccumulator, TakeStartsANewWindow) {
    DutyAccumulator a;
    a.add(100.0F);
    float m = 0.0F;
    ASSERT_TRUE(a.take(&m));
    EXPECT_FLOAT_EQ(m, 100.0F);
    EXPECT_FALSE(a.take(&m));

    a.add(20.0F);
    a.add(40.0F);
    ASSERT_TRUE(a.take(&m));
    EXPECT_NEAR(m, 30.0F, 1e-4F);
}

TEST(DutyAccumulator, KeepsHundredthOfAPercentResolution) {
    DutyAccumulator a;
    a.add(2.14F);
    a.add(2.16F);
    float m = 0.0F;
    ASSERT_TRUE(a.take(&m));
    EXPECT_NEAR(m, 2.15F, 1e-4F);
}

TEST(DutyAccumulator, ClampsOutOfRangeAndNonFinite) {
    DutyAccumulator a;
    a.add(150.0F);   // -> 100
    a.add(-5.0F);    // -> 0
    a.add(NAN);      // -> 0
    a.add(INFINITY); // -> 0 (non-finite)
    float m = 0.0F;
    ASSERT_TRUE(a.take(&m));
    EXPECT_NEAR(m, 25.0F, 1e-4F);
}

TEST(DutyAccumulator, UndrainedWindowRestartsInsteadOfOverflowing) {
    DutyAccumulator a;
    for (std::uint32_t i = 0; i < DutyAccumulator::kMaxSamples; ++i) {
        a.add(100.0F);
    }
    a.add(10.0F);  // window full: restarts with this sample
    float m = 0.0F;
    ASSERT_TRUE(a.take(&m));
    EXPECT_FLOAT_EQ(m, 10.0F);
}

}  // namespace
