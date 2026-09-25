// ESPID-Coffee — post-steam boiler refill sequencer tests.

#include <gtest/gtest.h>

#include <cmath>

#include "pump/refill_sequencer.hpp"

namespace {

using espid::pump::RefillSequencer;
using State      = RefillSequencer::State;
using StopReason = RefillSequencer::StopReason;

RefillSequencer make() {
    RefillSequencer::Config c{};
    c.power_pct         = 30.0F;
    c.full_pressure_bar = 5.0F;
    c.confirm_ms        = 600;
    c.max_duration_ms   = 60'000;
    return RefillSequencer(c);
}

TEST(RefillSequencer, IdleNeverDrivesThePump) {
    auto r = make();
    EXPECT_FLOAT_EQ(r.step(0, true, true, 2.0F), 0.0F);
    EXPECT_EQ(r.state(), State::kIdle);
}

TEST(RefillSequencer, PumpsGentlyUntilPressureConfirmsFull) {
    auto r = make();
    r.start(1'000);
    // Steam space condensing: pressure low, pump keeps running.
    EXPECT_FLOAT_EQ(r.step(1'100, true, true, 2.1F), 30.0F);
    EXPECT_FLOAT_EQ(r.step(20'000, true, true, 1.8F), 30.0F);
    // Hydraulically full: pressure climbs, must hold for confirm_ms.
    EXPECT_FLOAT_EQ(r.step(30'000, true, true, 6.0F), 30.0F);
    EXPECT_FLOAT_EQ(r.step(30'500, true, true, 7.5F), 30.0F);
    EXPECT_FLOAT_EQ(r.step(30'600, true, true, 8.0F), 0.0F);
    EXPECT_TRUE(r.finished_now());
    EXPECT_EQ(r.reason(), StopReason::kFull);
    EXPECT_EQ(r.duration_ms(), 29'600U);
}

TEST(RefillSequencer, PressureSpikeDoesNotEndTheRefill) {
    auto r = make();
    r.start(0);
    (void)r.step(1'000, true, true, 6.0F);   // spike above threshold
    (void)r.step(1'300, true, true, 2.0F);   // back down: confirm restarts
    EXPECT_FLOAT_EQ(r.step(1'700, true, true, 6.0F), 30.0F);
    EXPECT_FLOAT_EQ(r.step(2'200, true, true, 6.0F), 30.0F);  // 500 ms only
    EXPECT_TRUE(r.running());
    EXPECT_FLOAT_EQ(r.step(2'300, true, true, 6.0F), 0.0F);   // 600 ms held
    EXPECT_EQ(r.reason(), StopReason::kFull);
}

TEST(RefillSequencer, UnusablePressureNeverConfirms) {
    auto r = make();
    r.start(0);
    (void)r.step(1'000, true, false, 9.0F);
    (void)r.step(2'000, true, false, 9.0F);
    (void)r.step(3'000, true, true, NAN);
    EXPECT_TRUE(r.running());
}

// Steam knob left open: the boiler never pressurises, the backstop ends it.
TEST(RefillSequencer, TimesOutWhenTheBoilerNeverPressurises) {
    auto r = make();
    r.start(0);
    EXPECT_FLOAT_EQ(r.step(59'999, true, true, 0.3F), 30.0F);
    EXPECT_FLOAT_EQ(r.step(60'000, true, true, 0.3F), 0.0F);
    EXPECT_EQ(r.reason(), StopReason::kTimeout);
    EXPECT_TRUE(r.finished_now());
}

TEST(RefillSequencer, ReleasingTheSwitchCancelsImmediately) {
    auto r = make();
    r.start(0);
    (void)r.step(5'000, true, true, 2.0F);
    EXPECT_FLOAT_EQ(r.step(5'100, false, true, 2.0F), 0.0F);
    EXPECT_TRUE(r.finished_now());
    EXPECT_EQ(r.reason(), StopReason::kCancelled);
    EXPECT_EQ(r.state(), State::kIdle);
}

// A switch left on after a finished refill must not restart it.
TEST(RefillSequencer, FinishedRefillWaitsForRelease) {
    auto r = make();
    r.start(0);
    (void)r.step(1'000, true, true, 6.0F);
    (void)r.step(1'600, true, true, 6.0F);
    ASSERT_EQ(r.state(), State::kFinished);

    r.start(2'000);  // ignored: not idle
    EXPECT_FLOAT_EQ(r.step(2'100, true, true, 2.0F), 0.0F);
    EXPECT_FALSE(r.finished_now());
    EXPECT_EQ(r.state(), State::kFinished);

    (void)r.step(3'000, false, true, 2.0F);  // released
    EXPECT_EQ(r.state(), State::kIdle);
    r.start(4'000);
    EXPECT_TRUE(r.running());
    EXPECT_EQ(r.reason(), StopReason::kNone);
}

TEST(RefillSequencer, StopReasonNames) {
    EXPECT_STREQ(espid::pump::to_string(StopReason::kFull), "full");
    EXPECT_STREQ(espid::pump::to_string(StopReason::kTimeout), "timeout");
    EXPECT_STREQ(espid::pump::to_string(StopReason::kCancelled), "cancelled");
}

}  // namespace
