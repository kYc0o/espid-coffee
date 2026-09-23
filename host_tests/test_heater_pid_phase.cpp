// ESPID-Coffee — heater PID phase selection + shot thermal stats tests.

#include <gtest/gtest.h>

#include "pid/heater_phase.hpp"
#include "pid/shot_thermal_stats.hpp"

namespace {

using espid::pid::HeaterPhase;
using espid::pid::HeaterPhaseTracker;
using espid::pid::ShotThermalStats;

TEST(HeaterPhaseTracker, StartsIdleWithoutChangeEvent) {
    HeaterPhaseTracker t;
    EXPECT_EQ(t.update(false, false, 0, 10'000), HeaterPhase::kIdle);
    EXPECT_FALSE(t.changed());
}

TEST(HeaterPhaseTracker, BrewThenTrailingThenIdle) {
    HeaterPhaseTracker t;
    (void)t.update(false, false, 0, 10'000);

    EXPECT_EQ(t.update(false, true, 100, 10'000), HeaterPhase::kBrew);
    EXPECT_TRUE(t.changed());
    EXPECT_EQ(t.update(false, true, 25'000, 10'000), HeaterPhase::kBrew);
    EXPECT_FALSE(t.changed());

    EXPECT_EQ(t.update(false, false, 25'100, 10'000), HeaterPhase::kBrewTrailing);
    EXPECT_TRUE(t.changed());
    EXPECT_EQ(t.previous(), HeaterPhase::kBrew);
    EXPECT_EQ(t.entered_ms(), 25'100U);

    EXPECT_EQ(t.update(false, false, 35'099, 10'000), HeaterPhase::kBrewTrailing);
    EXPECT_EQ(t.update(false, false, 35'100, 10'000), HeaterPhase::kIdle);
    EXPECT_TRUE(t.changed());
    EXPECT_EQ(t.previous(), HeaterPhase::kBrewTrailing);
}

TEST(HeaterPhaseTracker, ZeroTrailingGoesStraightToIdle) {
    HeaterPhaseTracker t;
    (void)t.update(false, true, 0, 0);
    EXPECT_EQ(t.update(false, false, 1'000, 0), HeaterPhase::kIdle);
}

TEST(HeaterPhaseTracker, NewShotDuringTrailingReturnsToBrew) {
    HeaterPhaseTracker t;
    (void)t.update(false, true, 0, 10'000);
    (void)t.update(false, false, 1'000, 10'000);
    EXPECT_EQ(t.update(false, true, 2'000, 10'000), HeaterPhase::kBrew);
}

TEST(HeaterPhaseTracker, SteamOverridesBrewAndCancelsTrailing) {
    HeaterPhaseTracker t;
    (void)t.update(false, true, 0, 10'000);
    EXPECT_EQ(t.update(true, true, 500, 10'000), HeaterPhase::kSteam);
    (void)t.update(false, false, 1'000, 10'000);
    EXPECT_EQ(t.phase(), HeaterPhase::kIdle);
}

TEST(HeaterPhaseTracker, BrewGainPhases) {
    EXPECT_TRUE(HeaterPhaseTracker::uses_brew_gains(HeaterPhase::kBrew));
    EXPECT_TRUE(HeaterPhaseTracker::uses_brew_gains(HeaterPhase::kBrewTrailing));
    EXPECT_FALSE(HeaterPhaseTracker::uses_brew_gains(HeaterPhase::kIdle));
    EXPECT_FALSE(HeaterPhaseTracker::uses_brew_gains(HeaterPhase::kSteam));
}

TEST(ShotThermalStats, DipOvershootSettleAndDuty) {
    ShotThermalStats s;
    s.begin(1'000, 92.0F, 92.0F, 0.5F);

    s.sample(1'000, 92.0F, 20.0F, true);
    s.sample(2'000, 91.0F, 80.0F, true);   // leaves band
    s.sample(3'000, 90.2F, 100.0F, true);  // bottom
    s.sample(4'000, 91.8F, 60.0F, false);  // back in band
    s.sample(5'000, 92.7F, 0.0F, false);   // overshoot, out of band
    s.sample(6'000, 92.3F, 10.0F, false);  // settles here

    const auto r = s.finish();
    EXPECT_FALSE(s.active());
    EXPECT_FLOAT_EQ(r.start_temp_c, 92.0F);
    EXPECT_FLOAT_EQ(r.min_temp_c, 90.2F);
    EXPECT_EQ(r.min_at_ms, 2'000U);
    EXPECT_NEAR(r.drop_c(), 1.8F, 1e-4F);
    EXPECT_FLOAT_EQ(r.max_after_min_c, 92.7F);
    EXPECT_NEAR(r.overshoot_c(), 0.7F, 1e-4F);
    EXPECT_TRUE(r.settled);
    EXPECT_EQ(r.settled_at_ms, 5'000U);
    EXPECT_EQ(r.brew_ms, 2'000U);
    EXPECT_EQ(r.window_ms, 5'000U);
    EXPECT_NEAR(r.avg_duty_brew, 200.0F / 3.0F, 1e-3F);
    EXPECT_NEAR(r.avg_duty_trail, 70.0F / 3.0F, 1e-3F);
}

TEST(ShotThermalStats, UnsettledWhenWindowEndsOutOfBand) {
    ShotThermalStats s;
    s.begin(0, 92.0F, 92.0F, 0.5F);
    s.sample(500, 91.0F, 100.0F, false);
    const auto r = s.finish();
    EXPECT_FALSE(r.settled);
    EXPECT_EQ(r.settled_at_ms, 500U);
}

TEST(ShotThermalStats, NeverLeavingBandSettlesAtZero) {
    ShotThermalStats s;
    s.begin(0, 92.0F, 92.0F, 0.5F);
    s.sample(500, 91.8F, 40.0F, true);
    const auto r = s.finish();
    EXPECT_TRUE(r.settled);
    EXPECT_EQ(r.settled_at_ms, 0U);
}

TEST(ShotThermalStats, IgnoresSamplesWhenInactiveOrNonFinite) {
    ShotThermalStats s;
    s.sample(0, 10.0F, 50.0F, true);
    EXPECT_FALSE(s.active());

    s.begin(0, 92.0F, 92.0F, 0.5F);
    s.sample(100, NAN, 50.0F, true);
    const auto r = s.finish();
    EXPECT_FLOAT_EQ(r.min_temp_c, 92.0F);
    EXPECT_FLOAT_EQ(r.avg_duty_brew, 0.0F);
}

}  // namespace
