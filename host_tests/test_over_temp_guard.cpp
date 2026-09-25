// ESPID-Coffee — over-temperature guard tests.

#include <gtest/gtest.h>

#include <cmath>

#include "pid/over_temp_guard.hpp"

namespace {

using espid::pid::OverTempGuard;

// Mirrors the firmware defaults: brew 120 / 100, steam 165 / 145, 5 C margin.
OverTempGuard make_guard() {
    OverTempGuard::Config c{};
    c.brew              = {120.0F, 100.0F};
    c.steam             = {165.0F, 145.0F};
    c.cooldown_margin_c = 5.0F;
    return OverTempGuard(c);
}

TEST(OverTempGuard, BrewModeTripsAtStopAndReleasesAtReset) {
    auto g = make_guard();
    EXPECT_FALSE(g.update(false, 119.9F).tripped);
    const auto t = g.update(false, 120.0F);
    EXPECT_TRUE(t.tripped);
    EXPECT_TRUE(g.tripped());

    // Latched until the release threshold, not merely below the stop.
    EXPECT_FALSE(g.update(false, 110.0F).released);
    EXPECT_TRUE(g.tripped());
    EXPECT_TRUE(g.update(false, 100.0F).released);
    EXPECT_FALSE(g.tripped());
}

TEST(OverTempGuard, TripAndReleaseAreReportedOnce) {
    auto g = make_guard();
    EXPECT_TRUE(g.update(false, 125.0F).tripped);
    EXPECT_FALSE(g.update(false, 126.0F).tripped);
    EXPECT_TRUE(g.update(false, 95.0F).released);
    EXPECT_FALSE(g.update(false, 94.0F).released);
}

TEST(OverTempGuard, SteamModeUsesSteamLimits) {
    auto g = make_guard();
    EXPECT_FALSE(g.update(true, 140.0F).tripped);
    EXPECT_FLOAT_EQ(g.limits().stop_c, 165.0F);
    EXPECT_TRUE(g.update(true, 165.0F).tripped);
    EXPECT_TRUE(g.update(true, 145.0F).released);
}

// The 2026-09-24 regression: steam off at 126.5 C must not trip the stop.
TEST(OverTempGuard, LeavingSteamHotStartsCooldownInsteadOfTripping) {
    auto g = make_guard();
    (void)g.update(true, 128.0F);

    const auto t = g.update(false, 126.5F);
    EXPECT_TRUE(t.cooldown_started);
    EXPECT_FALSE(t.tripped);
    EXPECT_FALSE(g.tripped());
    EXPECT_TRUE(g.cooling_down());
    EXPECT_FLOAT_EQ(g.limits().stop_c, 165.0F);

    // Cooling through the brew stop threshold stays quiet.
    for (float t_c = 126.0F; t_c >= 116.0F; t_c -= 0.5F) {
        EXPECT_FALSE(g.update(false, t_c).tripped) << t_c;
    }
    EXPECT_TRUE(g.cooling_down());
}

TEST(OverTempGuard, CooldownEndsBelowMarginAndReArmsBrewLimits) {
    auto g = make_guard();
    (void)g.update(true, 128.0F);
    (void)g.update(false, 126.0F);

    EXPECT_FALSE(g.update(false, 115.0F).cooldown_ended);  // exit is strictly below 115
    const auto t = g.update(false, 114.9F);
    EXPECT_TRUE(t.cooldown_ended);
    EXPECT_FALSE(g.cooling_down());
    EXPECT_FLOAT_EQ(g.limits().stop_c, 120.0F);

    // Brew limits are live again: a genuine rise back to 120 C trips.
    EXPECT_TRUE(g.update(false, 120.0F).tripped);
}

TEST(OverTempGuard, LeavingSteamAlreadyCoolSkipsCooldown) {
    auto g = make_guard();
    (void)g.update(true, 110.0F);
    const auto t = g.update(false, 110.0F);
    EXPECT_FALSE(t.cooldown_started);
    EXPECT_FALSE(g.cooling_down());
    EXPECT_FLOAT_EQ(g.limits().stop_c, 120.0F);
}

TEST(OverTempGuard, SteamBackOnCancelsCooldown) {
    auto g = make_guard();
    (void)g.update(true, 128.0F);
    (void)g.update(false, 126.0F);
    ASSERT_TRUE(g.cooling_down());

    (void)g.update(true, 125.0F);
    EXPECT_FALSE(g.cooling_down());

    // A fresh off-edge starts a new cool-down.
    EXPECT_TRUE(g.update(false, 125.0F).cooldown_started);
}

TEST(OverTempGuard, GenuineSteamTripReleasesOnSteamLimitsDuringCooldown) {
    auto g = make_guard();
    EXPECT_TRUE(g.update(true, 166.0F).tripped);
    (void)g.update(false, 160.0F);  // steam off while latched
    EXPECT_TRUE(g.cooling_down());
    EXPECT_TRUE(g.tripped());

    EXPECT_FALSE(g.update(false, 146.0F).released);
    EXPECT_TRUE(g.update(false, 145.0F).released);
}

TEST(OverTempGuard, NonFiniteSampleChangesNothingButSteamEdgeIsKept) {
    auto g = make_guard();
    (void)g.update(true, 128.0F);

    // Steam goes off on a bad sample: cool-down starts conservatively.
    const auto t = g.update(false, NAN);
    EXPECT_TRUE(t.cooldown_started);
    EXPECT_FALSE(t.tripped);
    EXPECT_TRUE(g.cooling_down());

    // The next good sample above the brew stop is still covered.
    EXPECT_FALSE(g.update(false, 125.0F).tripped);

    // A bad sample never trips, releases or ends the cool-down.
    const auto n = g.update(false, NAN);
    EXPECT_FALSE(n.tripped || n.released || n.cooldown_ended);
    EXPECT_TRUE(g.cooling_down());
}

}  // namespace
