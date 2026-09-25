// ESPID-Coffee — telemetry publish-rate policy tests.

#include <gtest/gtest.h>

#include "pid/telemetry_policy.hpp"

namespace {

using espid::pid::HeaterPhase;
using espid::pid::TelemetryPolicy;

constexpr std::uint64_t kAfterWarmup = 900'000;  // 15 min uptime

TelemetryPolicy::Config cfg() { return TelemetryPolicy::Config{}; }

TEST(TelemetryPolicy, WarmupWindowPublishesFast) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), 0, HeaterPhase::kIdle, 70.0F, 0.0F), 1'000U);
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), 599'999, HeaterPhase::kIdle, 0.0F, 0.0F), 1'000U);
}

TEST(TelemetryPolicy, SettledAfterWarmupFallsBackToHeartbeat) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), 600'000, HeaterPhase::kIdle, 0.1F, 0.01F), 10'000U);
}

TEST(TelemetryPolicy, BrewAndTrailingAlwaysFastest) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kBrew, 0.0F, 0.0F), 500U);
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kBrewTrailing, 0.0F, 0.0F), 500U);
}

// The shot switch is authoritative even while the heater phase still lags
// at "idle" on the first record of a shot.
TEST(TelemetryPolicy, LiveShotSwitchWinsOverLaggingPhase) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle,
                                         0.0F, 0.0F, /*shot_running=*/true), 500U);
}

TEST(TelemetryPolicy, LiveSteamSwitchWinsOverLaggingPhase) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle,
                                         0.0F, 0.0F, false, /*steam_active=*/true), 1'000U);
}

TEST(TelemetryPolicy, SteamUsesActiveRate) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kSteam, 0.0F, 0.0F), 1'000U);
}

TEST(TelemetryPolicy, OffSetpointOrMovingUsesActiveRate) {
    // Outside the band (e.g. post-shot recovery).
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle, -1.2F, 0.0F), 1'000U);
    // In band but drifting.
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle, 0.1F, 0.2F), 1'000U);
}

TEST(TelemetryPolicy, NonFiniteInputsPublishFast) {
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle, NAN, 0.0F), 1'000U);
    EXPECT_EQ(TelemetryPolicy::period_ms(cfg(), kAfterWarmup, HeaterPhase::kIdle, 0.0F, NAN), 1'000U);
}

TEST(TelemetryPolicy, BandsAreConfigurable) {
    auto c = cfg();
    c.settle_band_c    = 2.0F;
    c.active_slope_cps = 1.0F;
    c.idle_period_ms   = 30'000;
    EXPECT_EQ(TelemetryPolicy::period_ms(c, kAfterWarmup, HeaterPhase::kIdle, -1.2F, 0.2F), 30'000U);
}

}  // namespace
