// ESPID-Coffee — OLED machine-status selection tests.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "ui/machine_status.hpp"

namespace {

using espid::ui::format_header;
using espid::ui::MachineStatus;
using espid::ui::needs_attention;
using espid::ui::select_status;
using espid::ui::StatusInputs;
using espid::ui::SteamReadyTracker;

TEST(MachineStatus, DefaultIsIdle) {
    EXPECT_EQ(select_status(StatusInputs{}), MachineStatus::kIdle);
}

// The full post-steam sequence the operator walks through.
TEST(MachineStatus, SteamThenRefillWalkthrough) {
    StatusInputs in{};
    in.steam_active = true;
    EXPECT_EQ(select_status(in), MachineStatus::kSteamHeating);
    in.steam_ready = true;
    EXPECT_EQ(select_status(in), MachineStatus::kSteamReady);

    in.steam_active   = false;
    in.steam_ready    = false;
    in.steam_cooldown = true;
    EXPECT_EQ(select_status(in), MachineStatus::kRefillPrompt);

    in.refill_running = true;
    EXPECT_EQ(select_status(in), MachineStatus::kRefilling);

    in.refill_running         = false;
    in.refill_waiting_release = true;
    EXPECT_EQ(select_status(in), MachineStatus::kRefillFullRelease);
    in.refill_timed_out = true;
    EXPECT_EQ(select_status(in), MachineStatus::kRefillTimeoutRelease);

    // Switch off, boiler cooled below the cool-down exit: back to normal.
    in = StatusInputs{};
    EXPECT_EQ(select_status(in), MachineStatus::kIdle);
}

// A small steam session can leave the boiler above the cool-down exit after
// a full refill; it must not be asked to refill again.
TEST(MachineStatus, NoSecondRefillPromptAfterConfirmedFull) {
    StatusInputs in{};
    in.steam_cooldown         = true;
    in.refilled_this_cooldown = true;
    EXPECT_EQ(select_status(in), MachineStatus::kCoolingDown);
    EXPECT_FALSE(needs_attention(MachineStatus::kCoolingDown));
}

TEST(MachineStatus, SafetyOutranksEverything) {
    StatusInputs in{};
    in.steam_active   = true;
    in.refill_running = true;
    in.shot_running   = true;
    in.estop          = true;
    EXPECT_EQ(select_status(in), MachineStatus::kEmergencyStop);

    in.estop         = false;
    in.fill_complete = false;
    EXPECT_EQ(select_status(in), MachineStatus::kFilling);

    in.fill_complete = true;
    in.fill_ok       = false;
    EXPECT_EQ(select_status(in), MachineStatus::kFillFailed);
}

TEST(MachineStatus, ShotWinsOverSteamAndCooldownPrompt) {
    StatusInputs in{};
    in.shot_running   = true;
    in.steam_active   = true;
    in.steam_cooldown = true;
    EXPECT_EQ(select_status(in), MachineStatus::kBrewing);
}

TEST(MachineStatus, OnlyPromptsAndFaultsAskForAttention) {
    EXPECT_TRUE(needs_attention(MachineStatus::kRefillPrompt));
    EXPECT_TRUE(needs_attention(MachineStatus::kRefillFullRelease));
    EXPECT_TRUE(needs_attention(MachineStatus::kRefillTimeoutRelease));
    EXPECT_TRUE(needs_attention(MachineStatus::kEmergencyStop));
    EXPECT_TRUE(needs_attention(MachineStatus::kFillFailed));
    EXPECT_FALSE(needs_attention(MachineStatus::kIdle));
    EXPECT_FALSE(needs_attention(MachineStatus::kRefilling));
    EXPECT_FALSE(needs_attention(MachineStatus::kSteamReady));
    EXPECT_FALSE(needs_attention(MachineStatus::kBrewing));
}

// Every header must leave room for the 4-character Wi-Fi label (16 of the
// 21 characters a 6 px font fits on 128 px).
TEST(MachineStatus, EveryHeaderFitsNextToTheWifiLabel) {
    for (int s = 0; s <= static_cast<int>(MachineStatus::kCoolingDown); ++s) {
        char buf[32];
        const int n = format_header(static_cast<MachineStatus>(s), 99, buf, sizeof(buf));
        EXPECT_GT(n, 0) << s;
        EXPECT_LE(std::strlen(buf), 16U) << s << " '" << buf << "'";
    }
}

TEST(MachineStatus, RefillingShowsElapsedSeconds) {
    char buf[32];
    (void)format_header(MachineStatus::kRefilling, 7, buf, sizeof(buf));
    EXPECT_STREQ(buf, "REFILLING  7s");
    (void)format_header(MachineStatus::kRefillPrompt, 0, buf, sizeof(buf));
    EXPECT_STREQ(buf, "BREW to refill");
}

TEST(MachineStatus, FormatHeaderToleratesTinyBuffers) {
    char buf[4];
    EXPECT_GT(format_header(MachineStatus::kEmergencyStop, 0, buf, sizeof(buf)), 0);
    EXPECT_EQ(std::strlen(buf), 3U);
    EXPECT_EQ(format_header(MachineStatus::kIdle, 0, nullptr, 8), 0);
}

TEST(SteamReadyTracker, ReadyNearSetpointWithHysteresis) {
    SteamReadyTracker t(2.0F, 5.0F);
    EXPECT_FALSE(t.update(true, 120.0F, 130.0F));
    EXPECT_TRUE(t.update(true, 128.0F, 130.0F));   // within 2 C
    EXPECT_TRUE(t.update(true, 125.5F, 130.0F));   // dipping while steaming
    EXPECT_FALSE(t.update(true, 124.9F, 130.0F));  // more than 5 C below
    EXPECT_FALSE(t.update(true, 127.0F, 130.0F));  // needs 128 again
    EXPECT_TRUE(t.update(true, 128.0F, 130.0F));
}

TEST(SteamReadyTracker, ResetsWhenSteamOffOrReadingBad) {
    SteamReadyTracker t(2.0F, 5.0F);
    ASSERT_TRUE(t.update(true, 129.0F, 130.0F));
    EXPECT_FALSE(t.update(false, 129.0F, 130.0F));
    ASSERT_TRUE(t.update(true, 129.0F, 130.0F));
    EXPECT_FALSE(t.update(true, NAN, 130.0F));
}

}  // namespace
