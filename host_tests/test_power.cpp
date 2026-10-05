// ESPID-Coffee — standby timer, daily wake and deep-sleep planning tests.

#include <gtest/gtest.h>

#include <cstdlib>
#include <ctime>
#include <string>

#include "power/daily_wake.hpp"
#include "power/inactivity_timer.hpp"
#include "power/sleep_planner.hpp"
#include "power/time_of_day.hpp"
#include "power/wake_pins.hpp"
#include "power/wake_reason.hpp"

namespace {

using espid::power::InactivityTimer;

// ---------------- InactivityTimer ----------------

constexpr std::uint32_t kMin = 60'000;

TEST(InactivityTimer, ExpiresAfterTimeoutFromBoot) {
    InactivityTimer t({true, 90 * kMin}, 1'000);
    EXPECT_FALSE(t.expired(1'000 + 90 * kMin - 1));
    EXPECT_EQ(t.remaining_ms(1'000 + 90 * kMin - 1), 1U);
    EXPECT_TRUE(t.expired(1'000 + 90 * kMin));
    EXPECT_EQ(t.remaining_ms(1'000 + 90 * kMin), 0U);
}

TEST(InactivityTimer, ActivityRestartsTheCountdown) {
    InactivityTimer t({true, 10 * kMin}, 0);
    t.touch(9 * kMin);
    EXPECT_FALSE(t.expired(18 * kMin));
    EXPECT_TRUE(t.expired(19 * kMin));
}

TEST(InactivityTimer, DisabledNeverExpires) {
    InactivityTimer t({false, 1}, 0);
    EXPECT_FALSE(t.expired(1'000'000'000ULL));
    EXPECT_EQ(t.remaining_ms(5), 0U);
}

TEST(InactivityTimer, LoweringTheTimeoutKeepsTheLastActivity) {
    InactivityTimer t({true, 90 * kMin}, 0);
    t.set_config({true, 30 * kMin});
    EXPECT_TRUE(t.expired(45 * kMin));
}

// ---------------- next_daily_wake ----------------

// Runs each test in the machine's time zone and restores the host's.
class DailyWake : public ::testing::Test {
   protected:
    void SetUp() override {
        const char* old = std::getenv("TZ");
        had_tz_ = old != nullptr;
        if (had_tz_) {
            old_tz_ = old;
        }
        setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
        tzset();
    }
    void TearDown() override {
        if (had_tz_) {
            setenv("TZ", old_tz_.c_str(), 1);
        } else {
            unsetenv("TZ");
        }
        tzset();
    }

   private:
    bool        had_tz_ = false;
    std::string old_tz_;
};

constexpr int kNine = 9 * 60;

TEST_F(DailyWake, LaterTheSameDay) {
    // 2026-10-02 08:00 CEST -> 09:00 CEST the same day.
    EXPECT_EQ(espid::power::next_daily_wake(1790920800, kNine), 1790924400);
}

TEST_F(DailyWake, ExactlyAtOrPastTheTimeMeansTomorrow) {
    EXPECT_EQ(espid::power::next_daily_wake(1790924400, kNine), 1791010800);
    EXPECT_EQ(espid::power::next_daily_wake(1790924430, kNine), 1791010800);
}

TEST_F(DailyWake, KeepsTheLocalHourAcrossSpringForward) {
    // 2026-03-28 10:00 CET -> 2026-03-29 09:00 CEST: only 22 h later.
    EXPECT_EQ(espid::power::next_daily_wake(1774688400, kNine), 1774767600);
}

TEST_F(DailyWake, KeepsTheLocalHourAcrossFallBack) {
    // 2026-10-24 10:00 CEST -> 2026-10-25 09:00 CET: 24 h + 1 h later.
    EXPECT_EQ(espid::power::next_daily_wake(1792828800, kNine), 1792915200);
}

TEST_F(DailyWake, NonexistentLocalTimeStillWakesThatNight) {
    // 02:30 does not exist on 2026-03-29; mktime() normalises it.
    const std::time_t now = 1774735200;  // 2026-03-28 23:00 CET
    const std::time_t t   = espid::power::next_daily_wake(now, 2 * 60 + 30);
    EXPECT_GT(t, now);
    EXPECT_LT(t - now, 6 * 3600);
}

TEST_F(DailyWake, CrossesTheYear) {
    // 2026-12-31 23:30 CET, wake 00:15 -> 2027-01-01 00:15.
    EXPECT_EQ(espid::power::next_daily_wake(1798756200, 15), 1798758900);
}

TEST_F(DailyWake, RejectsBadMinuteAndUnsetClock) {
    EXPECT_EQ(espid::power::next_daily_wake(1790920800, -1), -1);
    EXPECT_EQ(espid::power::next_daily_wake(1790920800, 24 * 60), -1);
    EXPECT_EQ(espid::power::next_daily_wake(3600, kNine), -1);  // 1970: no NTP yet
}

// ---------------- plan_sleep ----------------

using espid::power::plan_sleep;
using espid::power::SleepPlanConfig;

constexpr std::uint64_t kUs = 1'000'000ULL;

TEST(SleepPlanner, LongSleepStopsShortByTheDriftMargin) {
    const SleepPlanConfig c{};  // 3 %, >= 60 s, last 120 s direct
    const auto p = plan_sleep(0, 20 * 3600, c);
    EXPECT_TRUE(p.resync_first);
    EXPECT_EQ(p.sleep_us, (20 * 3600 - 2160) * kUs);  // 3 % of 20 h = 36 min
}

TEST(SleepPlanner, ShortSleepUsesTheMinimumMargin) {
    const auto p = plan_sleep(0, 1000, SleepPlanConfig{});
    EXPECT_TRUE(p.resync_first);
    EXPECT_EQ(p.sleep_us, (1000 - 60) * kUs);
}

TEST(SleepPlanner, LastLegIsSleptStraightToTheTarget) {
    const auto p = plan_sleep(100, 220, SleepPlanConfig{});
    EXPECT_FALSE(p.resync_first);
    EXPECT_EQ(p.sleep_us, 120 * kUs);
}

TEST(SleepPlanner, DueOrPastMeansNoSleep) {
    EXPECT_EQ(plan_sleep(500, 500, SleepPlanConfig{}).sleep_us, 0U);
    EXPECT_EQ(plan_sleep(501, 500, SleepPlanConfig{}).sleep_us, 0U);
}

TEST(SleepPlanner, ConvergesInAFewRoundsDespiteWorstCaseDrift) {
    // Every early wake lands 3 % late (worst case); the target must still
    // be reached without ever overshooting it.
    const SleepPlanConfig c{};
    std::time_t now = 0;
    const std::time_t target = 23 * 3600;
    int rounds = 0;
    for (;;) {
        const auto p = plan_sleep(now, target, c);
        ++rounds;
        if (!p.resync_first) {
            break;
        }
        const double slept_s = static_cast<double>(p.sleep_us / kUs) * 1.03;
        now += static_cast<std::time_t>(slept_s);
        ASSERT_LT(now, target);
    }
    EXPECT_LE(rounds, 4);
}

TEST(SleepPlanner, MinimumMarginLargerThanTheGapStillProgresses) {
    SleepPlanConfig c{};
    c.precise_window_s = 0;
    c.min_margin_s     = 1000;
    const auto p = plan_sleep(0, 200, c);
    EXPECT_TRUE(p.resync_first);
    EXPECT_EQ(p.sleep_us, 1 * kUs);
}

// ---------------- select_wake_pins ----------------

using espid::power::select_wake_pins;
using espid::power::WakePins;

constexpr WakePins kPins{5, 4};

TEST(WakePins, BothOffWakeOnEitherGoingHigh) {
    const auto a = select_wake_pins(kPins, false, false);
    ASSERT_TRUE(a.ok);
    EXPECT_EQ(a.ext1_high_mask, (1ULL << 5) | (1ULL << 4));
    EXPECT_EQ(a.ext0_gpio, -1);
}

TEST(WakePins, SteamLeftOnWakesWhenTurnedOff) {
    const auto a = select_wake_pins(kPins, false, true);
    ASSERT_TRUE(a.ok);
    EXPECT_EQ(a.ext1_high_mask, 1ULL << 5);
    EXPECT_EQ(a.ext0_gpio, 4);
    EXPECT_EQ(a.ext0_level, 0);
}

TEST(WakePins, BothOnCannotBeArmed) {
    EXPECT_FALSE(select_wake_pins(kPins, true, true).ok);
}

TEST(WakePins, MissingPinCannotBeArmed) {
    EXPECT_FALSE(select_wake_pins(WakePins{-1, 4}, false, false).ok);
}

// ---------------- time of day ----------------

using espid::power::format_time_of_day;
using espid::power::parse_time_of_day;

TEST(TimeOfDay, ParsesTwentyFourHourText) {
    int m = -1;
    EXPECT_TRUE(parse_time_of_day("09:00", &m));
    EXPECT_EQ(m, 540);
    EXPECT_TRUE(parse_time_of_day("9:05", &m));
    EXPECT_EQ(m, 545);
    EXPECT_TRUE(parse_time_of_day("00:00", &m));
    EXPECT_EQ(m, 0);
    EXPECT_TRUE(parse_time_of_day("23:59", &m));
    EXPECT_EQ(m, 1439);
}

TEST(TimeOfDay, RejectsAnythingElse) {
    int m = 7;
    for (const char* bad : {"", "9", "24:00", "12:60", "123:00", "12:5", "12:345",
                            "12:00:00", "12:00 ", "7pm", ":30", "ab:cd"}) {
        EXPECT_FALSE(parse_time_of_day(bad, &m)) << bad;
    }
    EXPECT_FALSE(parse_time_of_day(nullptr, &m));
    EXPECT_EQ(m, 7);
}

TEST(TimeOfDay, FormatsAndRoundTrips) {
    char buf[6];
    ASSERT_TRUE(format_time_of_day(545, buf, sizeof(buf)));
    EXPECT_STREQ(buf, "09:05");
    int m = 0;
    ASSERT_TRUE(parse_time_of_day(buf, &m));
    EXPECT_EQ(m, 545);
    EXPECT_FALSE(format_time_of_day(1440, buf, sizeof(buf)));
    EXPECT_FALSE(format_time_of_day(0, buf, 5));
}

// ---------------- classify_wake ----------------

using espid::power::classify_wake;
using espid::power::WakeInputs;
using espid::power::WakeReason;

WakeInputs slept() {
    WakeInputs in{};
    in.from_deep_sleep = true;
    in.context_valid   = true;
    return in;
}

TEST(WakeReason, ResetOrUnknownContextIsAPowerOn) {
    EXPECT_EQ(classify_wake(WakeInputs{}), WakeReason::kPowerOn);
    WakeInputs in = slept();
    in.context_valid = false;
    in.by_timer      = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kPowerOn);
}

TEST(WakeReason, SwitchLineWakeIsAttributedByTheLevelThatMoved) {
    WakeInputs in = slept();
    in.by_switch_line = true;
    in.steam_changed  = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kSteamSwitch);
    in.brew_changed = true;  // both moved: brew first
    EXPECT_EQ(classify_wake(in), WakeReason::kBrewSwitch);
}

// 2026-10-05 07:29: woke through the switch lines, nobody touched anything.
TEST(WakeReason, SwitchLineWakeWithNoSwitchMovedIsSpurious) {
    WakeInputs in = slept();
    in.by_switch_line = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kSpurious);
}

TEST(WakeReason, TimerWakeIsScheduleOrResync) {
    WakeInputs in = slept();
    in.by_timer = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kSchedule);
    in.resync_leg = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kClockResync);
}

TEST(WakeReason, AMovedSwitchWinsOverTheTimer) {
    WakeInputs in = slept();
    in.by_timer     = true;
    in.resync_leg   = true;
    in.brew_changed = true;
    EXPECT_EQ(classify_wake(in), WakeReason::kBrewSwitch);
}

TEST(WakeReason, NoKnownSourceIsOther) {
    EXPECT_EQ(classify_wake(slept()), WakeReason::kOther);
}

TEST(WakeReason, StableNames) {
    EXPECT_STREQ(espid::power::to_string(WakeReason::kSpurious), "spurious");
    EXPECT_STREQ(espid::power::to_string(WakeReason::kClockResync), "clock_resync");
}

}  // namespace
