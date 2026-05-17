// ESPID-Coffee — brew button + shot timer unit tests.

#include <gtest/gtest.h>

#include "brew/brew_button.hpp"
#include "brew/shot_timer.hpp"

namespace {

using espid::brew::BrewButton;
using espid::brew::ShotTimer;

// ---------------- BrewButton ----------------

TEST(BrewButton, StartsReleased) {
    BrewButton btn;
    EXPECT_FALSE(btn.is_pressed());
    // Steady high (released, active-low) → no event.
    EXPECT_EQ(btn.poll(true, 0), BrewButton::Event::kNone);
}

TEST(BrewButton, EmitsPressedAfterDebounceWindow) {
    BrewButton::Config c{/*active_low=*/true, /*debounce_ms=*/25};
    BrewButton         btn(c);

    // Press at t=0; still within debounce window at t=24.
    EXPECT_EQ(btn.poll(false, 0),  BrewButton::Event::kNone);
    EXPECT_EQ(btn.poll(false, 10), BrewButton::Event::kNone);
    EXPECT_EQ(btn.poll(false, 24), BrewButton::Event::kNone);
    EXPECT_FALSE(btn.is_pressed());

    // At t=25 the candidate has held long enough → promoted.
    EXPECT_EQ(btn.poll(false, 25), BrewButton::Event::kPressed);
    EXPECT_TRUE(btn.is_pressed());

    // Further low samples produce no further events.
    EXPECT_EQ(btn.poll(false, 30), BrewButton::Event::kNone);
}

TEST(BrewButton, RejectsShortGlitch) {
    BrewButton::Config c{/*active_low=*/true, /*debounce_ms=*/25};
    BrewButton         btn(c);

    EXPECT_EQ(btn.poll(false, 0),  BrewButton::Event::kNone);   // glitch starts
    EXPECT_EQ(btn.poll(false, 10), BrewButton::Event::kNone);   // still pending
    EXPECT_EQ(btn.poll(true,  15), BrewButton::Event::kNone);   // glitch ends early
    EXPECT_FALSE(btn.is_pressed());

    // No spurious press after the glitch.
    EXPECT_EQ(btn.poll(true, 100), BrewButton::Event::kNone);
}

TEST(BrewButton, EmitsReleasedAfterDebounceWindow) {
    BrewButton::Config c{/*active_low=*/true, /*debounce_ms=*/25};
    BrewButton         btn(c);

    // Establish pressed.
    (void)btn.poll(false, 0);
    EXPECT_EQ(btn.poll(false, 25), BrewButton::Event::kPressed);

    // Begin release at t=100; promote at t=125.
    EXPECT_EQ(btn.poll(true, 100), BrewButton::Event::kNone);
    EXPECT_EQ(btn.poll(true, 124), BrewButton::Event::kNone);
    EXPECT_EQ(btn.poll(true, 125), BrewButton::Event::kReleased);
    EXPECT_FALSE(btn.is_pressed());
}

TEST(BrewButton, ActiveHighInverts) {
    BrewButton::Config c{/*active_low=*/false, /*debounce_ms=*/10};
    BrewButton         btn(c);

    // High = pressed.
    EXPECT_EQ(btn.poll(true, 0),  BrewButton::Event::kNone);
    EXPECT_EQ(btn.poll(true, 10), BrewButton::Event::kPressed);
}

TEST(BrewButton, ResetReturnsToReleasedNoCandidate) {
    BrewButton::Config c{/*active_low=*/true, /*debounce_ms=*/25};
    BrewButton         btn(c);

    (void)btn.poll(false, 0);
    (void)btn.poll(false, 25);
    EXPECT_TRUE(btn.is_pressed());

    btn.reset();
    EXPECT_FALSE(btn.is_pressed());
    // Subsequent steady-high samples must not emit a spurious release.
    EXPECT_EQ(btn.poll(true, 100), BrewButton::Event::kNone);
}

// ---------------- ShotTimer ----------------

TEST(ShotTimer, StartsIdleAtZero) {
    ShotTimer t;
    EXPECT_FALSE(t.running());
    EXPECT_EQ(t.elapsed_ms(0),     0U);
    EXPECT_EQ(t.elapsed_ms(10000), 0U);
}

TEST(ShotTimer, CountsWhileRunning) {
    ShotTimer t;
    t.start(1000);
    EXPECT_TRUE(t.running());
    EXPECT_EQ(t.elapsed_ms(1000), 0U);
    EXPECT_EQ(t.elapsed_ms(1500), 500U);
    EXPECT_EQ(t.elapsed_ms(8000), 7000U);
}

TEST(ShotTimer, StopFreezesElapsed) {
    ShotTimer t;
    t.start(0);
    t.stop(3000);
    EXPECT_FALSE(t.running());
    EXPECT_EQ(t.elapsed_ms(3000),   3000U);
    EXPECT_EQ(t.elapsed_ms(99'999), 3000U);
}

TEST(ShotTimer, ResumesFromAccumulated) {
    ShotTimer t;
    t.start(0);
    t.stop(2000);                          // 2 s banked
    t.start(10'000);                       // resume
    EXPECT_EQ(t.elapsed_ms(10'500), 2500U);
    t.stop(11'000);
    EXPECT_EQ(t.elapsed_ms(99'999), 3000U);
}

TEST(ShotTimer, ResetClearsEverything) {
    ShotTimer t;
    t.start(0);
    t.stop(5000);
    EXPECT_EQ(t.elapsed_ms(5000), 5000U);
    t.reset();
    EXPECT_FALSE(t.running());
    EXPECT_EQ(t.elapsed_ms(5000), 0U);
}

TEST(ShotTimer, DoubleStartIsIdempotent) {
    ShotTimer t;
    t.start(1000);
    t.start(5000);                         // ignored
    EXPECT_EQ(t.elapsed_ms(6000), 5000U);  // counts from the first start
}

TEST(ShotTimer, DoubleStopIsIdempotent) {
    ShotTimer t;
    t.start(0);
    t.stop(1000);
    t.stop(9999);                          // ignored
    EXPECT_EQ(t.elapsed_ms(9999), 1000U);
}

}  // namespace
