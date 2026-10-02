// Debounced momentary-button input.
//
// Pure logic class — no GPIO / FreeRTOS / ESP-IDF includes. The caller
// is responsible for sampling the raw line level and passing it (with a
// monotonic timestamp in ms) to `poll()` at any rate; a typical embedded
// caller polls every 5-20 ms from a task or timer.
//
// Debounce algorithm: a candidate level different from the current
// stable level must persist for `debounce_ms` before it is promoted.
// Glitches shorter than `debounce_ms` are rejected.
//
// Not thread-safe. Drive from a single task.

#pragma once

#include <cstdint>

namespace espid::brew {

class BrewButton {
   public:
    enum class Event : std::uint8_t {
        kNone     = 0,
        kPressed  = 1,
        kReleased = 2,
    };

    struct Config {
        bool          active_low  = true;  // momentary to GND, internal pull-up
        std::uint32_t debounce_ms = 25;
    };

    BrewButton() = default;
    explicit BrewButton(const Config& cfg) noexcept : cfg_(cfg) {}

    void          set_config(const Config& cfg) noexcept { cfg_ = cfg; }
    const Config& config() const noexcept                { return cfg_; }

    // Reset to the "released, no candidate" state.
    void reset() noexcept {
        stable_pressed_     = false;
        has_candidate_      = false;
        candidate_pressed_  = false;
        candidate_since_ms_ = 0;
    }

    // Take the current raw level as the stable state without emitting an
    // event. Call once before the first poll(): a latching switch that is
    // already on at boot is the baseline, so only a later off->on counts as
    // a press.
    void seed(bool raw_level) noexcept {
        stable_pressed_ = cfg_.active_low ? !raw_level : raw_level;
        has_candidate_  = false;
    }

    // Feed one raw sample. Returns kPressed/kReleased on the rising
    // edge of a debounced transition, otherwise kNone.
    Event poll(bool raw_level, std::uint64_t now_ms) noexcept {
        const bool raw_pressed = cfg_.active_low ? !raw_level : raw_level;

        if (raw_pressed == stable_pressed_) {
            // Raw matches stable — drop any pending candidate.
            has_candidate_ = false;
            return Event::kNone;
        }

        if (!has_candidate_ || raw_pressed != candidate_pressed_) {
            // Begin (or restart) a candidate window.
            has_candidate_      = true;
            candidate_pressed_  = raw_pressed;
            candidate_since_ms_ = now_ms;
            return Event::kNone;
        }

        if ((now_ms - candidate_since_ms_) < cfg_.debounce_ms) {
            return Event::kNone;
        }

        // Candidate has held long enough — promote.
        stable_pressed_ = candidate_pressed_;
        has_candidate_  = false;
        return stable_pressed_ ? Event::kPressed : Event::kReleased;
    }

    bool is_pressed() const noexcept { return stable_pressed_; }

   private:
    Config        cfg_{};
    bool          stable_pressed_     = false;
    bool          has_candidate_      = false;
    bool          candidate_pressed_  = false;
    std::uint64_t candidate_since_ms_ = 0;
};

}  // namespace espid::brew
