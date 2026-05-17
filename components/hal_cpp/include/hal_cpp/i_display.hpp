// IDisplay — minimal text-oriented display contract.
//
// Designed for monochrome character/pixel displays (e.g. SSD1306, SH1106).
// Keeps the interface free of vendor types so any concrete driver can
// implement it without leaking u8g2/Adafruit/etc into the call sites.
//
// Lifecycle: init() once; then per-frame: clear() -> draw_text(...)+ -> flush().
//
// Coordinates are pixels with the origin (0,0) at the top-left.  draw_text
// uses (x, baseline_y) — `baseline_y` is the font baseline, matching the
// u8g2 convention so concrete drivers can pass it through unchanged.

#pragma once

#include <cstddef>
#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

// Logical font sizes.  Concrete drivers map these to specific font glyph
// sets (e.g. u8g2_font_6x10_tr / _9x15B_tr / _logisoso24_tn).
enum class Font : std::uint8_t {
    kSmall  = 0,  // ~6 px tall, body text
    kMedium = 1,  // ~10 px tall, labels
    kLarge  = 2,  // ~24 px tall, single big numeric readout
};

class IDisplay {
   public:
    virtual ~IDisplay() = default;

    // Initialise the controller and its transport.  Idempotent failure
    // returns kAlreadyInit; backend faults return kHardwareFault or
    // kBusError.
    virtual Status init() noexcept = 0;

    // Width / height in pixels.  Valid after a successful init().
    virtual std::uint16_t width()  const noexcept = 0;
    virtual std::uint16_t height() const noexcept = 0;

    // Clear the off-screen buffer.  Does not touch the panel; call flush()
    // to commit.
    virtual void clear() noexcept = 0;

    // Render `text` (null-terminated, ASCII) at (x, baseline_y) using
    // `font`.  Coordinates beyond the panel are clipped by the backend.
    virtual void draw_text(std::int16_t x, std::int16_t baseline_y,
                           Font font, const char* text) noexcept = 0;

    // Push the off-screen buffer to the panel.
    virtual Status flush() noexcept = 0;
};

}  // namespace espid::hal_cpp
