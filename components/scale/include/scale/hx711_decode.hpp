// HX711 raw-count decoding -- pure logic, host-testable.
//
// The HX711 outputs a 24-bit two's-complement conversion, MSB first. This
// helper sign-extends that 24-bit value into a full 32-bit signed integer.
// No ESP-IDF dependencies; shared between the driver and host_tests.

#pragma once

#include <cstdint>

namespace espid::scale {

// Sign-extend a 24-bit two's-complement value (low 24 bits of `v`) to
// int32. Bits above bit 23 in `v` are ignored.
inline std::int32_t sign_extend24(std::uint32_t v) noexcept {
    v &= 0x00FF'FFFFU;
    if ((v & 0x0080'0000U) != 0U) {
        v |= 0xFF00'0000U;
    }
    return static_cast<std::int32_t>(v);
}

}  // namespace espid::scale
