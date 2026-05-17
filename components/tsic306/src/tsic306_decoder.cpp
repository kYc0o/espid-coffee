#include "tsic306/tsic306_decoder.hpp"

#include <cmath>

namespace espid::tsic306 {

namespace {
constexpr std::uint8_t kPacket1Ok = 0b01;
constexpr std::uint8_t kPacket2Ok = 0b10;
constexpr std::uint8_t kBothOk    = kPacket1Ok | kPacket2Ok;

constexpr int   kRawMax   = 2047;  // 11-bit
constexpr float kSpanC    = 200.0F;
constexpr float kOffsetC  = -50.0F;
}  // namespace

hal_cpp::Result<float> Tsic306Decoder::decode(const hal_cpp::ZacwireFrame& frame) noexcept {
    if ((frame.parity_mask & kBothOk) != kBothOk) {
        return hal_cpp::Error::kCrcMismatch;
    }

    // Raw value: high 8 bits from packet1 (upper 3 bits used → 11-bit total),
    // low 8 bits from packet2. ZACwire is MSB-first; capture layer packs
    // packet1 in the high byte of `raw`.
    const std::uint16_t raw11 = frame.raw & 0x07FFU;  // mask to 11 bits

    const float celsius =
        (static_cast<float>(raw11) / static_cast<float>(kRawMax)) * kSpanC + kOffsetC;

    if (celsius < cfg_.min_valid_c || celsius > cfg_.max_valid_c) {
        return hal_cpp::Error::kRangeError;
    }

    if (cfg_.max_change_rate_c > 0.0F && last_value_.has_value()) {
        const float delta = std::fabs(celsius - *last_value_);
        if (delta > cfg_.max_change_rate_c) {
            return hal_cpp::Error::kRangeError;
        }
    }

    last_value_ = celsius;
    return celsius;
}

}  // namespace espid::tsic306
