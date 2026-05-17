// ESPID-Coffee — Tsic306Decoder unit tests.
//
// We construct synthetic ZacwireFrame values and verify:
//   - the transfer function maps 0/half/full-scale correctly
//   - bad parity → kCrcMismatch
//   - out-of-range raw → kRangeError
//   - rate-of-change filter rejects implausible jumps
//   - reset() clears the rate filter so a fresh value is accepted

#include <gtest/gtest.h>

#include "hal_cpp/i_zacwire_capture.hpp"
#include "tsic306/tsic306_decoder.hpp"

namespace {

using espid::hal_cpp::Error;
using espid::hal_cpp::ZacwireFrame;
using espid::tsic306::DecoderConfig;
using espid::tsic306::Tsic306Decoder;

constexpr std::uint8_t kBothParityOk = 0b11;

ZacwireFrame frame_with_raw(std::uint16_t raw11, std::uint8_t parity = kBothParityOk) {
    return ZacwireFrame{
        .raw          = static_cast<std::uint16_t>(raw11 & 0x07FFU),
        .parity_mask  = parity,
        .timestamp_us = 0U,
    };
}

constexpr float kEps = 0.1F;  // °C

TEST(Tsic306Decoder, ZeroRawDecodesToMinusFifty) {
    Tsic306Decoder dec{};
    auto r = dec.decode(frame_with_raw(0));
    ASSERT_TRUE(r.ok()) << "got error " << static_cast<int>(r.error());
    EXPECT_NEAR(r.value(), -50.0F, kEps);
}

TEST(Tsic306Decoder, HalfScaleRawDecodesToFifty) {
    // raw=1024 → ~(1024/2047)*200 - 50 ≈ 50.07
    Tsic306Decoder dec{};
    auto r = dec.decode(frame_with_raw(1024));
    ASSERT_TRUE(r.ok());
    EXPECT_NEAR(r.value(), 50.07F, kEps);
}

TEST(Tsic306Decoder, FullScaleRawDecodesToOneFifty) {
    Tsic306Decoder dec{};
    auto r = dec.decode(frame_with_raw(2047));
    ASSERT_TRUE(r.ok());
    EXPECT_NEAR(r.value(), 150.0F, kEps);
}

TEST(Tsic306Decoder, BadParityReturnsCrcMismatch) {
    Tsic306Decoder dec{};
    auto r = dec.decode(frame_with_raw(1024, /*parity=*/0b10));  // packet1 bad
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kCrcMismatch);
    EXPECT_FALSE(dec.last_value().has_value());
}

TEST(Tsic306Decoder, RateOfChangeRejectsImplausibleJump) {
    Tsic306Decoder dec{DecoderConfig{.max_change_rate_c = 15.0F}};
    // First sample: 25 °C → raw ≈ (25 - (-50)) / 200 * 2047 ≈ 767
    ASSERT_TRUE(dec.decode(frame_with_raw(767)).ok());
    // Next sample claims 100 °C → raw ≈ 1535. Δ=75 > 15 → rejected.
    auto r = dec.decode(frame_with_raw(1535));
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kRangeError);
    // last_value should remain the previously good 25 °C.
    ASSERT_TRUE(dec.last_value().has_value());
    EXPECT_NEAR(*dec.last_value(), 25.0F, 1.0F);
}

TEST(Tsic306Decoder, RateOfChangeAcceptsGradualDrift) {
    Tsic306Decoder dec{};
    ASSERT_TRUE(dec.decode(frame_with_raw(767)).ok());      // 25 °C
    auto r = dec.decode(frame_with_raw(820));               // ~30 °C, Δ≈5
    ASSERT_TRUE(r.ok());
    EXPECT_NEAR(r.value(), 30.16F, 0.5F);
}

TEST(Tsic306Decoder, ResetClearsRateFilter) {
    Tsic306Decoder dec{};
    ASSERT_TRUE(dec.decode(frame_with_raw(767)).ok());      // 25 °C
    dec.reset();
    // After reset, a big jump should be accepted because there's no prior.
    auto r = dec.decode(frame_with_raw(1535));              // ~100 °C
    ASSERT_TRUE(r.ok());
    EXPECT_NEAR(r.value(), 100.0F, 1.0F);
}

TEST(Tsic306Decoder, OutOfRangeBelowMinReturnsRangeError) {
    Tsic306Decoder dec{DecoderConfig{
        .max_change_rate_c = 0.0F,  // disabled
        .min_valid_c       = 0.0F,
        .max_valid_c       = 150.0F,
    }};
    auto r = dec.decode(frame_with_raw(0));  // -50 °C, below min
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kRangeError);
}

}  // namespace
