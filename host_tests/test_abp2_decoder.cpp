// ESPID-Coffee — ABP2 decoder unit tests.

#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "abp2/abp2_decoder.hpp"

namespace {

using espid::abp2::DecodedSample;
using espid::abp2::FrameStatus;
using espid::abp2::TransferFunction;
using espid::abp2::decode_frame;
using espid::hal_cpp::Error;

std::array<std::uint8_t, 7> make_frame(FrameStatus status,
                                       std::uint32_t pressure_counts,
                                       std::uint32_t temperature_counts,
                                       std::uint8_t extra_status_bits = 0U) {
    return {
        static_cast<std::uint8_t>(
            (static_cast<std::uint8_t>(status) << 6) | extra_status_bits),
        static_cast<std::uint8_t>((pressure_counts >> 16) & 0xFFU),
        static_cast<std::uint8_t>((pressure_counts >> 8) & 0xFFU),
        static_cast<std::uint8_t>(pressure_counts & 0xFFU),
        static_cast<std::uint8_t>((temperature_counts >> 16) & 0xFFU),
        static_cast<std::uint8_t>((temperature_counts >> 8) & 0xFFU),
        static_cast<std::uint8_t>(temperature_counts & 0xFFU),
    };
}

constexpr float kEpsPsi = 0.05F;
constexpr float kEpsTemp = 0.2F;

TEST(Abp2Decoder, DecodesNominalFrame) {
    const auto raw = make_frame(FrameStatus::kPowered, 8'388'608, 8'388'608);
    const TransferFunction tf{};

    auto r = decode_frame(raw.data(), tf);
    ASSERT_TRUE(r.ok()) << static_cast<int>(r.error());

    const DecodedSample s = r.value();
    EXPECT_EQ(s.status, FrameStatus::kPowered);
    EXPECT_FALSE(s.busy);
    EXPECT_EQ(s.pressure_counts, 8'388'608U);
    EXPECT_EQ(s.temperature_counts, 8'388'608U);
    EXPECT_NEAR(s.pressure_psi, 15.0F, kEpsPsi);
    EXPECT_NEAR(s.pressure_bar, 1.034F, 0.01F);
    EXPECT_NEAR(s.temperature_c, 95.0F, kEpsTemp);
}

TEST(Abp2Decoder, BusyBitIsReportedButStillDecodes) {
    const auto raw = make_frame(FrameStatus::kPowered, 7'000'000, 9'000'000,
                                /*extra_status_bits=*/0x20U);
    auto r = decode_frame(raw.data(), TransferFunction{});
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value().status, FrameStatus::kPowered);
    EXPECT_TRUE(r.value().busy);
}

// An absent or garbled response reads back as all zeros. It must not decode as
// a valid 0 bar sample: the pump PID would act on it as a real reading, and the
// startup fill guard would treat it as a pressure measurement.
TEST(Abp2Decoder, AllZeroFrameRejected) {
    const std::array<std::uint8_t, 7> raw{};
    auto r = decode_frame(raw.data(), TransferFunction{});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kBusError);
}

TEST(Abp2Decoder, UnpoweredStatusRejectedEvenWithPlausibleCounts) {
    const auto raw = make_frame(FrameStatus::kUnpowered, 8'388'608, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kBusError);
}

TEST(Abp2Decoder, ReservedStatusRejected) {
    const auto raw = make_frame(FrameStatus::kReserved, 8'388'608, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kBusError);
}

TEST(Abp2Decoder, CountsFarBelowFloorRejected) {
    // Zero counts on an otherwise healthy frame are not a reading the sensor
    // produces; only a small undershoot of the floor is legitimate.
    const auto raw = make_frame(FrameStatus::kPowered, 0, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kRangeError);
}

TEST(Abp2Decoder, DiagnosticStatusReturnsHardwareFault) {
    const auto raw = make_frame(FrameStatus::kDiagnostic, 8'388'608, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kHardwareFault);
}

TEST(Abp2Decoder, BelowMinCountsClampedToZeroPressure) {
    // Counts below the 10% floor clamp to 0 bar instead of error
    // (atmospheric pressure at rest sits just under the nominal minimum).
    const auto raw = make_frame(FrameStatus::kPowered, 1'000'000, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    EXPECT_TRUE(r.ok());
    EXPECT_FLOAT_EQ(r.value().pressure_psi, 0.0F);
    EXPECT_FLOAT_EQ(r.value().pressure_bar, 0.0F);
}

TEST(Abp2Decoder, AboveMaxCountsReturnsRangeError) {
    const auto raw = make_frame(FrameStatus::kPowered, 16'000'000, 8'388'608);
    auto r = decode_frame(raw.data(), TransferFunction{});
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kRangeError);
}

TEST(Abp2Decoder, InvalidTransferConfigReturnsInvalidArg) {
    const auto raw = make_frame(FrameStatus::kPowered, 8'388'608, 8'388'608);
    TransferFunction tf{};
    tf.output_min_counts = tf.output_max_counts;

    auto r = decode_frame(raw.data(), tf);
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.error(), Error::kInvalidArg);
}

}  // namespace
