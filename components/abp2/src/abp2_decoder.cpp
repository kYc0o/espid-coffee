#include "abp2/abp2_decoder.hpp"

#include <cmath>

namespace espid::abp2 {

namespace {

constexpr float kPsiToBar = 0.0689476F;
constexpr float kTempScaleC = 270.0F;
constexpr float kTempOffsetC = -40.0F;
constexpr float kCountsMax24 = 16'777'215.0F;

bool valid_transfer(const TransferFunction& tf) noexcept {
    if (tf.output_max_counts <= tf.output_min_counts) {
        return false;
    }
    if (!(std::isfinite(tf.pressure_min_psi) && std::isfinite(tf.pressure_max_psi))) {
        return false;
    }
    if (tf.pressure_max_psi <= tf.pressure_min_psi) {
        return false;
    }
    return true;
}

}  // namespace

hal_cpp::Result<DecodedSample> decode_frame(const std::uint8_t raw[7],
                                            const TransferFunction& tf) noexcept {
    if (raw == nullptr || !valid_transfer(tf)) {
        return hal_cpp::Error::kInvalidArg;
    }

    const auto status = static_cast<FrameStatus>((raw[0] >> 6) & 0x03U);
    if (status == FrameStatus::kDiagnostic) {
        return hal_cpp::Error::kHardwareFault;
    }

    // ABP2 command-response payload uses 24-bit pressure and temperature words.
    const std::uint32_t pressure_counts =
        static_cast<std::uint32_t>((static_cast<std::uint32_t>(raw[1]) << 16) |
                                   (static_cast<std::uint32_t>(raw[2]) << 8) |
                                   static_cast<std::uint32_t>(raw[3]));
    const std::uint32_t temperature_counts =
        static_cast<std::uint32_t>((static_cast<std::uint32_t>(raw[4]) << 16) |
                                   (static_cast<std::uint32_t>(raw[5]) << 8) |
                                   static_cast<std::uint32_t>(raw[6]));

    // Clamp counts below the calibrated floor to 0 (minimum) pressure:
    // at atmospheric/zero gauge the sensor naturally reads slightly under the
    // nominal 10% output span floor. Treat above-max as a hard range error.
    const std::uint32_t clamped_counts =
        (pressure_counts < tf.output_min_counts) ? tf.output_min_counts
                                                  : pressure_counts;
    if (clamped_counts > tf.output_max_counts ||
        static_cast<float>(temperature_counts) > kCountsMax24) {
        return hal_cpp::Error::kRangeError;
    }

    const float counts_span =
        static_cast<float>(tf.output_max_counts - tf.output_min_counts);
    const float psi_span = tf.pressure_max_psi - tf.pressure_min_psi;
    const float pressure_psi =
        (static_cast<float>(clamped_counts - tf.output_min_counts) * psi_span /
         counts_span) +
        tf.pressure_min_psi;
    const float pressure_bar = pressure_psi * kPsiToBar;
    const float temperature_c =
        (static_cast<float>(temperature_counts) * kTempScaleC / kCountsMax24) +
        kTempOffsetC;

    return DecodedSample{
        .status = status,
        .pressure_counts = pressure_counts,
        .temperature_counts = temperature_counts,
        .pressure_psi = pressure_psi,
        .pressure_bar = pressure_bar,
        .temperature_c = temperature_c,
    };
}

}  // namespace espid::abp2
