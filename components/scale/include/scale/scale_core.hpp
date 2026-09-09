// ScaleCore -- pure dual-cell weight computation for the drip-tray scale.
//
// Two HX711 amplifiers (one load cell per tray side) each report a signed
// raw count. This core converts raw counts to grams via a per-cell
// calibration factor (counts-per-gram) and a per-cell tare offset, then
// sums the two cells into a total weight. No ESP-IDF dependencies; the
// HX711 bit-bang driver feeds raw counts in via update().
//
// Not thread-safe. Drive from a single task.

#pragma once

#include <cmath>
#include <cstdint>

namespace espid::scale {

class ScaleCore {
   public:
    struct Config {
        float calibration_factor1 = 1.0F;  // raw counts per gram, cell 1
        float calibration_factor2 = 1.0F;  // raw counts per gram, cell 2
    };

    ScaleCore() = default;
    explicit ScaleCore(const Config& cfg) noexcept { set_config(cfg); }

    // Rejects non-finite or zero factors (would divide by zero).
    bool set_config(const Config& cfg) noexcept {
        if (!valid_factor_(cfg.calibration_factor1) ||
            !valid_factor_(cfg.calibration_factor2)) {
            return false;
        }
        cfg_ = cfg;
        return true;
    }

    const Config& config() const noexcept { return cfg_; }

    // Store the latest raw counts from both cells.
    void update(std::int32_t raw1, std::int32_t raw2) noexcept {
        raw1_ = raw1;
        raw2_ = raw2;
    }

    // Zero the scale: capture the current raw counts as the tare offsets.
    void tare() noexcept {
        offset1_ = raw1_;
        offset2_ = raw2_;
    }

    float channel1_g() const noexcept {
        return static_cast<float>(raw1_ - offset1_) / cfg_.calibration_factor1;
    }

    float channel2_g() const noexcept {
        return static_cast<float>(raw2_ - offset2_) / cfg_.calibration_factor2;
    }

    float weight_g() const noexcept { return channel1_g() + channel2_g(); }

   private:
    static bool valid_factor_(float f) noexcept {
        return std::isfinite(f) && f != 0.0F;
    }

    Config       cfg_{};
    std::int32_t raw1_{0};
    std::int32_t raw2_{0};
    std::int32_t offset1_{0};
    std::int32_t offset2_{0};
};

}  // namespace espid::scale
