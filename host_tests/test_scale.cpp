// ESPID-Coffee -- scale::ScaleCore unit tests.

#include <gtest/gtest.h>

#include "scale/scale_core.hpp"

namespace {

using espid::scale::ScaleCore;

TEST(ScaleCore, RejectsZeroOrNonFiniteFactors) {
    ScaleCore core;
    EXPECT_FALSE(core.set_config({0.0F, 1.0F}));
    EXPECT_FALSE(core.set_config({1.0F, 0.0F}));
    EXPECT_TRUE(core.set_config({420.0F, 420.0F}));
}

TEST(ScaleCore, SumsBothCellsWithCalibration) {
    ScaleCore core({100.0F, 200.0F});  // counts per gram, per cell
    core.update(1000, 4000);           // 10 g + 20 g

    EXPECT_FLOAT_EQ(core.channel1_g(), 10.0F);
    EXPECT_FLOAT_EQ(core.channel2_g(), 20.0F);
    EXPECT_FLOAT_EQ(core.weight_g(), 30.0F);
}

TEST(ScaleCore, TareZeroesCurrentReading) {
    ScaleCore core({100.0F, 100.0F});
    core.update(5000, 8000);
    core.tare();

    EXPECT_FLOAT_EQ(core.weight_g(), 0.0F);

    core.update(5500, 8500);  // +5 g on each cell
    EXPECT_FLOAT_EQ(core.weight_g(), 10.0F);
}

TEST(ScaleCore, HandlesNegativeNetWeight) {
    ScaleCore core({100.0F, 100.0F});
    core.update(2000, 2000);
    core.tare();

    core.update(1000, 2000);  // cell 1 drops 10 g
    EXPECT_FLOAT_EQ(core.weight_g(), -10.0F);
}

TEST(ScaleCore, DefaultFactorsAreUnity) {
    ScaleCore core;
    core.update(3, 4);
    EXPECT_FLOAT_EQ(core.weight_g(), 7.0F);
}

}  // namespace
