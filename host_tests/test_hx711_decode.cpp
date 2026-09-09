// ESPID-Coffee -- scale::sign_extend24 unit tests.

#include <cstdint>

#include <gtest/gtest.h>

#include "scale/hx711_decode.hpp"

namespace {

using espid::scale::sign_extend24;

TEST(Hx711Decode, ZeroIsZero) {
    EXPECT_EQ(sign_extend24(0x000000U), 0);
}

TEST(Hx711Decode, PositiveMaxIsHalfScaleMinusOne) {
    EXPECT_EQ(sign_extend24(0x7FFFFFU), 8'388'607);  // 2^23 - 1
}

TEST(Hx711Decode, MostSignificantBitIsNegative) {
    EXPECT_EQ(sign_extend24(0x800000U), -8'388'608);  // -2^23
}

TEST(Hx711Decode, AllOnesIsMinusOne) {
    EXPECT_EQ(sign_extend24(0xFFFFFFU), -1);
}

TEST(Hx711Decode, NegativeMax) {
    EXPECT_EQ(sign_extend24(0x800001U), -8'388'607);
}

TEST(Hx711Decode, IgnoresBitsAboveBit23) {
    // Stray high bits (e.g. left over from a wider shift) must not leak in.
    EXPECT_EQ(sign_extend24(0xFF000001U), 1);
    EXPECT_EQ(sign_extend24(0xAB800000U), -8'388'608);
}

}  // namespace
