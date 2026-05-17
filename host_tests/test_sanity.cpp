// Sanity check — proves the gtest harness builds and runs on the host.
#include <gtest/gtest.h>

TEST(Sanity, ArithmeticIsBoring) {
    EXPECT_EQ(2 + 2, 4);
}
