#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <utility>
#include <units/angle.h>
#include <units/length.h>
#include "Neo.h"

using namespace ::testing;

using ::testing::A;
using ::testing::An;
using ::testing::NotNull;
using ::testing::Matcher;
using ::testing::WhenDynamicCastTo;
using ::testing::Eq;
using ::testing::DoubleEq;

TEST(DegreeTo550UnitTest, ConvertsDegreesToNeo550Units) {
  EXPECT_THAT(t34::Neo::DegreeTo550Unit(180_deg), DoubleEq(21.0));
  EXPECT_THAT(t34::Neo::DegreeTo550Unit(360_deg), DoubleEq(42.0));
}

// class DegreeTo550UnitTest : public TestWithParam<std::pair<units::degree_t, double>> {};

// TEST_P(DegreeTo550UnitTest, ConvertsDegreesToNeo550Units) {
//   auto [degrees, expected] = GetParam();

//   EXPECT_THAT(t34::Neo::DegreeTo550Unit(degrees), DoubleEq(expected));
// }

// INSTANTIATE_TEST_SUITE_P(NeoTests, DegreeTo550UnitTest, Values(
//   std::make_pair(180_deg, 21.0),
//   std::make_pair(360_deg, 42.0)
// ));


TEST(InchTo550UnitTest, ConvertsInchesToNeo550Units) {
  EXPECT_THAT(t34::Neo::InchTo550Unit(0.0625_in * M_PI), DoubleEq(21.0));
  EXPECT_THAT(t34::Neo::InchTo550Unit(0.1250_in * M_PI), DoubleEq(42.0));
}

// class InchTo550UnitTest : public TestWithParam<std::pair<units::inch_t, double>> {};

// TEST_P(InchTo550UnitTest, ConvertsInchTo550Unit) {
//   auto [inches, expected] = GetParam();

//   EXPECT_THAT(t34::Neo::InchTo550Unit(inches), DoubleEq(expected));
// }

// INSTANTIATE_TEST_SUITE_P(NeoTests, InchTo550UnitTest, Values(
//   std::make_pair(0.0625_in * M_PI,  42.0),
//   std::make_pair(0.1250_in * M_PI,  42.0)
// ));
