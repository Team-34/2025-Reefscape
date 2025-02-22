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


class DegreeTo550UnitTest : public TestWithParam<std::pair<units::degree_t, double>> {};

TEST_P(DegreeTo550UnitTest, ConvertsDegreesToNeo550Units) {
  auto [degrees, expected] = GetParam();

  EXPECT_THAT(t34::Neo::DegreeTo550Unit(degrees), DoubleEq(expected));
}

INSTANTIATE_TEST_SUITE_P(NeoTests, DegreeTo550UnitTest, Values(
  std::make_pair( 90_deg, 10.5),
  std::make_pair(180_deg, 21.0),
  std::make_pair(270_deg, 31.5),
  std::make_pair(360_deg, 42.0)
));


class InchTo550UnitTest : public TestWithParam<std::pair<units::inch_t, double>> {};

TEST_P(InchTo550UnitTest, ConvertsInchTo550Unit) {
  auto [inches, expected] = GetParam();

  EXPECT_THAT(t34::Neo::InchTo550Unit(inches), DoubleEq(expected));
}

INSTANTIATE_TEST_SUITE_P(NeoTests, InchTo550UnitTest, Values(
  std::make_pair(0.03125_in * M_PI, 10.5),
  std::make_pair(0.0625_in * M_PI,  21.0),
  std::make_pair(0.09375_in * M_PI, 31.5),
  std::make_pair(0.1250_in * M_PI,  42.0)
));
