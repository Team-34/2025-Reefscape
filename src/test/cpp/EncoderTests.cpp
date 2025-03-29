#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <utility>
#include <units/angle.h>
#include <units/length.h>
#include "subsystems/Elevator.h"

using namespace ::testing;

using ::testing::A;
using ::testing::An;
using ::testing::NotNull;
using ::testing::Matcher;
using ::testing::WhenDynamicCastTo;
using ::testing::Eq;
using ::testing::DoubleEq;


class UpdateEncoderUnitTest : public TestWithParam<std::tuple<double, double, double, double>> {
  public:
  t34::Elevator test_elevator;
};

TEST_P(UpdateEncoderUnitTest, update_pos_handles_roll_over) {
  auto [acc, last, next, expected] = GetParam();

  EXPECT_THAT(test_elevator.UpdatePosition(acc, last, next), DoubleEq(expected));
}

INSTANTIATE_TEST_SUITE_P(EncoderTests, UpdateEncoderUnitTest, Values(
  std::make_tuple(0.9, 0.9, 0.1, 1.1),
  std::make_tuple(1.5, 0.1, 0.9, 1.3),
  std::make_tuple(15.6, 0.9, 0.15, 15.85),
  std::make_tuple(25.5, 0.5, 0.7, 25.7),
  std::make_tuple(-15.6, 0.9, 0.15, -15.35)
));