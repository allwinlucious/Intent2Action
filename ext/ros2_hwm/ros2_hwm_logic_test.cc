#include "ext/ros2_hwm/ros2_hwm_logic.h"

#include <array>
#include <cmath>
#include <numbers>
#include <string>

#include "gtest/gtest.h"

namespace ros2_hwm {
namespace {

constexpr double EPS = 1e-12;

TEST(RotateHandToFlange, MatchesRzMinusQuarterTurn) {
  const double c = std::cos(-std::numbers::pi / 4);
  const double s = std::sin(-std::numbers::pi / 4);
  const Wrench hand = {1.0, 2.0, 3.0, -4.0, 5.0, -6.0};
  const Wrench flange = rotateHandToFlange(hand);
  for (int offset : {0, 3}) {
    const double x = hand[offset], y = hand[offset + 1];
    EXPECT_NEAR(flange[offset], c * x - s * y, EPS);
    EXPECT_NEAR(flange[offset + 1], s * x + c * y, EPS);
    EXPECT_NEAR(flange[offset + 2], hand[offset + 2], EPS);
  }
}

TEST(RotateHandToFlange, HandXIsBetweenFlangeXAndMinusY) {
  const Wrench flange = rotateHandToFlange({1.0, 0, 0, 0, 0, 0});
  EXPECT_NEAR(flange[0], std::numbers::sqrt2 / 2, EPS);
  EXPECT_NEAR(flange[1], -std::numbers::sqrt2 / 2, EPS);
}

TEST(Gripper, WidthIsTwiceFinger) {
  EXPECT_DOUBLE_EQ(gripperWidth(0.0381), 0.0762);
  EXPECT_DOUBLE_EQ(fingerPosition(0.075), 0.0375);
  EXPECT_FALSE((GripperCommand{0.0, 30.0} == GripperCommand{0.075, 30.0}));
}

TEST(FiniteDifference, DividesByDtAndGuardsZero) {
  EXPECT_DOUBLE_EQ(finiteDifference(1.0, 0.5, 0.004), 125.0);
  EXPECT_DOUBLE_EQ(finiteDifference(1.0, 0.5, 0.0), 0.0);
}

TEST(StampsMatch, WithinToleranceBothWays) {
  EXPECT_TRUE(stampsMatch(1000, 1999, 1000));
  EXPECT_TRUE(stampsMatch(1999, 1000, 1000));
  EXPECT_FALSE(stampsMatch(1000, 2001, 1000));
}

TEST(MapJointIndices, FindsEachNameOrFails) {
  const std::array<std::string, 3> names = {"b", "a", "c"};
  const std::array<std::string, 2> wanted = {"a", "c"};
  std::array<int, 2> indices{};
  ASSERT_TRUE(mapJointIndices(names, wanted, indices));
  EXPECT_EQ(indices[0], 1);
  EXPECT_EQ(indices[1], 2);
  const std::array<std::string, 1> missing = {"d"};
  std::array<int, 1> none{};
  EXPECT_FALSE(mapJointIndices(names, missing, none));
}

StateSample stateAt(int64_t stamp_ns) {
  StateSample s;
  s.stamp_ns = stamp_ns;
  s.position[0] = static_cast<double>(stamp_ns);
  return s;
}

TEST(StampPairer, PairsInEitherOrder) {
  StampPairer pairer(1000);
  EXPECT_FALSE(pairer.addState(stateAt(4'000'000)).has_value());
  auto entry = pairer.addWrench(4'000'500, {1, 0, 0, 0, 0, 0});
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->state.stamp_ns, 4'000'000);
  EXPECT_EQ(entry->wrench[0], 1);

  EXPECT_FALSE(pairer.addWrench(8'000'000, {2, 0, 0, 0, 0, 0}).has_value());
  entry = pairer.addState(stateAt(8'000'000));
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->wrench[0], 2);
}

TEST(StampPairer, UsesEachMessageOnceAndIgnoresFarStamps) {
  StampPairer pairer(1000);
  pairer.addState(stateAt(4'000'000));
  EXPECT_FALSE(pairer.addWrench(5'000'000, {}).has_value());
  ASSERT_TRUE(pairer.addWrench(4'000'000, {}).has_value());
  // The state is consumed, so a second wrench with its stamp waits.
  EXPECT_FALSE(pairer.addWrench(4'000'000, {}).has_value());
  EXPECT_FALSE(pairer.addState(stateAt(9'000'000)).has_value());
}

TEST(StampPairer, KeepsOnlyLatestUnpaired) {
  StampPairer pairer(0);
  for (int64_t i = 1; i <= 5; ++i) pairer.addState(stateAt(i));
  EXPECT_FALSE(pairer.addWrench(1, {}).has_value());  // overwritten
  EXPECT_TRUE(pairer.addWrench(5, {}).has_value());
  pairer.clear();
  EXPECT_FALSE(pairer.addWrench(4, {}).has_value());
}

TEST(DropOldestQueue, DropsOldestWhenFull) {
  DropOldestQueue<int, 2> queue;
  EXPECT_TRUE(queue.empty());
  EXPECT_FALSE(queue.push(1));
  EXPECT_FALSE(queue.push(2));
  EXPECT_TRUE(queue.push(3));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.front(), 2);
  queue.pop();
  EXPECT_EQ(queue.front(), 3);
  queue.pop();
  EXPECT_TRUE(queue.empty());
  queue.push(4);
  queue.clear();
  EXPECT_TRUE(queue.empty());
}

TEST(Taring, StartsCompletedWithZeroBias) {
  TaringState taring;
  EXPECT_TRUE(taring.completed);
  EXPECT_EQ(applyTaring(taring, {1, 2, 3, 4, 5, 6})[2], 3);
}

TEST(Taring, AveragesNextCyclesThenSubtracts) {
  TaringState taring;
  startTaring(taring, 2);
  EXPECT_FALSE(taring.completed);
  applyTaring(taring, {1, 0, 0, 0, 0, 2});
  EXPECT_FALSE(taring.completed);
  const Wrench last = applyTaring(taring, {3, 0, 0, 0, 0, 4});
  EXPECT_TRUE(taring.completed);
  EXPECT_DOUBLE_EQ(taring.bias[0], 2.0);
  EXPECT_DOUBLE_EQ(taring.bias[5], 3.0);
  EXPECT_DOUBLE_EQ(last[0], 1.0);
  const Wrench after = applyTaring(taring, {2, 0, 0, 0, 0, 3});
  EXPECT_DOUBLE_EQ(after[0], 0.0);
  EXPECT_DOUBLE_EQ(after[5], 0.0);
}

TEST(Taring, NonPositiveCyclesMeansOne) {
  TaringState taring;
  startTaring(taring, 0);
  applyTaring(taring, {5, 0, 0, 0, 0, 0});
  EXPECT_TRUE(taring.completed);
  EXPECT_DOUBLE_EQ(taring.bias[0], 5.0);
}

}  // namespace
}  // namespace ros2_hwm
