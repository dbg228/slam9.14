#include <limits>
#include "gtest/gtest.h"
#include "inspection_geometry_tools/ordered_path_progress.hpp"

using inspection_geometry_tools::OrderedPathProgress;

TEST(OrderedPathProgress, EndBeforeStartCannotComplete)
{
  OrderedPathProgress progress;
  progress.setPath({{0, 0}, {1, 0}, {2, 0}, {2, 2}}, .25);
  EXPECT_FALSE(progress.update(2, 2));
  EXPECT_EQ(progress.visited(), 0U);
}

TEST(OrderedPathProgress, ClosedLoopDoesNotFinishAtInitialEnd)
{
  OrderedPathProgress progress;
  progress.setPath({{-1, 0}, {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}}, .25);
  EXPECT_FALSE(progress.update(-1, 0));
  EXPECT_FALSE(progress.update(0, 0));
  EXPECT_FALSE(progress.update(0, 0));
  EXPECT_EQ(progress.visited(), 2U);
  EXPECT_FALSE(progress.update(1, 0));
  EXPECT_FALSE(progress.update(1, 1));
  EXPECT_FALSE(progress.update(0, 1));
  EXPECT_TRUE(progress.update(0, 0));
}

TEST(OrderedPathProgress, ShortcutCannotSkipMiddle)
{
  OrderedPathProgress progress;
  progress.setPath({{0, 0}, {1, 0}, {2, 0}, {3, 0}}, .25);
  EXPECT_FALSE(progress.update(0, 0));
  EXPECT_FALSE(progress.update(3, 0));
  EXPECT_EQ(progress.visited(), 1U);
  EXPECT_FALSE(progress.update(1, 0));
  EXPECT_FALSE(progress.update(2, 0));
  EXPECT_TRUE(progress.update(3, 0));
  progress.clear();
  EXPECT_FALSE(progress.update(3, 0));
}

TEST(OrderedPathProgress, RejectsInvalidAndHandlesDuplicatePoints)
{
  OrderedPathProgress progress;
  EXPECT_THROW(progress.setPath({{0, 0}, {0, 0}}, .25), std::invalid_argument);
  EXPECT_THROW(progress.setPath({{0, 0}, {1, 0}}, 0), std::invalid_argument);
  EXPECT_THROW(progress.setPath({{0, 0}, {NAN, 0}}, .25), std::invalid_argument);
  progress.setPath({{0, 0}, {0, 0}, {1, 0}}, .25);
  EXPECT_FALSE(progress.update(std::numeric_limits<double>::quiet_NaN(), 0));
  EXPECT_FALSE(progress.update(0, 0));
  EXPECT_TRUE(progress.update(1, 0));
}
