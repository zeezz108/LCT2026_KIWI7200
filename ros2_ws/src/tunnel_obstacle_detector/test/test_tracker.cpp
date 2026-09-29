#include <gtest/gtest.h>

#include "tunnel_obstacle_detector/core/tracker.hpp"

using tod::Cluster;
using tod::Level;
using tod::Tracker;

namespace
{
Cluster makeCluster(float x, float lat, int points, Level level = Level::kDanger)
{
  Cluster c;
  c.x_min = x;
  c.x_max = x + 0.4F;
  c.lat_center = lat;
  c.lat_min = lat - 0.25F;
  c.lat_max = lat + 0.25F;
  c.h_min = 0.3F;
  c.h_max = 1.8F;
  c.num_points = points;
  c.danger_points = level == Level::kDanger ? points : 0;
  c.level = level;
  return c;
}
}  // namespace

TEST(Tracker, ConfirmsApproachingObjectAndEstimatesSpeed)
{
  Tracker tracker;
  const double v = 15.0;
  for (int f = 0; f < 6; ++f) {
    tracker.update({makeCluster(static_cast<float>(120.0 - v * 0.1 * f), 0.1F, 12)}, 0.1 * f);
    if (f < 2) {
      EXPECT_TRUE(tracker.reported().empty()) << "confirmed too early at frame " << f;
    }
  }
  const auto rep = tracker.reported();
  ASSERT_EQ(rep.size(), 1U);
  EXPECT_EQ(rep[0].level, Level::kDanger);
  ASSERT_TRUE(rep[0].has_speed);
  EXPECT_NEAR(rep[0].speed, -v, 1.0);
}

TEST(Tracker, SingleFrameNoiseIsNotReported)
{
  Tracker tracker;
  tracker.update({makeCluster(80.0F, 0.0F, 4)}, 0.0);
  for (int f = 1; f < 10; ++f) {
    tracker.update({}, 0.1 * f);
  }
  EXPECT_TRUE(tracker.reported().empty());
}

TEST(Tracker, DenseCloseObjectIsConfirmedImmediately)
{
  Tracker tracker;
  tracker.update({makeCluster(12.0F, 0.0F, 400)}, 0.0);
  EXPECT_EQ(tracker.reported().size(), 1U);
}

TEST(Tracker, SeparateObjectsKeepSeparateTracks)
{
  Tracker tracker;
  for (int f = 0; f < 4; ++f) {
    tracker.update({makeCluster(50.0F, 0.0F, 30), makeCluster(50.5F, 2.2F, 30, Level::kWarning)}, 0.1 * f);
  }
  const auto rep = tracker.reported();
  ASSERT_EQ(rep.size(), 2U);
  EXPECT_NE(rep[0].id, rep[1].id);
}

TEST(Tracker, BagRestartClearsHistory)
{
  Tracker tracker;
  for (int f = 0; f < 4; ++f) {
    tracker.update({makeCluster(50.0F, 0.0F, 30)}, 100.0 + 0.1 * f);
  }
  ASSERT_EQ(tracker.reported().size(), 1U);
  tracker.update({}, 10.0);  // time went backwards
  EXPECT_TRUE(tracker.tracks().empty());
}
