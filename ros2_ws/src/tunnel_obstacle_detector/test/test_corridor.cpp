#include <gtest/gtest.h>

#include <cmath>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/track_axis.hpp"

using tod::AxisModel;
using tod::Corridor;
using tod::CorridorEstimator;
using tod::TrackAxisDetector;

namespace
{
Corridor estimateFor(const tod_test::TunnelSpec & spec)
{
  const auto cloud = tod_test::renderTunnelTrackFrame(spec);
  TrackAxisDetector axis_detector;
  std::vector<tod::AxisDetection> inliers;
  const AxisModel axis = axis_detector.fit(axis_detector.detect(cloud, AxisModel()), &inliers);
  CorridorEstimator estimator;
  return estimator.estimate(cloud, inliers, axis, nullptr);
}
}  // namespace

TEST(Corridor, StraightTunnel)
{
  tod_test::TunnelSpec spec;
  const Corridor c = estimateFor(spec);
  ASSERT_TRUE(c.valid);
  for (double x : {10.0, 50.0, 100.0, 150.0}) {
    EXPECT_NEAR(c.lateralAt(static_cast<float>(x)), 0.0, 0.3) << "x=" << x;
    EXPECT_NEAR(c.bedAt(static_cast<float>(x)), 0.0, 0.3) << "x=" << x;
  }
  // walls of the synthetic tunnel stand 2.3 m from the axis
  EXPECT_NEAR(c.wall_left, 2.3, 0.15);
  EXPECT_NEAR(c.wall_right, -2.3, 0.2);
}

TEST(Corridor, FollowsLeftCurveIntoTheWalls)
{
  tod_test::TunnelSpec spec;
  spec.curvature = 1.0 / 400.0;
  const Corridor c = estimateFor(spec);
  ASSERT_TRUE(c.valid);
  // a straight corridor would be 4.5 m off at 60 m and hit the wall
  EXPECT_NEAR(c.lateralAt(30.0F), 0.5 * spec.curvature * 900.0, 0.35);
  EXPECT_NEAR(c.lateralAt(60.0F), 0.5 * spec.curvature * 3600.0, 0.6);
  EXPECT_NEAR(c.lateralAt(90.0F), 0.5 * spec.curvature * 8100.0, 1.0);
  EXPECT_GT(c.kappas[3], 0.0015);
}

TEST(Corridor, CurveIsFoundFromAnyInitialHeading)
{
  // the rail fit may be 0.3 deg off; near segments alone cannot tell such headings apart, pruning must not lose the
  // hypothesis that meets the rails further ahead
  tod_test::TunnelSpec spec;
  spec.curvature = 1.0 / 500.0;
  const auto cloud = tod_test::renderTunnelTrackFrame(spec, 300);
  TrackAxisDetector axis_detector;
  std::vector<tod::AxisDetection> inliers;
  const AxisModel axis = axis_detector.fit(axis_detector.detect(cloud, AxisModel()), &inliers);
  ASSERT_TRUE(axis.locked);
  for (double dth : {-0.006, -0.003, 0.0, 0.003, 0.006}) {
    for (double dy : {-0.1, 0.0, 0.1}) {
      AxisModel a = axis;
      a.theta0 += dth;
      a.y0 += dy;
      CorridorEstimator estimator;
      const Corridor c = estimator.estimate(cloud, inliers, a, nullptr);
      EXPECT_NEAR(c.lateralAt(60.0F), 0.5 * spec.curvature * 3600.0, 0.5) << "dtheta=" << dth << " dy=" << dy;
    }
  }
}

TEST(Corridor, ObstacleDoesNotBendTheCorridor)
{
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({50.0, 0.5, 0.0, 0.5, 1.7});  // person-sized box on the axis
  const Corridor c = estimateFor(spec);
  EXPECT_NEAR(c.lateralAt(50.0F), 0.0, 0.3);
}
