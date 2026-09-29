#include <gtest/gtest.h>

#include <cmath>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/core/track_axis.hpp"

using tod::AxisModel;
using tod::TrackAxisDetector;

TEST(TrackAxis, FindsRailsOfStraightTrack)
{
  tod_test::TunnelSpec spec;
  spec.sensor_lateral = 0.25;  // lidar 25 cm left of the axis
  // detector works in the lidar-centred track frame: shift so the lidar is at y = 0
  auto cloud = tod_test::renderTunnelTrackFrame(spec);
  for (auto & p : cloud) {
    p.y -= static_cast<float>(spec.sensor_lateral);
  }

  TrackAxisDetector detector;
  const auto detections = detector.detect(cloud, AxisModel());
  ASSERT_GE(detections.size(), 8U);
  std::vector<tod::AxisDetection> inliers;
  const AxisModel model = detector.fit(detections, &inliers);
  ASSERT_TRUE(model.locked);
  EXPECT_NEAR(model.y0, -0.25, 0.05);
  EXPECT_NEAR(model.theta0, 0.0, 0.005);
  EXPECT_NEAR(model.lateralAt(25.0), -0.25, 0.08);
}

TEST(TrackAxis, FollowsCurvedTrackWithBootstrapPrior)
{
  tod_test::TunnelSpec spec;
  spec.curvature = 1.0 / 300.0;
  const auto cloud = tod_test::renderTunnelTrackFrame(spec);

  // same bootstrap as the pipeline: near rails first, then the full range around that model
  tod::TrackAxisParams near_params;
  near_params.x_max = 16.0;
  near_params.min_inliers = 4;
  const TrackAxisDetector near_detector(near_params);
  const AxisModel prior = near_detector.fit(near_detector.detect(cloud, AxisModel()), nullptr);
  ASSERT_TRUE(prior.locked);

  TrackAxisDetector detector;
  std::vector<tod::AxisDetection> inliers;
  const AxisModel model = detector.fit(detector.detect(cloud, prior), &inliers);
  ASSERT_TRUE(model.locked);
  ASSERT_GE(inliers.size(), 8U);
  for (const auto & d : inliers) {
    EXPECT_NEAR(d.y, 0.5 * spec.curvature * d.x * d.x, 0.12) << "at x=" << d.x;
  }
}

TEST(TrackAxis, NoRailsNoLock)
{
  tod_test::TunnelSpec spec;
  spec.gauge_half = 3.5;  // rails hidden inside the walls
  const auto cloud = tod_test::renderTunnelTrackFrame(spec);
  TrackAxisDetector detector;
  const AxisModel model = detector.fit(detector.detect(cloud, AxisModel()), nullptr);
  EXPECT_FALSE(model.locked);
}
