#include <gtest/gtest.h>

#include <cmath>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/core/pipeline.hpp"

using tod::Level;
using tod::Pipeline;

TEST(Pipeline, EmptyCurvedTunnelStaysClear)
{
  tod_test::TunnelSpec spec;
  spec.curvature = 1.0 / 350.0;
  spec.sensor_lateral = 0.1;
  Pipeline pipeline;
  for (int f = 0; f < 8; ++f) {
    const auto & r = pipeline.process(tod_test::renderTunnel(spec, 100 + f), 0.1 * f);
    ASSERT_TRUE(r.valid);
    EXPECT_EQ(r.level, Level::kClear) << "frame " << f;
  }
  EXPECT_NEAR(pipeline.result().pose.height, spec.sensor_height, 0.03);
  EXPECT_TRUE(pipeline.result().axis.locked);
  EXPECT_GT(pipeline.result().free_distance, 60.0);
}

TEST(Pipeline, ApproachingObstacleIsReportedWithDistanceAndTtc)
{
  tod_test::TunnelSpec spec;
  Pipeline pipeline;
  const double speed = 12.0;
  double last_distance = 0.0;
  for (int f = 0; f < 8; ++f) {
    spec.boxes = {{100.0 - speed * 0.1 * f, 0.4, 0.0, 0.6, 1.2}};
    const auto & r = pipeline.process(tod_test::renderTunnel(spec, 200 + f), 0.1 * f);
    last_distance = spec.boxes[0].x_near;
    if (f >= 4) {
      ASSERT_EQ(r.level, Level::kDanger) << "frame " << f;
      EXPECT_NEAR(r.nearest_distance, last_distance, 1.0);
      EXPECT_NEAR(r.nearest_lateral, 0.0, 0.3);
      ASSERT_TRUE(std::isfinite(r.time_to_collision));
      EXPECT_NEAR(r.time_to_collision, last_distance / speed, 1.5);
      EXPECT_LE(r.free_distance, last_distance + 0.5);
    }
  }
}

TEST(Pipeline, CloudWithoutRingFieldStillFindsTheTrack)
{
  tod_test::TunnelSpec spec;
  spec.curvature = 1.0 / 500.0;
  spec.boxes = {{60.0, 0.5, 0.0, 0.5, 1.7}};
  Pipeline pipeline;
  for (int f = 0; f < 6; ++f) {
    auto raw = tod_test::renderTunnel(spec, 300 + f);
    for (auto & p : raw) {
      p.ring = 0;  // e.g. a driver that publishes only x, y, z, intensity
    }
    ASSERT_TRUE(pipeline.process(raw, 0.1 * f).valid);
  }
  const auto & r = pipeline.result();
  EXPECT_TRUE(r.axis.locked);
  EXPECT_EQ(r.level, Level::kDanger);
  EXPECT_NEAR(r.nearest_distance, 60.0, 1.0);
  EXPECT_NEAR(r.nearest_lateral, 0.0, 0.3);
}

TEST(Pipeline, ForwardAxisIsDetectedAutomatically)
{
  tod_test::TunnelSpec spec;
  spec.boxes = {{50.0, 0.5, 0.0, 0.5, 1.7}};
  for (const std::string axis : {"-y", "x"}) {
    tod::PipelineParams params;
    params.forward_axis = "auto";
    params.forward_axis_fallback = "y";  // wrong on purpose
    Pipeline pipeline(params);
    for (int f = 0; f < 5; ++f) {
      auto raw = tod_test::renderTunnel(spec, 400 + f);  // forward = -y
      if (axis == "x") {
        for (auto & p : raw) {  // rotate the cloud: forward = +x, left = +y
          const float x = p.x;
          p.x = -p.y;
          p.y = x;
        }
      }
      pipeline.process(raw, 0.1 * f);
    }
    EXPECT_EQ(pipeline.forwardAxis(), axis);
    EXPECT_EQ(pipeline.result().level, Level::kDanger) << axis;
    EXPECT_NEAR(pipeline.result().nearest_distance, 50.0, 1.0) << axis;
  }
}

TEST(Pipeline, TrackToCloudRoundTrip)
{
  tod_test::TunnelSpec spec;
  Pipeline pipeline;
  pipeline.process(tod_test::renderTunnel(spec, 7), 0.0);
  // a bed point 20 m ahead on the lidar's line must map to cloud y = -20, z = -height
  const Eigen::Vector3d c = pipeline.trackToCloud(Eigen::Vector3d(20.0, 0.0, 0.0));
  EXPECT_NEAR(c.y(), -20.0, 0.05);
  EXPECT_NEAR(c.x(), 0.0, 0.05);
  EXPECT_NEAR(c.z(), -spec.sensor_height, 0.05);
}
