#include <gtest/gtest.h>

#include <sstream>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/obstacle_detector.hpp"
#include "tunnel_obstacle_detector/core/track_axis.hpp"

using tod::Cluster;
using tod::Level;

namespace
{
std::vector<Cluster> detectIn(
  const tod_test::TunnelSpec & spec, std::vector<Cluster> * rejected = nullptr,
  const tod::DetectorParams * params = nullptr)
{
  const auto cloud = tod_test::renderTunnelTrackFrame(spec);
  tod::TrackAxisDetector axis_detector;
  std::vector<tod::AxisDetection> inliers;
  const auto axis = axis_detector.fit(axis_detector.detect(cloud, tod::AxisModel()), &inliers);
  tod::CorridorEstimator estimator;
  const auto corridor = estimator.estimate(cloud, inliers, axis, nullptr);
  tod::ObstacleDetector detector(params ? *params : tod::DetectorParams());
  return detector.detect(cloud, corridor, [&](float x, float y, float z, float r) {
      return r > 0.0F ? estimator.touchesStructure(x, y, z, r) : estimator.isStructure(x, y, z);
    }, nullptr, rejected);
}

/// parameters with the band below the low one switched on (the customer's 300 x 300 x 100 mm object)
tod::DetectorParams withVeryLowBand()
{
  tod::DetectorParams p;
  p.very_low_max_distance = 30.0;  // the default is 25 m
  return p;
}

std::string describe(const std::vector<Cluster> & clusters)
{
  std::ostringstream os;
  for (const auto & c : clusters) {
    os << "\n  x=" << c.x_min << ".." << c.x_max << " lat=" << c.lat_min << ".." << c.lat_max << " h=" << c.h_min
       << ".." << c.h_max << " n=" << c.num_points << " danger=" << c.danger_points << " level="
       << static_cast<int>(c.level) << " reason=" << static_cast<int>(c.reason);
  }
  return os.str();
}
}  // namespace

TEST(Detector, EmptyTunnelHasNoCandidates)
{
  tod_test::TunnelSpec spec;
  const auto clusters = detectIn(spec);
  EXPECT_TRUE(clusters.empty()) << describe(clusters);
}

TEST(Detector, PersonOnTrackIsDanger)
{
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({60.0, 0.4, 0.3, 0.5, 1.75});
  const auto clusters = detectIn(spec);
  ASSERT_EQ(clusters.size(), 1U) << describe(clusters);
  EXPECT_EQ(clusters[0].level, Level::kDanger);
  EXPECT_NEAR(clusters[0].x_min, 60.0, 0.5);
  EXPECT_NEAR(clusters[0].lat_center, 0.3, 0.25);
}

TEST(Detector, PersonNextToTrackIsWarning)
{
  // the clearance is the car cross-section (2.1 m wide), so 1.6 m from the axis is outside it but close enough
  // to be worth a warning; the band reaches to half_width_upper + warning_margin = 2.05 m
  tod_test::TunnelSpec spec;
  spec.wall_half_width = 4.5;  // double-track like space
  spec.boxes.push_back({30.0, 0.4, 1.6, 0.5, 1.75});
  const auto clusters = detectIn(spec);
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters[0].level, Level::kWarning);
}

TEST(Detector, PersonWellClearOfTheTrackIsIgnored)
{
  tod_test::TunnelSpec spec;
  spec.wall_half_width = 4.5;
  spec.boxes.push_back({30.0, 0.4, 2.6, 0.5, 1.75});  // 1.5 m of clearance from the car side
  EXPECT_TRUE(detectIn(spec).empty());
}

TEST(Detector, LowObjectBelowEnvelopeBottomIsIgnored)
{
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({40.0, 0.5, 0.0, 0.5, 0.15});  // lower than rail heads
  EXPECT_TRUE(detectIn(spec).empty());
}

TEST(Detector, PersonIsFoundFarAway)
{
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({120.0, 0.4, 0.0, 0.5, 1.75});
  std::vector<Cluster> rejected;
  const auto clusters = detectIn(spec, &rejected);
  ASSERT_FALSE(clusters.empty()) << "rejected:" << describe(rejected);
  EXPECT_NEAR(clusters.front().x_min, 120.0, 1.0);
}

TEST(Detector, PersonLyingOnTheTrackIsDanger)
{
  // a fallen person is ~0.35 m high: below the clearance envelope, but still on the track and dangerous.
  // The rails (0.2 m) and anything running along the track must stay silent in the same band.
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({50.0, 1.7, 0.0, 0.5, 0.35});
  std::vector<Cluster> rejected;
  const auto clusters = detectIn(spec, &rejected);
  ASSERT_FALSE(clusters.empty()) << "rejected:" << describe(rejected);
  EXPECT_EQ(clusters.front().level, Level::kDanger) << describe(clusters);
  EXPECT_NEAR(clusters.front().x_min, 50.0, 1.0);
  EXPECT_LT(clusters.front().h_max, 0.6);
}

TEST(Detector, LowObjectAcrossTheRailsIsDanger)
{
  // lying across both rails: the object must not be swallowed by the rail lines
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({35.0, 0.6, 0.0, 2.0, 0.3});
  std::vector<Cluster> rejected;
  const auto clusters = detectIn(spec, &rejected);
  ASSERT_FALSE(clusters.empty()) << "rejected:" << describe(rejected);
  EXPECT_EQ(clusters.front().level, Level::kDanger) << describe(clusters);
}

TEST(Detector, MinimumObjectOnTheTrackIsDetected)
{
  // the size the customer named as the main criterion: 300 x 300 x 100 mm, lying between the rails
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({20.0, 0.3, 0.0, 0.3, 0.1});
  std::vector<Cluster> rejected;
  const auto params = withVeryLowBand();
  const auto clusters = detectIn(spec, &rejected, &params);
  ASSERT_FALSE(clusters.empty()) << "rejected:" << describe(rejected);
  EXPECT_EQ(clusters.front().level, Level::kWarning) << describe(clusters);  // reported one level down
  EXPECT_NEAR(clusters.front().x_min, 20.0, 1.0);
  EXPECT_LT(clusters.front().h_max, 0.2) << describe(clusters);
}

TEST(Detector, TrackDetailRunningAlongTheTrackIsNotAnObstacle)
{
  // a duct or cover of the same height as the minimum object, but metres long: this is what the band must not report
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({20.0, 3.0, 0.0, 0.5, 0.12});
  std::vector<Cluster> rejected;
  const auto params = withVeryLowBand();
  const auto clusters = detectIn(spec, &rejected, &params);
  EXPECT_TRUE(clusters.empty()) << describe(clusters);
}

TEST(Detector, MinimumObjectIsIgnoredWhenTheBandIsOff)
{
  // switching the band off is what every earlier configuration did: then the object is below every threshold
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({20.0, 0.3, 0.0, 0.3, 0.1});
  tod::DetectorParams off;
  off.very_low_max_distance = 0.0;
  const auto clusters = detectIn(spec, nullptr, &off);
  EXPECT_TRUE(clusters.empty()) << describe(clusters);
}

TEST(Detector, MinimumObjectBeyondTheBandIsIgnored)
{
  // the band ends at 25 m: further away the rings that graze the floor are too far apart to tell a bump from a duct
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({40.0, 0.3, 0.0, 0.3, 0.1});
  const auto clusters = detectIn(spec);
  EXPECT_TRUE(clusters.empty()) << describe(clusters);
}
