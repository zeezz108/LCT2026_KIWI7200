#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "tunnel_obstacle_detector/core/ground_calibration.hpp"

using tod::GroundCalibrationParams;
using tod::GroundCalibrator;

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

/// Floor points of a plane seen by a lidar at `height`, tilted by pitch/roll, plus clutter above the floor.
std::vector<Eigen::Vector3f> tiltedFloor(double height, double pitch_deg, double roll_deg, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(3.0, 20.0);
  std::uniform_real_distribution<double> uy(-2.5, 2.5);
  std::normal_distribution<double> noise(0.0, 0.01);
  const double nx = std::tan(pitch_deg * kDegToRad);
  const double ny = std::tan(roll_deg * kDegToRad);
  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < 6000; ++i) {
    const double x = ux(rng);
    const double y = uy(rng);
    // plane nx*x + ny*y + z + height = 0 (normal (nx, ny, 1) up to scale)
    const double z = -height - nx * x - ny * y + noise(rng);
    pts.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  }
  std::uniform_real_distribution<double> uz(-1.3, -0.6);
  for (int i = 0; i < 1500; ++i) {  // clutter: rails, equipment
    pts.emplace_back(static_cast<float>(ux(rng)), static_cast<float>(uy(rng)), static_cast<float>(uz(rng)));
  }
  return pts;
}
}  // namespace

TEST(GroundCalibration, RecoversHeightPitchRoll)
{
  const auto pts = tiltedFloor(1.73, 0.85, 2.6, 3);
  GroundCalibrator calib;
  const auto & pose = calib.update(pts);
  ASSERT_TRUE(pose.valid);
  EXPECT_NEAR(pose.height, 1.73 * std::cos(2.6 * kDegToRad) * std::cos(0.85 * kDegToRad), 0.02);
  EXPECT_NEAR(pose.pitch_deg, 0.85, 0.2);
  EXPECT_NEAR(pose.roll_deg, 2.6, 0.2);

  // bed points map to z ~ 0 in the track frame
  const Eigen::Vector3d p(10.0, 1.0, -1.73 - std::tan(0.85 * kDegToRad) * 10.0 - std::tan(2.6 * kDegToRad) * 1.0);
  EXPECT_NEAR(pose.toTrack(p).z(), 0.0, 0.03);
}

TEST(GroundCalibration, RejectsSuddenJumpButAcceptsPersistentChange)
{
  GroundCalibrationParams params;
  params.reinit_after_rejects = 5;
  GroundCalibrator calib(params);
  calib.update(tiltedFloor(1.34, 0.0, 0.0, 1));
  ASSERT_NEAR(calib.pose().height, 1.34, 0.02);

  calib.update(tiltedFloor(1.80, 0.0, 0.0, 2));  // single outlier frame
  EXPECT_NEAR(calib.pose().height, 1.34, 0.02);

  for (unsigned i = 0; i < 6; ++i) {
    calib.update(tiltedFloor(1.80, 0.0, 0.0, 10 + i));
  }
  EXPECT_NEAR(calib.pose().height, 1.80, 0.05);
}

TEST(GroundCalibration, FixedPoseIsUsedAsIs)
{
  GroundCalibrationParams params;
  params.fixed = true;
  params.fixed_height = 1.5;
  params.fixed_pitch_deg = 1.0;
  GroundCalibrator calib(params);
  ASSERT_TRUE(calib.pose().valid);
  calib.update(tiltedFloor(1.0, 0.0, 0.0, 4));
  EXPECT_DOUBLE_EQ(calib.pose().height, 1.5);
  EXPECT_NEAR(calib.pose().pitch_deg, 1.0, 1e-6);
}
