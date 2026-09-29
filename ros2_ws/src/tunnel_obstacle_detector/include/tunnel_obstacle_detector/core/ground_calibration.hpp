#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace tod
{

/// Lidar pose relative to the track bed, estimated online.
struct SensorPose
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();  ///< mount frame -> track frame
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();       ///< bed normal in the mount frame
  double height = 0.0;                                     ///< lidar height above the bed [m]
  double pitch_deg = 0.0;                                  ///< positive when the lidar looks up
  double roll_deg = 0.0;
  bool valid = false;

  Eigen::Vector3d toTrack(const Eigen::Vector3d & mount_point) const
  {
    return rotation * mount_point + Eigen::Vector3d(0.0, 0.0, height);
  }
};

struct GroundCalibrationParams
{
  // candidate floor points in the mount frame (in front of the train, below the sensor)
  double x_min = 3.0;
  double x_max = 20.0;
  double y_half = 2.5;
  double z_max = -0.5;
  std::size_t max_samples = 5000;

  int ransac_iterations = 200;
  double inlier_threshold = 0.04;  ///< [m]
  double max_tilt_deg = 15.0;      ///< reject plane hypotheses steeper than this
  std::size_t min_inliers = 300;

  double smoothing = 0.2;            ///< weight of a new estimate in the running pose
  double max_step_height = 0.15;     ///< [m] jump that is treated as an outlier
  double max_step_angle_deg = 2.0;   ///< [deg] jump that is treated as an outlier
  int reinit_after_rejects = 20;     ///< accept a persistent new pose after this many rejections

  bool fixed = false;       ///< use the pose below instead of estimating it
  double fixed_height = 1.34;
  double fixed_pitch_deg = 0.0;
  double fixed_roll_deg = 0.0;
};

/// Estimates the track-bed plane (the dominant near-horizontal surface in front of the train) with RANSAC
/// and keeps a smoothed, outlier-protected pose.
class GroundCalibrator
{
public:
  explicit GroundCalibrator(const GroundCalibrationParams & params = GroundCalibrationParams());

  /// `mount_points`: points in the mount frame (X forward, Y left, Z up). Returns the current pose.
  const SensorPose & update(const std::vector<Eigen::Vector3f> & mount_points);

  const SensorPose & pose() const { return pose_; }
  void reset();

  /// One RANSAC + least-squares plane fit. `normal` points up, `height` is the origin's distance above the plane.
  static bool fitPlane(
    const std::vector<Eigen::Vector3f> & candidates, const GroundCalibrationParams & params, uint32_t seed,
    Eigen::Vector3d & normal, double & height, std::size_t & inliers);

private:
  void setPose(const Eigen::Vector3d & normal, double height);

  GroundCalibrationParams params_;
  SensorPose pose_;
  int rejects_ = 0;
  uint32_t frame_counter_ = 0;
  std::vector<Eigen::Vector3f> candidates_;
};

}  // namespace tod
