#include "tunnel_obstacle_detector/core/ground_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "tunnel_obstacle_detector/core/geometry.hpp"

namespace tod
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
}

GroundCalibrator::GroundCalibrator(const GroundCalibrationParams & params)
: params_(params)
{
  reset();
}

void GroundCalibrator::reset()
{
  pose_ = SensorPose();
  rejects_ = 0;
  if (params_.fixed) {
    const double pitch = params_.fixed_pitch_deg * kDegToRad;
    const double roll = params_.fixed_roll_deg * kDegToRad;
    Eigen::Vector3d normal(std::tan(pitch), std::tan(roll), 1.0);
    setPose(normal.normalized(), params_.fixed_height);
  }
}

bool GroundCalibrator::fitPlane(
  const std::vector<Eigen::Vector3f> & candidates, const GroundCalibrationParams & params, uint32_t seed,
  Eigen::Vector3d & normal, double & height, std::size_t & inliers)
{
  const std::size_t n = candidates.size();
  if (n < std::max<std::size_t>(params.min_inliers, 3)) {
    return false;
  }
  std::mt19937 rng(seed);
  std::uniform_int_distribution<std::size_t> pick(0, n - 1);
  const double min_cos = std::cos(params.max_tilt_deg * kDegToRad);

  std::size_t best_count = 0;
  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_d = 0.0;
  for (int it = 0; it < params.ransac_iterations; ++it) {
    const Eigen::Vector3d a = candidates[pick(rng)].cast<double>();
    const Eigen::Vector3d b = candidates[pick(rng)].cast<double>();
    const Eigen::Vector3d c = candidates[pick(rng)].cast<double>();
    Eigen::Vector3d nrm = (b - a).cross(c - a);
    const double len = nrm.norm();
    if (len < 1e-9) {
      continue;
    }
    nrm /= len;
    if (nrm.z() < 0.0) {
      nrm = -nrm;
    }
    if (nrm.z() < min_cos) {
      continue;
    }
    const double d = -nrm.dot(a);
    std::size_t count = 0;
    for (const auto & p : candidates) {
      if (std::abs(nrm.x() * p.x() + nrm.y() * p.y() + nrm.z() * p.z() + d) < params.inlier_threshold) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_normal = nrm;
      best_d = d;
    }
  }
  if (best_count < params.min_inliers) {
    return false;
  }

  // least-squares refinement on the inliers (smallest principal axis)
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  std::size_t count = 0;
  for (const auto & pf : candidates) {
    const Eigen::Vector3d p = pf.cast<double>();
    if (std::abs(best_normal.dot(p) + best_d) < params.inlier_threshold) {
      centroid += p;
      ++count;
    }
  }
  centroid /= static_cast<double>(count);
  for (const auto & pf : candidates) {
    const Eigen::Vector3d p = pf.cast<double>();
    if (std::abs(best_normal.dot(p) + best_d) < params.inlier_threshold) {
      const Eigen::Vector3d q = p - centroid;
      cov += q * q.transpose();
    }
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  Eigen::Vector3d refined = solver.eigenvectors().col(0);
  if (refined.z() < 0.0) {
    refined = -refined;
  }
  if (refined.z() < min_cos) {
    refined = best_normal;
  }
  normal = refined.normalized();
  height = -normal.dot(centroid);
  inliers = count;
  return true;
}

const SensorPose & GroundCalibrator::update(const std::vector<Eigen::Vector3f> & mount_points)
{
  if (params_.fixed) {
    return pose_;
  }
  // collect candidates with a stride so that the sample stays spread over the whole area
  candidates_.clear();
  std::size_t total = 0;
  for (const auto & p : mount_points) {
    if (p.x() > params_.x_min && p.x() < params_.x_max && std::abs(p.y()) < params_.y_half && p.z() < params_.z_max) {
      ++total;
    }
  }
  const std::size_t stride = std::max<std::size_t>(1, total / std::max<std::size_t>(1, params_.max_samples));
  std::size_t k = 0;
  for (const auto & p : mount_points) {
    if (p.x() > params_.x_min && p.x() < params_.x_max && std::abs(p.y()) < params_.y_half && p.z() < params_.z_max) {
      if (k++ % stride == 0) {
        candidates_.push_back(p);
      }
    }
  }

  Eigen::Vector3d normal;
  double height = 0.0;
  std::size_t inliers = 0;
  if (!fitPlane(candidates_, params_, 12345U + frame_counter_++, normal, height, inliers)) {
    return pose_;
  }
  if (!pose_.valid) {
    setPose(normal, height);
    return pose_;
  }
  const double angle = std::acos(std::clamp(normal.dot(pose_.normal), -1.0, 1.0));
  const bool jump = std::abs(height - pose_.height) > params_.max_step_height ||
    angle > params_.max_step_angle_deg * kDegToRad;
  if (jump) {
    if (++rejects_ < params_.reinit_after_rejects) {
      return pose_;
    }
    setPose(normal, height);
    rejects_ = 0;
    return pose_;
  }
  rejects_ = 0;
  const double a = params_.smoothing;
  setPose(((1.0 - a) * pose_.normal + a * normal).normalized(), (1.0 - a) * pose_.height + a * height);
  return pose_;
}

void GroundCalibrator::setPose(const Eigen::Vector3d & normal, double height)
{
  pose_.normal = normal;
  pose_.height = height;
  pose_.rotation = alignToPlane(normal);
  planeNormalToPitchRoll(normal, pose_.pitch_deg, pose_.roll_deg);
  pose_.valid = true;
}

}  // namespace tod
