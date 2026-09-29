#include "tunnel_obstacle_detector/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace tod
{

Eigen::Matrix3d mountRotationFromForwardAxis(const std::string & forward_axis)
{
  Eigen::Vector3d forward;
  if (forward_axis == "x") {
    forward = Eigen::Vector3d::UnitX();
  } else if (forward_axis == "-x") {
    forward = -Eigen::Vector3d::UnitX();
  } else if (forward_axis == "y") {
    forward = Eigen::Vector3d::UnitY();
  } else if (forward_axis == "-y") {
    forward = -Eigen::Vector3d::UnitY();
  } else {
    throw std::invalid_argument("forward_axis must be one of: x, -x, y, -y (got '" + forward_axis + "')");
  }
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d left = up.cross(forward);

  Eigen::Matrix3d rotation;
  rotation.row(0) = forward.transpose();
  rotation.row(1) = left.transpose();
  rotation.row(2) = up.transpose();
  return rotation;
}

std::string detectForwardAxis(const std::vector<RawPoint> & raw, double far_range, int min_votes)
{
  constexpr double kTanSector = 0.364;    // tan 20 deg
  constexpr double kTanElevation = 0.176;  // tan 10 deg
  const double far2 = far_range * far_range;
  long votes[4] = {0, 0, 0, 0};  // x, -x, y, -y
  for (std::size_t i = 0; i < raw.size(); i += 3) {
    const double x = raw[i].x;
    const double y = raw[i].y;
    const double z = raw[i].z;
    const double h2 = x * x + y * y;
    if (!(h2 > far2) || z * z > kTanElevation * kTanElevation * h2) {
      continue;  // also NaN
    }
    if (std::abs(y) < kTanSector * std::abs(x)) {
      ++votes[x > 0.0 ? 0 : 1];
    } else if (std::abs(x) < kTanSector * std::abs(y)) {
      ++votes[y > 0.0 ? 2 : 3];
    }
  }
  int best = 0;
  for (int k = 1; k < 4; ++k) {
    if (votes[k] > votes[best]) {
      best = k;
    }
  }
  long second = 0;
  for (int k = 0; k < 4; ++k) {
    if (k != best) {
      second = std::max(second, votes[k]);
    }
  }
  static const char * const kNames[4] = {"x", "-x", "y", "-y"};
  return votes[best] >= min_votes && votes[best] >= 3 * second ? kNames[best] : "";
}

Eigen::Matrix3d alignToPlane(const Eigen::Vector3d & up_normal)
{
  const Eigen::Vector3d z = up_normal.normalized();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX() - z.x() * z;
  x.normalize();
  const Eigen::Vector3d y = z.cross(x);

  Eigen::Matrix3d rotation;
  rotation.row(0) = x.transpose();
  rotation.row(1) = y.transpose();
  rotation.row(2) = z.transpose();
  return rotation;
}

void planeNormalToPitchRoll(const Eigen::Vector3d & up_normal, double & pitch_deg, double & roll_deg)
{
  constexpr double kRadToDeg = 180.0 / M_PI;
  pitch_deg = std::atan2(up_normal.x(), up_normal.z()) * kRadToDeg;
  roll_deg = std::atan2(up_normal.y(), up_normal.z()) * kRadToDeg;
}

}  // namespace tod
