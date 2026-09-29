#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

/// Fixed rotation from the point cloud frame to the "mount" frame (X forward, Y left, Z up).
/// `forward_axis` names the cloud axis that looks ahead of the train: "x", "-x", "y" or "-y";
/// the cloud Z axis is assumed to point up.
Eigen::Matrix3d mountRotationFromForwardAxis(const std::string & forward_axis);

/// Guesses which horizontal cloud axis looks ahead of the train: in a tunnel the view along the track is open for
/// 100+ m, sideways it ends at the walls and backwards the train body blocks it. Counts far, nearly horizontal
/// returns in a +-20 deg sector around each axis; returns "" when no axis clearly dominates.
std::string detectForwardAxis(const std::vector<RawPoint> & raw, double far_range = 50.0, int min_votes = 100);

/// Rotation that maps the mount frame to a frame whose Z axis is `up_normal` while keeping the
/// mount X axis (projected on the plane) as forward. Rows are the new basis vectors.
Eigen::Matrix3d alignToPlane(const Eigen::Vector3d & up_normal);

/// Pitch (positive when the track bed rises ahead in the mount frame) and roll in degrees of a plane normal.
void planeNormalToPitchRoll(const Eigen::Vector3d & up_normal, double & pitch_deg, double & roll_deg);

}  // namespace tod
