#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod_ros
{

/// Byte offsets and datatypes of the fields the detector needs.
struct CloudLayout
{
  int x = -1;
  int y = -1;
  int z = -1;
  int intensity = -1;
  int ring = -1;
  uint8_t xyz_type = 0;
  uint8_t intensity_type = 0;
  uint8_t ring_type = 0;
  uint32_t point_step = 0;
  bool valid = false;
};

/// Inspects the fields of a cloud. x/y/z (FLOAT32 or FLOAT64) are required; intensity and ring are optional
/// (the track axis detector needs ring; without it the corridor falls back to wall-only estimation).
CloudLayout analyzeLayout(const sensor_msgs::msg::PointCloud2 & msg, std::string & error);

/// Copies all points (including invalid zeros/NaNs, filtered later by the pipeline) into `out`.
void convertCloud(
  const sensor_msgs::msg::PointCloud2 & msg, const CloudLayout & layout, std::vector<tod::RawPoint> & out);

}  // namespace tod_ros
