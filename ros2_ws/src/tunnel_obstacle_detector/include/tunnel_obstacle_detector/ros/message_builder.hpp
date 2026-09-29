#pragma once

#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tunnel_obstacle_msgs/msg/obstacle_status.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tunnel_obstacle_detector/core/pipeline.hpp"

namespace tod_ros
{

tunnel_obstacle_msgs::msg::ObstacleStatus buildStatus(
  const tod::FrameResult & result, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header,
  double transport_latency_ms);

vision_msgs::msg::Detection3DArray buildDetections(
  const tod::FrameResult & result, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header);

/// Corridor edges, obstacle boxes, distance labels and a status banner, all in the cloud frame.
visualization_msgs::msg::MarkerArray buildMarkers(
  const tod::FrameResult & result, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header);

/// x/y/z/intensity cloud (cloud frame) of the given track-cloud points.
sensor_msgs::msg::PointCloud2 buildPointCloud(
  const std::vector<tod::RawPoint> & raw, const tod::TrackCloud & cloud, const std::vector<uint32_t> & indices,
  const std_msgs::msg::Header & header);

}  // namespace tod_ros
