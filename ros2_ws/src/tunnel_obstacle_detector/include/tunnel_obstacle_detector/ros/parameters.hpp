#pragma once

#include <rclcpp/node.hpp>

#include "tunnel_obstacle_detector/core/pipeline.hpp"

namespace tod_ros
{

/// Declares every algorithm parameter on the node (defaults = core defaults) and returns the filled structure.
tod::PipelineParams declarePipelineParameters(rclcpp::Node & node);

}  // namespace tod_ros
