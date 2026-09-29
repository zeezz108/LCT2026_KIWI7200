#pragma once

#include <cstdint>
#include <vector>

namespace tod
{

/// Lidar return in the frame of the incoming point cloud.
struct RawPoint
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  uint16_t ring = 0;
};

/// Lidar return in the track frame: X ahead along the train, Y to the left, Z up from the track bed.
struct TrackPoint
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  uint16_t ring = 0;
  uint32_t raw_index = 0;  ///< index of the source point in the frame's RawPoint vector
};

/// Obstacle severity, mirrors tunnel_obstacle_msgs/ObstacleStatus levels.
enum class Level : uint8_t
{
  kClear = 0,
  kWarning = 1,  ///< close to the clearance envelope, outside it
  kDanger = 2,   ///< inside the clearance envelope
};

using TrackCloud = std::vector<TrackPoint>;

}  // namespace tod
