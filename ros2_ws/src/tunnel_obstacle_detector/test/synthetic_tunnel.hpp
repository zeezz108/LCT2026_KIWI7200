#pragma once

#include <vector>

#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod_test
{

/// Axis-aligned box in the track frame; `lateral` is measured from the curved track axis at `x_near`.
struct Box
{
  double x_near = 50.0;
  double length = 0.5;
  double lateral = 0.0;
  double width = 0.5;
  double height = 1.7;
};

struct TunnelSpec
{
  double sensor_height = 1.35;
  double sensor_lateral = 0.0;  ///< lidar offset from the track axis
  double curvature = 0.0;       ///< track axis y = curvature * x^2 / 2
  double wall_half_width = 2.3;
  double ceiling = 4.4;
  double gauge_half = 0.795;
  double rail_height = 0.2;
  double max_range = 200.0;
  double azimuth_step_deg = 0.2;
  double azimuth_half_fov_deg = 50.0;
  double noise = 0.01;
  std::vector<Box> boxes;
};

/// Ray-casts a Pandar128-like scan pattern against a simple tunnel.
/// Returns points in the *cloud* frame of the bags: forward = -y, left = +x, up = +z.
std::vector<tod::RawPoint> renderTunnel(const TunnelSpec & spec, unsigned seed = 1);

/// Same scan converted to the track frame (X forward, Y left, Z up from the bed), for component tests.
tod::TrackCloud renderTunnelTrackFrame(const TunnelSpec & spec, unsigned seed = 1);

}  // namespace tod_test
