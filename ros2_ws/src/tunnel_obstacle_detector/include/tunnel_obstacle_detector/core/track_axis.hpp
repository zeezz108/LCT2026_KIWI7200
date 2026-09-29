#pragma once

#include <vector>

#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

/// Lateral position of the track axis found in one lidar ring (one scan line across the track).
struct AxisDetection
{
  double x = 0.0;      ///< distance ahead [m]
  double y = 0.0;      ///< lateral position of the track axis [m]
  double score = 0.0;  ///< template strength (rail height over bed, channel bonus)
};

/// Near-field track axis model y(x) = y0 + theta0 * x + kappa0 * x^2 / 2.
struct AxisModel
{
  double y0 = 0.0;
  double theta0 = 0.0;
  double kappa0 = 0.0;
  int num_detections = 0;
  bool locked = false;  ///< enough consistent detections

  double lateralAt(double x) const { return y0 + theta0 * x + 0.5 * kappa0 * x * x; }
};

struct TrackAxisParams
{
  double x_min = 3.0;
  double x_max = 32.0;
  double z_min = -0.7;
  double z_max = 0.6;
  double window = 1.8;           ///< lateral half-window around the prior axis [m]
  double bin = 0.04;             ///< lateral profile bin [m]
  double center_search = 1.0;    ///< how far from the prior the axis may be [m]
  double gauge_half = 0.795;     ///< half distance between rail head centres [m]
  double rail_half_window = 0.06;
  double rail_min_height = 0.08;     ///< rail head above the bed plane [m]
  double rail_above_bed = 0.08;      ///< rail head above the bed between the rails [m]
  double channel_bonus_depth = 0.10; ///< drainage channel deeper than this adds to the score [m]
  int min_ring_points = 25;

  int ransac_iterations = 150;
  double ransac_threshold = 0.10;
  double kappa_limit = 0.006;
  double kappa_ridge = 1600.0;
  int min_inliers = 5;
};

/// Finds the track axis near the train from the rails (two narrow bumps at +-gauge/2 above the bed between them)
/// and, when present, the drainage channel between them. Works per lidar ring.
class TrackAxisDetector
{
public:
  explicit TrackAxisDetector(const TrackAxisParams & params = TrackAxisParams());

  std::vector<AxisDetection> detect(const TrackCloud & cloud, const AxisModel & prior) const;

  /// Robust quadratic fit of detections; `inliers` (optional) receives the consistent detections.
  AxisModel fit(const std::vector<AxisDetection> & detections, std::vector<AxisDetection> * inliers) const;

  const TrackAxisParams & params() const { return params_; }

private:
  TrackAxisParams params_;
};

}  // namespace tod
