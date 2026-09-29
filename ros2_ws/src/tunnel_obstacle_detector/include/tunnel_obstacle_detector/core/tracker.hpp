#pragma once

#include <cstdint>
#include <vector>

#include "tunnel_obstacle_detector/core/obstacle_detector.hpp"
#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

struct TrackerParams
{
  double max_relative_speed = 30.0;  ///< [m/s] gate along the track before a track has a speed estimate
  double gate_along_base = 1.5;      ///< [m]
  double gate_along_rel = 0.03;      ///< fraction of the distance
  double gate_lateral = 0.8;         ///< [m]
  int confirm_hits = 3;              ///< hits within confirm_window frames to confirm a track
  int confirm_window = 5;
  // far objects need more evidence: +confirm_hits_per_25m beyond confirm_far_from, up to confirm_hits_max hits
  // within (required + 3) frames. At 15 m/s an obstacle is seen for ~70 frames between 150 and 50 m.
  double confirm_far_from = 50.0;
  int confirm_hits_per_25m = 1;
  int confirm_hits_max = 5;
  // A cluster with fewer returns than its size implies (`weak`) is not trusted on one frame: the smallest obstacle
  // of the case (300 x 300 x 100 mm) gives 3-6 returns at 56 m, and so does noise. A track that never saw a dense
  // cluster is confirmed only after this much evidence, which noise does not reach at a fixed place.
  int weak_confirm_hits = 6;
  int weak_confirm_window = 10;
  int danger_hits = 2;               ///< danger-level hits within the window to report DANGER
  int max_misses = 4;                ///< frames without a hit before the track is dropped
  int report_max_misses = 1;         ///< confirmed tracks are reported while missed at most this many frames
  int strong_points = 40;            ///< a cluster this dense ...
  double strong_max_distance = 40.0; ///< ... and this close is confirmed immediately
  double speed_smoothing = 0.5;

  // expected returns from a reference target, used for confidence
  double target_width = 0.5;
  double target_height = 1.0;
  double azimuth_resolution_deg = 0.1;
  double elevation_resolution_deg = 0.125;
  double return_ratio = 0.6;
};

struct Track
{
  uint32_t id = 0;
  Level level = Level::kWarning;
  double distance = 0.0;  ///< nearest point along the track [m]
  double lateral = 0.0;   ///< centre offset from the track axis [m]
  double h_min = 0.0;
  double h_max = 0.0;
  double length = 0.0;
  double width = 0.0;
  double speed = 0.0;     ///< d(distance)/dt [m/s], negative when approaching
  bool has_speed = false;
  int hits = 0;
  int misses = 0;
  uint32_t history = 0;         ///< bit i set: hit i frames ago
  uint32_t danger_history = 0;  ///< bit i set: danger-level hit i frames ago
  uint32_t strong_history = 0;  ///< bit i set: the hit i frames ago came from a cluster dense enough for its size
  bool confirmed = false;
  double confidence = 0.0;
  int num_points = 0;
  float cx = 0.0F;
  float cy = 0.0F;
  float cz = 0.0F;
  std::vector<uint32_t> points;  ///< indices into the latest track cloud (empty when missed)
};

/// Associates clusters over frames (gated along/across the track), confirms persistent objects and estimates
/// their approach speed. Persistence is what separates real objects from sensor noise at long range.
class Tracker
{
public:
  explicit Tracker(const TrackerParams & params = TrackerParams());

  void update(const std::vector<Cluster> & clusters, double stamp);

  /// Confirmed tracks seen recently, nearest first.
  std::vector<Track> reported() const;

  const std::vector<Track> & tracks() const { return tracks_; }
  double expectedPoints(double distance) const;
  void reset();

private:
  TrackerParams params_;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 1;
  double last_stamp_ = 0.0;
  bool has_stamp_ = false;
};

}  // namespace tod
