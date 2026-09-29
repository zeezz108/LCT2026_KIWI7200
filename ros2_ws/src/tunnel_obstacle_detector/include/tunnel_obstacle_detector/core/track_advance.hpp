#pragma once

#include <vector>

#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

/// How far the train drove between two frames, and nothing else.
///
/// In coordinates tied to the track - distance along it and offset from its axis - the tunnel does not move: the
/// same physical point keeps its offset and only slides towards the train. So a single number per frame is enough
/// to bring the returns of several frames into one, which is what makes a far object detectable: at 175 m it gives
/// 4-6 returns per frame but 12-16 over five frames, while inside the clearance envelope at that range an empty
/// tunnel accumulates nothing (measured, docs/research_log.md).
///
/// The number comes from the structures that cross the tunnel - portals, columns, brackets, joints. Their density
/// along the track forms a signature; between two frames the signature shifts by the driven distance, and the shift
/// is found by correlation. Walls running along the track carry no signature and are kept out by the narrow band.
struct TrackAdvanceParams
{
  double x_min = 5.0;            ///< profile window along the track [m]
  double x_max = 90.0;
  double cell = 0.25;            ///< profile resolution [m]
  double lateral_half = 3.0;     ///< only near the axis: along-track walls carry no signature
  double z_min = 1.0;            ///< above the contact rail, below the vault
  double z_max = 3.2;
  double search = 4.0;           ///< searched around the prior (or around zero without one) [m]
  double detrend_window = 4.0;   ///< the profile is divided by a moving average over this length
  double min_correlation = 0.25;
  double max_speed = 30.0;       ///< m/s; a larger shift is a mismatch, not a train
  int min_cells = 40;            ///< fewer occupied cells than this: no signature to match
  /// The raw measurement is excellent (median error 4 mm against tools/ego_motion.py) but jumps to a wrong peak
  /// once in a few hundred frames. A median over the last few raw values removes those without the lock-in that a
  /// speed prior with an acceleration clamp turned out to have (it held a wrong speed for a whole recording).
  int median_window = 3;
  /// ... and a measurement this far from the recent median is not a train either (0.3 m is already 3 m/s^2
  /// at 10 Hz): it is dropped before it can enter the median at all.
  double outlier_gate = 1.0;     ///< [m]
};

class TrackAdvance
{
public:
  explicit TrackAdvance(const TrackAdvanceParams & params = TrackAdvanceParams());

  /// Metres driven since the previous frame, or NaN when it could not be measured.
  double update(const TrackCloud & cloud, const Corridor & corridor, double dt);

  double advance() const { return advance_; }
  double correlation() const { return correlation_; }
  /// true when the last value came from the speed prior rather than from a matched profile
  bool predicted() const { return predicted_; }
  double speed() const { return speed_; }
  void reset();

  const TrackAdvanceParams & params() const { return params_; }

private:
  std::vector<float> profile(const TrackCloud & cloud, const Corridor & corridor) const;

  TrackAdvanceParams params_;
  std::vector<float> previous_;
  double advance_ = 0.0;
  double correlation_ = 0.0;
  double prior_ = 0.0;
  bool has_prior_ = false;
  std::vector<double> recent_;   ///< last raw measurements [m]
  double speed_ = 0.0;           ///< m/s, from the reported advance
  bool predicted_ = false;
};

}  // namespace tod
