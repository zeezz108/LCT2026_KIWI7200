#pragma once

#include <vector>

#include "tunnel_obstacle_detector/core/occupancy_grid.hpp"
#include "tunnel_obstacle_detector/core/track_axis.hpp"
#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

struct CorridorParams
{
  double x_max = 200.0;  ///< corridor length [m]
  double x_res = 0.5;    ///< cell size along the track [m]
  double y_half = 20.0;  ///< lateral extent of the occupancy grids [m]
  double y_res = 0.1;
  std::vector<double> segment_edges{0.0, 10.0, 20.0, 35.0, 60.0, 100.0, 150.0, 200.0};

  // curvature hypotheses per segment
  double kappa_max = 1.0 / 250.0;
  double kappa_step = 1.0 / 10000.0;
  double kappa_change_max = 0.0015;  ///< curvature may change by at most this between consecutive segments
  int beam_per_parent = 8;
  int beam_width = 24;
  double coarse_from = 60.0;         ///< beyond this distance the cost is evaluated on every second cell

  // uncertainty: hypotheses whose cost is within max(abs, rel * best) of the best one
  double alternative_cost_abs = 8.0;
  double alternative_cost_rel = 0.1;
  double diversity_heading_deg = 0.25;
  double diversity_offset = 0.25;

  // cost weights
  double w_smooth = 15.0;      ///< per (m of lateral deviation that a curvature change causes over smooth_base)^2
  double smooth_base = 40.0;
  double w_straight = 2.0;     ///< per metre of lateral deviation that |curvature| causes over smooth_base (L1)
  double w_straight_far = 10.0;  ///< same beyond straight_far_from, where walls are sparse and evidence is weak
  double straight_far_from = 60.0;
  double w_margin = 0.0;       ///< per structure cell in the clearance margin (off: columns at the gauge limit push)
  double w_axis = 40.0;        ///< per metre of centreline deviation from a rail detection
  double axis_residual_cap = 0.3;
  double w_axis_lookahead = 40.0;  ///< pruning only: rail detections beyond a segment scored along its curvature
  double w_temporal = 0.5;     ///< pull towards the previous frame's curvature profile (relative to w_smooth)
  // The train rides on the track, so the curvature under it follows a transition curve: at 15 m/s it changes by
  // ~1e-3 1/m per second at most. Near the train the curvature may therefore move only so fast between frames -
  // without this the search jumps onto the diverging route of a switch, where both rail pairs fit equally well.
  double kappa_change_rate = 0.0;          ///< [1/(m s)] allowed change of a near curvature; 0 = off
  double kappa_rate_limit_until = 35.0;    ///< only segments starting closer than this are limited [m]

  // search corridor and structure bands (heights above the track bed)
  double search_half_width_upper = 1.45;
  double search_half_width_lower = 1.20;
  double margin = 0.55;
  double band_upper_min = 0.9;
  double band_upper_max = 3.0;
  double band_lower_min = 0.35;
  double band_lower_max = 0.9;
  double long_structure_min_length = 4.0;
  // low band: things lying on the track (a fallen person is ~0.35 m high). Rails and anything running along the
  // track are infrastructure; a compact object is not.
  double band_low_min = 0.15;
  double band_low_max = 0.35;
  double low_structure_min_length = 3.0;
  double gauge_half = 0.795;       ///< rail head offset from the axis [m]
  // The rail head is a horizontal surface ~0.2 m above the bed. Near the train the bed itself is seen
  // between the rails, far away the rails are often the only floor returns left (steel reflects better at
  // a grazing angle), so their height is measured here and subtracted instead of dropping them.
  double rail_height_ref_min = 5.0;    ///< window where the rail height above the bed is measured [m]
  double rail_height_ref_max = 20.0;
  double rail_height_default = 0.19;   ///< used when the near field does not show both surfaces [m]
  double rail_height_min = 0.10;
  double rail_height_max = 0.30;
  /// ... and only where the axis is accurate enough to tell the rail strip from the bed beside it: the
  /// strip is 0.24 m wide, the axis error passes that at about 60 m.
  double rail_correct_max = 60.0;
  double rail_half_width = 0.12;   ///< the rail strips themselves are never obstacle candidates
  double dilate_along_base = 1.0;   ///< [m] linking distance along the track near the train
  double dilate_along_quad = 4.4e-4;  ///< [1/m] linking distance grows as k*x^2 (azimuth sampling of far walls)
  double aligned_half_width = 6.0;  ///< lateral extent of the axis-aligned structure grids [m]
  double inner_half_width = 1.6;    ///< inside this band around the axis far samples are never linked [m]
  double tall_low = 1.2;       ///< compact objects reaching below this ...
  double tall_high = 2.4;      ///< ... and above this height are structures (columns), people are shorter

  // walls run parallel to the track: structures seen next to the rails near the train are expected at the same
  // lateral offsets far ahead, where the rails are no longer visible and the walls are only sparse azimuth stripes
  double w_wall = 1.0;                  ///< reward per row where a structure is found at a near-field offset
  double wall_ref_min = 5.0;            ///< near-field window where the offsets are measured [m]
  double wall_ref_max = 30.0;
  double wall_scan_max = 8.0;           ///< offsets are looked for between the search half-width and this [m]
  double wall_ref_min_fraction = 0.3;   ///< an offset must be seen in at least this fraction of the window rows
  double wall_tolerance = 0.2;          ///< [m]
  double wall_tolerance_per_m = 0.002;  ///< far samples are noisier across the track

  // initial offset/heading hypotheses around the rail-based axis model
  double init_offset_span = 0.10;
  double init_offset_step = 0.05;
  double init_heading_span_deg = 0.6;
  double init_heading_step_deg = 0.15;
  double unlocked_offset_span = 0.3;
  double unlocked_offset_step = 0.1;
  double unlocked_heading_span_deg = 1.5;
  double unlocked_heading_step_deg = 0.25;

  // "the corridor points into a wall": where the tunnel turns out of sight, infrastructure crosses the corridor
  // cross-section. Beyond that row nothing about the track is known, and the vertical profile must not dive to
  // escape the wall (that lifts the floor behind it into the clearance envelope).
  double blocked_half_width = 1.45;  ///< half width of the tested cross-section [m]
  double blocked_window = 10.0;      ///< length of the window the intrusions are counted over [m]
  int blocked_cells = 6;             ///< structure cells inside the window that mean the corridor is closed
  double blocked_from = 30.0;        ///< never truncate closer than this [m]

  // vertical profile
  bool estimate_vertical = true;
  double grade_max = 0.04;
  double grade_step = 0.0025;
  double w_vertical_smooth = 40.0;  ///< per (m of height change that a grade change causes over the segment)^2
  double w_vertical_temporal = 0.5;   ///< pull towards the previous frame's grades (relative to w_vertical_smooth)
  double vertical_temporal_from = 60.0;  ///< only beyond this distance, where the bed is barely visible
  double vertical_fixed_until = 20.0;
  double z_grid_min = -4.0;
  double z_res = 0.1;
  int z_cells = 120;
  double vertical_zone_bottom = 0.45;
  double vertical_zone_top = 2.9;
  double vertical_ceiling_margin = 0.5;
  double w_vertical_ceiling = 0.3;
  double vertical_lateral_half_width = 1.35;
  double vertical_run_max_gap = 1.0;     ///< gaps allowed inside a horizontal surface run [m]
  double vertical_bed_tolerance = 0.35;  ///< bed surface may lie this far below the profile without cost [m]
  double vertical_below_depth = 1.5;
  double vertical_above_depth = 0.8;   ///< a horizontal surface this far above the profile is the bed, not an object
  double w_vertical_above = 1.0;       ///< cost per cell of bed found above the profile (the profile is too low)
  // ceiling clearance tracking
  double ceiling_ref_min = 8.0;          ///< near-field window where the ceiling clearance is measured [m]
  double ceiling_ref_max = 30.0;
  double ceiling_min_height = 2.8;       ///< lowest horizontal surface above this is the ceiling [m]
  int ceiling_ref_min_columns = 10;
  double ceiling_tolerance = 0.3;
  double w_ceiling_offset = 1.0;         ///< reward per column where the ceiling is found at its clearance
  // the bed is a horizontal surface, so where one is visible the profile belongs on it: this reward keeps the
  // profile on the floor instead of letting it sink to make structures ahead leave the clearance zone
  double w_floor = 1.0;                  ///< reward per row where a horizontal surface sits at the profile
  double floor_tolerance = 0.2;          ///< [m]
  /// the ceiling is a fallback cue: where the bed itself is visible it decides, so the clearance reward is
  /// skipped there. Without this a station vault (7-8 m) makes the profile dive when the tunnel (4 m) follows.
  bool ceiling_only_without_floor = true;
};

/// Clearance corridor: curved track axis and track-bed height profile ahead of the train.
struct Corridor
{
  double x_res = 0.5;
  std::vector<float> lateral;   ///< axis Y at cell centres
  std::vector<float> bed;       ///< track-bed Z at cell centres
  std::vector<float> heading;   ///< axis heading at cell centres [rad]
  std::vector<float> spread;    ///< lateral disagreement of near-optimal hypotheses at cell centres [m]
  std::vector<double> kappas;   ///< curvature per segment [1/m]
  std::vector<double> grades;   ///< grade per segment
  double ceiling_offset = 0.0;  ///< ceiling clearance above the bed near the train (NaN if not seen)
  double rail_height = 0.0;     ///< measured height of the rail heads above the bed [m]
  double wall_left = 0.0;       ///< near-field lateral offset of the structures left of the axis (NaN if none)
  double wall_right = 0.0;      ///< same on the right (negative)
  double evidence_range = 0.0;  ///< farthest distance where a wall line confirms the axis; beyond it the axis is
                                ///< an extrapolation and the envelope must shrink faster
  double blocked_range = 0.0;   ///< where infrastructure crosses the corridor (the tunnel turns out of sight);
                                ///< +inf when the corridor stays open over the whole length
  double y0 = 0.0;
  double theta0 = 0.0;
  double cost = 0.0;
  bool valid = false;

  double length() const { return static_cast<double>(lateral.size()) * x_res; }
  float lateralAt(float x) const { return interpolate(lateral, x); }
  float bedAt(float x) const { return interpolate(bed, x); }
  float headingAt(float x) const { return interpolate(heading, x); }
  float spreadAt(float x) const { return interpolate(spread, x); }

private:
  float interpolate(const std::vector<float> & values, float x) const;
};

/// Estimates the corridor by a diverse beam search over (initial offset, heading, per-segment curvature):
/// rail detections drive the near segments, the requirement that the train envelope passes through the
/// tunnel without touching structures drives the far ones.
class CorridorEstimator
{
public:
  explicit CorridorEstimator(const CorridorParams & params = CorridorParams());

  /// `dt` is the time since the frame `previous` was estimated from [s]; it scales the limits that follow from
  /// how far the train can have moved.
  Corridor estimate(
    const TrackCloud & cloud, const std::vector<AxisDetection> & axis_detections, const AxisModel & axis,
    const Corridor * previous, double dt = 0.1);

  const CorridorParams & params() const { return params_; }

  /// Structure grids of the last estimate (track frame, rows along x, columns across y), for diagnostics.
  const OccupancyGrid & upperGrid() const { return upper_; }
  const OccupancyGrid & lowerGrid() const { return lower_; }

  /// True if the track-frame point falls into a cell of a long or tall structure found in the last estimate
  /// (walls, columns, platform edges). Compact objects such as people or boxes are never structures.
  bool isStructure(float x, float y, float z) const;

  /// True if a structure cell lies within `lateral_radius` across and one cell along the track of the point.
  /// Points in the low band (below the clearance envelope) are tested against the track structures instead:
  /// rails and anything running along the track.
  bool touchesStructure(float x, float y, float z, float lateral_radius) const;

private:
  void buildLateralGrids(const TrackCloud & cloud);
  /// Offset of the most consistent structure line on one side (+1 left, -1 right) of the reference axis within the
  /// near-field window, NaN if no line is seen in enough rows.
  double measureWallOffset(const std::vector<float> & reference, int side) const;
  void searchLateral(
    const std::vector<AxisDetection> & detections, const AxisModel & axis, const Corridor * previous, double dt,
    Corridor & out);
  /// Distance at which a long structure crosses the corridor cross-section, +inf if it stays open.
  double blockedRange(const Corridor & out) const;
  void searchVertical(const TrackCloud & cloud, const Corridor * previous, Corridor & out);
  void buildAlignedGrids(const TrackCloud & cloud, const Corridor & corridor);

  CorridorParams params_;
  OccupancyGrid upper_;
  OccupancyGrid lower_;
  OccupancyGrid side_;
  OccupancyGrid aligned_upper_;
  OccupancyGrid aligned_lower_;
  OccupancyGrid aligned_low_;
  std::vector<float> aligned_lateral_;
  std::vector<float> aligned_bed_;
  double aligned_x_res_ = 0.5;
  std::vector<double> kappa_grid_;
  std::vector<double> grade_grid_;
  mutable std::vector<int> blocked_rows_;  ///< scratch: structure cells inside the envelope per row
};

}  // namespace tod
