#pragma once

#include <functional>
#include <limits>
#include <vector>

#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"
#include "tunnel_obstacle_detector/core/types.hpp"

namespace tod
{

struct DetectorParams
{
  // danger zone = train clearance envelope: stepped cross-section around the track axis, heights above the bed
  double zone_split_height = 0.9;
  double zone_half_width_lower = 1.10;  ///< below split height: 5 cm of margin over the car half width, because
                                        ///< the axis uncertainty is subtracted from it and objects lying on the
                                        ///< track are the critical case
  double zone_half_width_upper = 1.05;  ///< the customer defines the clearance as the car cross-section,
                                        ///< 3 m high and 2.1 m wide
  double zone_bottom = 0.30;
  /// objects lying on the track (a fallen person is ~0.35 m) are checked from this height up to zone_bottom,
  /// but only where the track itself is not: not on a rail, not on anything running along the track
  double zone_low_bottom = 0.15;

  /// ... and only this close: further away the bed profile is less certain than such an object is high
  double low_max_distance = 75.0;

  /// The customer's minimum obstacle, 300 x 300 x 100 mm, is *lower* than the track's own details (0.16-0.25 m),
  /// so no height threshold can separate them. A band below the low one takes what height cannot: an object that
  /// is compact, lies between the rails, rises above the floor actually seen there, and is close enough that the
  /// floor is measured rather than extrapolated. Such a cluster goes to the tracker as `weak`.
  double zone_very_low_bottom = 0.05;    ///< above the local track bed [m]: a 0.1 m object has to fit *inside*
                                         ///< the band, so it starts just above the floor's own noise
  double very_low_max_distance = 25.0;   ///< 0 disables the band; beyond 25 m the rings that graze the floor are
                                         ///< further apart than the joining radius and every duct breaks into
                                         ///< "objects" again
  /// Closer than this the lidar resolves the four-foot itself: concrete joints, duct edges and cable clips rise
  /// the same 0.06-0.08 m over the same 0.3 m, and at 2-5 m they gave 12.9% of frames a false DANGER.
  double very_low_min_distance = 12.0;
  /// What this band finds is by construction at the limit of what the data can tell apart, so it is reported one
  /// level down: the operator sees the object, and a mistake costs a warning rather than an emergency stop.
  bool very_low_warning_only = true;
  double very_low_half_width = 0.60;     ///< strictly *between* the rails: the rail foot, its fasteners and the
                                         ///< baseplates sit at 0.65-0.9 m and rise 0.07-0.14 m above the bed -
                                         ///< with 0.85 m they produced ~3 false objects per frame at 20 m
  double very_low_max_length = 0.8;      ///< along the track [m]: details of the track are metres long
  double very_low_max_width = 0.8;       ///< across the track [m]
  double very_low_rise = 0.06;           ///< above the ground actually seen around it [m]
  /// Rings graze the floor metres apart (spacing ~ elevation step * x^2 / lidar height: 0.65 m at 20 m, 2.6 m at
  /// 40 m). Without joining them a duct running along the track breaks into a row of compact "objects", which is
  /// exactly what the length test is meant to catch.
  double very_low_eps_quad = 2.5e-3;   ///< measured spacing is 1.67e-3 * x^2; the radius needs a margin over it
  double very_low_eps_max = 1.5;
  double zone_top = 3.0;                ///< ... and 3 m high
  double lateral_sigma0 = 0.05;         ///< axis uncertainty at the lidar [m]; the zone shrinks by it
  double lateral_sigma_per_m = 0.001;   ///< axis uncertainty growth with distance
  double lateral_sigma_beyond_per_m = 0.010;  ///< extra growth beyond the distance where walls still confirm the axis
  double lateral_shrink_max = 0.90;     ///< the envelope never shrinks below +-(half width - this)
  double bottom_per_m = 0.001;          ///< bed height uncertainty growth: the zone bottom rises with distance

  // warning zone = danger zone widened laterally
  double warning_margin = 1.0;
  double warning_bottom = 0.75;  ///< above the contact rail cover
  double warning_top = 2.5;
  /// beyond this the axis is too uncertain to tell "next to the envelope" from "on the wall" ...
  double warning_max_distance = 60.0;
  /// ... but that was set before the axis error was measured: where walls still confirm the axis it is 0.11 m at
  /// 100 m (docs/experiments.md § 4), so the band may follow the confirmed length instead of a fixed number.
  bool warning_to_evidence = false;
  int min_warning_points = 5;

  /// detection stops where near-optimal corridor hypotheses disagree by more than this [m]
  double max_axis_spread = 0.9;

  double x_min = 1.5;
  double x_max = 200.0;

  // clustering
  double eps_near = 0.35;
  double eps_far = 0.7;
  double eps_far_from = 60.0;
  /// rings hit a flat object metres apart (spacing ~ x^2 * elevation step / lidar height), so returns from one
  /// object lying on the track are far apart along the track
  double eps_low_per_m = 0.02;
  double eps_low_max = 1.5;
  int min_points = 4;
  int min_danger_points = 2;
  double danger_fraction = 0.2;

  /// clusters with more than this share of points on structure cells (walls, columns) are infrastructure
  double max_structure_fraction = 0.5;
  /// warning-level clusters touching a structure (cabinets, platform edges, signals) are infrastructure
  double warning_touch_radius = 0.25;
  double max_warning_touch_fraction = 0.2;
  /// far away the axis is uncertain: danger-level clusters next to structures are infrastructure too
  double far_touch_from = 60.0;
  double far_touch_radius = 0.5;
  double max_far_touch_fraction = 0.3;

  /// objects must rise above the locally observed ground by min(object_height_far, object_height_near + k * x);
  /// flatter clusters are rails, sleepers, switch parts
  double object_height_near = 0.20;
  double object_height_per_m = 0.0;
  double object_height_far = 0.20;
  /// An object lying on the track is sampled by the few rings that graze it: at 50 m they are 0.10 m apart in
  /// height, so a 0.35 m object shows up as one to three lines at 0.14, 0.24 and 0.35 m above the floor, and which
  /// of them exists changes from frame to frame. A lower rise threshold for such clusters was measured and
  /// rejected: at 0.13 m it lets track details through (11 more false frames on the station recording) because
  /// their rise, width and length are the same as a lying person's. What stays is the rule that a low cluster is
  /// judged only against a floor that was really seen, never against the extrapolated profile.
  /// Raised from 0.20 to 0.26 m on the customer's 20-minute recording: flat track details there (ducts, covers)
  /// rise 0.20-0.25 m above the floor and gave 49 of its 70 false DANGER frames. A lying person (0.35 m) and the
  /// object on the rails (0.5 m) clear it, and close-range detection of lying objects stays at 100%.
  double object_height_low = 0.26;
  bool low_requires_ground = true;
  double low_cluster_margin = 0.1;  ///< a cluster whose top is this far above the envelope bottom is "low"
  /// The floor is sampled by rings whose spacing along the track grows as elevation_step * x^2 / lidar height:
  /// 4 m at 50 m, 9 m at 75 m. A fixed 2 m window therefore finds no ground at all around a far object, and the
  /// height falls back to the extrapolated bed profile - exactly the quantity that is uncertain there.
  double ground_window_along = 2.0;
  double ground_window_quad = 2.4e-3;  ///< window grows as this * x^2 [m]
  double ground_window_max = 15.0;
  double ground_window_across = 2.5;
  double ground_max_above_bed = 1.0;  ///< returns higher than this above the bed profile are not ground candidates

  /// a cluster needs at least max(min_points, density_factor * beams hitting a reference small object) returns;
  /// the reference object is reference_width x reference_height seen with the given angular resolution
  double reference_width = 0.4;
  double reference_height = 0.5;
  /// of a low object only the face towards the train is seen, so fewer returns are expected
  double reference_height_low = 0.25;
  double azimuth_resolution_deg = 0.1;
  double elevation_resolution_deg = 0.125;
  double density_factor = 0.4;
  /// Keep clusters that fail the density test (marked `weak`) instead of dropping them - but only where the test
  /// cannot discriminate anyway: when it demands no more than this many returns. Close to the train a real object
  /// gives hundreds of returns, so a sparse cluster there is clutter and is still dropped.
  bool keep_weak_clusters = true;
  double weak_max_required = 10.0;
  /// Far away an object gives a handful of returns per frame - 3 after dual returns are merged at 175 m - while
  /// the smallest cluster we accept is 4. But the object does not move and the train does: in coordinates tied to
  /// the track (distance along it, offset from its axis) the returns of several frames fall on the same place, and
  /// only the driven distance is needed to bring them together (core/track_advance.hpp). Beyond `accum_from` the
  /// clustering therefore works on the returns of the last `accum_frames` frames - and asks for *more* points than
  /// a single frame ever could, because an empty tunnel accumulates nothing inside the clearance envelope there.
  int accum_frames = 0;              ///< 0 disables the accumulation
  double accum_from = 120.0;         ///< [m] beyond this the accumulated clustering runs *in addition*
  int accum_min_points = 10;         ///< a cluster of the accumulated returns needs at least this many
  /// An obstacle stands on the track, so its returns of several frames land on the same place and the cluster
  /// stays short along the track. Anything that moves with the train smears over the driven distance (6 m over
  /// five frames) and is not what this is for.
  double accum_max_length = 1.5;     ///< [m] along the track
  /// ... and if the current frame alone already found something there, the accumulated cluster is the same object
  double accum_merge_gap = 3.0;      ///< [m]
  /// A thing that stands on the track is seen again and again in the same place, so its accumulated cluster holds
  /// returns of several frames. A trail left by something moving with the train breaks into short pieces, and each
  /// piece comes from one frame only - which is what this asks about.
  int accum_min_frames = 3;
  /// An obstacle stands on the bed, and at this range the lidar still sees its lower part (the ray to the feet of
  /// a person at 150 m is only half a degree below the horizon). What accumulates *above* that is the tunnel:
  /// brackets, portals, the vault brought into the envelope by the extrapolated axis.
  double accum_max_bottom = 1.0;     ///< [m] above the bed profile
  /// Beyond `accum_from` the axis is an extrapolation and the evidence is a dozen returns spread over five frames.
  /// Reporting that one level down says exactly as much as it is worth: something is standing there.
  bool accum_warning_only = true;
  double max_bottom_height = 1.2;
  /// A torn cable hanging into the clearance is an obstacle the customer explicitly asked for, so the rule
  /// above applies only far away, where a "hanging" cluster is far more likely a ceiling fixture that the
  /// axis error has dragged into the envelope (measured: dropping the rule everywhere costs 1.1% of frames,
  /// and every one of those false alarms sits beyond 100 m).
  double floating_from = 80.0;
  double max_bottom_height_per_m = 0.004;  ///< the bed profile itself is less certain far away
  /// far away a single lidar ring grazing the bed or the ceiling leaves a thin arc of returns without extent in
  /// height or along the track. Such an arc crosses the whole tunnel, so the same ring is seen at the same height
  /// on both sides of the envelope; a real object is not.
  double min_far_extent = 0.15;
  double surface_side_min = 0.9;    ///< lateral band just outside the envelope where the arc must continue [m]
  double surface_side_max = 1.7;
  double surface_height_bin = 0.20;  ///< height resolution of that check [m]
  int surface_side_points = 2;       ///< returns needed on each side
};

/// Why a cluster was dropped (kKept for reported candidates).
enum class RejectReason : uint8_t
{
  kKept = 0,
  kStructure = 1,     ///< mostly on wall / column cells
  kTouch = 2,         ///< warning-level cluster touching a structure
  kFarTouch = 3,       ///< far danger-level cluster next to a structure
  kLowAboveGround = 4, ///< does not rise enough above the ground seen around it
  kSparse = 5,         ///< fewer returns than even a small object would give at this distance
  kFloating = 6,       ///< bottom far above the bed: hanging or distant structure, not standing on the track
  kFlat = 7,           ///< thin arc from one ring grazing a surface, no extent in height or along the track
  kVeryLow = 8         ///< in the band below the low one, but too long, too wide or barely above the floor
};

struct Cluster
{
  Level level = Level::kWarning;
  RejectReason reason = RejectReason::kKept;
  /// fewer returns than a reference object of that size would give at this distance. Such a cluster is not thrown
  /// away: the organizers' smallest obstacle (300 x 300 x 100 mm) gives 3-6 returns at 56 m, which is exactly this
  /// case. It is passed to the tracker instead, where it needs more frames of evidence than a dense one.
  bool weak = false;
  int num_points = 0;
  int danger_points = 0;
  int structure_points = 0;
  float x_min = 0.0F;         ///< nearest point along the track [m]
  float x_max = 0.0F;
  float lat_min = 0.0F;       ///< relative to the corridor axis, left positive
  float lat_max = 0.0F;
  float lat_center = 0.0F;
  float h_min = 0.0F;         ///< above the local track bed
  float h_max = 0.0F;
  float above_ground = 0.0F;  ///< top of the cluster above the locally observed ground
  float cx = 0.0F;            ///< centroid, track frame
  float cy = 0.0F;
  float cz = 0.0F;
  std::vector<uint32_t> points;  ///< indices into the track cloud
};

/// Zone labels per track point.
enum ZoneLabel : uint8_t
{
  kOutside = 0,
  kWarningZone = 1,
  kDangerZone = 2,
};

/// Finds returns inside the danger/warning zones around the corridor and groups them into clusters,
/// dropping clusters that belong to tunnel structures.
class ObstacleDetector
{
public:
  /// (x, y, z, lateral_radius) -> is there a tunnel structure at / next to this track-frame point.
  /// radius 0 asks for the point's own cell.
  using StructureOracle = std::function<bool(float x, float y, float z, float radius)>;

  explicit ObstacleDetector(const DetectorParams & params = DetectorParams());

  /// `advance` is how far the train drove since the previous call [m]; NaN or a negative value empties the
  /// accumulation buffer, which only costs the far range its extra returns for a few frames.
  std::vector<Cluster> detect(
    const TrackCloud & cloud, const Corridor & corridor, const StructureOracle & is_structure,
    std::vector<uint8_t> * labels = nullptr, std::vector<Cluster> * rejected = nullptr,
    double advance = std::numeric_limits<double>::quiet_NaN());

  const DetectorParams & params() const { return params_; }

  /// Length of the corridor used by the last detect() call (limited by axis uncertainty).
  double trustedLength() const { return trusted_length_; }

private:
  /// Height of the floor around a cluster, measured as the median offset of the lowest returns from the bed
  /// profile (so a wide window does not pick up the grade) and put back at the cluster's distance.
  float localGround(const Corridor & corridor, float x_min, float x_max, float y_centre) const;


  DetectorParams params_;
  double trusted_length_ = 0.0;
  // lowest return per 1 m x 0.5 m cell, for the ground check
  std::vector<float> ground_min_;
  int ground_rows_ = 0;
  int ground_cols_ = 0;
  std::vector<uint32_t> candidates_;
  /// One return kept for the far accumulation: where it is relative to the track, what zone it fell into, and
  /// (for the current frame only) its index in the cloud.
  struct AccPoint
  {
    float s = 0.0F;        ///< along the track [m]
    float e = 0.0F;        ///< offset from the axis, left positive [m]
    float h = 0.0F;        ///< above the bed profile [m]
    uint8_t label = 0;     ///< kDangerZone / kWarningZone
    uint8_t structure = 0;
    uint8_t frame = 0;     ///< which buffered frame it came from
    uint32_t index = 0xFFFFFFFFU;
  };

  /// Clusters built from the returns of the last frames, for the range where one frame is not enough.
  void accumulatedClusters(
    const TrackCloud & cloud, const Corridor & corridor, const StructureOracle & is_structure,
    std::vector<Cluster> & clusters, std::vector<Cluster> * rejected);

  std::vector<std::vector<AccPoint>> accum_;  ///< newest last; `s` of older frames is kept up to date
  std::vector<std::size_t> low_;   ///< indices into candidates_ that sit below the envelope bottom
  std::vector<std::size_t> very_low_;  ///< ... and of those, the ones below the low band as well
  // returns beside the envelope per (distance cell, height bin, side): a surface grazed by one ring shows up on
  // both sides at the same height
  std::vector<uint16_t> side_counts_;
  int side_rows_ = 0;
  int side_bins_ = 0;
  int sideCount(float x, float h, int side) const;
  int sideCountNear(float x, float h, int side) const;
  std::vector<uint8_t> labels_;
};

}  // namespace tod
