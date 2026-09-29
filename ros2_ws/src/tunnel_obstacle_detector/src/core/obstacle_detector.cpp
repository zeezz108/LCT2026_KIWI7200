#include "tunnel_obstacle_detector/core/obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace tod
{

namespace
{
constexpr double kSideCellAlong = 0.5;
constexpr double kSideHeightMin = -1.0;
constexpr double kGroundCellAlong = 1.0;
constexpr double kGroundCellAcross = 0.5;
constexpr double kGroundHalfWidth = 20.0;

inline int64_t voxelKey(int ix, int iy, int iz)
{
  return (static_cast<int64_t>(ix + 32768) << 32) | (static_cast<int64_t>(iy + 32768) << 16) |
         static_cast<int64_t>(iz + 32768);
}
}  // namespace

float ObstacleDetector::localGround(
  const Corridor & corridor, float x_min, float x_max, float y_centre) const
{
  const DetectorParams & p = params_;
  const double x_mid = 0.5 * (x_min + x_max);
  const double window = std::clamp(p.ground_window_quad * x_mid * x_mid, p.ground_window_along, p.ground_window_max);
  const int r0 = std::max(0, static_cast<int>(std::floor((x_min - window) / kGroundCellAlong)));
  const int r1 = std::min(ground_rows_ - 1, static_cast<int>(std::floor((x_max + window) / kGroundCellAlong)));
  const int c0 = std::max(0, static_cast<int>(std::floor((y_centre - p.ground_window_across + kGroundHalfWidth) / kGroundCellAcross)));
  const int c1 = std::min(ground_cols_ - 1, static_cast<int>(std::floor((y_centre + p.ground_window_across + kGroundHalfWidth) / kGroundCellAcross)));
  std::vector<float> offsets;
  for (int r = r0; r <= r1; ++r) {
    // the window is long, so the grade must not enter the estimate: measure every cell against the bed profile
    const float bed = corridor.bedAt(static_cast<float>((r + 0.5) * kGroundCellAlong));
    for (int c = c0; c <= c1; ++c) {
      const float v = ground_min_[static_cast<std::size_t>(r) * ground_cols_ + c];
      if (std::isfinite(v)) {
        offsets.push_back(v - bed);
      }
    }
  }
  if (offsets.size() < 4) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  std::nth_element(offsets.begin(), offsets.begin() + offsets.size() / 2, offsets.end());
  return offsets[offsets.size() / 2] + corridor.bedAt(static_cast<float>(x_mid));
}

int ObstacleDetector::sideCount(float x, float h, int side) const
{
  const int row = static_cast<int>(x / kSideCellAlong);
  const int bin = static_cast<int>((h - kSideHeightMin) / params_.surface_height_bin);
  if (row < 0 || row >= side_rows_ || bin < 0 || bin >= side_bins_) {
    return 0;
  }
  return side_counts_[(static_cast<std::size_t>(row) * side_bins_ + bin) * 2 + side];
}

int ObstacleDetector::sideCountNear(float x, float h, int side) const
{
  const float bin = static_cast<float>(params_.surface_height_bin);
  return sideCount(x, h - bin, side) + sideCount(x, h, side) + sideCount(x, h + bin, side);
}

ObstacleDetector::ObstacleDetector(const DetectorParams & params)
: params_(params)
{
}

void ObstacleDetector::accumulatedClusters(
  const TrackCloud & cloud, const Corridor & corridor, const StructureOracle & is_structure,
  std::vector<Cluster> & clusters, std::vector<Cluster> * rejected)
{
  const DetectorParams & p = params_;
  std::vector<AccPoint> all;
  uint8_t frame_id = 0;
  for (const auto & frame : accum_) {
    for (AccPoint q : frame) {
      q.frame = frame_id;
      all.push_back(q);
    }
    ++frame_id;
  }
  if (static_cast<int>(all.size()) < p.accum_min_points) {
    return;
  }
  // euclidean clustering in track coordinates; the radius is the far one, the returns of one object land within it
  const double eps = p.eps_far;
  const double eps2 = eps * eps;
  std::vector<char> seen(all.size(), 0);
  std::vector<std::size_t> queue;
  for (std::size_t seed = 0; seed < all.size(); ++seed) {
    if (seen[seed]) {
      continue;
    }
    queue.assign(1, seed);
    seen[seed] = 1;
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
      const AccPoint & a = all[queue[qi]];
      for (std::size_t j = 0; j < all.size(); ++j) {
        if (seen[j]) {
          continue;
        }
        const AccPoint & b = all[j];
        const double d2 = (a.s - b.s) * (a.s - b.s) + (a.e - b.e) * (a.e - b.e) + (a.h - b.h) * (a.h - b.h);
        if (d2 < eps2) {
          seen[j] = 1;
          queue.push_back(j);
        }
      }
    }
    if (static_cast<int>(queue.size()) < p.accum_min_points) {
      continue;
    }
    float s_min = all[queue[0]].s, s_max = s_min;
    for (std::size_t k : queue) {
      s_min = std::min(s_min, all[k].s);
      s_max = std::max(s_max, all[k].s);
    }
    if (s_max - s_min > p.accum_max_length) {
      continue;  // smeared over the driven distance: whatever this is, it is not standing on the track
    }
    uint32_t frames_seen = 0;
    for (std::size_t k : queue) {
      frames_seen |= 1U << all[k].frame;
    }
    if (__builtin_popcount(frames_seen) < p.accum_min_frames) {
      continue;  // one frame only: a piece of a trail, or noise that happened to fall together
    }
    const bool already_found = std::any_of(
      clusters.begin(), clusters.end(), [&](const Cluster & e) {
        return e.x_min < s_max + p.accum_merge_gap && e.x_max > s_min - p.accum_merge_gap;
      });
    if (already_found) {
      continue;  // this frame alone saw it too; one report is enough
    }

    Cluster c;
    c.num_points = static_cast<int>(queue.size());
    c.x_min = c.x_max = all[queue[0]].s;
    c.lat_min = c.lat_max = all[queue[0]].e;
    c.h_min = c.h_max = all[queue[0]].h;
    double ss = 0.0, se = 0.0, sh = 0.0;
    for (std::size_t k : queue) {
      const AccPoint & q = all[k];
      c.x_min = std::min(c.x_min, q.s);
      c.x_max = std::max(c.x_max, q.s);
      c.lat_min = std::min(c.lat_min, q.e);
      c.lat_max = std::max(c.lat_max, q.e);
      c.h_min = std::min(c.h_min, q.h);
      c.h_max = std::max(c.h_max, q.h);
      ss += q.s;
      se += q.e;
      sh += q.h;
      c.danger_points += q.label == kDangerZone ? 1 : 0;
      c.structure_points += q.structure;
      if (q.index != 0xFFFFFFFFU) {
        c.points.push_back(q.index);
      }
    }
    const double n = static_cast<double>(c.num_points);
    c.cx = static_cast<float>(ss / n);
    c.lat_center = static_cast<float>(se / n);
    c.cy = static_cast<float>(c.lat_center + corridor.lateralAt(c.cx));
    c.cz = static_cast<float>(sh / n + corridor.bedAt(c.cx));
    const bool danger = c.danger_points >= p.min_danger_points && c.danger_points >= p.danger_fraction * n;
    c.level = danger ? Level::kDanger : Level::kWarning;
    if (!danger) {
      continue;  // the warning band is not checked at this range anyway
    }

    // the same physical tests as for a single frame, minus the ones that only apply near the train
    const float ground = localGround(corridor, c.x_min, c.x_max, c.cy);
    c.above_ground = std::isfinite(ground) ?
      static_cast<float>(c.h_max + corridor.bedAt(c.cx) - ground) : c.h_max;
    if (c.h_min > p.accum_max_bottom) {
      c.reason = RejectReason::kFloating;  // not standing on the track: this band is where the tunnel itself is
    } else if (c.above_ground < std::min(p.object_height_far, p.object_height_near + p.object_height_per_m * c.x_min)) {
      c.reason = RejectReason::kLowAboveGround;
    } else if (c.structure_points > p.max_structure_fraction * n) {
      c.reason = RejectReason::kStructure;
    } else if (is_structure) {
      int far_touching = 0;
      for (std::size_t k : queue) {
        const AccPoint & q = all[k];
        if (is_structure(q.s, q.e + corridor.lateralAt(q.s), q.h + corridor.bedAt(q.s),
          static_cast<float>(p.far_touch_radius)))
        {
          ++far_touching;
        }
      }
      if (far_touching > p.max_far_touch_fraction * n) {
        c.reason = RejectReason::kFarTouch;
      }
    }
    if (c.reason != RejectReason::kKept) {
      if (rejected) {
        rejected->push_back(std::move(c));
      }
      continue;
    }
    if (p.accum_warning_only) {
      c.level = Level::kWarning;
    }
    clusters.push_back(std::move(c));
  }
  (void)cloud;
}

std::vector<Cluster> ObstacleDetector::detect(
  const TrackCloud & cloud, const Corridor & corridor, const StructureOracle & is_structure,
  std::vector<uint8_t> * labels_out, std::vector<Cluster> * rejected, double advance)
{
  const DetectorParams & p = params_;
  labels_.assign(cloud.size(), kOutside);
  candidates_.clear();
  low_.clear();
  very_low_.clear();
  const double warning_limit = p.warning_to_evidence ?
    std::max(p.warning_max_distance, corridor.evidence_range) : p.warning_max_distance;
  const bool accumulate = p.accum_frames > 0;
  std::vector<AccPoint> far_points;
  if (accumulate) {
    // the buffer only survives while the driven distance is known: without it the frames cannot be aligned
    if (!(advance >= 0.0) || !std::isfinite(advance)) {
      accum_.clear();
    } else {
      for (auto & frame : accum_) {
        for (auto & q : frame) {
          q.s -= static_cast<float>(advance);
          q.index = 0xFFFFFFFFU;
        }
      }
    }
  } else {
    accum_.clear();
  }
  if (rejected) {
    rejected->clear();
  }
  // the corridor is trusted up to where near-optimal axis hypotheses start to disagree strongly
  double x_end = std::min(p.x_max, corridor.length());
  for (std::size_t i = 0; i < corridor.spread.size(); ++i) {
    if (corridor.spread[i] > p.max_axis_spread) {
      x_end = std::min(x_end, i * corridor.x_res);
      break;
    }
  }
  trusted_length_ = x_end;

  ground_rows_ = static_cast<int>(std::ceil(p.x_max / kGroundCellAlong));
  ground_cols_ = static_cast<int>(std::ceil(2.0 * kGroundHalfWidth / kGroundCellAcross));
  ground_min_.assign(static_cast<std::size_t>(ground_rows_) * ground_cols_, std::numeric_limits<float>::infinity());
  side_rows_ = static_cast<int>(std::ceil(p.x_max / kSideCellAlong));
  side_bins_ = static_cast<int>(std::ceil((3.0 - kSideHeightMin) / p.surface_height_bin));
  side_counts_.assign(static_cast<std::size_t>(side_rows_) * side_bins_ * 2, 0);

  // 1. zone labelling (and lowest return per ground cell)
  for (uint32_t i = 0; i < cloud.size(); ++i) {
    const TrackPoint & pt = cloud[i];
    // only returns near the expected bed are ground candidates (cells seeing just the ceiling must stay empty)
    if (pt.x >= 0.0F && pt.x < p.x_max && std::abs(pt.y) < kGroundHalfWidth &&
      pt.z < corridor.bedAt(pt.x) + p.ground_max_above_bed)
    {
      float & g = ground_min_[
        static_cast<std::size_t>(pt.x / kGroundCellAlong) * ground_cols_ +
        static_cast<std::size_t>((pt.y + kGroundHalfWidth) / kGroundCellAcross)];
      g = std::min(g, pt.z);
    }
    if (pt.x < p.x_min || pt.x >= x_end) {
      continue;
    }
    const double signed_lat = pt.y - corridor.lateralAt(pt.x);
    const double lat = std::abs(signed_lat);
    const double h = pt.z - corridor.bedAt(pt.x);
    if (lat > p.surface_side_min && lat < p.surface_side_max && h > kSideHeightMin && h < 3.0) {
      const int row = static_cast<int>(pt.x / kSideCellAlong);
      const int bin = static_cast<int>((h - kSideHeightMin) / p.surface_height_bin);
      if (row >= 0 && row < side_rows_ && bin >= 0 && bin < side_bins_) {
        uint16_t & cell = side_counts_[(static_cast<std::size_t>(row) * side_bins_ + bin) * 2 +
          (signed_lat > 0.0 ? 0 : 1)];
        cell = static_cast<uint16_t>(std::min<int>(cell + 1, 65535));
      }
    }
    const double beyond = std::max(0.0, pt.x - corridor.evidence_range);
    const double shrink = std::min(p.lateral_shrink_max, std::max<double>(
        p.lateral_sigma0 + p.lateral_sigma_per_m * pt.x + p.lateral_sigma_beyond_per_m * beyond,
        corridor.spreadAt(pt.x)));
    const double bottom = p.zone_bottom + p.bottom_per_m * pt.x;
    const double half_width = h <= p.zone_split_height ? p.zone_half_width_lower : p.zone_half_width_upper;
    const double low_bottom = p.zone_low_bottom + p.bottom_per_m * pt.x;
    if (h > bottom && h < p.zone_top && lat < half_width - shrink) {
      labels_[i] = kDangerZone;
    } else if (pt.x < p.low_max_distance && h > low_bottom && h <= bottom &&
      lat < p.zone_half_width_lower - shrink)
    {
      // a low object between the rails: candidate unless it is the track itself
      if (is_structure && is_structure(pt.x, pt.y, pt.z, 0.0F)) {
        continue;
      }
      labels_[i] = kDangerZone;
      low_.push_back(candidates_.size());
    } else if (pt.x < p.very_low_max_distance && pt.x >= p.very_low_min_distance &&
      h > p.zone_very_low_bottom + p.bottom_per_m * pt.x && h <= low_bottom && lat < p.very_low_half_width)
    {
      // below the low band: the customer's 300 x 300 x 100 mm object. What it is, is decided on the cluster
      // (compact, between the rails, rising above the floor seen there) - height alone cannot tell it from the track
      if (is_structure && is_structure(pt.x, pt.y, pt.z, 0.0F)) {
        continue;
      }
      // deliberately a *warning* point: it must not turn a cluster that also holds normal returns into a DANGER
      // one, and on its own it is exactly what this band promises - "something is there", one level down
      labels_[i] = kWarningZone;
      low_.push_back(candidates_.size());
      very_low_.push_back(candidates_.size());
    } else if (pt.x < warning_limit && h > std::max(bottom, p.warning_bottom) && h < p.warning_top &&
      lat < p.zone_half_width_upper + p.warning_margin - shrink)
    {
      if (is_structure && is_structure(pt.x, pt.y, pt.z, 0.0F)) {
        continue;  // wall or column inside the warning band: never a candidate
      }
      labels_[i] = kWarningZone;
    } else {
      continue;
    }
    if (accumulate && pt.x >= p.accum_from) {
      // beyond that distance a single frame rarely holds enough returns to cluster; this one also waits for the
      // next frames, without being taken away from the ordinary per-frame path
      AccPoint q;
      q.s = pt.x;
      q.e = static_cast<float>(signed_lat);
      q.h = static_cast<float>(h);
      q.label = labels_[i];
      q.structure = (is_structure && is_structure(pt.x, pt.y, pt.z, 0.0F)) ? 1U : 0U;
      q.index = i;
      far_points.push_back(q);
    }
    candidates_.push_back(i);
  }
  if (accumulate) {
    accum_.push_back(std::move(far_points));
    while (static_cast<int>(accum_.size()) > p.accum_frames) {
      accum_.erase(accum_.begin());
    }
  }

  // 2. euclidean clustering on a voxel hash, radius grows with distance
  std::vector<char> is_low(candidates_.size(), 0);
  for (std::size_t k : low_) {
    if (k < is_low.size()) {
      is_low[k] = 1;
    }
  }
  std::vector<char> is_very_low(candidates_.size(), 0);
  for (std::size_t k : very_low_) {
    if (k < is_very_low.size()) {
      is_very_low[k] = 1;
    }
  }
  const double voxel = std::max({p.eps_near, p.eps_far, p.eps_low_max});
  std::unordered_map<int64_t, std::vector<uint32_t>> grid;
  grid.reserve(candidates_.size() * 2 + 1);
  for (uint32_t k = 0; k < candidates_.size(); ++k) {
    const TrackPoint & pt = cloud[candidates_[k]];
    grid[voxelKey(
        static_cast<int>(std::floor(pt.x / voxel)), static_cast<int>(std::floor(pt.y / voxel)),
        static_cast<int>(std::floor(pt.z / voxel)))].push_back(k);
  }

  std::vector<char> visited(candidates_.size(), 0);
  std::vector<uint32_t> queue;
  std::vector<Cluster> clusters;
  for (uint32_t seed = 0; seed < candidates_.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    queue.clear();
    queue.push_back(seed);
    visited[seed] = 1;
    for (std::size_t q = 0; q < queue.size(); ++q) {
      const TrackPoint & a = cloud[candidates_[queue[q]]];
      const int ix = static_cast<int>(std::floor(a.x / voxel));
      const int iy = static_cast<int>(std::floor(a.y / voxel));
      const int iz = static_cast<int>(std::floor(a.z / voxel));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto it = grid.find(voxelKey(ix + dx, iy + dy, iz + dz));
            if (it == grid.end()) {
              continue;
            }
            for (uint32_t k : it->second) {
              if (visited[k]) {
                continue;
              }
              if (is_very_low[queue[q]] != is_very_low[k]) {
                continue;  // the band below the low one clusters on its own: a chain of floor returns must not
              }            // attach itself to a real object and drag its reported distance towards the train
              const TrackPoint & b = cloud[candidates_[k]];
              const double far = std::max(a.x, b.x);
              double eps = far >= p.eps_far_from ? p.eps_far : p.eps_near;
              if (is_low[queue[q]] && is_low[k]) {
                eps = std::max(eps, std::min(p.eps_low_max, p.eps_low_per_m * far));
              }
              if (is_very_low[k]) {
                eps = std::max(eps, std::min(p.very_low_eps_max, p.very_low_eps_quad * far * far));
              }
              const double d2 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
              if (d2 < eps * eps) {
                visited[k] = 1;
                queue.push_back(k);
              }
            }
          }
        }
      }
    }
    if (static_cast<int>(queue.size()) < p.min_points) {
      continue;
    }

    // 3. cluster features
    Cluster c;
    c.num_points = static_cast<int>(queue.size());
    c.x_min = c.lat_min = c.h_min = std::numeric_limits<float>::infinity();
    c.x_max = c.lat_max = c.h_max = -std::numeric_limits<float>::infinity();
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    double slat = 0.0;
    int touching = 0;
    int far_touching = 0;
    c.points.reserve(queue.size());
    for (uint32_t k : queue) {
      const uint32_t idx = candidates_[k];
      const TrackPoint & pt = cloud[idx];
      const float lat = pt.y - corridor.lateralAt(pt.x);
      const float h = pt.z - corridor.bedAt(pt.x);
      c.points.push_back(idx);
      c.x_min = std::min(c.x_min, pt.x);
      c.x_max = std::max(c.x_max, pt.x);
      c.lat_min = std::min(c.lat_min, lat);
      c.lat_max = std::max(c.lat_max, lat);
      c.h_min = std::min(c.h_min, h);
      c.h_max = std::max(c.h_max, h);
      sx += pt.x;
      sy += pt.y;
      sz += pt.z;
      slat += lat;
      if (labels_[idx] == kDangerZone) {
        ++c.danger_points;
      }
      if (is_structure) {  // grids are built in raw track-frame heights
        if (is_structure(pt.x, pt.y, pt.z, 0.0F)) {
          ++c.structure_points;
          ++touching;
          ++far_touching;
        } else if (is_structure(pt.x, pt.y, pt.z, static_cast<float>(p.warning_touch_radius))) {
          ++touching;
          ++far_touching;
        } else if (pt.x >= p.far_touch_from && is_structure(pt.x, pt.y, pt.z, static_cast<float>(p.far_touch_radius))) {
          ++far_touching;
        }
      }
    }
    const double n = static_cast<double>(c.num_points);
    c.cx = static_cast<float>(sx / n);
    c.cy = static_cast<float>(sy / n);
    c.cz = static_cast<float>(sz / n);
    c.lat_center = static_cast<float>(slat / n);
    const bool danger = c.danger_points >= p.min_danger_points && c.danger_points >= p.danger_fraction * n;
    c.level = danger ? Level::kDanger : Level::kWarning;
    if (!danger && c.num_points < p.min_warning_points) {
      continue;
    }

    // physical plausibility: enough returns for the distance, and standing on the track bed
    constexpr double kDegToRad = M_PI / 180.0;
    const double dist = std::max(1.0F, c.x_min);
    const bool low_cluster = c.h_max < p.zone_bottom + p.bottom_per_m * dist + p.low_cluster_margin;
    // a cluster that lives entirely below the low band can only be the minimum object - or the track itself
    const bool very_low_cluster = p.very_low_max_distance > 0.0 &&
      c.h_max < p.zone_low_bottom + p.bottom_per_m * dist;
    const double reference_height = low_cluster ? p.reference_height_low : p.reference_height;
    const double beams = (p.reference_width / (dist * p.azimuth_resolution_deg * kDegToRad)) *
      (reference_height / (dist * p.elevation_resolution_deg * kDegToRad));
    const double required_points = std::max<double>(p.min_points, p.density_factor * beams);
    if (c.num_points < required_points) {
      // the density model describes a 0.4 x 0.25 m object; the minimum obstacle is 0.3 x 0.1 m and never passes it,
      // so in the band below the low one density is not the test - compactness, rise and the tracker are
      if (p.keep_weak_clusters && c.num_points >= p.min_points &&
        (required_points <= p.weak_max_required || very_low_cluster))
      {
        c.weak = true;  // too few returns for its size: the tracker decides it over several frames
      } else {
        c.reason = RejectReason::kSparse;
      }
    }
    if (c.reason != RejectReason::kKept) {
      // already rejected above
    } else if (dist >= p.floating_from &&
      c.h_min > p.max_bottom_height + p.max_bottom_height_per_m * dist)
    {
      c.reason = RejectReason::kFloating;
    } else if (dist >= p.far_touch_from &&
      std::max(c.h_max - c.h_min, c.x_max - c.x_min) < p.min_far_extent &&
      sideCountNear(c.x_min, 0.5F * (c.h_min + c.h_max), 0) >= p.surface_side_points &&
      sideCountNear(c.x_min, 0.5F * (c.h_min + c.h_max), 1) >= p.surface_side_points)
    {
      c.reason = RejectReason::kFlat;  // the same ring crosses the whole tunnel at this height: a grazed surface
    }
    if (c.reason != RejectReason::kKept) {
      if (rejected) {
        rejected->push_back(std::move(c));
      }
      continue;
    }

    // rise above the ground actually seen around the object (not the extrapolated bed profile)
    float z_top = -std::numeric_limits<float>::infinity();
    for (uint32_t idx : c.points) {
      z_top = std::max(z_top, cloud[idx].z);
    }
    const float ground = localGround(corridor, c.x_min, c.x_max, c.cy);
    c.above_ground = std::isfinite(ground) ? z_top - ground : c.h_max;
    double min_height = std::min(p.object_height_far, p.object_height_near + p.object_height_per_m * c.x_min);
    if (low_cluster) {
      min_height = p.object_height_low;
    }
    if (very_low_cluster) {
      // the track's own details are metres long and run along it; the minimum object is a compact bump
      if (c.x_max - c.x_min > p.very_low_max_length || c.lat_max - c.lat_min > p.very_low_max_width ||
        !std::isfinite(ground) || z_top - ground < p.very_low_rise)
      {
        c.reason = RejectReason::kVeryLow;
        if (rejected) {
          rejected->push_back(std::move(c));
        }
        continue;
      }
      min_height = p.very_low_rise;
      c.weak = true;  // this band is the riskiest: let the tracker ask for several frames of evidence
      if (p.very_low_warning_only) {
        c.level = Level::kWarning;
      }
    }
    if (low_cluster && p.low_requires_ground && !std::isfinite(ground)) {
      c.reason = RejectReason::kLowAboveGround;  // its height would be measured against the extrapolated profile
      if (rejected) {
        rejected->push_back(std::move(c));
      }
      continue;
    }
    if (c.above_ground < min_height) {
      c.reason = RejectReason::kLowAboveGround;
      if (rejected) {
        rejected->push_back(std::move(c));
      }
      continue;
    }

    if (c.structure_points > p.max_structure_fraction * n) {
      c.reason = RejectReason::kStructure;
    } else if (!danger && touching > p.max_warning_touch_fraction * n) {
      c.reason = RejectReason::kTouch;
    } else if (c.x_min >= p.far_touch_from && far_touching > p.max_far_touch_fraction * n) {
      c.reason = RejectReason::kFarTouch;
    }
    if (c.reason != RejectReason::kKept) {
      if (rejected) {
        rejected->push_back(std::move(c));
      }
      continue;
    }
    clusters.push_back(std::move(c));
  }

  if (accumulate) {
    accumulatedClusters(cloud, corridor, is_structure, clusters, rejected);
  }

  std::sort(clusters.begin(), clusters.end(), [](const Cluster & a, const Cluster & b) {return a.x_min < b.x_min;});
  if (labels_out) {
    *labels_out = labels_;
  }
  return clusters;
}

}  // namespace tod
