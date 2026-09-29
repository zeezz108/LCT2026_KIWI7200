#include "tunnel_obstacle_detector/core/corridor_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_set>

namespace tod
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

inline double sq(double v) {return v * v;}

/// Partial search state at a segment boundary.
struct Node
{
  double cost = 0.0;
  double rank = 0.0;     ///< cost plus the lookahead estimate used for pruning
  double y = 0.0;        ///< offset at the end of the segment
  double theta = 0.0;    ///< heading at the end of the segment
  double kappa = 0.0;    ///< curvature of the segment that ended here
  double y_start = 0.0;  ///< offset at the start of that segment
  double theta_start = 0.0;
  int parent = -1;
  bool has_segment = false;
};

std::vector<double> symmetricGrid(double centre, double span, double step)
{
  std::vector<double> grid;
  const int n = static_cast<int>(std::lround(span / step));
  for (int i = -n; i <= n; ++i) {
    grid.push_back(centre + i * step);
  }
  return grid;
}

/// Keeps the best-ranked candidate per (heading, offset) cell, up to `width` candidates.
std::vector<Node> selectDiverse(std::vector<Node> & candidates, int width, double heading_cell, double offset_cell)
{
  std::sort(candidates.begin(), candidates.end(), [](const Node & a, const Node & b) {return a.rank < b.rank;});
  std::vector<Node> selected;
  std::unordered_set<long long> seen;
  for (const Node & c : candidates) {
    const long long hk = std::llround(c.theta / heading_cell);
    const long long ok = std::llround(c.y / offset_cell);
    const long long key = hk * 1000003LL + ok;
    if (!seen.insert(key).second) {
      continue;
    }
    selected.push_back(c);
    if (static_cast<int>(selected.size()) >= width) {
      break;
    }
  }
  return selected;
}
}  // namespace

float Corridor::interpolate(const std::vector<float> & values, float x) const
{
  if (values.empty()) {
    return 0.0F;
  }
  const double f = x / x_res - 0.5;
  if (f <= 0.0) {
    return values.front();
  }
  const auto i = static_cast<std::size_t>(f);
  if (i + 1 >= values.size()) {
    return values.back();
  }
  const float t = static_cast<float>(f - static_cast<double>(i));
  return values[i] * (1.0F - t) + values[i + 1] * t;
}

CorridorEstimator::CorridorEstimator(const CorridorParams & params)
: params_(params)
{
  const int rows = static_cast<int>(std::lround(params_.x_max / params_.x_res));
  const int cols = static_cast<int>(std::lround(2.0 * params_.y_half / params_.y_res));
  upper_.configure(rows, cols, params_.x_res, params_.y_res, -params_.y_half);
  lower_.configure(rows, cols, params_.x_res, params_.y_res, -params_.y_half);
  const int aligned_cols = static_cast<int>(std::lround(2.0 * params_.aligned_half_width / params_.y_res));
  aligned_upper_.configure(rows, aligned_cols, params_.x_res, params_.y_res, -params_.aligned_half_width);
  aligned_lower_.configure(rows, aligned_cols, params_.x_res, params_.y_res, -params_.aligned_half_width);
  aligned_low_.configure(rows, aligned_cols, params_.x_res, params_.y_res, -params_.aligned_half_width);
  side_.configure(rows, params_.z_cells, params_.x_res, params_.z_res, params_.z_grid_min);
  kappa_grid_ = symmetricGrid(0.0, params_.kappa_max, params_.kappa_step);
  grade_grid_ = symmetricGrid(0.0, params_.grade_max, params_.grade_step);
}

void CorridorEstimator::buildLateralGrids(const TrackCloud & cloud)
{
  const CorridorParams & p = params_;
  upper_.clear();
  lower_.clear();
  for (const TrackPoint & pt : cloud) {
    if (pt.x <= 0.5 || pt.x >= p.x_max || std::abs(pt.y) >= p.y_half) {
      continue;
    }
    if (pt.z > p.band_upper_min && pt.z < p.band_upper_max) {
      upper_.mark(pt.x, pt.y, pt.z);
    } else if (pt.z > p.band_lower_min && pt.z < p.band_lower_max) {
      lower_.mark(pt.x, pt.y);
    }
  }
  // no distance-growing linking here: the axis is not known yet, and a far object on the track must never become a
  // "structure" that the corridor search steers around
  upper_.keepLongStructures(
    p.dilate_along_base, 0.0, 1, p.long_structure_min_length, static_cast<float>(p.tall_low),
    static_cast<float>(p.tall_high));
  lower_.keepLongStructures(p.dilate_along_base, 0.0, 1, p.long_structure_min_length);
  upper_.buildPrefix();
  lower_.buildPrefix();
}

void CorridorEstimator::buildAlignedGrids(const TrackCloud & cloud, const Corridor & corridor)
{
  // same structure test in coordinates relative to the found axis: walls parallel to a curved track keep a
  // constant offset, so their sparse far samples link up
  const CorridorParams & p = params_;
  aligned_upper_.clear();
  aligned_lower_.clear();
  aligned_low_.clear();
  for (const TrackPoint & pt : cloud) {
    if (pt.x <= 0.5 || pt.x >= p.x_max) {
      continue;
    }
    const float lat = pt.y - corridor.lateralAt(pt.x);
    if (std::abs(lat) >= p.aligned_half_width) {
      continue;
    }
    if (pt.z > p.band_upper_min && pt.z < p.band_upper_max) {
      aligned_upper_.mark(pt.x, lat, pt.z);
    } else if (pt.z > p.band_lower_min && pt.z < p.band_lower_max) {
      aligned_lower_.mark(pt.x, lat);
    }
    // low band, measured from the bed profile; the rail strips are left out so that an object lying across the
    // rails does not become part of their (endless) component
    const float h = pt.z - corridor.bedAt(pt.x);
    if (h > p.band_low_min && h < p.band_low_max &&
      std::abs(std::abs(lat) - p.gauge_half) > p.rail_half_width)
    {
      aligned_low_.mark(pt.x, lat);
    }
  }
  // far walls are linked by the x^2-growing dilation only outside the envelope; inside it objects stay separate
  const int inner_lo = static_cast<int>(std::floor((p.aligned_half_width - p.inner_half_width) / p.y_res));
  const int inner_hi = static_cast<int>(std::ceil((p.aligned_half_width + p.inner_half_width) / p.y_res)) - 1;
  aligned_upper_.keepLongStructures(
    p.dilate_along_base, p.dilate_along_quad, 1, p.long_structure_min_length, static_cast<float>(p.tall_low),
    static_cast<float>(p.tall_high), inner_lo, inner_hi);
  aligned_lower_.keepLongStructures(
    p.dilate_along_base, p.dilate_along_quad, 1, p.long_structure_min_length, 0.0F, 0.0F, inner_lo, inner_hi);
  // rails, cable ducts, drainage covers and switch rods run along the track; objects on it do not
  aligned_low_.keepLongStructures(p.dilate_along_base, 0.0, 1, p.low_structure_min_length);
  aligned_lateral_ = corridor.lateral;
  aligned_bed_ = corridor.bed;
  aligned_x_res_ = corridor.x_res;
}

double CorridorEstimator::measureWallOffset(const std::vector<float> & reference, int side) const
{
  const CorridorParams & p = params_;
  const int r0 = std::max(0, static_cast<int>(p.wall_ref_min / p.x_res));
  const int r1 = std::min(upper_.rows(), static_cast<int>(p.wall_ref_max / p.x_res));
  const double tol = p.wall_tolerance;
  const double first = p.search_half_width_upper + tol;
  const int n = static_cast<int>(std::floor((p.wall_scan_max - first) / p.y_res)) + 1;
  if (r1 <= r0 || n <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // rows that have a structure within +-tol of each candidate offset
  std::vector<int> seen(static_cast<std::size_t>(n), 0);
  int best_rows = 0;
  int best_k = -1;
  for (int k = 0; k < n; ++k) {
    const double offset = side * (first + k * p.y_res);
    int rows_seen = 0;
    for (int i = r0; i < r1; ++i) {
      const double y = reference[static_cast<std::size_t>(i)] + offset;
      rows_seen += upper_.count(i, y - tol, y + tol) > 0 ? 1 : 0;
    }
    seen[static_cast<std::size_t>(k)] = rows_seen;
    if (rows_seen > best_rows) {  // nearest line wins ties
      best_rows = rows_seen;
      best_k = k;
    }
  }
  if (best_k < 0 || best_rows < p.wall_ref_min_fraction * (r1 - r0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // a thin line is "seen" from every candidate within +-tol: take the middle of that plateau
  const int threshold = static_cast<int>(std::ceil(0.9 * best_rows));
  int lo = best_k;
  int hi = best_k;
  while (lo > 0 && seen[static_cast<std::size_t>(lo - 1)] >= threshold) {
    --lo;
  }
  while (hi + 1 < n && seen[static_cast<std::size_t>(hi + 1)] >= threshold) {
    ++hi;
  }
  return side * (first + 0.5 * (lo + hi) * p.y_res);
}

void CorridorEstimator::searchLateral(
  const std::vector<AxisDetection> & detections, const AxisModel & axis, const Corridor * previous, double dt,
  Corridor & out)
{
  const CorridorParams & p = params_;
  const std::size_t n_seg = p.segment_edges.size() - 1;
  const int rows = upper_.rows();
  const double hwu = p.search_half_width_upper;
  const double hwl = p.search_half_width_lower;
  const double base2 = 0.5 * p.smooth_base * p.smooth_base;
  const bool use_temporal = previous && previous->valid && previous->kappas.size() == n_seg;

  // structure lines beside the rails near the train (walls, column rows, platform edges), measured from the rail axis
  // of this frame or, without rails, from the previous corridor
  out.wall_left = std::numeric_limits<double>::quiet_NaN();
  out.wall_right = std::numeric_limits<double>::quiet_NaN();
  if (p.w_wall > 0.0 && (axis.locked || (previous && previous->valid))) {
    std::vector<float> reference(static_cast<std::size_t>(rows));
    for (int i = 0; i < rows; ++i) {
      const double x = (i + 0.5) * p.x_res;
      reference[static_cast<std::size_t>(i)] =
        static_cast<float>(axis.locked ? axis.lateralAt(x) : previous->lateralAt(static_cast<float>(x)));
    }
    out.wall_left = measureWallOffset(reference, 1);
    out.wall_right = measureWallOffset(reference, -1);
  }
  const bool wall_left = std::isfinite(out.wall_left);
  const bool wall_right = std::isfinite(out.wall_right);
  std::vector<double> wall_tol(static_cast<std::size_t>(rows));
  for (int i = 0; i < rows; ++i) {
    wall_tol[static_cast<std::size_t>(i)] = p.wall_tolerance + p.wall_tolerance_per_m * (i + 0.5) * p.x_res;
  }

  const std::vector<double> offsets = axis.locked ?
    symmetricGrid(axis.y0, p.init_offset_span, p.init_offset_step) :
    symmetricGrid(axis.y0, p.unlocked_offset_span, p.unlocked_offset_step);
  const std::vector<double> headings = axis.locked ?
    symmetricGrid(axis.theta0, p.init_heading_span_deg * kDegToRad, p.init_heading_step_deg * kDegToRad) :
    symmetricGrid(axis.theta0, p.unlocked_heading_span_deg * kDegToRad, p.unlocked_heading_step_deg * kDegToRad);

  std::vector<Node> beams;
  for (double y : offsets) {
    for (double t : headings) {
      Node root;
      root.y = y;
      root.theta = t;
      beams.push_back(root);
    }
  }

  std::vector<std::vector<Node>> layers;
  layers.push_back(beams);
  std::vector<const AxisDetection *> seg_det;
  std::vector<const AxisDetection *> future_det;
  const std::size_t n_kappa = kappa_grid_.size();
  const auto kappa_index = [&](double kappa) {
      return std::clamp(
        static_cast<long>(std::lround((kappa + p.kappa_max) / p.kappa_step)), 0L, static_cast<long>(n_kappa) - 1);
    };
  const long window = static_cast<long>(std::lround(p.kappa_change_max / p.kappa_step));
  const std::size_t keep = static_cast<std::size_t>(p.beam_per_parent);

  for (std::size_t s = 0; s < n_seg; ++s) {
    const double x0 = p.segment_edges[s];
    const double x1 = p.segment_edges[s + 1];
    const int i0 = std::clamp(static_cast<int>(std::floor(x0 / p.x_res)), 0, rows);
    const int i1 = std::clamp(static_cast<int>(std::floor(x1 / p.x_res)), 0, rows);
    const int stride = x0 >= p.coarse_from ? 2 : 1;
    const double len = x1 - x0;
    seg_det.clear();
    future_det.clear();
    for (const auto & d : detections) {
      if (d.x >= x0 && d.x < x1) {
        seg_det.push_back(&d);
      } else if (d.x >= x1) {
        future_det.push_back(&d);
      }
    }

    // near the train the curvature is the one the train is riding on: between frames it may change only as fast
    // as a transition curve does (a switch offers a second, equally good rail pair - the route is not one of them)
    long t_lo = 0;
    long t_hi = static_cast<long>(n_kappa) - 1;
    if (use_temporal && p.kappa_change_rate > 0.0 && x0 < p.kappa_rate_limit_until) {
      const long kc = kappa_index(previous->kappas[s]);
      const double allowed = p.kappa_change_rate * std::clamp(dt, 0.05, 1.0);
      const long w = std::max(1L, static_cast<long>(std::lround(allowed / p.kappa_step)));
      t_lo = std::max(0L, kc - w);
      t_hi = std::min(static_cast<long>(n_kappa) - 1, kc + w);
    }

    std::vector<Node> candidates(beams.size() * keep);
    for (auto & c : candidates) {
      c.cost = std::numeric_limits<double>::infinity();
    }
    const long n_beams = static_cast<long>(beams.size());
    #pragma omp parallel for schedule(dynamic)
    for (long bi = 0; bi < n_beams; ++bi) {
      const Node & b = beams[static_cast<std::size_t>(bi)];
      long k_lo = 0;
      long k_hi = static_cast<long>(n_kappa) - 1;
      if (b.has_segment) {
        const long kc = kappa_index(b.kappa);
        k_lo = std::max(0L, kc - window);
        k_hi = std::min(static_cast<long>(n_kappa) - 1, kc + window);
      }
      if (t_lo > k_lo || t_hi < k_hi) {  // temporal limit on the near segments
        const long mid = std::clamp((k_lo + k_hi) / 2, t_lo, t_hi);
        k_lo = std::max(k_lo, t_lo);
        k_hi = std::min(k_hi, t_hi);
        if (k_lo > k_hi) {
          k_lo = k_hi = mid;
        }
      }
      struct Scored
      {
        double rank;
        double cost;
        long k;
        bool operator<(const Scored & o) const {return rank < o.rank;}
      };
      std::vector<Scored> scored;
      scored.reserve(static_cast<std::size_t>(k_hi - k_lo + 1));
      for (long k = k_lo; k <= k_hi; ++k) {
        const double kappa = kappa_grid_[static_cast<std::size_t>(k)];
        double c = 0.0;
        for (int i = i0; i < i1; i += stride) {
          const double dx = (i + 0.5) * p.x_res - x0;
          const double yc = b.y + b.theta * dx + 0.5 * kappa * dx * dx;
          c += upper_.count(i, yc - hwu, yc + hwu) + lower_.count(i, yc - hwl, yc + hwl);
          if (p.w_margin > 0.0) {
            c += p.w_margin *
              (upper_.count(i, yc + hwu, yc + hwu + p.margin) + upper_.count(i, yc - hwu - p.margin, yc - hwu));
          }
          const double tol = wall_tol[static_cast<std::size_t>(i)];
          if (wall_left && upper_.count(i, yc + out.wall_left - tol, yc + out.wall_left + tol) > 0) {
            c -= p.w_wall;
          }
          if (wall_right && upper_.count(i, yc + out.wall_right - tol, yc + out.wall_right + tol) > 0) {
            c -= p.w_wall;
          }
        }
        c *= stride;
        if (b.has_segment) {
          c += p.w_smooth * sq((kappa - b.kappa) * base2);
        }
        // weak L1 prior towards a straight track: breaks ties between near-equivalent curvatures (noisy rail
        // detections cannot tell 1e-4 from 0, but it moves the axis by ~1 m at 120 m); real curves win on walls
        c += (x0 >= p.straight_far_from ? p.w_straight_far : p.w_straight) * std::abs(kappa) * base2;
        for (const AxisDetection * d : seg_det) {
          const double dx = d->x - x0;
          const double r = d->y - (b.y + b.theta * dx + 0.5 * kappa * dx * dx);
          c += p.w_axis * std::min(std::abs(r), p.axis_residual_cap);
        }
        if (use_temporal) {
          c += p.w_temporal * p.w_smooth * sq((kappa - previous->kappas[s]) * base2);
        }
        // lookahead for pruning only: rails further ahead scored along this curvature. Near segments are short and
        // their own few detections cannot tell headings apart that miss the rails at 30 m by 0.3 m.
        double look = 0.0;
        if (p.w_axis_lookahead > 0.0) {
          const double y1 = b.y + b.theta * len + 0.5 * kappa * len * len;
          const double t1 = b.theta + kappa * len;
          for (const AxisDetection * d : future_det) {
            const double dx = d->x - x1;
            const double r = d->y - (y1 + t1 * dx + 0.5 * kappa * dx * dx);
            look += p.w_axis_lookahead * std::min(std::abs(r), p.axis_residual_cap);
          }
        }
        scored.push_back({b.cost + c + look, c, k});
      }
      const std::size_t n_keep = std::min(keep, scored.size());
      std::partial_sort(scored.begin(), scored.begin() + static_cast<long>(n_keep), scored.end());
      for (std::size_t j = 0; j < n_keep; ++j) {
        const double kappa = kappa_grid_[static_cast<std::size_t>(scored[j].k)];
        Node & n = candidates[static_cast<std::size_t>(bi) * keep + j];
        n.cost = b.cost + scored[j].cost;
        n.rank = scored[j].rank;
        n.y_start = b.y;
        n.theta_start = b.theta;
        n.y = b.y + b.theta * len + 0.5 * kappa * len * len;
        n.theta = b.theta + kappa * len;
        n.kappa = kappa;
        n.parent = static_cast<int>(bi);
        n.has_segment = true;
      }
    }
    candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(), [](const Node & n) {return !std::isfinite(n.cost);}),
      candidates.end());
    beams = selectDiverse(candidates, p.beam_width, p.diversity_heading_deg * kDegToRad, p.diversity_offset);
    layers.push_back(beams);
  }

  // lateral profile of the path ending at a node of the last layer
  const auto profile = [&](std::size_t final_index, std::vector<double> * kappas_out,
    std::vector<double> * y_starts_out, std::vector<double> * theta_starts_out, std::vector<float> & lateral,
    std::vector<float> * heading) {
      std::vector<double> kappas(n_seg);
      std::vector<double> y_starts(n_seg);
      std::vector<double> theta_starts(n_seg);
      int idx = static_cast<int>(final_index);
      for (std::size_t s = n_seg; s-- > 0;) {
        const Node & n = layers[s + 1][static_cast<std::size_t>(idx)];
        kappas[s] = n.kappa;
        y_starts[s] = n.y_start;
        theta_starts[s] = n.theta_start;
        idx = n.parent;
      }
      lateral.assign(static_cast<std::size_t>(rows), 0.0F);
      if (heading) {
        heading->assign(static_cast<std::size_t>(rows), 0.0F);
      }
      std::size_t s = 0;
      for (int i = 0; i < rows; ++i) {
        const double x = (i + 0.5) * p.x_res;
        while (s + 1 < n_seg && x >= p.segment_edges[s + 1]) {
          ++s;
        }
        const double dx = x - p.segment_edges[s];
        lateral[static_cast<std::size_t>(i)] =
          static_cast<float>(y_starts[s] + theta_starts[s] * dx + 0.5 * kappas[s] * dx * dx);
        if (heading) {
          (*heading)[static_cast<std::size_t>(i)] = static_cast<float>(theta_starts[s] + kappas[s] * dx);
        }
      }
      if (kappas_out) {
        *kappas_out = kappas;
        *y_starts_out = y_starts;
        *theta_starts_out = theta_starts;
      }
    };

  std::vector<double> kappas;
  std::vector<double> y_starts;
  std::vector<double> theta_starts;
  profile(0, &kappas, &y_starts, &theta_starts, out.lateral, &out.heading);
  out.kappas = kappas;
  out.y0 = y_starts[0];
  out.theta0 = theta_starts[0];
  out.cost = layers.back()[0].cost;

  // how far the walls still confirm the chosen axis: beyond that the corridor is extrapolated
  out.evidence_range = 0.0;
  for (int i = 0; i < rows; ++i) {
    const double yc = out.lateral[static_cast<std::size_t>(i)];
    const double tol = wall_tol[static_cast<std::size_t>(i)];
    const bool seen =
      (wall_left && upper_.count(i, yc + out.wall_left - tol, yc + out.wall_left + tol) > 0) ||
      (wall_right && upper_.count(i, yc + out.wall_right - tol, yc + out.wall_right + tol) > 0);
    if (seen) {
      out.evidence_range = (i + 0.5) * p.x_res;
    }
  }

  // disagreement of near-optimal alternatives: large where the tunnel does not constrain the track
  out.spread.assign(static_cast<std::size_t>(rows), 0.0F);
  const double limit = out.cost + std::max(p.alternative_cost_abs, p.alternative_cost_rel * out.cost);
  std::vector<float> alt;
  for (std::size_t k = 1; k < layers.back().size(); ++k) {
    if (layers.back()[k].cost > limit) {
      break;  // beams are sorted by cost
    }
    profile(k, nullptr, nullptr, nullptr, alt, nullptr);
    for (int i = 0; i < rows; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      out.spread[ui] = std::max(out.spread[ui], std::abs(alt[ui] - out.lateral[ui]));
    }
  }
}

double CorridorEstimator::blockedRange(const Corridor & out) const
{
  const CorridorParams & p = params_;
  if (p.blocked_cells <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  // The clearance envelope of the train is empty by definition, so long structures inside it mean the corridor
  // does not go there: the tunnel has turned out of sight and we are looking at its wall. Only long structures
  // are in the grid, so a person or a box can never close the corridor.
  const int rows = upper_.rows();
  const int window = std::max(1, static_cast<int>(std::lround(p.blocked_window / p.x_res)));
  const int first = std::clamp(static_cast<int>(p.blocked_from / p.x_res), 0, rows);
  blocked_rows_.assign(static_cast<std::size_t>(std::max(0, rows - first)), 0);
  int inside = 0;
  for (int i = first; i < rows; ++i) {
    const double yc = out.lateralAt(static_cast<float>((i + 0.5) * p.x_res));
    blocked_rows_[static_cast<std::size_t>(i - first)] =
      upper_.count(i, yc - p.blocked_half_width, yc + p.blocked_half_width);
    inside += blocked_rows_[static_cast<std::size_t>(i - first)];
    if (i - first >= window) {
      inside -= blocked_rows_[static_cast<std::size_t>(i - first - window)];
    }
    if (inside >= p.blocked_cells) {
      return std::max(p.blocked_from, (i - window + 1) * p.x_res);
    }
  }
  return std::numeric_limits<double>::infinity();
}

void CorridorEstimator::searchVertical(const TrackCloud & cloud, const Corridor * previous, Corridor & out)
{
  const CorridorParams & p = params_;
  const std::size_t n_seg = p.segment_edges.size() - 1;
  const int rows = side_.rows();
  out.bed.assign(static_cast<std::size_t>(rows), 0.0F);
  out.grades.assign(n_seg, 0.0);
  if (!p.estimate_vertical) {
    return;
  }

  const bool use_temporal = previous && previous->valid && previous->grades.size() == n_seg;
  // How high the rail heads sit above the bed: measured near the train, where both surfaces are seen densely.
  double rail_height = p.rail_height_default;
  {
    std::vector<float> on_rail;
    std::vector<float> between;
    for (const TrackPoint & pt : cloud) {
      if (pt.x < p.rail_height_ref_min || pt.x > p.rail_height_ref_max) {
        continue;
      }
      const double lat = pt.y - out.lateralAt(pt.x);
      const double h = pt.z - out.bedAt(pt.x);
      if (h < -0.5 || h > 0.5) {
        continue;
      }
      if (std::abs(std::abs(lat) - p.gauge_half) <= p.rail_half_width) {
        on_rail.push_back(static_cast<float>(h));
      } else if (std::abs(lat) < p.gauge_half - p.rail_half_width) {
        between.push_back(static_cast<float>(h));
      }
    }
    if (on_rail.size() > 50 && between.size() > 50) {
      const auto median = [](std::vector<float> & v) {
          std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
          return static_cast<double>(v[v.size() / 2]);
        };
      rail_height = std::clamp(median(on_rail) - median(between), p.rail_height_min, p.rail_height_max);
    }
  }
  out.rail_height = rail_height;

  side_.clear();
  const double z_max = p.z_grid_min + p.z_cells * p.z_res;
  for (const TrackPoint & pt : cloud) {
    if (pt.x <= 0.5 || pt.x >= p.x_max || pt.z <= p.z_grid_min || pt.z >= z_max) {
      continue;
    }
    const double lat = pt.y - out.lateralAt(pt.x);
    if (std::abs(lat) >= p.vertical_lateral_half_width) {
      continue;
    }
    // a return off a rail head stands for the bed 0.2 m below it: without the correction the profile lands on the
    // rails and every object on the track loses that much of its measured height
    const bool on_rail = pt.x < p.rail_correct_max &&
      std::abs(std::abs(lat) - p.gauge_half) <= p.rail_half_width;
    side_.mark(pt.x, on_rail ? pt.z - rail_height : pt.z);
  }
  // only horizontal surfaces (bed, ceiling) define the profile; vertical things (columns, portals, people)
  // must not drag the envelope up or down
  side_.keepRowRuns(
    static_cast<int>(std::lround(p.long_structure_min_length / p.x_res)),
    static_cast<int>(std::lround(p.vertical_run_max_gap / p.x_res)));
  side_.buildPrefix();

  // ceiling height above the bed measured near the train; far away, where the bed is no longer visible, the
  // profile keeps this clearance to the (still visible) ceiling
  double ceiling_offset = std::numeric_limits<double>::quiet_NaN();
  {
    std::vector<double> lows;
    const int r0 = static_cast<int>(p.ceiling_ref_min / p.x_res);
    const int r1 = std::min(rows, static_cast<int>(p.ceiling_ref_max / p.x_res));
    const int c_min = static_cast<int>(std::ceil((p.ceiling_min_height - p.z_grid_min) / p.z_res));
    for (int i = r0; i < r1; ++i) {
      for (int k = c_min; k < side_.cols(); ++k) {
        if (side_.occupied(i, k)) {
          lows.push_back(p.z_grid_min + (k + 0.5) * p.z_res);
          break;
        }
      }
    }
    if (static_cast<int>(lows.size()) >= p.ceiling_ref_min_columns) {
      std::nth_element(lows.begin(), lows.begin() + lows.size() / 2, lows.end());
      ceiling_offset = lows[lows.size() / 2];
    }
  }
  out.ceiling_offset = ceiling_offset;

  // Rows where a horizontal surface is visible below the ceiling: there the bed itself defines the profile and the
  // ceiling clearance reward is switched off. A station vault sits 3-4 m above the tunnel ceiling that follows it,
  // and keeping the near-field clearance would make the profile dive by that much.
  std::vector<uint8_t> floor_seen(static_cast<std::size_t>(rows), 0);
  if (std::isfinite(ceiling_offset) && p.ceiling_only_without_floor) {
    for (int i = 0; i < rows; ++i) {
      floor_seen[static_cast<std::size_t>(i)] =
        side_.count(i, p.z_grid_min, p.ceiling_min_height) > 0 ? 1 : 0;
    }
  }

  std::vector<Node> beams(1);  // y = bed height, theta = grade
  std::vector<std::vector<Node>> layers{beams};
  const std::vector<double> fixed_grid{0.0};
  // beyond a wall that crosses the corridor the cloud says nothing about the track: the grade is continued
  // (the smoothness term keeps the parent's grade when no row contributes a cost) instead of diving to escape it
  const int i_blocked = out.blocked_range >= static_cast<double>(rows) * p.x_res
    ? rows
    : std::clamp(static_cast<int>(out.blocked_range / p.x_res), 0, rows);
  for (std::size_t s = 0; s < n_seg; ++s) {
    const double x0 = p.segment_edges[s];
    const double x1 = p.segment_edges[s + 1];
    const int i0 = std::clamp(static_cast<int>(std::floor(x0 / p.x_res)), 0, rows);
    const int i1 = std::clamp(static_cast<int>(std::floor(x1 / p.x_res)), 0, rows);
    const double len = x1 - x0;
    const std::vector<double> & grid = x0 >= p.vertical_fixed_until ? grade_grid_ : fixed_grid;

    std::vector<Node> candidates;
    for (std::size_t bi = 0; bi < beams.size(); ++bi) {
      const Node & b = beams[bi];
      std::vector<std::pair<double, double>> scored;
      scored.reserve(grid.size());
      for (double g : grid) {
        double c = 0.0;
        for (int i = i0; i < i1; ++i) {
          const double zb = b.y + g * ((i + 0.5) * p.x_res - x0);
          // Beyond a wall that crosses the corridor the tunnel has turned out of sight: what fills the clearance
          // zone there is that wall, not something the profile should dodge. The floor, if still visible, keeps
          // anchoring the profile, so only the structure terms are dropped.
          if (i < i_blocked) {
            c += side_.count(i, zb + p.vertical_zone_bottom, zb + p.vertical_zone_top);
            c += p.w_vertical_ceiling *
              side_.count(i, zb + p.vertical_zone_top, zb + p.vertical_zone_top + p.vertical_ceiling_margin);
          }
          // bed surface found clearly below the profile: the profile is too high
          c += side_.count(i, zb - p.vertical_below_depth, zb - p.vertical_bed_tolerance);
          // ... and the other way round: a horizontal surface just above the profile is the bed as well. Without
          // this the profile dives after a rising ceiling (station vault) and the floor lands inside the zone.
          c += p.w_vertical_above *
            side_.count(i, zb + p.vertical_bed_tolerance, zb + p.vertical_above_depth);
          // the profile belongs on the floor wherever one is visible (the penalty above keeps it off the
          // drainage trough: the bed 0.4 m higher would then cost more than this reward gives)
          if (p.w_floor > 0.0) {
            c -= p.w_floor *
              std::min(1, side_.count(i, zb - p.floor_tolerance, zb + p.floor_tolerance));
          }
          // reward the ceiling seen at its near-field clearance above the profile
          if (std::isfinite(ceiling_offset) && i < i_blocked && !floor_seen[static_cast<std::size_t>(i)]) {
            c -= p.w_ceiling_offset *
              std::min(1, side_.count(i, zb + ceiling_offset - p.ceiling_tolerance, zb + ceiling_offset + p.ceiling_tolerance));
          }
        }
        c += p.w_vertical_smooth * sq((g - b.theta) * len);
        // far away the bed is barely visible and the profile jumps between frames; near the train the data is good
        // and a temporal pull would only delay real grade changes
        if (use_temporal && x0 >= p.vertical_temporal_from) {
          c += p.w_vertical_temporal * p.w_vertical_smooth * sq((g - previous->grades[s]) * len);
        }
        scored.emplace_back(c, g);
      }
      const std::size_t keep = std::min<std::size_t>(static_cast<std::size_t>(p.beam_per_parent), scored.size());
      std::partial_sort(scored.begin(), scored.begin() + keep, scored.end());
      for (std::size_t j = 0; j < keep; ++j) {
        Node n;
        n.cost = b.cost + scored[j].first;
        n.rank = n.cost;
        n.y_start = b.y;
        n.theta_start = b.theta;
        n.kappa = scored[j].second;
        n.y = b.y + n.kappa * len;
        n.theta = n.kappa;
        n.parent = static_cast<int>(bi);
        n.has_segment = true;
        candidates.push_back(n);
      }
    }
    beams = selectDiverse(candidates, p.beam_width, p.grade_step, 0.1);
    layers.push_back(beams);
  }

  std::vector<double> z_starts(n_seg);
  int idx = 0;
  for (std::size_t s = n_seg; s-- > 0;) {
    const Node & n = layers[s + 1][static_cast<std::size_t>(idx)];
    out.grades[s] = n.kappa;
    z_starts[s] = n.y_start;
    idx = n.parent;
  }
  for (int i = 0; i < rows; ++i) {
    const double x = (i + 0.5) * p.x_res;
    std::size_t s = 0;
    while (s + 1 < n_seg && x >= p.segment_edges[s + 1]) {
      ++s;
    }
    out.bed[static_cast<std::size_t>(i)] = static_cast<float>(z_starts[s] + out.grades[s] * (x - p.segment_edges[s]));
  }
}

bool CorridorEstimator::isStructure(float x, float y, float z) const
{
  return touchesStructure(x, y, z, 0.0F);
}

bool CorridorEstimator::touchesStructure(float x, float y, float z, float lateral_radius) const
{
  const CorridorParams & p = params_;
  if (aligned_lateral_.empty()) {
    return false;
  }
  const int row_raw = static_cast<int>(std::floor(x / p.x_res));
  if (row_raw < 0 || row_raw >= aligned_upper_.rows()) {
    return false;
  }
  {
    // low band: the rail strips and everything that runs along the track
    const float bed = aligned_bed_.empty() ? 0.0F : aligned_bed_[static_cast<std::size_t>(row_raw)];
    const float height = z - bed;
    if (height > p.band_low_min && height < p.band_low_max) {
      const float lat_low = y - aligned_lateral_[static_cast<std::size_t>(row_raw)];
      if (std::abs(std::abs(lat_low) - p.gauge_half) <= p.rail_half_width) {
        return true;  // a rail
      }
      const int col_low = static_cast<int>(std::floor((lat_low + p.aligned_half_width) / p.y_res));
      const int dc_low = static_cast<int>(std::ceil(lateral_radius / p.y_res));
      for (int cc = std::max(0, col_low - dc_low); cc <= std::min(aligned_low_.cols() - 1, col_low + dc_low); ++cc) {
        if (aligned_low_.occupied(row_raw, cc)) {
          return true;
        }
      }
      return false;
    }
  }
  if (z <= p.band_lower_min || z >= p.band_upper_max) {
    return false;
  }
  const int row = row_raw;
  const float lat = y - aligned_lateral_[static_cast<std::size_t>(row)];
  const int col = static_cast<int>(std::floor((lat + p.aligned_half_width) / p.y_res));
  const int dc = static_cast<int>(std::ceil(lateral_radius / p.y_res));
  const int dr = lateral_radius > 0.0F ? 1 : 0;
  // a point is tested against its own band; a low point is also a structure if a structure stands right above its
  // cell (the foot of a column). Never the other way round: a person above a rail line is not the rail.
  const bool upper_band = z > p.band_upper_min;
  for (int r = std::max(0, row - dr); r <= std::min(aligned_upper_.rows() - 1, row + dr); ++r) {
    for (int c = std::max(0, col - dc); c <= std::min(aligned_upper_.cols() - 1, col + dc); ++c) {
      const bool own_cell = r == row && c == col;
      const bool upper = aligned_upper_.occupied(r, c);
      const bool lower = aligned_lower_.occupied(r, c);
      if (upper_band ? upper : (lower || (own_cell && upper))) {
        return true;
      }
    }
  }
  return false;
}

Corridor CorridorEstimator::estimate(
  const TrackCloud & cloud, const std::vector<AxisDetection> & axis_detections, const AxisModel & axis,
  const Corridor * previous, double dt)
{
  Corridor corridor;
  corridor.x_res = params_.x_res;
  buildLateralGrids(cloud);
  searchLateral(axis_detections, axis, previous, dt, corridor);
  corridor.blocked_range = blockedRange(corridor);
  searchVertical(cloud, previous, corridor);
  buildAlignedGrids(cloud, corridor);
  corridor.valid = true;
  return corridor;
}

}  // namespace tod
