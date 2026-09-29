#include "tunnel_obstacle_detector/core/track_axis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Dense>

namespace tod
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
}

TrackAxisDetector::TrackAxisDetector(const TrackAxisParams & params)
: params_(params)
{
}

std::vector<AxisDetection> TrackAxisDetector::detect(const TrackCloud & cloud, const AxisModel & prior) const
{
  const TrackAxisParams & p = params_;

  std::vector<std::vector<uint32_t>> rings;
  for (uint32_t i = 0; i < cloud.size(); ++i) {
    const TrackPoint & pt = cloud[i];
    if (pt.x <= p.x_min || pt.x >= p.x_max || pt.z <= p.z_min || pt.z >= p.z_max) {
      continue;
    }
    if (std::abs(pt.y - prior.lateralAt(pt.x)) >= p.window) {
      continue;
    }
    if (pt.ring >= rings.size()) {
      rings.resize(pt.ring + 1U);
    }
    rings[pt.ring].push_back(i);
  }

  const int nbins = static_cast<int>(std::ceil(2.0 * p.window / p.bin));
  std::vector<float> zmin(nbins);
  std::vector<float> zmax(nbins);
  std::vector<float> zmax_x(nbins);  // distance of the highest return in the bin (rail heads are hit closer than the bed)
  std::vector<float> xs;
  std::vector<float> scratch;
  std::vector<AxisDetection> detections;

  for (const auto & indices : rings) {
    if (static_cast<int>(indices.size()) < p.min_ring_points) {
      continue;
    }
    xs.clear();
    for (uint32_t i : indices) {
      xs.push_back(cloud[i].x);
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    const double xm = xs[xs.size() / 2];
    const double yp = prior.lateralAt(xm);
    const double y_lo = yp - p.window;

    std::fill(zmin.begin(), zmin.end(), kNaN);
    std::fill(zmax.begin(), zmax.end(), kNaN);
    for (uint32_t i : indices) {
      const int b = static_cast<int>(std::floor((cloud[i].y - y_lo) / p.bin));
      if (b < 0 || b >= nbins) {
        continue;
      }
      const float z = cloud[i].z;
      zmin[b] = std::isnan(zmin[b]) ? z : std::min(zmin[b], z);
      if (std::isnan(zmax[b]) || z > zmax[b]) {
        zmax[b] = z;
        zmax_x[b] = cloud[i].x;
      }
    }

    auto center = [&](int b) {return y_lo + (b + 0.5) * p.bin;};
    auto bin_range = [&](double a, double c, int & b0, int & b1) {
        b0 = std::max(0, static_cast<int>(std::ceil((a - y_lo) / p.bin - 0.5)));
        b1 = std::min(nbins - 1, static_cast<int>(std::floor((c - y_lo) / p.bin - 0.5)));
      };
    auto max_in = [&](double a, double c, float * x_at_max = nullptr) {
        int b0, b1;
        bin_range(a, c, b0, b1);
        float m = kNaN;
        for (int b = b0; b <= b1; ++b) {
          if (center(b) > a && center(b) < c && !std::isnan(zmax[b]) && (std::isnan(m) || zmax[b] > m)) {
            m = zmax[b];
            if (x_at_max) {
              *x_at_max = zmax_x[b];
            }
          }
        }
        return m;
      };
    auto collect_min = [&](double a, double c, std::vector<float> & out) {
        out.clear();
        int b0, b1;
        bin_range(a, c, b0, b1);
        for (int b = b0; b <= b1; ++b) {
          if (center(b) > a && center(b) < c && !std::isnan(zmin[b])) {
            out.push_back(zmin[b]);
          }
        }
      };

    bool found = false;
    double best_c = 0.0;
    double best_x = xm;
    double best_score = -std::numeric_limits<double>::infinity();
    const double g = p.gauge_half;
    const double w = p.rail_half_window;
    for (int ci = 0; ci < nbins; ++ci) {
      const double c = center(ci);
      if (std::abs(c - yp) > p.center_search) {
        continue;
      }
      float xl = 0.0F;
      float xr = 0.0F;
      const float rl = max_in(c - g - w, c - g + w, &xl);
      const float rr = max_in(c + g - w, c + g + w, &xr);
      if (std::isnan(rl) || std::isnan(rr)) {
        continue;
      }
      collect_min(c - 0.55, c + 0.55, scratch);
      if (scratch.empty()) {
        continue;
      }
      const std::size_t k60 = static_cast<std::size_t>(0.6 * (scratch.size() - 1));
      std::nth_element(scratch.begin(), scratch.begin() + k60, scratch.end());
      const float bed = scratch[k60];
      if (rl < p.rail_min_height || rr < p.rail_min_height || std::min(rl, rr) - bed < p.rail_above_bed) {
        continue;
      }
      const float out_l = max_in(c - g - 0.30, c - g - 0.12);
      const float out_r = max_in(c + g + 0.12, c + g + 0.30);
      if ((!std::isnan(out_l) && out_l > rl - 0.05F) || (!std::isnan(out_r) && out_r > rr - 0.05F)) {
        continue;
      }
      collect_min(c - 0.12, c + 0.12, scratch);
      double channel = 0.0;
      if (!scratch.empty()) {
        std::nth_element(scratch.begin(), scratch.begin() + scratch.size() / 2, scratch.end());
        channel = scratch[scratch.size() / 2];
      }
      const double score = 0.5 * (rl + rr) - bed - 0.5 * std::abs(rl - rr) +
        0.5 * std::max(0.0, -channel - p.channel_bonus_depth);
      if (score > best_score) {
        best_score = score;
        best_c = c;
        best_x = 0.5 * (xl + xr);
        found = true;
      }
    }
    if (found) {
      detections.push_back({best_x, best_c, best_score});
    }
  }
  return detections;
}

AxisModel TrackAxisDetector::fit(const std::vector<AxisDetection> & det, std::vector<AxisDetection> * inliers) const
{
  const TrackAxisParams & p = params_;
  AxisModel model;
  model.num_detections = static_cast<int>(det.size());
  if (inliers) {
    inliers->clear();
  }
  const std::size_t n = det.size();
  if (static_cast<int>(n) < p.min_inliers) {
    return model;
  }

  std::mt19937 rng(777U);
  std::uniform_int_distribution<std::size_t> pick(0, n - 1);
  std::vector<char> best_mask;
  int best_count = 0;
  std::vector<char> mask(n);
  for (int it = 0; it < p.ransac_iterations; ++it) {
    const std::size_t a = pick(rng);
    const std::size_t b = pick(rng);
    const std::size_t c = pick(rng);
    if (a == b || b == c || a == c) {
      continue;
    }
    const double xmin = std::min({det[a].x, det[b].x, det[c].x});
    const double xmax = std::max({det[a].x, det[b].x, det[c].x});
    if (xmax - xmin < 4.0) {
      continue;
    }
    Eigen::Matrix3d m;
    Eigen::Vector3d v;
    for (int r = 0; r < 3; ++r) {
      const AxisDetection & d = det[r == 0 ? a : (r == 1 ? b : c)];
      m.row(r) << 1.0, d.x, 0.5 * d.x * d.x;
      v(r) = d.y;
    }
    const Eigen::Vector3d coef = m.fullPivLu().solve(v);
    if (!coef.allFinite() || std::abs(coef(2)) > p.kappa_limit) {
      continue;
    }
    int count = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const double r = det[i].y - (coef(0) + coef(1) * det[i].x + 0.5 * coef(2) * det[i].x * det[i].x);
      mask[i] = std::abs(r) < p.ransac_threshold;
      count += mask[i];
    }
    if (count > best_count) {
      best_count = count;
      best_mask = mask;
    }
  }
  if (best_count < p.min_inliers) {
    return model;
  }

  Eigen::Matrix3d normal_matrix = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    if (!best_mask[i]) {
      continue;
    }
    const Eigen::Vector3d row(1.0, det[i].x, 0.5 * det[i].x * det[i].x);
    normal_matrix += row * row.transpose();
    rhs += row * det[i].y;
    if (inliers) {
      inliers->push_back(det[i]);
    }
  }
  normal_matrix(2, 2) += p.kappa_ridge;
  const Eigen::Vector3d coef = normal_matrix.ldlt().solve(rhs);
  model.y0 = coef(0);
  model.theta0 = coef(1);
  model.kappa0 = std::clamp(coef(2), -p.kappa_limit, p.kappa_limit);
  model.locked = true;
  return model;
}

}  // namespace tod
