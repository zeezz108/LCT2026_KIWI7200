#include "tunnel_obstacle_detector/core/tracker.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>

namespace tod
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

int recentHits(uint32_t history, int window)
{
  const uint32_t mask = window >= 32 ? 0xFFFFFFFFU : ((1U << window) - 1U);
  return static_cast<int>(std::bitset<32>(history & mask).count());
}
}  // namespace

Tracker::Tracker(const TrackerParams & params)
: params_(params)
{
}

void Tracker::reset()
{
  tracks_.clear();
  has_stamp_ = false;
}

double Tracker::expectedPoints(double distance) const
{
  const double d = std::max(distance, 1.0);
  const double cols = params_.target_width / (d * params_.azimuth_resolution_deg * kDegToRad);
  const double rows = params_.target_height / (d * params_.elevation_resolution_deg * kDegToRad);
  return std::max(1.0, std::max(cols, 1.0) * std::max(rows, 1.0) * params_.return_ratio);
}

void Tracker::update(const std::vector<Cluster> & clusters, double stamp)
{
  const TrackerParams & p = params_;
  double dt = 0.1;
  if (has_stamp_) {
    dt = stamp - last_stamp_;
    if (dt < 0.0 || dt > 5.0) {  // bag restarted or long gap: history is meaningless
      tracks_.clear();
      dt = 0.1;
    }
  }
  last_stamp_ = stamp;
  has_stamp_ = true;

  // gated candidate pairs
  struct Pair
  {
    double cost;
    std::size_t track;
    std::size_t cluster;
  };
  std::vector<Pair> pairs;
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    const Track & tr = tracks_[t];
    const double predicted = tr.distance + (tr.has_speed ? tr.speed * dt : 0.0);
    const double gate_along = p.gate_along_base + p.gate_along_rel * std::max(predicted, 0.0) +
      (tr.has_speed ? 0.0 : p.max_relative_speed * dt);
    for (std::size_t c = 0; c < clusters.size(); ++c) {
      const double along = std::abs(clusters[c].x_min - predicted);
      const double lat = std::abs(clusters[c].lat_center - tr.lateral);
      if (along <= gate_along && lat <= p.gate_lateral) {
        pairs.push_back({along / gate_along + lat / p.gate_lateral, t, c});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.cost < b.cost;});

  std::vector<char> track_used(tracks_.size(), 0);
  std::vector<char> cluster_used(clusters.size(), 0);
  auto apply = [&](Track & tr, const Cluster & c, bool is_new) {
      if (!is_new && dt > 1e-3) {
        const double inst = (c.x_min - tr.distance) / dt;
        tr.speed = tr.has_speed ? (1.0 - p.speed_smoothing) * tr.speed + p.speed_smoothing * inst : inst;
        tr.has_speed = true;
      }
      tr.distance = c.x_min;
      tr.lateral = c.lat_center;
      tr.h_min = c.h_min;
      tr.h_max = c.h_max;
      tr.length = c.x_max - c.x_min;
      tr.width = c.lat_max - c.lat_min;
      tr.num_points = c.num_points;
      tr.cx = c.cx;
      tr.cy = c.cy;
      tr.cz = c.cz;
      tr.points = c.points;
      tr.misses = 0;
      ++tr.hits;
      tr.history = (tr.history << 1U) | 1U;
      tr.danger_history = (tr.danger_history << 1U) | (c.level == Level::kDanger ? 1U : 0U);
      tr.strong_history = (tr.strong_history << 1U) | (c.weak ? 0U : 1U);
    };

  for (const Pair & pr : pairs) {
    if (track_used[pr.track] || cluster_used[pr.cluster]) {
      continue;
    }
    track_used[pr.track] = 1;
    cluster_used[pr.cluster] = 1;
    apply(tracks_[pr.track], clusters[pr.cluster], false);
  }
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    if (track_used[t]) {
      continue;
    }
    Track & tr = tracks_[t];
    ++tr.misses;
    tr.history <<= 1U;
    tr.danger_history <<= 1U;
    tr.strong_history <<= 1U;
    tr.points.clear();
    if (tr.has_speed) {
      tr.distance += tr.speed * dt;
    }
  }
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {return tr.misses > p.max_misses;}),
    tracks_.end());
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    if (cluster_used[c]) {
      continue;
    }
    Track tr;
    tr.id = next_id_++;
    apply(tr, clusters[c], true);
    tracks_.push_back(std::move(tr));
  }

  for (Track & tr : tracks_) {
    const int extra = static_cast<int>(std::floor(std::max(0.0, tr.distance - p.confirm_far_from) / 25.0)) *
      p.confirm_hits_per_25m;
    const int required = std::min(p.confirm_hits_max, p.confirm_hits + extra);
    const int window = std::min(31, std::max(p.confirm_window, required + 3));
    const int hits = recentHits(tr.history, window);
    const bool strong = tr.misses == 0 && tr.num_points >= p.strong_points && tr.distance <= p.strong_max_distance;
    if (recentHits(tr.strong_history, window) > 0) {
      if (hits >= required || strong) {
        tr.confirmed = true;
      }
    } else if (recentHits(tr.history, p.weak_confirm_window) >= std::max(required, p.weak_confirm_hits)) {
      tr.confirmed = true;  // only weak clusters so far: more frames of the same thing in the same place
    }
    const int danger = recentHits(tr.danger_history, p.confirm_window);
    tr.level = (danger >= std::min(p.danger_hits, std::max(hits, 1)) && danger > 0) ? Level::kDanger : Level::kWarning;
    if (tr.misses == 0) {
      const double persistence = std::min(1.0, static_cast<double>(hits) / p.confirm_hits);
      const double density = std::min(1.0, tr.num_points / expectedPoints(tr.distance));
      tr.confidence = persistence * (0.5 + 0.5 * density);
    } else {
      tr.confidence *= 0.7;
    }
  }
}

std::vector<Track> Tracker::reported() const
{
  std::vector<Track> out;
  for (const Track & tr : tracks_) {
    if (tr.confirmed && tr.misses <= params_.report_max_misses) {
      out.push_back(tr);
    }
  }
  std::sort(out.begin(), out.end(), [](const Track & a, const Track & b) {return a.distance < b.distance;});
  return out;
}

}  // namespace tod
