#include "tunnel_obstacle_detector/core/track_advance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tod
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Zero-mean normalised correlation of `a` shifted by `shift` cells against `b`.
double correlate(const std::vector<float> & a, const std::vector<float> & b, int shift)
{
  const int n = static_cast<int>(a.size());
  double sa = 0.0, sb = 0.0, saa = 0.0, sbb = 0.0, sab = 0.0;
  int used = 0;
  for (int i = 0; i < n; ++i) {
    const int j = i + shift;
    if (j < 0 || j >= n) {
      continue;
    }
    const double x = a[static_cast<std::size_t>(i)];
    const double y = b[static_cast<std::size_t>(j)];
    sa += x;
    sb += y;
    saa += x * x;
    sbb += y * y;
    sab += x * y;
    ++used;
  }
  if (used < 20) {
    return -1.0;
  }
  const double cov = sab / used - (sa / used) * (sb / used);
  const double va = saa / used - (sa / used) * (sa / used);
  const double vb = sbb / used - (sb / used) * (sb / used);
  if (va <= 1e-9 || vb <= 1e-9) {
    return -1.0;
  }
  return cov / std::sqrt(va * vb);
}
}  // namespace

TrackAdvance::TrackAdvance(const TrackAdvanceParams & params)
: params_(params)
{
}

void TrackAdvance::reset()
{
  previous_.clear();
  advance_ = 0.0;
  correlation_ = 0.0;
  prior_ = 0.0;
  has_prior_ = false;
  recent_.clear();
  speed_ = 0.0;
  predicted_ = false;
}

std::vector<float> TrackAdvance::profile(const TrackCloud & cloud, const Corridor & corridor) const
{
  const auto & p = params_;
  const int cells = std::max(1, static_cast<int>((p.x_max - p.x_min) / p.cell));
  std::vector<float> hist(static_cast<std::size_t>(cells), 0.0F);
  for (const auto & pt : cloud) {
    if (pt.x < p.x_min || pt.x >= p.x_max || pt.z < p.z_min || pt.z > p.z_max) {
      continue;
    }
    if (std::abs(pt.y - corridor.lateralAt(pt.x)) > p.lateral_half) {
      continue;
    }
    const int cell = static_cast<int>((pt.x - p.x_min) / p.cell);
    if (cell >= 0 && cell < cells) {
      hist[static_cast<std::size_t>(cell)] += 1.0F;
    }
  }
  // the density of returns falls as 1/x^2; dividing by a moving average leaves only what stands out locally
  const int half = std::max(1, static_cast<int>(0.5 * p.detrend_window / p.cell));
  std::vector<float> out(hist.size(), 0.0F);
  for (int i = 0; i < cells; ++i) {
    double sum = 0.0;
    int n = 0;
    for (int j = std::max(0, i - half); j <= std::min(cells - 1, i + half); ++j) {
      sum += hist[static_cast<std::size_t>(j)];
      ++n;
    }
    const double mean = n > 0 ? sum / n : 0.0;
    out[static_cast<std::size_t>(i)] = mean > 1e-6 ?
      static_cast<float>(hist[static_cast<std::size_t>(i)] / mean) : 0.0F;
  }
  return out;
}

double TrackAdvance::update(const TrackCloud & cloud, const Corridor & corridor, double dt)
{
  advance_ = kNaN;
  correlation_ = 0.0;
  if (!corridor.valid || dt <= 0.0) {
    previous_.clear();
    return advance_;
  }
  std::vector<float> current = profile(cloud, corridor);
  const int occupied = static_cast<int>(std::count_if(
      current.begin(), current.end(), [](float v) {return v > 1e-6F;}));
  if (occupied < params_.min_cells) {
    previous_.clear();
    return advance_;
  }
  if (previous_.size() != current.size()) {
    previous_ = std::move(current);
    return advance_;
  }

  // the train moves forward, so the structures of the previous frame are found at smaller x: positive shift
  const double max_shift = std::min(params_.search + (has_prior_ ? std::abs(prior_) : 0.0),
      params_.max_speed * dt);
  const int max_cells = std::max(1, static_cast<int>(max_shift / params_.cell));
  const int centre = has_prior_ ? static_cast<int>(std::lround(prior_ / params_.cell)) : 0;
  const int window = has_prior_ ? std::max(1, static_cast<int>(params_.search / params_.cell)) : max_cells;

  int best = 0;
  double best_corr = -2.0;
  double corr_prev = -2.0, corr_next = -2.0;
  for (int s = std::max(0, centre - window); s <= std::min(max_cells, centre + window); ++s) {
    const double c = correlate(previous_, current, -s);  // previous profile slides towards the train
    if (c > best_corr) {
      best_corr = c;
      best = s;
      corr_prev = s > 0 ? correlate(previous_, current, -(s - 1)) : -2.0;
      corr_next = correlate(previous_, current, -(s + 1));
    }
  }
  previous_ = std::move(current);
  correlation_ = best_corr;
  if (best_corr < params_.min_correlation) {
    has_prior_ = false;
    if (!recent_.empty()) {       // the profile did not match; the train did not stop because of that
      advance_ = speed_ * dt;
      predicted_ = true;
    }
    return advance_;
  }
  double shift = best * params_.cell;
  if (corr_prev > -1.5 && corr_next > -1.5) {  // parabola through three samples: sub-cell resolution
    const double denom = corr_prev - 2.0 * best_corr + corr_next;
    if (std::abs(denom) > 1e-9) {
      shift += params_.cell * 0.5 * (corr_prev - corr_next) / denom;
    }
  }
  shift = std::max(0.0, shift);
  predicted_ = false;
  if (!recent_.empty() && std::abs(shift - advance_) > params_.outlier_gate) {
    advance_ = speed_ * dt;       // the peak jumped to another structure: keep what the train was doing
    predicted_ = true;
    has_prior_ = false;
    return advance_;
  }
  recent_.push_back(shift);
  if (static_cast<int>(recent_.size()) > std::max(1, params_.median_window)) {
    recent_.erase(recent_.begin());
  }
  std::vector<double> sorted = recent_;
  std::sort(sorted.begin(), sorted.end());
  advance_ = sorted[sorted.size() / 2];
  speed_ = advance_ / dt;
  prior_ = shift;                 // the search starts from the raw value, the output is the median
  has_prior_ = true;
  return advance_;
}

}  // namespace tod
