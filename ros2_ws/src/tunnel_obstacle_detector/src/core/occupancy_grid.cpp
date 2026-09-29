#include "tunnel_obstacle_detector/core/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tod
{

void OccupancyGrid::configure(int rows, int cols, double row_res, double col_res, double col_origin)
{
  rows_ = rows;
  cols_ = cols;
  row_res_ = row_res;
  col_res_ = col_res;
  col_origin_ = col_origin;
  const std::size_t n = static_cast<std::size_t>(rows) * cols;
  occ_.assign(n, 0);
  vmin_.assign(n, std::numeric_limits<float>::infinity());
  vmax_.assign(n, -std::numeric_limits<float>::infinity());
  dilated_.assign(n, 0);
  tmp_.assign(n, 0);
  labels_.assign(n, 0);
  prefix_.assign(static_cast<std::size_t>(rows) * (cols + 1), 0);
}

void OccupancyGrid::clear()
{
  std::fill(occ_.begin(), occ_.end(), 0);
  std::fill(vmin_.begin(), vmin_.end(), std::numeric_limits<float>::infinity());
  std::fill(vmax_.begin(), vmax_.end(), -std::numeric_limits<float>::infinity());
}

void OccupancyGrid::mark(double row_coord, double col_coord)
{
  const int r = static_cast<int>(std::floor(row_coord / row_res_));
  const int c = static_cast<int>(std::floor((col_coord - col_origin_) / col_res_));
  if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
    occ_[static_cast<std::size_t>(r) * cols_ + c] = 1;
  }
}

void OccupancyGrid::mark(double row_coord, double col_coord, float value)
{
  const int r = static_cast<int>(std::floor(row_coord / row_res_));
  const int c = static_cast<int>(std::floor((col_coord - col_origin_) / col_res_));
  if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
    const std::size_t i = static_cast<std::size_t>(r) * cols_ + c;
    occ_[i] = 1;
    vmin_[i] = std::min(vmin_[i], value);
    vmax_[i] = std::max(vmax_[i], value);
  }
}

void OccupancyGrid::keepLongStructures(
  double dilate_base, double dilate_quad, int dilate_cols, double min_extent, float tall_low, float tall_high,
  int base_only_col_lo, int base_only_col_hi)
{
  const bool use_tall = tall_low < tall_high;
  const int R = rows_;
  const int C = cols_;
  // dilation along the rows with a distance-dependent half width, via per-column prefix counts
  std::vector<int32_t> counts(static_cast<std::size_t>(R + 1));
  std::vector<int> half(static_cast<std::size_t>(R));
  const int base_cells = static_cast<int>(std::lround(dilate_base / row_res_));
  for (int r = 0; r < R; ++r) {
    const double x = (r + 0.5) * row_res_;
    half[static_cast<std::size_t>(r)] = static_cast<int>(std::lround(std::max(dilate_base, dilate_quad * x * x) / row_res_));
  }
  for (int c = 0; c < C; ++c) {
    const bool base_only = c >= base_only_col_lo && c <= base_only_col_hi;
    counts[0] = 0;
    for (int r = 0; r < R; ++r) {
      counts[static_cast<std::size_t>(r + 1)] = counts[static_cast<std::size_t>(r)] + occ_[static_cast<std::size_t>(r) * C + c];
    }
    for (int r = 0; r < R; ++r) {
      const int d = base_only ? base_cells : half[static_cast<std::size_t>(r)];
      const int lo = std::max(0, r - d);
      const int hi = std::min(R - 1, r + d);
      tmp_[static_cast<std::size_t>(r) * C + c] =
        counts[static_cast<std::size_t>(hi + 1)] - counts[static_cast<std::size_t>(lo)] > 0 ? 1 : 0;
    }
  }
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      uint8_t v = 0;
      for (int dc = -dilate_cols; dc <= dilate_cols && !v; ++dc) {
        const int cc = c + dc;
        if (cc >= 0 && cc < C) {
          v = tmp_[static_cast<std::size_t>(r) * C + cc];
        }
      }
      dilated_[static_cast<std::size_t>(r) * C + c] = v;
    }
  }

  // 4-connected components of the dilated grid, keep those long along the rows
  std::fill(labels_.begin(), labels_.end(), 0);
  std::vector<char> keep(1, 0);
  int next = 1;
  for (int start = 0; start < R * C; ++start) {
    if (!dilated_[start] || labels_[start]) {
      continue;
    }
    int rmin = std::numeric_limits<int>::max();  // extent of the *occupied* cells of the component
    int rmax = -1;
    float vlow = std::numeric_limits<float>::infinity();
    float vhigh = -std::numeric_limits<float>::infinity();
    stack_.clear();
    stack_.push_back(start);
    labels_[start] = next;
    while (!stack_.empty()) {
      const int cell = stack_.back();
      stack_.pop_back();
      const int r = cell / C;
      const int c = cell % C;
      if (occ_[cell]) {
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
        vlow = std::min(vlow, vmin_[cell]);
        vhigh = std::max(vhigh, vmax_[cell]);
      }
      const int neighbours[4][2] = {{r - 1, c}, {r + 1, c}, {r, c - 1}, {r, c + 1}};
      for (const auto & nb : neighbours) {
        if (nb[0] < 0 || nb[0] >= R || nb[1] < 0 || nb[1] >= C) {
          continue;
        }
        const int idx = nb[0] * C + nb[1];
        if (dilated_[idx] && !labels_[idx]) {
          labels_[idx] = next;
          stack_.push_back(idx);
        }
      }
    }
    const bool is_long = rmax >= rmin && (rmax - rmin + 1) * row_res_ > min_extent;
    const bool is_tall = use_tall && vlow < tall_low && vhigh > tall_high;
    keep.push_back(is_long || is_tall ? 1 : 0);
    ++next;
  }
  for (int i = 0; i < R * C; ++i) {
    if (occ_[i] && !keep[labels_[i]]) {
      occ_[i] = 0;
    }
  }
}

void OccupancyGrid::keepRowRuns(int min_run, int max_gap)
{
  // runs are scanned along the rows for every column (a column = one lateral/vertical coordinate)
  std::fill(tmp_.begin(), tmp_.end(), 0);
  for (int c = 0; c < cols_; ++c) {
    int r = 0;
    while (r < rows_) {
      if (!occ_[static_cast<std::size_t>(r) * cols_ + c]) {
        ++r;
        continue;
      }
      int start = r;
      int last = r;
      int count = 0;
      while (r < rows_ && r - last <= max_gap + 1) {
        if (occ_[static_cast<std::size_t>(r) * cols_ + c]) {
          last = r;
          ++count;
        }
        ++r;
      }
      if (last - start + 1 >= min_run) {
        for (int k = start; k <= last; ++k) {
          const std::size_t i = static_cast<std::size_t>(k) * cols_ + c;
          tmp_[i] = occ_[i];
        }
      }
      r = last + 1;
    }
  }
  occ_.swap(tmp_);
}

void OccupancyGrid::buildPrefix()
{
  for (int r = 0; r < rows_; ++r) {
    int32_t acc = 0;
    const std::size_t base = static_cast<std::size_t>(r) * (cols_ + 1);
    prefix_[base] = 0;
    for (int c = 0; c < cols_; ++c) {
      acc += occ_[static_cast<std::size_t>(r) * cols_ + c];
      prefix_[base + c + 1] = acc;
    }
  }
}

int OccupancyGrid::count(int row, double col_lo, double col_hi) const
{
  if (row < 0 || row >= rows_) {
    return 0;
  }
  // round-half-up via an offset that keeps the argument positive (much cheaper than lround)
  constexpr double kOffset = 1 << 20;
  const int lo = std::clamp(
    static_cast<int>((col_lo - col_origin_) / col_res_ + 0.5 + kOffset) - (1 << 20), 0, cols_);
  const int hi = std::clamp(
    static_cast<int>((col_hi - col_origin_) / col_res_ + 0.5 + kOffset) - (1 << 20), 0, cols_);
  if (hi <= lo) {
    return 0;
  }
  const std::size_t base = static_cast<std::size_t>(row) * (cols_ + 1);
  return prefix_[base + hi] - prefix_[base + lo];
}

std::size_t OccupancyGrid::occupiedCount() const
{
  return static_cast<std::size_t>(std::count(occ_.begin(), occ_.end(), 1));
}

}  // namespace tod
