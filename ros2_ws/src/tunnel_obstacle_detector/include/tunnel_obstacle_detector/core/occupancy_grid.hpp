#pragma once

#include <cstdint>
#include <vector>

namespace tod
{

/// Binary grid indexed by (row = distance ahead, col = lateral or vertical coordinate) with
/// "long structure" filtering and per-row prefix sums for O(1) interval counts.
class OccupancyGrid
{
public:
  void configure(int rows, int cols, double row_res, double col_res, double col_origin);
  void clear();
  void mark(double row_coord, double col_coord);
  /// Marks a cell and tracks the min/max of `value` (e.g. height) seen in it.
  void mark(double row_coord, double col_coord, float value);

  /// Keeps only occupied cells of structures whose occupied cells span more than `min_extent` metres along the rows.
  /// Cells are linked through a dilation of +-max(dilate_base, dilate_quad * x^2) metres along the rows (x = row
  /// distance; far walls are sampled by azimuth columns spaced ~x^2) and +-dilate_cols cells across.
  /// If `tall_low < tall_high`, compact structures whose tracked values span below `tall_low` and above `tall_high`
  /// are kept as well (columns, portal frames).
  /// Columns in [base_only_col_lo, base_only_col_hi] (e.g. inside the clearance envelope) use only `dilate_base`, so
  /// objects on the track never link to distant samples.
  void keepLongStructures(
    double dilate_base, double dilate_quad, int dilate_cols, double min_extent, float tall_low = 0.0F,
    float tall_high = 0.0F, int base_only_col_lo = 0, int base_only_col_hi = -1);

  /// Keeps only cells that belong to runs along the rows (distance) of at least `min_run` cells, allowing gaps of up
  /// to `max_gap` empty cells: horizontal surfaces (floor, ceiling) survive, vertical objects (columns, people) do not.
  void keepRowRuns(int min_run, int max_gap);

  void buildPrefix();

  /// Occupied cells in `row` whose column index lies in [round((lo - origin)/res), round((hi - origin)/res)).
  int count(int row, double col_lo, double col_hi) const;

  bool occupied(int row, int col) const { return occ_[static_cast<std::size_t>(row) * cols_ + col] != 0; }
  int rows() const { return rows_; }
  int cols() const { return cols_; }
  double rowRes() const { return row_res_; }
  std::size_t occupiedCount() const;

private:
  int rows_ = 0;
  int cols_ = 0;
  double row_res_ = 1.0;
  double col_res_ = 1.0;
  double col_origin_ = 0.0;
  std::vector<uint8_t> occ_;
  std::vector<float> vmin_;
  std::vector<float> vmax_;
  std::vector<uint8_t> dilated_;
  std::vector<uint8_t> tmp_;
  std::vector<int32_t> labels_;
  std::vector<int32_t> stack_;
  std::vector<int32_t> prefix_;
};

}  // namespace tod
