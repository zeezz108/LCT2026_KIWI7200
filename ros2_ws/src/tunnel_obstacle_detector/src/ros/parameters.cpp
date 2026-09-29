#include "tunnel_obstacle_detector/ros/parameters.hpp"

#include <string>
#include <vector>

namespace tod_ros
{

namespace
{
void param(rclcpp::Node & node, const std::string & name, double & value)
{
  value = node.declare_parameter<double>(name, value);
}
void param(rclcpp::Node & node, const std::string & name, int & value)
{
  value = static_cast<int>(node.declare_parameter<int64_t>(name, value));
}
void param(rclcpp::Node & node, const std::string & name, std::size_t & value)
{
  value = static_cast<std::size_t>(node.declare_parameter<int64_t>(name, static_cast<int64_t>(value)));
}
void param(rclcpp::Node & node, const std::string & name, bool & value)
{
  value = node.declare_parameter<bool>(name, value);
}
void param(rclcpp::Node & node, const std::string & name, std::string & value)
{
  value = node.declare_parameter<std::string>(name, value);
}
void param(rclcpp::Node & node, const std::string & name, std::vector<double> & value)
{
  value = node.declare_parameter<std::vector<double>>(name, value);
}
}  // namespace

tod::PipelineParams declarePipelineParameters(rclcpp::Node & node)
{
  tod::PipelineParams p;
  param(node, "sensor.forward_axis", p.forward_axis);
  param(node, "sensor.forward_axis_fallback", p.forward_axis_fallback);
  param(node, "sensor.min_range", p.min_range);
  param(node, "sensor.max_range", p.max_range);
  param(node, "visibility_lateral", p.visibility_lateral);
  param(node, "sensor.dedupe_dual_returns", p.dedupe_dual_returns);
  param(node, "sensor.virtual_ring_deg", p.virtual_ring_deg);

  auto & c = p.calibration;
  param(node, "calibration.fixed", c.fixed);
  param(node, "calibration.fixed_height", c.fixed_height);
  param(node, "calibration.fixed_pitch_deg", c.fixed_pitch_deg);
  param(node, "calibration.fixed_roll_deg", c.fixed_roll_deg);
  param(node, "calibration.x_min", c.x_min);
  param(node, "calibration.x_max", c.x_max);
  param(node, "calibration.y_half", c.y_half);
  param(node, "calibration.z_max", c.z_max);
  param(node, "calibration.max_samples", c.max_samples);
  param(node, "calibration.ransac_iterations", c.ransac_iterations);
  param(node, "calibration.inlier_threshold", c.inlier_threshold);
  param(node, "calibration.max_tilt_deg", c.max_tilt_deg);
  param(node, "calibration.min_inliers", c.min_inliers);
  param(node, "calibration.smoothing", c.smoothing);
  param(node, "calibration.max_step_height", c.max_step_height);
  param(node, "calibration.max_step_angle_deg", c.max_step_angle_deg);
  param(node, "calibration.reinit_after_rejects", c.reinit_after_rejects);

  auto & a = p.axis;
  param(node, "track_axis.x_min", a.x_min);
  param(node, "track_axis.x_max", a.x_max);
  param(node, "track_axis.gauge_half", a.gauge_half);
  param(node, "track_axis.rail_min_height", a.rail_min_height);
  param(node, "track_axis.rail_above_bed", a.rail_above_bed);
  param(node, "track_axis.center_search", a.center_search);
  param(node, "track_axis.min_ring_points", a.min_ring_points);
  param(node, "track_axis.min_inliers", a.min_inliers);

  auto & k = p.corridor;
  param(node, "corridor.length", k.x_max);
  param(node, "corridor.cell", k.x_res);
  param(node, "corridor.segment_edges", k.segment_edges);
  param(node, "corridor.kappa_max", k.kappa_max);
  param(node, "corridor.kappa_step", k.kappa_step);
  param(node, "corridor.kappa_change_max", k.kappa_change_max);
  param(node, "corridor.coarse_from", k.coarse_from);
  param(node, "corridor.alternative_cost_abs", k.alternative_cost_abs);
  param(node, "corridor.alternative_cost_rel", k.alternative_cost_rel);
  param(node, "corridor.beam_per_parent", k.beam_per_parent);
  param(node, "corridor.beam_width", k.beam_width);
  param(node, "corridor.w_smooth", k.w_smooth);
  param(node, "corridor.smooth_base", k.smooth_base);
  param(node, "corridor.w_straight", k.w_straight);
  param(node, "corridor.w_straight_far", k.w_straight_far);
  param(node, "corridor.straight_far_from", k.straight_far_from);
  param(node, "corridor.w_margin", k.w_margin);
  param(node, "corridor.w_axis", k.w_axis);
  param(node, "corridor.w_axis_lookahead", k.w_axis_lookahead);
  param(node, "corridor.w_temporal", k.w_temporal);
  param(node, "corridor.kappa_change_rate", k.kappa_change_rate);
  param(node, "corridor.kappa_rate_limit_until", k.kappa_rate_limit_until);
  param(node, "corridor.search_half_width_upper", k.search_half_width_upper);
  param(node, "corridor.search_half_width_lower", k.search_half_width_lower);
  param(node, "corridor.margin", k.margin);
  param(node, "corridor.band_upper_min", k.band_upper_min);
  param(node, "corridor.band_upper_max", k.band_upper_max);
  param(node, "corridor.band_lower_min", k.band_lower_min);
  param(node, "corridor.band_lower_max", k.band_lower_max);
  param(node, "corridor.long_structure_min_length", k.long_structure_min_length);
  param(node, "corridor.band_low_min", k.band_low_min);
  param(node, "corridor.band_low_max", k.band_low_max);
  param(node, "corridor.low_structure_min_length", k.low_structure_min_length);
  param(node, "corridor.gauge_half", k.gauge_half);
  param(node, "corridor.rail_half_width", k.rail_half_width);
  param(node, "corridor.rail_height_default", k.rail_height_default);
  param(node, "corridor.rail_correct_max", k.rail_correct_max);
  param(node, "corridor.rail_height_ref_min", k.rail_height_ref_min);
  param(node, "corridor.rail_height_ref_max", k.rail_height_ref_max);
  param(node, "corridor.dilate_along_base", k.dilate_along_base);
  param(node, "corridor.dilate_along_quad", k.dilate_along_quad);
  param(node, "corridor.tall_low", k.tall_low);
  param(node, "corridor.tall_high", k.tall_high);
  param(node, "corridor.w_wall", k.w_wall);
  param(node, "corridor.wall_ref_min", k.wall_ref_min);
  param(node, "corridor.wall_ref_max", k.wall_ref_max);
  param(node, "corridor.wall_scan_max", k.wall_scan_max);
  param(node, "corridor.wall_ref_min_fraction", k.wall_ref_min_fraction);
  param(node, "corridor.wall_tolerance", k.wall_tolerance);
  param(node, "corridor.wall_tolerance_per_m", k.wall_tolerance_per_m);
  param(node, "corridor.blocked_half_width", k.blocked_half_width);
  param(node, "corridor.blocked_window", k.blocked_window);
  param(node, "corridor.blocked_cells", k.blocked_cells);
  param(node, "corridor.blocked_from", k.blocked_from);
  param(node, "corridor.estimate_vertical", k.estimate_vertical);
  param(node, "corridor.grade_max", k.grade_max);
  param(node, "corridor.w_vertical_smooth", k.w_vertical_smooth);
  param(node, "corridor.w_vertical_above", k.w_vertical_above);
  param(node, "corridor.vertical_above_depth", k.vertical_above_depth);
  param(node, "corridor.w_vertical_temporal", k.w_vertical_temporal);
  param(node, "corridor.vertical_temporal_from", k.vertical_temporal_from);
  param(node, "corridor.w_ceiling_offset", k.w_ceiling_offset);
  param(node, "corridor.w_floor", k.w_floor);
  param(node, "corridor.vertical_lateral_half_width", k.vertical_lateral_half_width);
  param(node, "corridor.floor_tolerance", k.floor_tolerance);
  param(node, "corridor.ceiling_ref_min", k.ceiling_ref_min);
  param(node, "corridor.ceiling_ref_max", k.ceiling_ref_max);
  param(node, "corridor.ceiling_min_height", k.ceiling_min_height);
  param(node, "corridor.ceiling_tolerance", k.ceiling_tolerance);
  param(node, "corridor.ceiling_only_without_floor", k.ceiling_only_without_floor);

  auto & d = p.detector;
  param(node, "zone.split_height", d.zone_split_height);
  param(node, "zone.half_width_lower", d.zone_half_width_lower);
  param(node, "zone.half_width_upper", d.zone_half_width_upper);
  param(node, "zone.bottom", d.zone_bottom);
  param(node, "zone.low_bottom", d.zone_low_bottom);
  param(node, "zone.low_max_distance", d.low_max_distance);
  param(node, "zone.very_low_bottom", d.zone_very_low_bottom);
  param(node, "zone.very_low_max_distance", d.very_low_max_distance);
  param(node, "zone.very_low_min_distance", d.very_low_min_distance);
  param(node, "zone.very_low_warning_only", d.very_low_warning_only);
  param(node, "zone.very_low_half_width", d.very_low_half_width);
  param(node, "zone.very_low_max_length", d.very_low_max_length);
  param(node, "zone.very_low_max_width", d.very_low_max_width);
  param(node, "zone.very_low_rise", d.very_low_rise);
  param(node, "detection.very_low_eps_quad", d.very_low_eps_quad);
  param(node, "detection.very_low_eps_max", d.very_low_eps_max);
  param(node, "detection.accum_frames", d.accum_frames);
  param(node, "detection.accum_from", d.accum_from);
  param(node, "detection.accum_min_points", d.accum_min_points);
  param(node, "detection.accum_min_frames", d.accum_min_frames);
  param(node, "detection.accum_max_length", d.accum_max_length);
  param(node, "detection.accum_max_bottom", d.accum_max_bottom);
  param(node, "detection.accum_warning_only", d.accum_warning_only);
  param(node, "zone.top", d.zone_top);
  param(node, "zone.lateral_sigma0", d.lateral_sigma0);
  param(node, "zone.lateral_sigma_per_m", d.lateral_sigma_per_m);
  param(node, "zone.lateral_sigma_beyond_per_m", d.lateral_sigma_beyond_per_m);
  param(node, "zone.lateral_shrink_max", d.lateral_shrink_max);
  param(node, "zone.bottom_per_m", d.bottom_per_m);
  param(node, "zone.warning_margin", d.warning_margin);
  param(node, "zone.warning_bottom", d.warning_bottom);
  param(node, "zone.warning_top", d.warning_top);
  param(node, "zone.warning_max_distance", d.warning_max_distance);
  param(node, "zone.warning_to_evidence", d.warning_to_evidence);
  param(node, "zone.max_axis_spread", d.max_axis_spread);
  param(node, "detection.min_warning_points", d.min_warning_points);
  param(node, "detection.x_min", d.x_min);
  param(node, "detection.x_max", d.x_max);
  param(node, "detection.eps_near", d.eps_near);
  param(node, "detection.eps_far", d.eps_far);
  param(node, "detection.eps_far_from", d.eps_far_from);
  param(node, "detection.eps_low_per_m", d.eps_low_per_m);
  param(node, "detection.eps_low_max", d.eps_low_max);
  param(node, "detection.reference_height_low", d.reference_height_low);
  param(node, "detection.min_points", d.min_points);
  param(node, "detection.min_danger_points", d.min_danger_points);
  param(node, "detection.danger_fraction", d.danger_fraction);
  param(node, "detection.max_structure_fraction", d.max_structure_fraction);
  param(node, "detection.object_height_near", d.object_height_near);
  param(node, "detection.object_height_low", d.object_height_low);
  param(node, "detection.low_requires_ground", d.low_requires_ground);
  param(node, "detection.low_cluster_margin", d.low_cluster_margin);
  param(node, "detection.ground_window_quad", d.ground_window_quad);
  param(node, "detection.ground_window_max", d.ground_window_max);
  param(node, "detection.object_height_per_m", d.object_height_per_m);
  param(node, "detection.object_height_far", d.object_height_far);
  param(node, "detection.density_factor", d.density_factor);
  param(node, "detection.keep_weak_clusters", d.keep_weak_clusters);
  param(node, "detection.weak_max_required", d.weak_max_required);
  param(node, "detection.max_bottom_height", d.max_bottom_height);
  param(node, "detection.floating_from", d.floating_from);
  param(node, "detection.max_bottom_height_per_m", d.max_bottom_height_per_m);
  param(node, "detection.min_far_extent", d.min_far_extent);
  param(node, "detection.surface_side_min", d.surface_side_min);
  param(node, "detection.surface_side_max", d.surface_side_max);
  param(node, "detection.surface_side_points", d.surface_side_points);
  param(node, "detection.reference_width", d.reference_width);
  param(node, "detection.reference_height", d.reference_height);

  auto & t = p.tracker;
  param(node, "tracking.max_relative_speed", t.max_relative_speed);
  param(node, "tracking.gate_along_base", t.gate_along_base);
  param(node, "tracking.gate_along_rel", t.gate_along_rel);
  param(node, "tracking.gate_lateral", t.gate_lateral);
  param(node, "tracking.confirm_hits", t.confirm_hits);
  param(node, "tracking.confirm_window", t.confirm_window);
  param(node, "tracking.confirm_far_from", t.confirm_far_from);
  param(node, "tracking.confirm_hits_per_25m", t.confirm_hits_per_25m);
  param(node, "tracking.confirm_hits_max", t.confirm_hits_max);
  param(node, "tracking.danger_hits", t.danger_hits);
  param(node, "tracking.weak_confirm_hits", t.weak_confirm_hits);
  param(node, "tracking.weak_confirm_window", t.weak_confirm_window);
  param(node, "tracking.max_misses", t.max_misses);
  param(node, "tracking.report_max_misses", t.report_max_misses);
  param(node, "tracking.strong_points", t.strong_points);
  param(node, "tracking.strong_max_distance", t.strong_max_distance);
  return p;
}

}  // namespace tod_ros
