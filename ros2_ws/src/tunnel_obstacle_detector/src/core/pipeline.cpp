#include "tunnel_obstacle_detector/core/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "tunnel_obstacle_detector/core/geometry.hpp"

namespace tod
{

namespace
{
using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

Pipeline::Pipeline(const PipelineParams & params)
: params_(params),
  forward_axis_(params.forward_axis == "auto" ? "" : params.forward_axis),
  mount_rotation_(mountRotationFromForwardAxis(params.forward_axis == "auto" ? params.forward_axis_fallback : params.forward_axis)),
  calibrator_(params.calibration),
  axis_detector_(params.axis),
  corridor_estimator_(params.corridor),
  detector_(params.detector),
  tracker_(params.tracker)
{
}

void Pipeline::reset()
{
  calibrator_.reset();
  tracker_.reset();
  last_axis_ = AxisModel();
  last_corridor_ = Corridor();
  has_stamp_ = false;
  result_ = FrameResult();
  if (params_.forward_axis == "auto") {
    forward_axis_.clear();
    forward_axis_frames_ = 0;
  }
}

void Pipeline::updateForwardAxis(const std::vector<RawPoint> & raw)
{
  constexpr int kMaxFrames = 10;
  const std::string detected = detectForwardAxis(raw);
  ++forward_axis_frames_;
  if (detected.empty() && forward_axis_frames_ < kMaxFrames) {
    return;  // keep the fallback for this frame and look again
  }
  forward_axis_ = detected.empty() ? params_.forward_axis_fallback : detected;
  mount_rotation_ = mountRotationFromForwardAxis(forward_axis_);
  calibrator_.reset();  // poses estimated with the fallback axis are meaningless
  last_axis_ = AxisModel();
  last_corridor_ = Corridor();
}

Eigen::Vector3d Pipeline::trackToCloud(const Eigen::Vector3d & track_point) const
{
  const SensorPose & pose = calibrator_.pose();
  const Eigen::Vector3d mount = pose.rotation.transpose() * (track_point - Eigen::Vector3d(0.0, 0.0, pose.height));
  return mount_rotation_.transpose() * mount;
}

Eigen::Matrix3d Pipeline::trackToCloudRotation() const
{
  return mount_rotation_.transpose() * calibrator_.pose().rotation.transpose();
}

const FrameResult & Pipeline::process(const std::vector<RawPoint> & raw, double stamp)
{
  const auto t_start = Clock::now();
  FrameResult & r = result_;
  r = FrameResult();
  r.stamp = stamp;
  r.nearest_distance = kNaN;
  r.nearest_lateral = kNaN;
  r.time_to_collision = kNaN;

  // time since the last frame: it scales how far the corridor may have moved. A jump backwards or a long gap
  // means a restarted recording - the previous corridor and axis say nothing about this frame.
  double dt = 0.1;
  if (has_stamp_) {
    dt = stamp - last_stamp_;
    if (dt < 0.0 || dt > 5.0) {
      last_axis_ = AxisModel();
      last_corridor_ = Corridor();
      dt = 0.1;
    }
  }
  last_stamp_ = stamp;
  has_stamp_ = true;

  // 1. valid returns into the mount frame (X forward, Y left, Z up)
  auto t0 = Clock::now();
  if (forward_axis_.empty()) {
    updateForwardAxis(raw);
  }
  mount_points_.clear();
  mount_points_.reserve(raw.size());
  std::vector<uint32_t> source;
  source.reserve(raw.size());
  const Eigen::Matrix3f rot = mount_rotation_.cast<float>();
  const double min_r2 = params_.min_range * params_.min_range;
  const double max_r2 = params_.max_range * params_.max_range;
  // organized layout: points of one column are ring 0..N-1; the same ring of the previous column is N points back
  uint32_t stride = 0;
  if (params_.dedupe_dual_returns && !raw.empty()) {
    uint16_t max_ring = 0;
    for (std::size_t i = 0; i < std::min<std::size_t>(raw.size(), 4096); ++i) {
      max_ring = std::max(max_ring, raw[i].ring);
    }
    const uint32_t rings = static_cast<uint32_t>(max_ring) + 1U;
    if (rings > 1 && raw.size() % rings == 0 && raw[rings - 1].ring == max_ring && raw[rings].ring == 0) {
      stride = rings;
    }
  }
  for (uint32_t i = 0; i < raw.size(); ++i) {
    const RawPoint & p = raw[i];
    const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
    if (!std::isfinite(r2) || r2 < min_r2 || r2 > max_r2) {
      continue;
    }
    if (stride && i >= stride) {
      const RawPoint & q = raw[i - stride];
      if (q.x == p.x && q.y == p.y && q.z == p.z) {
        continue;  // second return identical to the first one
      }
    }
    mount_points_.push_back(rot * Eigen::Vector3f(p.x, p.y, p.z));
    source.push_back(i);
  }
  r.num_points = mount_points_.size();
  r.timings.transform_ms = msSince(t0);

  // 2. track-bed calibration
  t0 = Clock::now();
  r.pose = calibrator_.update(mount_points_);
  r.timings.calibration_ms = msSince(t0);
  if (!r.pose.valid) {
    r.timings.total_ms = msSince(t_start);
    return r;
  }

  t0 = Clock::now();
  const Eigen::Matrix3f pose_rot = r.pose.rotation.cast<float>();
  const float height = static_cast<float>(r.pose.height);
  // clouds without a ring field: beams of constant elevation above the bed plane act as rings for the rail search
  bool has_rings = false;
  for (std::size_t k = 0; k < source.size() && !has_rings; k += 97) {
    has_rings = raw[source[k]].ring != 0;
  }
  cloud_.resize(mount_points_.size());
  for (std::size_t k = 0; k < mount_points_.size(); ++k) {
    const Eigen::Vector3f q = pose_rot * mount_points_[k];
    TrackPoint & tp = cloud_[k];
    tp.x = q.x();
    tp.y = q.y();
    tp.z = q.z() + height;
    tp.intensity = raw[source[k]].intensity;
    if (has_rings) {
      tp.ring = raw[source[k]].ring;
    } else {
      const double elevation_deg = std::atan2(q.z(), std::hypot(q.x(), q.y())) * 180.0 / M_PI;
      tp.ring = static_cast<uint16_t>(std::clamp((elevation_deg + 90.0) / params_.virtual_ring_deg, 0.0, 65535.0));
    }
    tp.raw_index = source[k];
  }
  r.timings.transform_ms += msSince(t0);

  // 3. near-field track axis from rails
  t0 = Clock::now();
  AxisModel prior;
  if (last_axis_.locked) {
    prior = last_axis_;
  } else {
    // no track history: bootstrap from the nearest rails, where a curve has not yet moved the axis much
    TrackAxisParams near_params = axis_detector_.params();
    near_params.x_max = std::min(near_params.x_max, 16.0);
    near_params.min_inliers = std::max(4, near_params.min_inliers - 1);
    const TrackAxisDetector near_detector(near_params);
    const AxisModel near_model = near_detector.fit(near_detector.detect(cloud_, prior), nullptr);
    if (near_model.locked) {
      prior = near_model;
    }
  }
  std::vector<AxisDetection> detections = axis_detector_.detect(cloud_, prior);
  AxisModel axis = axis_detector_.fit(detections, &r.axis_detections);
  if (!axis.locked && last_axis_.locked) {
    axis.y0 = last_axis_.y0;
    axis.theta0 = last_axis_.theta0;
  }
  r.axis = axis;
  r.timings.axis_ms = msSince(t0);

  // 4. corridor
  t0 = Clock::now();
  r.corridor = corridor_estimator_.estimate(
    cloud_, r.axis_detections, axis, last_corridor_.valid ? &last_corridor_ : nullptr, dt);
  r.timings.corridor_ms = msSince(t0);
  if (axis.locked) {
    last_axis_ = axis;
    last_axis_.y0 = r.corridor.y0;
    last_axis_.theta0 = r.corridor.theta0;
  }
  last_corridor_ = r.corridor;

  // 4a. how far the train drove since the previous frame: one number, but it is what lets the returns of several
  // frames be added up in coordinates tied to the track (see track_advance.hpp)
  r.advance = advance_.update(cloud_, r.corridor, dt);
  r.advance_correlation = advance_.correlation();
  r.speed = std::isfinite(r.advance) && dt > 0.0 ? r.advance / dt :
    std::numeric_limits<double>::quiet_NaN();

  // 5. zones and clusters
  t0 = Clock::now();
  const CorridorEstimator & estimator = corridor_estimator_;
  r.clusters = detector_.detect(
    cloud_, r.corridor,
    [&estimator](float x, float y, float z, float radius) {
      return radius > 0.0F ? estimator.touchesStructure(x, y, z, radius) : estimator.isStructure(x, y, z);
    },
    &labels_, &r.rejected_clusters, r.advance);
  r.trusted_length = detector_.trustedLength();
  r.timings.detection_ms = msSince(t0);

  // 6. temporal confirmation
  t0 = Clock::now();
  tracker_.update(r.clusters, stamp);
  r.obstacles = tracker_.reported();
  r.timings.tracking_ms = msSince(t0);

  // 7. verdict
  for (const Track & tr : r.obstacles) {
    if (static_cast<int>(tr.level) > static_cast<int>(r.level)) {
      r.level = tr.level;
    }
    if (tr.level == Level::kDanger && std::isnan(r.nearest_distance)) {
      r.nearest_distance = tr.distance;
      r.nearest_lateral = tr.lateral;
      r.confidence = tr.confidence;
      if (tr.has_speed && tr.speed < -0.5) {
        r.time_to_collision = tr.distance / -tr.speed;
      }
    }
  }
  if (r.level == Level::kClear && !r.obstacles.empty()) {
    r.level = Level::kWarning;
  }

  // observed length of the corridor: how far returns are seen around the track axis
  std::vector<float> xs;
  xs.reserve(cloud_.size() / 8);
  for (const TrackPoint & tp : cloud_) {
    if (tp.x > 2.0F && tp.x < r.corridor.length() &&
      std::abs(tp.y - r.corridor.lateralAt(tp.x)) < params_.visibility_lateral)
    {
      xs.push_back(tp.x);
    }
  }
  if (!xs.empty()) {
    const std::size_t k = static_cast<std::size_t>(0.995 * static_cast<double>(xs.size() - 1));
    std::nth_element(xs.begin(), xs.begin() + k, xs.end());
    r.free_distance = xs[k];
  }
  r.free_distance = std::min(r.free_distance, r.trusted_length);
  if (!std::isnan(r.nearest_distance)) {
    r.free_distance = std::min(r.free_distance, r.nearest_distance);
  }

  r.valid = true;
  r.timings.total_ms = msSince(t_start);
  return r;
}

}  // namespace tod
