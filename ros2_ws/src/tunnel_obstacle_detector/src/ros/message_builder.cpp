#include "tunnel_obstacle_detector/ros/message_builder.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include <Eigen/Geometry>

namespace tod_ros
{

namespace
{
using visualization_msgs::msg::Marker;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker baseMarker(const std_msgs::msg::Header & header, const std::string & ns, int id, int type)
{
  Marker m;
  m.header = header;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

/// Pose of a track-frame box whose long side follows the corridor heading.
geometry_msgs::msg::Pose boxPose(const tod::Pipeline & pipeline, const Eigen::Vector3d & track_centre, double heading)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d c = pipeline.trackToCloud(track_centre);
  pose.position.x = c.x();
  pose.position.y = c.y();
  pose.position.z = c.z();
  const Eigen::Matrix3d rot = pipeline.trackToCloudRotation() * Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Quaterniond q(rot);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

const char * levelName(tod::Level level)
{
  switch (level) {
    case tod::Level::kDanger: return "DANGER";
    case tod::Level::kWarning: return "WARNING";
    default: return "CLEAR";
  }
}
}  // namespace

tunnel_obstacle_msgs::msg::ObstacleStatus buildStatus(
  const tod::FrameResult & r, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header,
  double transport_latency_ms)
{
  tunnel_obstacle_msgs::msg::ObstacleStatus s;
  s.header = header;
  s.level = static_cast<uint8_t>(r.level);
  s.obstacle_detected = r.level == tod::Level::kDanger;
  s.nearest_distance = static_cast<float>(r.nearest_distance);
  s.nearest_lateral = static_cast<float>(r.nearest_lateral);
  s.time_to_collision = static_cast<float>(r.time_to_collision);
  s.confidence = static_cast<float>(r.confidence);
  s.free_distance = static_cast<float>(r.free_distance);
  s.processing_time_ms = static_cast<float>(r.timings.total_ms);
  s.transport_latency_ms = static_cast<float>(transport_latency_ms);
  s.num_points = static_cast<uint32_t>(r.num_points);
  s.sensor_height = static_cast<float>(r.pose.height);
  s.sensor_pitch_deg = static_cast<float>(r.pose.pitch_deg);
  s.sensor_roll_deg = static_cast<float>(r.pose.roll_deg);
  s.track_curvature = r.corridor.kappas.empty() ? kNaN : static_cast<float>(r.corridor.kappas.front());
  s.track_axis_locked = r.axis.locked;
  s.train_speed = static_cast<float>(r.speed);

  for (const tod::Track & tr : r.obstacles) {
    tunnel_obstacle_msgs::msg::Obstacle o;
    o.id = tr.id;
    o.level = static_cast<uint8_t>(tr.level);
    o.distance = static_cast<float>(tr.distance);
    o.lateral = static_cast<float>(tr.lateral);
    o.height_min = static_cast<float>(tr.h_min);
    o.height_max = static_cast<float>(tr.h_max);
    o.length = static_cast<float>(tr.length);
    o.width = static_cast<float>(tr.width);
    o.num_points = static_cast<uint32_t>(tr.num_points);
    o.hits = static_cast<uint32_t>(tr.hits);
    o.confidence = static_cast<float>(tr.confidence);
    o.relative_speed = tr.has_speed ? static_cast<float>(tr.speed) : kNaN;
    o.position = toPoint(pipeline.trackToCloud(Eigen::Vector3d(tr.cx, tr.cy, tr.cz)));
    s.obstacles.push_back(o);
  }
  return s;
}

vision_msgs::msg::Detection3DArray buildDetections(
  const tod::FrameResult & r, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection3DArray out;
  out.header = header;
  for (const tod::Track & tr : r.obstacles) {
    vision_msgs::msg::Detection3D det;
    det.header = header;
    det.id = std::to_string(tr.id);
    const double heading = r.corridor.headingAt(static_cast<float>(tr.distance));
    const double bed = r.corridor.bedAt(static_cast<float>(tr.distance));
    const Eigen::Vector3d centre(
      tr.distance + 0.5 * tr.length, r.corridor.lateralAt(static_cast<float>(tr.distance)) + tr.lateral,
      bed + 0.5 * (tr.h_min + tr.h_max));
    det.bbox.center = boxPose(pipeline, centre, heading);
    det.bbox.size.x = std::max(0.2, tr.length);
    det.bbox.size.y = std::max(0.2, tr.width);
    det.bbox.size.z = std::max(0.2, tr.h_max - tr.h_min);
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = tr.level == tod::Level::kDanger ? "danger" : "warning";
    hyp.hypothesis.score = tr.confidence;
    hyp.pose.pose = det.bbox.center;
    det.results.push_back(hyp);
    out.detections.push_back(det);
  }
  return out;
}

visualization_msgs::msg::MarkerArray buildMarkers(
  const tod::FrameResult & r, const tod::Pipeline & pipeline, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray arr;
  Marker clear;
  clear.header = header;
  clear.action = Marker::DELETEALL;
  arr.markers.push_back(clear);
  if (!r.valid) {
    return arr;
  }
  const auto & dp = pipeline.params().detector;
  const tod::Corridor & corr = r.corridor;
  const double length = std::min(corr.length(), dp.x_max);

  // corridor axis and envelope edges
  auto edge = [&](int id, double offset, double height, const std_msgs::msg::ColorRGBA & col, double width) {
      Marker m = baseMarker(header, "corridor", id, Marker::LINE_STRIP);
      m.scale.x = width;
      m.color = col;
      for (double x = 1.0; x <= length; x += 2.0) {
        const auto xf = static_cast<float>(x);
        m.points.push_back(toPoint(pipeline.trackToCloud(
          Eigen::Vector3d(x, corr.lateralAt(xf) + offset, corr.bedAt(xf) + height))));
      }
      arr.markers.push_back(m);
    };
  const double hw = dp.zone_half_width_upper;
  edge(0, 0.0, 0.05, color(1.0F, 1.0F, 1.0F, 0.9F), 0.06);
  edge(1, hw, 0.05, color(1.0F, 0.2F, 0.2F, 0.8F), 0.05);
  edge(2, -hw, 0.05, color(1.0F, 0.2F, 0.2F, 0.8F), 0.05);
  edge(3, hw + dp.warning_margin, 0.05, color(1.0F, 0.8F, 0.0F, 0.5F), 0.04);
  edge(4, -hw - dp.warning_margin, 0.05, color(1.0F, 0.8F, 0.0F, 0.5F), 0.04);

  // envelope cross-sections every 25 m
  int ring_id = 0;
  for (double x = 25.0; x <= length; x += 25.0) {
    const auto xf = static_cast<float>(x);
    Marker m = baseMarker(header, "envelope", ring_id++, Marker::LINE_STRIP);
    m.scale.x = 0.04;
    m.color = color(1.0F, 0.3F, 0.3F, 0.6F);
    const double yc = corr.lateralAt(xf);
    const double zb = corr.bedAt(xf);
    const double lo = dp.zone_half_width_lower;
    const double s = dp.zone_split_height;
    const std::vector<std::pair<double, double>> outline{
      {-lo, dp.zone_bottom}, {lo, dp.zone_bottom}, {lo, s}, {hw, s}, {hw, dp.zone_top},
      {-hw, dp.zone_top}, {-hw, s}, {-lo, s}, {-lo, dp.zone_bottom}};
    for (const auto & [dy, dz] : outline) {
      m.points.push_back(toPoint(pipeline.trackToCloud(Eigen::Vector3d(x, yc + dy, zb + dz))));
    }
    arr.markers.push_back(m);
  }

  // observed free part of the track
  {
    Marker m = baseMarker(header, "free", 0, Marker::LINE_STRIP);
    m.scale.x = 0.25;
    m.color = r.level == tod::Level::kDanger ? color(1.0F, 0.1F, 0.1F, 0.9F) : color(0.1F, 1.0F, 0.3F, 0.9F);
    for (double x = 1.0; x <= std::max(1.0, r.free_distance); x += 2.0) {
      const auto xf = static_cast<float>(x);
      m.points.push_back(toPoint(pipeline.trackToCloud(Eigen::Vector3d(x, corr.lateralAt(xf), corr.bedAt(xf) + 0.1))));
    }
    if (m.points.size() >= 2) {
      arr.markers.push_back(m);
    }
  }

  // obstacles
  int id = 0;
  for (const tod::Track & tr : r.obstacles) {
    const bool danger = tr.level == tod::Level::kDanger;
    const auto xf = static_cast<float>(tr.distance);
    const Eigen::Vector3d centre(
      tr.distance + 0.5 * tr.length, corr.lateralAt(xf) + tr.lateral, corr.bedAt(xf) + 0.5 * (tr.h_min + tr.h_max));
    Marker box = baseMarker(header, "obstacles", id, Marker::CUBE);
    box.pose = boxPose(pipeline, centre, corr.headingAt(xf));
    box.scale.x = std::max(0.3, tr.length);
    box.scale.y = std::max(0.3, tr.width);
    box.scale.z = std::max(0.3, tr.h_max - tr.h_min);
    box.color = danger ? color(1.0F, 0.0F, 0.0F, 0.55F) : color(1.0F, 0.6F, 0.0F, 0.45F);
    arr.markers.push_back(box);

    Marker text = baseMarker(header, "labels", id, Marker::TEXT_VIEW_FACING);
    text.pose.position = toPoint(pipeline.trackToCloud(centre + Eigen::Vector3d(0.0, 0.0, 1.6 + 0.01 * tr.distance)));
    text.scale.z = 0.6 + 0.012 * tr.distance;
    text.color = danger ? color(1.0F, 0.2F, 0.2F, 1.0F) : color(1.0F, 0.8F, 0.2F, 1.0F);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s %.1f m", levelName(tr.level), tr.distance);
    text.text = buf;
    arr.markers.push_back(text);
    ++id;
  }

  // banner above the train
  Marker banner = baseMarker(header, "status", 0, Marker::TEXT_VIEW_FACING);
  banner.pose.position = toPoint(pipeline.trackToCloud(Eigen::Vector3d(8.0, 0.0, 4.5)));
  banner.scale.z = 1.0;
  char buf[160];
  if (r.level == tod::Level::kDanger) {
    banner.color = color(1.0F, 0.15F, 0.15F, 1.0F);
    std::snprintf(buf, sizeof(buf), "OBSTACLE %.1f m - BRAKE", r.nearest_distance);
  } else if (r.level == tod::Level::kWarning) {
    banner.color = color(1.0F, 0.75F, 0.1F, 1.0F);
    std::snprintf(buf, sizeof(buf), "WARNING: object near track | free %.0f m", r.free_distance);
  } else {
    banner.color = color(0.2F, 1.0F, 0.4F, 1.0F);
    std::snprintf(buf, sizeof(buf), "PATH CLEAR %.0f m", r.free_distance);
  }
  banner.text = buf;
  arr.markers.push_back(banner);
  return arr;
}

sensor_msgs::msg::PointCloud2 buildPointCloud(
  const std::vector<tod::RawPoint> & raw, const tod::TrackCloud & cloud, const std::vector<uint32_t> & indices,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(indices.size());
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (uint32_t i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step);
  uint8_t * out = msg.data.data();
  for (uint32_t idx : indices) {
    const tod::RawPoint & p = raw[cloud[idx].raw_index];
    const float values[4] = {p.x, p.y, p.z, p.intensity};
    std::memcpy(out, values, 16);
    out += 16;
  }
  return msg;
}

}  // namespace tod_ros
