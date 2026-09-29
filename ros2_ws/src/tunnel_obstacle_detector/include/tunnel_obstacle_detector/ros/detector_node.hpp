// ROS 2 node: subscribes to the lidar point cloud, runs the detection pipeline and publishes the verdict,
// confirmed obstacles and RViz markers. Header-only so that the node itself can be tested (test_node.cpp).
#pragma once

#include <algorithm>
#include <chrono>
#include <map>
#include <iomanip>
#include <array>
#include <cmath>
#include <limits>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tunnel_obstacle_msgs/msg/obstacle_status.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tunnel_obstacle_detector/core/pipeline.hpp"
#include "tunnel_obstacle_detector/ros/cloud_conversion.hpp"
#include "tunnel_obstacle_detector/ros/message_builder.hpp"
#include "tunnel_obstacle_detector/ros/parameters.hpp"

namespace tod_ros
{

class DetectorNode : public rclcpp::Node
{
public:
  ~DetectorNode() override {writeSummary();}

  explicit DetectorNode(const rclcpp::NodeOptions & options)
  : Node("tunnel_obstacle_detector", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    publish_zone_points_ = declare_parameter<bool>("publish_zone_points", false);
    log_period_s_ = declare_parameter<double>("log_period_s", 1.0);
    csv_path_ = declare_parameter<std::string>("csv_log_path", "");
    summary_path_ = declare_parameter<std::string>("summary_path", "");
    track_frame_ = declare_parameter<std::string>("track_frame", "track");
    view_decimation_ = std::max<int64_t>(1, declare_parameter<int64_t>("view_decimation", 2));
    view_max_range_ = declare_parameter<double>("view_max_range", 210.0);
    pipeline_ = std::make_unique<tod::Pipeline>(declarePipelineParameters(*this));

    status_pub_ = create_publisher<tunnel_obstacle_msgs::msg::ObstacleStatus>("~/status", 10);
    detections_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/obstacles", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    obstacle_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacle_points", rclcpp::SensorDataQoS());
    zone_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/zone_points", rclcpp::SensorDataQoS());
    view_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/view_points", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    if (!csv_path_.empty()) {
      csv_.open(csv_path_);
      csv_ << "stamp,level,nearest_distance,nearest_lateral,ttc,confidence,free_distance,num_obstacles,"
              "processing_ms,transport_ms,num_points,height,pitch_deg,roll_deg,kappa0,axis_locked\n";
    }

    if (input_topic_.empty()) {
      RCLCPP_INFO(get_logger(), "input_topic is empty: waiting for any sensor_msgs/msg/PointCloud2 topic");
      discovery_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {discoverTopic();});
    } else {
      subscribe(input_topic_);
    }
    log_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, log_period_s_)), [this]() {logSummary();});
  }

private:
  void discoverTopic()
  {
    const std::string own_prefix = std::string(get_fully_qualified_name()) + "/";
    for (const auto & [name, types] : get_topic_names_and_types()) {
      if (name.rfind(own_prefix, 0) == 0) {
        continue;
      }
      if (std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end()) {
        subscribe(name);
        discovery_timer_->cancel();
        return;
      }
    }
  }

  void subscribe(const std::string & topic)
  {
    RCLCPP_INFO(get_logger(), "subscribing to %s", topic.c_str());
    input_topic_ = topic;
    std::function<void(std::shared_ptr<const sensor_msgs::msg::PointCloud2>, const rclcpp::MessageInfo &)> cb =
      [this](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg, const rclcpp::MessageInfo & info) {
        onCloud(*msg, info);
      };
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS().keep_last(2), cb);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & msg, const rclcpp::MessageInfo & info)
  {
    const auto wall_start = std::chrono::system_clock::now();
    const int64_t source_ns = info.get_rmw_message_info().source_timestamp;
    const double transport_ms = source_ns > 0 ?
      std::chrono::duration<double, std::milli>(wall_start.time_since_epoch()).count() - source_ns * 1e-6 :
      std::numeric_limits<double>::quiet_NaN();

    if (!layout_.valid || layout_.point_step != msg.point_step) {
      std::string error;
      layout_ = analyzeLayout(msg, error);
      if (!layout_.valid) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported point cloud: %s", error.c_str());
        return;
      }
      if (layout_.ring < 0) {
        RCLCPP_WARN(get_logger(), "cloud has no ring field: rail-based track axis is disabled");
      }
    }
    convertCloud(msg, layout_, raw_);
    const auto t_converted = std::chrono::system_clock::now();
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const tod::FrameResult & r = pipeline_->process(raw_, stamp);
    const auto t_processed = std::chrono::system_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(t_processed - wall_start).count();
    last_convert_ms_ = std::chrono::duration<double, std::milli>(t_converted - wall_start).count();
    last_pipeline_ms_ = std::chrono::duration<double, std::milli>(t_processed - t_converted).count();

    auto status = buildStatus(r, *pipeline_, msg.header, transport_ms);
    status.processing_time_ms = static_cast<float>(total_ms);
    status_pub_->publish(status);
    detections_pub_->publish(buildDetections(r, *pipeline_, msg.header));
    if (publish_markers_ && markers_pub_->get_subscription_count() > 0) {
      markers_pub_->publish(buildMarkers(r, *pipeline_, msg.header));
    }
    if (obstacle_points_pub_->get_subscription_count() > 0) {
      std::vector<uint32_t> idx;
      for (const auto & tr : r.obstacles) {
        idx.insert(idx.end(), tr.points.begin(), tr.points.end());
      }
      obstacle_points_pub_->publish(buildPointCloud(raw_, pipeline_->trackCloud(), idx, msg.header));
    }
    if (r.valid) {
      publishTrackFrame(r, msg.header.frame_id);
    }
    if (r.valid && view_pub_->get_subscription_count() > 0) {
      view_pub_->publish(buildViewCloud(msg.header));
    }
    if (publish_zone_points_ && zone_points_pub_->get_subscription_count() > 0) {
      std::vector<uint32_t> idx;
      const auto & labels = pipeline_->zoneLabels();
      for (uint32_t i = 0; i < labels.size(); ++i) {
        if (labels[i] != tod::kOutside) {
          idx.push_back(i);
        }
      }
      zone_points_pub_->publish(buildPointCloud(raw_, pipeline_->trackCloud(), idx, msg.header));
    }
    if (csv_.is_open()) {
      csv_ << std::fixed << stamp << ',' << static_cast<int>(r.level) << ',' << r.nearest_distance << ','
           << r.nearest_lateral << ',' << r.time_to_collision << ',' << r.confidence << ',' << r.free_distance << ','
           << r.obstacles.size() << ',' << total_ms << ',' << transport_ms << ',' << r.num_points << ','
           << r.pose.height << ',' << r.pose.pitch_deg << ',' << r.pose.roll_deg << ','
           << (r.corridor.kappas.empty() ? 0.0 : r.corridor.kappas.front()) << ',' << r.axis.locked << '\n';
    }

    if (!summary_path_.empty()) {
      ++summary_frames_;
      summary_levels_[static_cast<std::size_t>(r.level)] += 1;
      for (const auto & o : r.obstacles) {
        ObstacleSummary & sum = summary_objects_[o.id];
        if (sum.frames == 0) {
          sum.first_stamp = stamp;
          sum.first_distance = o.distance;
          sum.min_distance = o.distance;
        }
        sum.last_stamp = stamp;
        sum.last_distance = o.distance;
        sum.min_distance = std::min(sum.min_distance, static_cast<double>(o.distance));
        sum.lateral = o.lateral;
        sum.height = o.h_max;
        sum.max_level = std::max<int>(sum.max_level, static_cast<int>(o.level));
        sum.max_points = std::max<int>(sum.max_points, static_cast<int>(o.num_points));
        ++sum.frames;
      }
    }

    last_publish_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::system_clock::now() - t_processed).count();
    ++frames_;
    proc_ms_.push_back(total_ms);
    if (proc_ms_.size() > 200) {
      proc_ms_.pop_front();
    }
    last_level_ = r.level;
    last_nearest_ = r.nearest_distance;
    last_free_ = r.free_distance;
    last_obstacles_ = r.obstacles.size();
    last_locked_ = r.axis.locked;
    last_height_ = r.pose.height;
    last_speed_ = r.speed;
    if (std::isfinite(r.speed)) {
      summary_speeds_.push_back(r.speed);
    }
    last_transport_ms_ = transport_ms;
  }

  /// Static TF track -> cloud frame from the online calibration, re-sent when the pose drifts noticeably.
  void publishTrackFrame(const tod::FrameResult & r, const std::string & cloud_frame)
  {
    const bool changed = cloud_frame != tf_child_ || pipeline_->forwardAxis() != tf_forward_ ||
      std::abs(r.pose.height - tf_height_) > 0.01 || std::abs(r.pose.pitch_deg - tf_pitch_) > 0.1 ||
      std::abs(r.pose.roll_deg - tf_roll_) > 0.1;
    if (!changed) {
      return;
    }
    if (pipeline_->forwardAxis() != tf_forward_) {
      RCLCPP_INFO(get_logger(), "forward axis of the cloud: %s", pipeline_->forwardAxis().c_str());
    }
    tf_child_ = cloud_frame;
    tf_forward_ = pipeline_->forwardAxis();
    tf_height_ = r.pose.height;
    tf_pitch_ = r.pose.pitch_deg;
    tf_roll_ = r.pose.roll_deg;
    // p_track = R_cloud_to_track * p_cloud + (0, 0, h)
    const Eigen::Matrix3d rot = pipeline_->trackToCloudRotation().transpose();
    const Eigen::Quaterniond q(rot);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = track_frame_;
    tf.child_frame_id = cloud_frame;
    tf.transform.translation.z = r.pose.height;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  /// Decimated cloud for RViz with a `zone` channel (0 outside, 1 warning zone, 2 danger zone).
  sensor_msgs::msg::PointCloud2 buildViewCloud(const std_msgs::msg::Header & header)
  {
    const auto & cloud = pipeline_->trackCloud();
    const auto & labels = pipeline_->zoneLabels();
    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
    out.height = 1;
    const char * names[5] = {"x", "y", "z", "intensity", "zone"};
    for (uint32_t i = 0; i < 5; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      out.fields.push_back(f);
    }
    out.point_step = 20;
    out.is_bigendian = false;
    out.is_dense = true;
    out.data.resize(cloud.size() / static_cast<std::size_t>(view_decimation_) * 20 + 20);
    uint8_t * dst = out.data.data();
    uint32_t count = 0;
    const float max_r = static_cast<float>(view_max_range_);
    for (std::size_t i = 0; i < cloud.size(); i += static_cast<std::size_t>(view_decimation_)) {
      if (cloud[i].x > max_r) {
        continue;
      }
      const tod::RawPoint & p = raw_[cloud[i].raw_index];
      const float zone = i < labels.size() ? static_cast<float>(labels[i]) : 0.0F;
      const float values[5] = {p.x, p.y, p.z, p.intensity, zone};
      std::memcpy(dst, values, 20);
      dst += 20;
      ++count;
    }
    out.width = count;
    out.row_step = out.point_step * count;
    out.data.resize(out.row_step);
    return out;
  }

  /// A file a reviewer can open instead of watching RViz: how many frames, what was reported, where and for how
  /// long. Rewritten every log period, so it is complete even if the container is stopped mid-run.
  void writeSummary()
  {
    if (summary_path_.empty() || summary_frames_ == 0) {
      return;
    }
    std::vector<double> ms(proc_ms_.begin(), proc_ms_.end());
    std::sort(ms.begin(), ms.end());
    const auto pct = [&ms](double q) {
        return ms.empty() ? 0.0 : ms[std::min(ms.size() - 1, static_cast<std::size_t>(q * ms.size()))];
      };
    // the speed of the train, measured from the lidar alone (no odometry is available on these recordings)
    std::vector<double> speeds = summary_speeds_;
    std::sort(speeds.begin(), speeds.end());
    const double speed_kmh = speeds.empty() ? 0.0 : 3.6 * speeds[speeds.size() / 2];
    std::ofstream out(summary_path_);
    out << std::fixed << std::setprecision(3);
    out << "{\n  \"frames\": " << summary_frames_
        << ",\n  \"frames_clear\": " << summary_levels_[0]
        << ",\n  \"frames_warning\": " << summary_levels_[1]
        << ",\n  \"frames_danger\": " << summary_levels_[2]
        << ",\n  \"processing_ms_p50\": " << pct(0.5)
        << ",\n  \"processing_ms_p95\": " << pct(0.95)
        << ",\n  \"train_speed_kmh_median\": " << speed_kmh
        << ",\n  \"input_topic\": \"" << input_topic_ << "\""
        << ",\n  \"objects\": [";
    bool first = true;
    for (const auto & [id, o] : summary_objects_) {
      out << (first ? "\n    " : ",\n    ")
          << "{\"id\": " << id
          << ", \"level\": \"" << (o.max_level == 2 ? "DANGER" : "WARNING") << "\""
          << ", \"frames\": " << o.frames
          << ", \"seen_from_m\": " << o.first_distance
          << ", \"nearest_m\": " << o.min_distance
          << ", \"lateral_m\": " << o.lateral
          << ", \"height_m\": " << o.height
          << ", \"max_points\": " << o.max_points
          << ", \"seconds\": " << (o.last_stamp - o.first_stamp) << "}";
      first = false;
    }
    out << (first ? "" : "\n  ") << "]\n}\n";
  }

  void logSummary()
  {
    writeSummary();
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_log_).count();
    last_log_ = now;
    if (frames_ == frames_at_last_log_) {
      return;
    }
    const double fps = (frames_ - frames_at_last_log_) / std::max(dt, 1e-3);
    frames_at_last_log_ = frames_;
    std::vector<double> sorted(proc_ms_.begin(), proc_ms_.end());
    std::sort(sorted.begin(), sorted.end());
    const double p50 = sorted[sorted.size() / 2];
    const double p95 = sorted[std::min(sorted.size() - 1, static_cast<std::size_t>(0.95 * sorted.size()))];
    const char * level = last_level_ == tod::Level::kDanger ? "DANGER " :
      (last_level_ == tod::Level::kWarning ? "WARNING" : "CLEAR  ");
    char speed[32] = "";
    if (std::isfinite(last_speed_)) {  // measured from the lidar alone, no odometry on the train
      std::snprintf(speed, sizeof(speed), " v=%.0f km/h", 3.6 * last_speed_);
    }
    char timing[160];
    std::snprintf(timing, sizeof(timing), "%.1f fps proc p50=%.1f p95=%.1f ms (convert %.1f + algo %.1f + publish %.1f) "
      "transport=%.1f ms", fps, p50, p95, last_convert_ms_, last_pipeline_ms_, last_publish_ms_, last_transport_ms_);
    if (last_level_ == tod::Level::kDanger) {
      RCLCPP_WARN(get_logger(), "[%s] obstacle at %.1f m | objects=%zu free=%.0f m%s | %s | lidar h=%.2f m axis=%s",
        level, last_nearest_, last_obstacles_, last_free_, speed, timing, last_height_,
        last_locked_ ? "rails" : "walls");
    } else {
      RCLCPP_INFO(get_logger(), "[%s] objects=%zu free=%.0f m%s | %s | lidar h=%.2f m axis=%s", level, last_obstacles_,
        last_free_, speed, timing, last_height_, last_locked_ ? "rails" : "walls");
    }
  }

  std::string input_topic_;
  bool publish_markers_ = true;
  bool publish_zone_points_ = false;
  double log_period_s_ = 1.0;
  struct ObstacleSummary
  {
    int frames = 0;
    int max_level = 0;
    int max_points = 0;
    double first_stamp = 0.0;
    double last_stamp = 0.0;
    double first_distance = 0.0;
    double last_distance = 0.0;
    double min_distance = 0.0;
    double lateral = 0.0;
    double height = 0.0;
  };

  std::string csv_path_;
  std::ofstream csv_;
  std::string summary_path_;
  std::map<uint32_t, ObstacleSummary> summary_objects_;
  std::size_t summary_frames_ = 0;
  std::array<std::size_t, 3> summary_levels_{0, 0, 0};
  std::string track_frame_;
  int64_t view_decimation_ = 2;
  double view_max_range_ = 210.0;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::string tf_child_;
  std::string tf_forward_;
  double tf_height_ = 0.0;
  double tf_pitch_ = 0.0;
  double tf_roll_ = 0.0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr view_pub_;

  std::unique_ptr<tod::Pipeline> pipeline_;
  CloudLayout layout_;
  std::vector<tod::RawPoint> raw_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<tunnel_obstacle_msgs::msg::ObstacleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr zone_points_pub_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  std::size_t frames_ = 0;
  std::size_t frames_at_last_log_ = 0;
  std::chrono::steady_clock::time_point last_log_ = std::chrono::steady_clock::now();
  std::deque<double> proc_ms_;
  tod::Level last_level_ = tod::Level::kClear;
  double last_nearest_ = 0.0;
  double last_free_ = 0.0;
  std::size_t last_obstacles_ = 0;
  bool last_locked_ = false;
  double last_speed_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> summary_speeds_;  ///< every measured frame speed of the run, for the summary median
  double last_height_ = 0.0;
  double last_transport_ms_ = 0.0;
  double last_convert_ms_ = 0.0;
  double last_pipeline_ms_ = 0.0;
  double last_publish_ms_ = 0.0;
};

}  // namespace tod_ros
