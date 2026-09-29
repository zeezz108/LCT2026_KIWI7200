// Offline evaluation: reads a bag directly (no DDS, no real-time pacing), runs the pipeline on every frame and
// writes one JSON line per frame. Parameters are the same ROS parameters as the node:
//   ros2 run tunnel_obstacle_detector offline_runner --ros-args --params-file detector.yaml
//        -p bag:=/data/doubleT_obstacle -p output:=/tmp/result.jsonl

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tunnel_obstacle_detector/core/pipeline.hpp"
#include "tunnel_obstacle_detector/ros/cloud_conversion.hpp"
#include "tunnel_obstacle_detector/ros/parameters.hpp"

namespace
{
std::string num(double v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tunnel_obstacle_offline_runner");
  const std::string bag = node->declare_parameter<std::string>("bag", "");
  std::string topic = node->declare_parameter<std::string>("input_topic", "");
  const std::string output = node->declare_parameter<std::string>("output", "offline_result.jsonl");
  const int64_t max_frames = node->declare_parameter<int64_t>("max_frames", -1);
  const int64_t skip_frames = node->declare_parameter<int64_t>("skip_frames", 0);
  // frames whose corridor, zones and clusters are dumped for plotting (tools/plot_debug_frame.py)
  const std::vector<int64_t> debug_frames = node->declare_parameter<std::vector<int64_t>>("debug_frames", {-1});
  const std::string debug_dir = node->declare_parameter<std::string>("debug_dir", ".");
  tod::Pipeline pipeline(tod_ros::declarePipelineParameters(*node));
  if (bag.empty()) {
    RCLCPP_ERROR(node->get_logger(), "parameter 'bag' is required");
    return 1;
  }

  rosbag2_cpp::Reader reader;
  reader.open(bag);
  if (topic.empty()) {
    for (const auto & meta : reader.get_all_topics_and_types()) {
      if (meta.type == "sensor_msgs/msg/PointCloud2") {
        topic = meta.name;
        break;
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "bag=%s topic=%s output=%s", bag.c_str(), topic.c_str(), output.c_str());

  std::ofstream out(output);
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  tod_ros::CloudLayout layout;
  std::vector<tod::RawPoint> raw;
  std::vector<double> times;
  int64_t frame = -1;
  int64_t processed = 0;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != topic) {
      continue;
    }
    ++frame;
    if (frame < skip_frames) {
      continue;
    }
    if (max_frames >= 0 && processed >= max_frames) {
      break;
    }
    sensor_msgs::msg::PointCloud2 msg;
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    serialization.deserialize_message(&serialized, &msg);

    const auto t0 = std::chrono::steady_clock::now();
    if (!layout.valid || layout.point_step != msg.point_step) {
      std::string error;
      layout = tod_ros::analyzeLayout(msg, error);
      if (!layout.valid) {
        RCLCPP_ERROR(node->get_logger(), "unsupported cloud: %s", error.c_str());
        return 1;
      }
    }
    tod_ros::convertCloud(msg, layout, raw);
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const tod::FrameResult & r = pipeline.process(raw, stamp);
    const double total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    times.push_back(total_ms);
    ++processed;

    out << "{\"frame\":" << frame << ",\"stamp\":" << std::fixed << stamp << ",\"bag_time\":" << bag_msg->time_stamp
        << ",\"valid\":" << (r.valid ? "true" : "false") << ",\"level\":" << static_cast<int>(r.level)
        << ",\"nearest\":" << num(r.nearest_distance) << ",\"lateral\":" << num(r.nearest_lateral)
        << ",\"ttc\":" << num(r.time_to_collision) << ",\"free\":" << num(r.free_distance)
        << ",\"trusted\":" << num(r.trusted_length)
        << ",\"height\":" << num(r.pose.height) << ",\"pitch\":" << num(r.pose.pitch_deg)
        << ",\"roll\":" << num(r.pose.roll_deg) << ",\"axis_locked\":" << (r.axis.locked ? "true" : "false")
        << ",\"axis_detections\":" << r.axis_detections.size() << ",\"theta0\":" << num(r.corridor.theta0)
        << ",\"wall_left\":" << num(r.corridor.wall_left) << ",\"wall_right\":" << num(r.corridor.wall_right)
        << ",\"evidence_range\":" << num(r.corridor.evidence_range)
        << ",\"advance\":" << num(r.advance) << ",\"advance_corr\":" << num(r.advance_correlation)
        << ",\"blocked\":" << num(std::isfinite(r.corridor.blocked_range) ? r.corridor.blocked_range : -1.0)
        << ",\"kappas\":[";
    for (std::size_t i = 0; i < r.corridor.kappas.size(); ++i) {
      out << (i ? "," : "") << num(r.corridor.kappas[i] * 1e3);
    }
    out << "],\"grades\":[";
    for (std::size_t i = 0; i < r.corridor.grades.size(); ++i) {
      out << (i ? "," : "") << num(r.corridor.grades[i] * 1e2);
    }
    // axis and bed every 10 m (reference axis for synthetic injection, plots)
    out << "],\"axis_lat\":[";
    for (int x = 0; x <= 200; x += 10) {
      out << (x ? "," : "") << (r.corridor.valid ? num(r.corridor.lateralAt(static_cast<float>(x))) : "null");
    }
    out << "],\"axis_bed\":[";
    for (int x = 0; x <= 200; x += 10) {
      out << (x ? "," : "") << (r.corridor.valid ? num(r.corridor.bedAt(static_cast<float>(x))) : "null");
    }
    // how far near-optimal axis hypotheses disagree: the detector's own uncertainty
    out << "],\"axis_spread\":[";
    for (int x = 0; x <= 200; x += 10) {
      out << (x ? "," : "") << (r.corridor.valid ? num(r.corridor.spreadAt(static_cast<float>(x))) : "null");
    }
    out << "],\"ms\":{\"total\":" << num(total_ms) << ",\"transform\":" << num(r.timings.transform_ms)
        << ",\"calib\":" << num(r.timings.calibration_ms) << ",\"axis\":" << num(r.timings.axis_ms)
        << ",\"corridor\":" << num(r.timings.corridor_ms) << ",\"detect\":" << num(r.timings.detection_ms)
        << ",\"track\":" << num(r.timings.tracking_ms) << "},\"points\":" << r.num_points << ",\"clusters\":[";
    for (std::size_t i = 0; i < r.clusters.size(); ++i) {
      const auto & c = r.clusters[i];
      out << (i ? "," : "") << "{\"x\":" << num(c.x_min) << ",\"lat\":" << num(c.lat_center) << ",\"n\":" << c.num_points
          << ",\"hmin\":" << num(c.h_min) << ",\"hmax\":" << num(c.h_max) << ",\"level\":" << static_cast<int>(c.level)
          << ",\"above\":" << num(c.above_ground) << ",\"len\":" << num(c.x_max - c.x_min)
          << ",\"wid\":" << num(c.lat_max - c.lat_min) << ",\"danger_pts\":" << c.danger_points
          << ",\"cx\":" << num(c.cx) << ",\"cy\":" << num(c.cy) << ",\"cz\":" << num(c.cz) << "}";
    }
    out << "],\"rejected\":[";
    for (std::size_t i = 0; i < r.rejected_clusters.size(); ++i) {
      const auto & c = r.rejected_clusters[i];
      out << (i ? "," : "") << "{\"x\":" << num(c.x_min) << ",\"lat\":" << num(c.lat_center) << ",\"n\":" << c.num_points
          << ",\"reason\":" << static_cast<int>(c.reason) << ",\"level\":" << static_cast<int>(c.level)
          << ",\"above\":" << num(c.above_ground) << ",\"hmin\":" << num(c.h_min) << ",\"hmax\":" << num(c.h_max)
          << ",\"len\":" << num(c.x_max - c.x_min) << ",\"wid\":" << num(c.lat_max - c.lat_min) << "}";
    }
    out << "],\"obstacles\":[";
    for (std::size_t i = 0; i < r.obstacles.size(); ++i) {
      const auto & t = r.obstacles[i];
      out << (i ? "," : "") << "{\"id\":" << t.id << ",\"level\":" << static_cast<int>(t.level)
          << ",\"distance\":" << num(t.distance) << ",\"lateral\":" << num(t.lateral) << ",\"n\":" << t.num_points
          << ",\"hits\":" << t.hits << ",\"conf\":" << num(t.confidence)
          << ",\"speed\":" << (t.has_speed ? num(t.speed) : "null") << ",\"hmax\":" << num(t.h_max)
          << ",\"cx\":" << num(t.cx) << ",\"cy\":" << num(t.cy) << ",\"cz\":" << num(t.cz) << "}";
    }
    out << "]}\n";

    if (std::find(debug_frames.begin(), debug_frames.end(), frame) != debug_frames.end()) {
      std::ofstream dbg(debug_dir + "/debug_" + std::to_string(frame) + ".json");
      const auto & c = r.corridor;
      const Eigen::Matrix3d rot = pipeline.trackToCloudRotation();
      dbg << "{\"frame\":" << frame << ",\"x_res\":" << c.x_res << ",\"height\":" << r.pose.height
          << ",\"track_to_cloud\":[";
      for (int i = 0; i < 9; ++i) {
        dbg << (i ? "," : "") << rot(i / 3, i % 3);
      }
      auto dump = [&dbg](const char * name, const std::vector<float> & v) {
          dbg << ",\"" << name << "\":[";
          for (std::size_t i = 0; i < v.size(); ++i) {
            dbg << (i ? "," : "") << num(v[i]);
          }
          dbg << "]";
        };
      dbg << "]";
      dump("lateral", c.lateral);
      dump("bed", c.bed);
      dump("spread", c.spread);
      dbg << ",\"trusted\":" << num(r.trusted_length) << ",\"axis_detections\":[";
      for (std::size_t i = 0; i < r.axis_detections.size(); ++i) {
        dbg << (i ? "," : "") << "[" << num(r.axis_detections[i].x) << "," << num(r.axis_detections[i].y) << "]";
      }
      dbg << "],\"clusters\":[";
      const auto & cloud = pipeline.trackCloud();
      auto dump_clusters = [&](const std::vector<tod::Cluster> & cls, bool rejected, bool & first) {
          for (const auto & cl : cls) {
            dbg << (first ? "" : ",") << "{\"rejected\":" << (rejected ? "true" : "false") << ",\"level\":"
                << static_cast<int>(cl.level) << ",\"reason\":" << static_cast<int>(cl.reason)
                << ",\"x\":" << num(cl.x_min) << ",\"lat\":" << num(cl.lat_center) << ",\"above_ground\":"
                << num(cl.above_ground) << ",\"structure_points\":" << cl.structure_points << ",\"points\":[";
            first = false;
            for (std::size_t k = 0; k < cl.points.size(); ++k) {
              const auto & tp = cloud[cl.points[k]];
              dbg << (k ? "," : "") << "[" << num(tp.x) << "," << num(tp.y) << "," << num(tp.z) << "]";
            }
            dbg << "]}";
          }
        };
      bool first = true;
      dump_clusters(r.clusters, false, first);
      dump_clusters(r.rejected_clusters, true, first);
      dbg << "]";
      // structure cells that the corridor search steers around
      const auto & est = pipeline.corridorEstimator();
      auto dump_grid = [&](const char * name, const tod::OccupancyGrid & g) {
          dbg << ",\"" << name << "\":[";
          bool first_cell = true;
          for (int row = 0; row < g.rows(); ++row) {
            for (int col = 0; col < g.cols(); ++col) {
              if (g.occupied(row, col)) {
                dbg << (first_cell ? "" : ",") << "[" << num((row + 0.5) * g.rowRes()) << ","
                    << num(-est.params().y_half + (col + 0.5) * est.params().y_res) << "]";
                first_cell = false;
              }
            }
          }
          dbg << "]";
        };
      dump_grid("upper_cells", est.upperGrid());
      dump_grid("lower_cells", est.lowerGrid());
      dbg << "}\n";
    }
  }

  if (!times.empty()) {
    std::sort(times.begin(), times.end());
    double sum = 0.0;
    for (double t : times) {
      sum += t;
    }
    RCLCPP_INFO(node->get_logger(), "frames=%ld mean=%.1f ms p50=%.1f ms p95=%.1f ms max=%.1f ms", processed,
      sum / times.size(), times[times.size() / 2], times[static_cast<std::size_t>(0.95 * (times.size() - 1))],
      times.back());
  }
  rclcpp::shutdown();
  return 0;
}
