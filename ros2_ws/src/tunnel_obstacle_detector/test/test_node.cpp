// ROS-level test: the node itself - parameters, subscription, conversion, pipeline, published message.
// The core is covered by the other tests; this one checks that a cloud arriving on a topic becomes a verdict.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tunnel_obstacle_msgs/msg/obstacle_status.hpp>

#include "synthetic_tunnel.hpp"
#include "tunnel_obstacle_detector/ros/detector_node.hpp"

using tunnel_obstacle_msgs::msg::ObstacleStatus;

namespace
{

/// Packs the rendered scan the way the recordings do: x, y, z, intensity (float32) and ring (uint16).
sensor_msgs::msg::PointCloud2 toPointCloud2(const std::vector<tod::RawPoint> & points, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "lidar";
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = 18;
  msg.row_step = msg.point_step * msg.width;
  const char * names[] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = static_cast<uint32_t>(4 * i);
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  sensor_msgs::msg::PointField ring;
  ring.name = "ring";
  ring.offset = 16;
  ring.datatype = sensor_msgs::msg::PointField::UINT16;
  ring.count = 1;
  msg.fields.push_back(ring);

  msg.data.resize(static_cast<std::size_t>(msg.point_step) * points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    uint8_t * p = msg.data.data() + i * msg.point_step;
    const float xyz[4] = {points[i].x, points[i].y, points[i].z, points[i].intensity};
    std::memcpy(p, xyz, sizeof(xyz));
    std::memcpy(p + 16, &points[i].ring, sizeof(uint16_t));
  }
  return msg;
}

/// Feeds `frames` identical scans into the node and returns the verdicts it published.
std::vector<ObstacleStatus> run(const tod_test::TunnelSpec & spec, int frames = 10)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("input_topic", "/test_points"),
    rclcpp::Parameter("publish_markers", false),
    rclcpp::Parameter("sensor.forward_axis", "-y"),
  });
  auto node = std::make_shared<tod_ros::DetectorNode>(options);
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>("/test_points", rclcpp::SensorDataQoS());

  std::vector<ObstacleStatus> received;
  auto sub = helper->create_subscription<ObstacleStatus>(
    "/tunnel_obstacle_detector/status", 10,
    [&received](ObstacleStatus::ConstSharedPtr msg) {received.push_back(*msg);});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  for (int i = 0; i < frames; ++i) {
    stamp = stamp + rclcpp::Duration(0, 100000000);  // 10 Hz
    pub->publish(toPointCloud2(tod_test::renderTunnel(spec, static_cast<unsigned>(i + 1)), stamp));
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    const std::size_t before = received.size();
    while (received.size() == before && std::chrono::steady_clock::now() < until) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  return received;
}

}  // namespace

TEST(Node, EmptyTunnelStaysClear)
{
  tod_test::TunnelSpec spec;
  const auto verdicts = run(spec, 6);
  ASSERT_GE(verdicts.size(), 5u) << "the node published no verdicts";
  for (const auto & v : verdicts) {
    EXPECT_EQ(v.level, ObstacleStatus::LEVEL_CLEAR);
    EXPECT_FALSE(v.obstacle_detected);
    EXPECT_GT(v.free_distance, 50.0F) << "the corridor should be seen far ahead in an empty tunnel";
  }
}

TEST(Node, PersonOnTheTrackIsPublishedAsDanger)
{
  tod_test::TunnelSpec spec;
  spec.boxes.push_back({50.0, 0.4, 0.0, 0.5, 1.75});  // person-sized box on the axis at 50 m
  const auto verdicts = run(spec, 10);
  ASSERT_FALSE(verdicts.empty()) << "the node published no verdicts";
  const auto danger = std::find_if(
    verdicts.begin(), verdicts.end(),
    [](const ObstacleStatus & v) {return v.level == ObstacleStatus::LEVEL_DANGER;});
  ASSERT_NE(danger, verdicts.end()) << "no DANGER verdict for a person standing on the track";
  EXPECT_TRUE(danger->obstacle_detected);
  EXPECT_NEAR(danger->nearest_distance, 50.0F, 2.0F);
  EXPECT_FALSE(danger->obstacles.empty());
  EXPECT_LT(std::abs(danger->obstacles.front().lateral), 0.5F);
  EXPECT_GT(danger->processing_time_ms, 0.0F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
