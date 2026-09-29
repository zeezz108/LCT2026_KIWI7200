#include "tunnel_obstacle_detector/ros/detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tod_ros::DetectorNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
