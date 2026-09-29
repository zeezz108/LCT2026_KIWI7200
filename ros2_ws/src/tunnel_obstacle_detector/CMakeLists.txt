cmake_minimum_required(VERSION 3.8)
project(tunnel_obstacle_detector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tunnel_obstacle_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# ---------------------------------------------------------------- core algorithm (no ROS dependencies)
add_library(tod_core STATIC
  src/core/geometry.cpp
  src/core/ground_calibration.cpp
  src/core/track_axis.cpp
  src/core/occupancy_grid.cpp
  src/core/corridor_estimator.cpp
  src/core/track_advance.cpp
  src/core/obstacle_detector.cpp
  src/core/tracker.cpp
  src/core/pipeline.cpp
)
target_include_directories(tod_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tod_core PUBLIC Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(tod_core PUBLIC OpenMP::OpenMP_CXX)
endif()
set_target_properties(tod_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------- ROS glue
add_library(tod_ros STATIC
  src/ros/cloud_conversion.cpp
  src/ros/parameters.cpp
  src/ros/message_builder.cpp
)
target_link_libraries(tod_ros tod_core)
ament_target_dependencies(tod_ros rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs vision_msgs tunnel_obstacle_msgs)

add_executable(detector_node src/ros/detector_node.cpp)
target_link_libraries(detector_node tod_ros)
ament_target_dependencies(detector_node rclcpp tf2_ros)

add_executable(offline_runner src/ros/offline_runner.cpp)
target_link_libraries(offline_runner tod_ros)
ament_target_dependencies(offline_runner rclcpp rosbag2_cpp)

install(TARGETS detector_node offline_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------- tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core
    test/synthetic_tunnel.cpp
    test/test_ground_calibration.cpp
    test/test_track_axis.cpp
    test/test_corridor.cpp
    test/test_track_advance.cpp
    test/test_detector.cpp
    test/test_tracker.cpp
    test/test_pipeline.cpp
  )
  target_link_libraries(test_core tod_core)

  # ROS-level test: the node, its parameters, topics and published messages
  ament_add_gtest(test_node test/synthetic_tunnel.cpp test/test_node.cpp)
  target_link_libraries(test_node tod_ros)
  ament_target_dependencies(test_node rclcpp sensor_msgs tunnel_obstacle_msgs vision_msgs visualization_msgs tf2_ros)
endif()

ament_package()
