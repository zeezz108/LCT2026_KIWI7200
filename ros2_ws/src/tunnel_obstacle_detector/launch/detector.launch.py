"""Obstacle detector, optionally with RViz2 and bag playback.

    ros2 launch tunnel_obstacle_detector detector.launch.py
    ros2 launch tunnel_obstacle_detector detector.launch.py rviz:=true bag:=/data/doubleT_obstacle loop:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("tunnel_obstacle_detector")
    bag = LaunchConfiguration("bag")
    has_bag = PythonExpression(["'", bag, "' != ''"])
    loop = LaunchConfiguration("loop")

    return LaunchDescription([
        DeclareLaunchArgument("omp_threads", default_value="4"),
        # OpenMP threads must not spin-wait: bag playback, DDS and RViz share the CPU with the detector
        SetEnvironmentVariable("OMP_WAIT_POLICY", "PASSIVE"),
        SetEnvironmentVariable("OMP_NUM_THREADS", LaunchConfiguration("omp_threads")),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "detector.yaml"])),
        DeclareLaunchArgument("input_topic", default_value="", description="empty: first PointCloud2 topic found"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value="detector.rviz", description="file in the package rviz/ dir"),
        DeclareLaunchArgument("bag", default_value="", description="path of a ROS 2 bag to play"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("csv_log_path", default_value="", description="per-frame CSV log"),
        DeclareLaunchArgument("summary_path", default_value="",
                              description="JSON summary of the run: frames, levels, every reported object"),

        Node(
            package="tunnel_obstacle_detector",
            executable="detector_node",
            name="tunnel_obstacle_detector",
            output="screen",
            parameters=[LaunchConfiguration("params_file"),
                        {"input_topic": LaunchConfiguration("input_topic"),
                         "csv_log_path": LaunchConfiguration("csv_log_path"),
                         "summary_path": LaunchConfiguration("summary_path")}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", LaunchConfiguration("rviz_config")])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        # small read-ahead queue: the default 1000 messages of 24 MB 360-degree clouds stall playback
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--rate", LaunchConfiguration("rate"), "--loop", "--read-ahead-queue-size", "20"],
            output="screen",
            condition=IfCondition(PythonExpression([has_bag, " and '", loop, "' == 'true'"])),
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--rate", LaunchConfiguration("rate"), "--read-ahead-queue-size", "20"],
            output="screen",
            condition=IfCondition(PythonExpression([has_bag, " and '", loop, "' != 'true'"])),
        ),
    ])
