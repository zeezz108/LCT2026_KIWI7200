#!/usr/bin/env bash
# Native ROS 2 Humble in the WSL distro for fast development iterations (run as root).
# The deliverable itself is the Docker image; this is only the dev environment.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

add-apt-repository -y universe

# Official apt source package (replaces the old manual ros.key setup)
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
apt-get install -y /tmp/ros2-apt-source.deb

apt-get update
apt-get install -y \
  ros-humble-desktop ros-dev-tools \
  ros-humble-vision-msgs ros-humble-rosbag2-storage-default-plugins \
  libeigen3-dev python3-numpy python3-pip

grep -q '/opt/ros/humble/setup.bash' /home/dev/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> /home/dev/.bashrc

bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg list | wc -l | xargs echo "ros2 packages:"'
echo "03_ros2_humble: OK"
