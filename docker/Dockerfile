# Metro tunnel obstacle detector — ROS 2 Humble, Ubuntu 22.04
#
#   docker build -t tunnel-obstacle-detector -f docker/Dockerfile .
#   docker run --rm -it --net=host --ipc=host tunnel-obstacle-detector
#   ros2 bag play <bag>            # on the host or in another container

ARG ROS_DISTRO=humble

# ------------------------------------------------------------------ build + unit tests
FROM ros:${ROS_DISTRO}-ros-base-jammy AS build
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-vision-msgs \
      ros-humble-tf2-ros \
      ros-humble-rosbag2-storage-default-plugins \
      libeigen3-dev \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ros2_ws/src ./src
# the node test publishes and subscribes: keep DDS inside the container
ENV ROS_LOCALHOST_ONLY=1
RUN . /opt/ros/humble/setup.sh \
 && colcon build --event-handlers console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
 && colcon test --packages-select tunnel_obstacle_detector \
 && colcon test-result --verbose

# ------------------------------------------------------------------ runtime
FROM ros:${ROS_DISTRO}-ros-base-jammy
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-vision-msgs \
      ros-humble-tf2-ros \
      ros-humble-rosbag2-storage-default-plugins \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-rviz2 \
      libgomp1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data /results && chmod 777 /results \
 && (id -u 1000 >/dev/null 2>&1 || useradd --create-home --uid 1000 --shell /bin/bash ros)

# OpenMP threads must not spin-wait: bag playback, DDS and RViz2 share the CPU with the detector
ENV OMP_NUM_THREADS=4 \
    OMP_WAIT_POLICY=PASSIVE \
    ROS_LOG_DIR=/tmp/ros_log
# Fast DDS passes clouds through shared memory (/dev/shm) that only processes of the same user can open: a detector
# running as root would discover a `ros2 bag play` of a normal host user but receive no data. UID 1000 is the usual
# desktop user; for another host user run with `--user $(id -u):$(id -g)`, for a root player with `--user root`.
USER 1000:1000
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "tunnel_obstacle_detector", "detector.launch.py"]
