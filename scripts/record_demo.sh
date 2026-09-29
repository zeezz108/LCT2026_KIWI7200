#!/usr/bin/env bash
# Record a live demo video: detector + RViz2 + bag playback, RViz window captured with ffmpeg (X11).
# Usage (WSL / Linux desktop): bash scripts/record_demo.sh <bag_dir> <out.mp4> [seconds] [start_offset_s] [rate]
set -eo pipefail
BAG="$1"; OUT="$2"; SECONDS_TO_RECORD="${3:-30}"; OFFSET="${4:-0}"; RATE="${5:-1.0}"
source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
export OMP_WAIT_POLICY=PASSIVE OMP_NUM_THREADS=4

ros2 launch tunnel_obstacle_detector detector.launch.py rviz:=true > /tmp/record_launch.log 2>&1 &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} 2>/dev/null || true; pkill -f rviz2 || true; pkill -f "ros2 bag play" || true' EXIT

# wait for the RViz window
WIN=""
for _ in $(seq 1 60); do
  WIN=$(xwininfo -root -tree 2>/dev/null | grep -i "rviz" | grep -oE "0x[0-9a-f]+" | head -1 || true)
  [[ -n "${WIN}" ]] && break
  sleep 1
done
[[ -z "${WIN}" ]] && { echo "RViz window not found"; exit 1; }
sleep 4

ros2 bag play "${BAG}" --start-offset "${OFFSET}" --rate "${RATE}" > /tmp/record_play.log 2>&1 &
sleep 1
ffmpeg -y -loglevel error -f x11grab -framerate 15 -window_id "${WIN}" -i "${DISPLAY}" -t "${SECONDS_TO_RECORD}" \
  -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -preset veryfast -pix_fmt yuv420p "${OUT}"
echo "saved ${OUT}"
grep -E "fps" /tmp/record_launch.log | tail -3 | sed 's/.*tunnel_obstacle_detector\]: //'
