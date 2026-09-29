#!/usr/bin/env bash
# Record a demo video without touching the desktop: detector + RViz2 on a virtual X server (Xvfb, software OpenGL),
# the bag played at real time, the virtual screen captured with ffmpeg.
# Usage (WSL / Linux): bash scripts/record_demo_headless.sh <bag_dir> <out.mp4> [bag_seconds] [start_offset_s] [rate] [rviz_config]
#   bag_seconds: how much of the bag to show. With rate < 1 (360° clouds of 24 MB are read slower than real time from
#   disk on a desktop) the capture is sped up back to real time, so the video always runs at the recorded pace.
set -eo pipefail
# ffmpeg reads stdin: without this it swallows the rest of a script piped into bash
exec < /dev/null
BAG="$1"; OUT="$2"; BAG_SECONDS="${3:-20}"; OFFSET="${4:-0}"; RATE="${5:-1.0}"; RVIZ_CONFIG="${6:-demo.rviz}"
W=1920; H=1080; DISP=":99"; LEAD=3
CAPTURE_SECONDS=$(python3 -c "print(round(${BAG_SECONDS} / ${RATE}, 2))")
SPEEDUP=$(python3 -c "print(1.0 / ${RATE})")

source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
export OMP_WAIT_POLICY=PASSIVE OMP_NUM_THREADS=4

cleanup() {
  pkill -f '[r]os2 bag play' || true
  pkill -f '[r]viz2' || true
  pkill -f '[d]etector_node' || true
  pkill -f '[r]os2 launch' || true
  [[ -n "${XVFB_PID:-}" ]] && kill "${XVFB_PID}" 2>/dev/null || true
}
trap cleanup EXIT

Xvfb "${DISP}" -screen 0 "${W}x${H}x24" -nolisten tcp > /tmp/xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2
# software OpenGL gets its own share of cores so that the player and the detector are not starved
export DISPLAY="${DISP}" QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=4
unset WAYLAND_DISPLAY

ros2 launch tunnel_obstacle_detector detector.launch.py rviz:=true rviz_config:="${RVIZ_CONFIG}" \
  > /tmp/headless_launch.log 2>&1 &
WIN=""
for _ in $(seq 1 60); do
  WIN=$(xdotool search --name "RViz" 2>/dev/null | head -1 || true)
  [[ -n "${WIN}" ]] && break
  sleep 1
done
[[ -z "${WIN}" ]] && { echo "RViz window not found"; tail -20 /tmp/headless_launch.log; exit 1; }
xdotool windowmove "${WIN}" 0 0 || true
sleep 5

# read the bag once so that playback is not throttled by the disk
cat "${BAG}"/*.db3 "${BAG}"/*.mcap > /dev/null 2>&1 || true
# the detector needs a moment to discover the topic: the player starts paused and is resumed when the capture runs.
# The default read-ahead queue (1000 messages) would pull a whole 5 GB 360° bag into RAM and stall playback.
ros2 bag play "${BAG}" --start-offset "${OFFSET}" --rate "${RATE}" --read-ahead-queue-size 20 --start-paused \
  > /tmp/headless_play.log 2>&1 &
sleep "${LEAD}"
RAW="${OUT%.mp4}_raw.mp4"
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size "${W}x${H}" -i "${DISP}" -t "${CAPTURE_SECONDS}" \
  -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p "${RAW}" &
FFMPEG_PID=$!
sleep 0.5
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > /dev/null
wait "${FFMPEG_PID}"
ffmpeg -y -loglevel error -i "${RAW}" -filter:v "setpts=PTS/${SPEEDUP}" -r 15 \
  -c:v libx264 -preset slow -crf 22 -pix_fmt yuv420p "${OUT}"
rm -f "${RAW}"
echo "saved ${OUT}"
grep -E "fps" /tmp/headless_launch.log | tail -3 | sed 's/.*tunnel_obstacle_detector\]: //'
