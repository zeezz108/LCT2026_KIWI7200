#!/usr/bin/env bash
# Dump C++ debug data for chosen frames of one bag and copy them to the Windows project for plotting.
# Usage (WSL): bash scripts/debug_frames.sh <bag_name> <frame> [frame ...] [-- -p name:=value ...]
set -eo pipefail
BAG="$1"; shift
FRAME_LIST=()
while [[ $# -gt 0 && "$1" != "--" ]]; do FRAME_LIST+=("$1"); shift; done
[[ "${1:-}" == "--" ]] && shift
FRAMES="[$(IFS=,; echo "${FRAME_LIST[*]}")]"
source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
BAG_PATH="${HOME}/data/for_hackathon/${BAG}"
if [[ "${BAG}" == /* ]]; then  # absolute path, e.g. a synthetic bag
  BAG_PATH="${BAG}"
  BAG="$(basename "${BAG}")"
fi
OUT="${HOME}/results/debug/${BAG}"
mkdir -p "${OUT}"
MAX=$(( $(printf '%s\n' "${FRAME_LIST[@]}" | sort -n | tail -1) + 1 ))
ros2 run tunnel_obstacle_detector offline_runner --ros-args \
  --params-file "${HOME}/lct_ws/install/tunnel_obstacle_detector/share/tunnel_obstacle_detector/config/detector.yaml" \
  -p bag:="${BAG_PATH}" -p output:="${OUT}/run.jsonl" \
  -p debug_frames:="${FRAMES}" -p debug_dir:="${OUT}" -p max_frames:=${MAX} "$@" 2>&1 | grep -E "frames=|ERROR" || true
WIN="/mnt/d/Python Projects/ЛЦТ 2026/data/results/debug/${BAG}"
mkdir -p "${WIN}"
cp "${OUT}"/debug_*.json "${WIN}/"
