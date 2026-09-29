#!/usr/bin/env bash
# Run the offline runner over every bag in a directory and collect JSONL results.
# Usage (WSL):  bash scripts/run_offline_all.sh [bag_dir] [result_dir] [extra ros args...]
set -eo pipefail

BAG_DIR="${1:-${HOME}/data/for_hackathon}"
OUT_DIR="${2:-${HOME}/results}"
shift 2 || true

source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
PARAMS="${HOME}/lct_ws/install/tunnel_obstacle_detector/share/tunnel_obstacle_detector/config/detector.yaml"
mkdir -p "${OUT_DIR}"

for bag in "${BAG_DIR}"/*/; do
  name="$(basename "${bag}")"
  echo "=== ${name}"
  ros2 run tunnel_obstacle_detector offline_runner --ros-args --params-file "${PARAMS}" \
    -p bag:="${bag%/}" -p output:="${OUT_DIR}/${name}.jsonl" "$@" 2>&1 | grep -E "frames=|ERROR" || true
done

WIN_RESULTS="/mnt/d/Python Projects/ЛЦТ 2026/data/results"
mkdir -p "${WIN_RESULTS}"
cp "${OUT_DIR}"/*.jsonl "${WIN_RESULTS}/"
