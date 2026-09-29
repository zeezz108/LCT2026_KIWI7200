#!/usr/bin/env bash
# Run all bags with extra parameter overrides into a named result directory (for A/B experiments).
# Usage (WSL): bash scripts/run_offline_variant.sh <variant_name> [-p name:=value ...]
set -eo pipefail
VARIANT="$1"; shift
source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
PARAMS="${HOME}/lct_ws/install/tunnel_obstacle_detector/share/tunnel_obstacle_detector/config/detector.yaml"
OUT="${HOME}/results/${VARIANT}"
mkdir -p "${OUT}"
for bag in "${HOME}"/data/for_hackathon/*/; do
  name="$(basename "${bag}")"
  ros2 run tunnel_obstacle_detector offline_runner --ros-args --params-file "${PARAMS}" \
    -p bag:="${bag%/}" -p output:="${OUT}/${name}.jsonl" "$@" 2>&1 | grep -E "frames=|ERROR|rror" || true
done
WIN="/mnt/d/Python Projects/ЛЦТ 2026/data/results/${VARIANT}"
mkdir -p "${WIN}"
cp "${OUT}"/*.jsonl "${WIN}/"
