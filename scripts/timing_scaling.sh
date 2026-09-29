#!/usr/bin/env bash
# Frame time against the number of OpenMP threads, for the table in docs/experiments.md § 6.
# Two clouds: the 360 deg recording (921 600 points per frame) and a +-50 deg one (307 200).
#
# Usage (from WSL):  bash "/mnt/d/Python Projects/ЛЦТ 2026/scripts/timing_scaling.sh" [threads...]
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/lct_ws/install/setup.bash"
CFG="${HOME}/lct_ws/install/tunnel_obstacle_detector/share/tunnel_obstacle_detector/config/detector.yaml"
OUT="/tmp/timing_scaling"
mkdir -p "${OUT}"
export OMP_WAIT_POLICY=PASSIVE

THREADS=${*:-"1 2 4 6"}
for bag in doubleT_obstacle roundT_pressureGate_roundT; do
  for n in ${THREADS}; do
    line=$(OMP_NUM_THREADS="${n}" ros2 run tunnel_obstacle_detector offline_runner --ros-args \
      --params-file "${CFG}" -p bag:="${HOME}/data/for_hackathon/${bag}" \
      -p output:="${OUT}/${bag}_${n}.jsonl" 2>&1 | grep -o 'frames=.*' | tail -1)
    echo "${bag} threads=${n} ${line}"
  done
done
