#!/usr/bin/env bash
# Dev build inside WSL: sync sources from the Windows project (path has spaces/Cyrillic) to ~/lct_ws and colcon build.
# Usage (from WSL):  bash "/mnt/d/Python Projects/ЛЦТ 2026/scripts/dev_build.sh" [--test] [extra colcon args]
set -eo pipefail

PROJECT="/mnt/d/Python Projects/ЛЦТ 2026"
WS="${HOME}/lct_ws"

RUN_TESTS=0
if [[ "${1:-}" == "--test" ]]; then RUN_TESTS=1; shift; fi

mkdir -p "${WS}/src"
rsync -a --delete "${PROJECT}/ros2_ws/src/" "${WS}/src/"

source /opt/ros/humble/setup.bash
cd "${WS}"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@" 2>&1 | tail -40

if [[ ${RUN_TESTS} -eq 1 ]]; then
  source install/setup.bash
  colcon test --packages-select tunnel_obstacle_detector --event-handlers console_direct+ 2>&1 | tail -60
  colcon test-result --verbose | tail -30
fi
