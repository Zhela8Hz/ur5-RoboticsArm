#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
exec ros2 run charuco_camera_calibration charuco_intrinsics --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p squares_x:=6 -p squares_y:=6 \
  -p square_length_m:=0.025 \
  -p marker_length_m:=0.018 \
  -p dictionary_id:=DICT_6X6_1000 \
  -p start_id:=233 \
  -p output_file:="${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml"
