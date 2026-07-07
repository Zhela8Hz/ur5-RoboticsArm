#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

exec python3 "${PROJECT_ROOT}/ros2_ws/src/charuco_camera_calibration/charuco_camera_calibration/charuco_intrinsics.py" --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p squares_x:=6 -p squares_y:=6 \
  -p square_length_m:=0.025 \
  -p marker_length_m:=0.018 \
  -p dictionary_id:=DICT_6X6_1000 \
  -p start_id:=233 \
  -p camera_name:=gemini335_color \
  -p min_charuco_corners:=20 \
  -p capture_dir:=/tmp/gemini335_rgb_intrinsics_captures \
  -p output_file:="${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080.yaml"
