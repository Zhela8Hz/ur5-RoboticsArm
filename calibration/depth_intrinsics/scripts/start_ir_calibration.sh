#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
if [ -d "${PROJECT_ROOT}/ros2_ws/install_charuco/charuco_camera_calibration" ]; then
  CHARUCO_PREFIX="${PROJECT_ROOT}/ros2_ws/install_charuco/charuco_camera_calibration"
  export AMENT_PREFIX_PATH="${CHARUCO_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export PATH="${CHARUCO_PREFIX}/lib/charuco_camera_calibration:${PATH}"
  export PYTHONPATH="${CHARUCO_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi

IR_WIDTH="${IR_WIDTH:-1024}"
IR_HEIGHT="${IR_HEIGHT:-768}"

exec ros2 run charuco_camera_calibration charuco_intrinsics --ros-args \
  -p image_topic:=/camera/ir/image_raw \
  -p squares_x:=6 -p squares_y:=6 \
  -p square_length_m:=0.040 \
  -p marker_length_m:=0.030 \
  -p dictionary_id:=DICT_6X6_1000 \
  -p start_id:=233 \
  -p use_clahe:=true \
  -p tuned_detector:=true \
  -p capture_dir:="${PROJECT_ROOT}/calibration/depth_intrinsics/captures" \
  -p output_file:="${PROJECT_ROOT}/calibration/depth_intrinsics/results/ir_intrinsics_${IR_WIDTH}x${IR_HEIGHT}.yaml"
