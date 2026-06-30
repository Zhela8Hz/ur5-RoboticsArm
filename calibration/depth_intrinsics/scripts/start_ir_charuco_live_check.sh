#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

exec python3 "${PROJECT_ROOT}/calibration/rgb_intrinsics/tools/charuco_live_check.py" \
  --image-topic /camera/ir/image_raw \
  --intrinsics "" \
  --output-dir "${PROJECT_ROOT}/calibration/depth_intrinsics/live_check" \
  --squares-x 6 --squares-y 6 \
  --square-length-m 0.040 \
  --marker-length-m 0.030 \
  --dictionary-id DICT_6X6_1000 \
  --start-id 233 \
  --min-charuco-corners 20 \
  --clahe \
  "$@"
