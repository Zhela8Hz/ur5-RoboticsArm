#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

exec python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_live_validate.py" \
  --image-topic /camera/color/image_raw \
  --intrinsics "${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml" \
  --handeye "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/latest/handeye_result.yaml" \
  --base-frame base \
  --tool-frame tool0
