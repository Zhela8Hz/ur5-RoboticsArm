#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

EXPOSURE="${1:-3000}"

cd "${PROJECT_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

if ros2 service list | grep -qx '/camera/set_ir_exposure'; then
  ros2 service call /camera/set_ir_exposure orbbec_camera_msgs/srv/SetInt32 "{data: ${EXPOSURE}}"
else
  echo "Service /camera/set_ir_exposure is not available."
  echo "Restart the IR camera with: IR_AUTO_EXPOSURE=false IR_EXPOSURE=${EXPOSURE} ./scripts/camera/start_dabai_ir_camera.sh"
  exit 1
fi
