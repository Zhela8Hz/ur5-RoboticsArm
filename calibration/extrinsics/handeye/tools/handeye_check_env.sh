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

echo "ROS distro: ${ROS_DISTRO:-unknown}"
echo "Checking Python dependencies..."
python3 - <<'PY'
import cv2

modules = ['rclpy', 'tf2_ros', 'cv_bridge', 'sensor_msgs', 'geometry_msgs', 'yaml']
for module in modules:
    __import__(module)
    print(f'{module}: OK')

print(f'OpenCV: {cv2.__version__}')
assert hasattr(cv2, 'aruco'), 'cv2.aruco missing'
assert hasattr(cv2.aruco, 'estimatePoseCharucoBoard'), 'estimatePoseCharucoBoard missing'
assert hasattr(cv2, 'calibrateHandEye'), 'calibrateHandEye missing'
print('OpenCV hand-eye support: OK')
PY

echo "Checking key files..."
test -f "${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml"
test -x "${PROJECT_ROOT}/scripts/camera/start_dabai_camera.sh"
echo "Environment looks ready."
