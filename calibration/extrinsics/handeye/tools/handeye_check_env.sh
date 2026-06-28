#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/z/Apps-my/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
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
test -f /home/z/Apps-my/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml
test -x /home/z/Apps-my/scripts/camera/start_dabai_camera.sh
echo "Environment looks ready."
