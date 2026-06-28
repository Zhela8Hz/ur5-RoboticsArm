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

session_dir="${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${session_dir}"
ln -sfn "${session_dir}" "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/latest"

echo "New hand-eye session: ${session_dir}"
echo "Only captures with at least 21 ChArUco corners will be accepted."

exec python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_capture.py" \
  --image-topic /camera/color/image_raw \
  --intrinsics "${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml" \
  --output-dir "${session_dir}" \
  --samples-file "${session_dir}/samples.jsonl" \
  --base-frame base \
  --tool-frame tool0 \
  --camera-frame camera_color_optical_frame \
  --min-charuco-corners 21 \
  --use-latest-tf
