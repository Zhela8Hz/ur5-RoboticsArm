#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
fi

cd "${PROJECT_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

exec python3 "${PROJECT_ROOT}/calibration/rgb_intrinsics/tools/interactive_charuco_intrinsics.py" \
  --image-topic /camera/color/image_raw \
  --squares-x 6 \
  --squares-y 6 \
  --square-length-m 0.025 \
  --marker-length-m 0.018 \
  --dictionary-id DICT_6X6_1000 \
  --start-id 233 \
  --camera-name gemini335_color \
  --min-charuco-corners 20 \
  --target-samples 20 \
  --min-samples 12 \
  --capture-dir /tmp/gemini335_rgb_intrinsics_interactive \
  --output-file "${PROJECT_ROOT}/calibration/rgb_intrinsics/results/rgb_intrinsics_gemini335_1920x1080_interactive.yaml"
