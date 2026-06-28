#!/usr/bin/env bash
set -eo pipefail

source /home/z/ros2_ws/setup_charuco_calibration.bash
ros2 service call /capture std_srvs/srv/Trigger '{}'
