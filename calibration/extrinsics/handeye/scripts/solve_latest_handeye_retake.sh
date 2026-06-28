#!/usr/bin/env bash
set -eo pipefail

session_dir="/home/z/Apps-my/calibration/extrinsics/handeye/sessions/latest"
test -f "${session_dir}/samples.jsonl"

python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_solve.py \
  --samples "${session_dir}/samples.jsonl" \
  --output "${session_dir}/handeye_result.yaml" \
  --method tsai

python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_validate_samples.py \
  --samples "${session_dir}/samples.jsonl" \
  --result "${session_dir}/handeye_result.yaml" \
  --output "${session_dir}/base_target_validation.csv"

echo
echo "Result: ${session_dir}/handeye_result.yaml"
echo "Validation CSV: ${session_dir}/base_target_validation.csv"
