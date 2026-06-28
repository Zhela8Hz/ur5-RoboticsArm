#!/usr/bin/env bash
set -eo pipefail

exec python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_validate_samples.py \
  --samples /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl \
  --result /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml \
  --output /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/base_target_validation.csv
