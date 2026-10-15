#!/usr/bin/env bash
# Reproduce the reBot B601-RS + C1 + TacCap G1 example from its static data (no hardware needed).
# Run from the repository root:  bash examples/rebot_b601_rs_taccap_v2/reproduce.sh
# Results go to examples/rebot_b601_rs_taccap_v2/output/ (not tracked by git).
set -euo pipefail
cd "$(dirname "$0")/../.."

EXAMPLE=examples/rebot_b601_rs_taccap_v2
OUT=$EXAMPLE/output
ARM=$EXAMPLE/data/arm
GRIPPER=$EXAMPLE/data/gripper
GAIN="1 1.032 1.08 1.004 1 1"            # torque calibration measured on this arm (feedback / commanded)
CLI="rebot_taccap_identification"
ROBOT=(--robot.type=rebot_b601_rs --robot.results_dir="$OUT")

# 1. Arm (link1-link5) from static data taken without the C1 adapter and gripper.
$CLI identify "${ROBOT[@]}" --torque-gain $GAIN --version v2 \
  --static "$ARM/drift_20261013_114441_identified_v1.csv@5" "$ARM/train_rand50_s0.csv" \
  --val "$ARM/drift_20261013_115149_identified_val_v1.csv" "$ARM/val_rand20_s100.csv"

# 2. Payload (C1 + TacCap G1) on the identified arm, from static data taken with it mounted.
$CLI payload "${ROBOT[@]}" --torque-gain $GAIN --name gripper \
  --urdf "$OUT/models/v2/rebot_b601_rs_identified_v2.urdf" \
  --out-urdf "$OUT/gripper/models/v2/rebot_b601_rs_gripper_identified_v2.urdf" \
  --static "$GRIPPER"/drift_20261013_15{2111,3021,3506,4032,5442,5847}_gripper_identified_v2.csv \
           "$GRIPPER"/train_rand40_s20{0,1,2,3}.csv \
  --val "$GRIPPER/val_rand20_s300.csv" "$GRIPPER/val_rand20_s301.csv"

# 3. Complete, self-contained URDF packages (urdf/ + meshes/), with and without the gripper.
$CLI package "${ROBOT[@]}" --model "$OUT/gripper/models/v2/rebot_b601_rs_gripper_identified_v2.urdf"
$CLI package "${ROBOT[@]}" --model "$OUT/models/v2/rebot_b601_rs_identified_v2.urdf"
