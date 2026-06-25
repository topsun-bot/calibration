#!/usr/bin/env bash
# 眼在手外专项：自动化测试 + 演示视频录制
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "==> [1/2] 运行眼在手外自动化测试..."
"$ROOT/scripts/run_sim_tests.sh" tests/test_eye_to_hand.py tests/test_offline_calib.py tests/test_mock_calib.py tests/test_sim_pipeline.py

echo ""
echo "==> [2/2] 录制完整演示视频..."
PY="${PYTHON:-/home/gy/anaconda3/envs/yolo/bin/python}"
if [[ ! -x "$PY" ]]; then PY="python3"; fi
export MUJOCO_GL="${MUJOCO_GL:-egl}"
"$PY" "$ROOT/scripts/record_eye_to_hand_demo.py" -o "$ROOT/output/eye_to_hand_demo.mp4"

echo ""
echo "==> 完成"
echo "    测试: 全部通过"
echo "    视频: $ROOT/output/eye_to_hand_demo.mp4"
