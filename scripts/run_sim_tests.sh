#!/usr/bin/env bash
# MuJoCo 仿真 + 离线/mock 自动化测试
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

PY="${PYTHON:-}"
if [[ -z "$PY" ]]; then
  if [[ -x "/home/gy/anaconda3/envs/yolo/bin/python" ]]; then
    PY="/home/gy/anaconda3/envs/yolo/bin/python"
  else
    PY="python3"
  fi
fi

echo "==> Python: $PY"
echo "==> 安装/更新依赖..."
"$PY" -m pip install -q -r get_object/requirements.txt -r requirements-dev.txt

echo "==> 确保 D1 URDF 存在..."
"$PY" calibate/tools/get_d1_urdf.py 2>/dev/null || true

export MUJOCO_GL="${MUJOCO_GL:-egl}"
echo "==> MUJOCO_GL=$MUJOCO_GL"
echo "==> 运行 pytest..."
"$PY" -m pytest tests/ -v --tb=short "$@"

echo "==> 全部测试通过"
