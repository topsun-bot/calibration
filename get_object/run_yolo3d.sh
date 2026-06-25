#!/bin/bash
# YOLO 3D 目标检测启动脚本
cd "$(dirname "$0")"
conda run -n yolo python yolo3d/run.py "$@"
