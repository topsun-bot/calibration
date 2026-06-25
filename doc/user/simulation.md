# 仿真与自动化测试

无真机时可用 **MuJoCo 仿真** 或 **mock 模式** 验证完整流程。

## 快速命令

```bash
# 全部自动化测试
./scripts/run_sim_tests.sh

# 眼在手外专项：测试 + 录制演示视频
./scripts/run_eye_to_hand_demo.sh

# 仅录制视频 → output/eye_to_hand_demo.mp4
python scripts/record_eye_to_hand_demo.py

# MuJoCo 仿真标定
python pick_place/run.py calibrate --sim

# MuJoCo 一步完成（标定 + 抓取）
python pick_place/run.py all --class bottle --sim --no-show

# 无 MuJoCo 时 mock 臂
python pick_place/run.py pick --class bottle --mock-arm

# Docker 内测试
docker run --rm calibration -m pytest tests/ -q
docker compose run --rm test
```

## 仿真标定精度

仿真模式使用解析 T_t2c（无渲染→检测误差），达到理论最优精度：

| 指标 | 结果 |
|------|------|
| 平移误差 | 0.0000 mm |
| 旋转误差 | 0.0000° |
| 质量评级 | Excellent |
| 所有 5 种算法 | 0.0000 mm / 0.000° |

## 测试覆盖

| 测试文件 | 覆盖内容 | 需要 |
|----------|----------|------|
| `tests/test_sim_pipeline.py` | MuJoCo 标定 + GT 对比 + 运动多样性 + 抓取放置 | mujoco |
| `tests/test_mock_calib.py` | Mock 臂+相机端到端标定 | - |
| `tests/test_eye_to_hand.py` | T_cam_base 往返、cam→base、`all --sim` 集成 | mujoco, ultralytics |
| `tests/test_offline_calib.py` | 离线合成数据标定 | - |
| `tests/test_intrinsics.py` | 内参 JSON / 动态读取逻辑 | - |
| `tests/test_ik.py` | 数值 IK | - |
| `tests/test_calib_camera.py` | 相机工厂 + USB/RealSense 配置 | pyrealsense2 (部分) |

```bash
./scripts/run_sim_tests.sh                              # 全部
./scripts/run_sim_tests.sh tests/test_sim_pipeline.py   # 指定测试
python -m pytest tests/ -k "not realsense" -q           # 跳过需硬件的测试
```

## 仿真标定 Python API

```python
from sim.mujoco_env import create_sim_env
from sim.sim_calibration import run_sim_calibration

env = create_sim_env(headless=True)
result = run_sim_calibration(env, num_poses=12)

print(f"平移误差: {result.translation_error_mm:.4f} mm")
print(f"旋转误差: {result.rotation_error_deg:.4f}°")
print(f"准确度:   {result.quality_label}")

# 各算法 GT 误差
for alg in result.per_algorithm:
    print(f"  {alg.name}: {alg.translation_error_mm:.4f}mm, {alg.rotation_error_deg:.4f}°")

# 输出文件: result.report_json, result.report_txt, result.report_png
```

## 可视化 API

```python
from common.calib_visualizer import (
    plot_coordinate_frames,      # 3D 坐标系场景（base/camera/gripper/target）
    plot_reprojection_errors,    # 重投影误差分布
    plot_sim_gt_comparison,      # 仿真 GT 对比柱状图
)

# 坐标系可视化
plot_coordinate_frames(result.T_est, save_path="output/frames.png")

# GT 对比
plot_sim_gt_comparison(result, save_path="output/gt_compare.png")
```

## 仿真说明

- 标定数据由解析模型直接计算 `T_t2c = T_base2cam @ T_g2b`（几何完全一致）
- 抓取检测在 `--sim` 下使用 MuJoCo 物体 ground truth
- MuJoCo 驱动臂运动与场景渲染
- `MUJOCO_GL=egl` 为 headless 渲染后端

## Docker 仿真

```bash
# 构建
docker build -t calibration .

# 仿真标定
docker run --rm calibration pick_place/run.py calibrate --sim --no-show

# 完整测试
docker run --rm calibration -m pytest tests/ -q
# 预期结果: 22 passed, 1 failed (需 RealSense 硬件), 1 skipped
```

## 脚本说明

| 脚本 | 作用 |
|------|------|
| `scripts/run_sim_tests.sh` | 安装依赖、下载 URDF、运行 pytest |
| `scripts/run_eye_to_hand_demo.sh` | 眼在手外测试 + 录制演示视频 |
| `scripts/record_eye_to_hand_demo.py` | 录制标定→报告→抓取全流程 MP4 |
| `scripts/check_cameras.py` | 检测 USB/RealSense 摄像头状态 |

更多脚本细节见 [scripts/README.md](../../scripts/README.md)。
