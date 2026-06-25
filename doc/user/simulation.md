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
```

## 测试覆盖

| 测试文件 | 覆盖内容 |
|----------|----------|
| `tests/test_eye_to_hand.py` | T_cam_base 往返、cam→base、`all --sim` 集成 |
| `tests/test_intrinsics.py` | 内参 JSON / 动态读取逻辑 |
| `tests/test_offline_calib.py` | 离线合成数据标定 |
| `tests/test_mock_calib.py` | Mock 臂+相机端到端标定 |
| `tests/test_sim_pipeline.py` | MuJoCo 标定 + 抓取放置 |
| `tests/test_ik.py` | 数值 IK |

```bash
./scripts/run_sim_tests.sh                              # 全部
./scripts/run_sim_tests.sh tests/test_eye_to_hand.py    # 指定测试
```

## 脚本说明

| 脚本 | 作用 |
|------|------|
| `scripts/run_sim_tests.sh` | 安装依赖、下载 URDF、运行 pytest |
| `scripts/run_eye_to_hand_demo.sh` | 眼在手外测试 + 录制演示视频 |
| `scripts/record_eye_to_hand_demo.py` | 录制标定→报告→抓取全流程 MP4 |

演示视频输出到 `output/eye_to_hand_demo.mp4`（侧视全景 + 固定相机双画面）。

录制参数：

```bash
python scripts/record_eye_to_hand_demo.py -o output/my_demo.mp4 --poses 8 --class bottle
```

## 仿真说明

- 标定图像由解析模型合成（与 URDF FK 自洽），MuJoCo 驱动臂运动与场景渲染
- 抓取检测在 `--sim` 下使用 MuJoCo 物体 ground truth，避免渲染深度与外参混用误差
- 仿真标定残差可能偏大（解析模型 vs MuJoCo 几何差异），真机以实际报告为准
- 需 `conda activate yolo`，脚本自动设置 `MUJOCO_GL=egl`

## 仿真标定 Python API

```python
from sim.mujoco_env import create_sim_env
from sim.sim_calibration import run_sim_calibration

env = create_sim_env(headless=True)
result = run_sim_calibration(env, num_poses=12)

print(f"平移误差: {result.translation_error_mm:.2f} mm")
print(f"旋转误差: {result.rotation_error_deg:.3f}°")
print(f"准确度:   {result.quality_label}")

# 输出: result.report_json, result.report_txt, result.report_png
```

仿真报告在标准报告基础上额外包含 GT 对比段、各算法 GT 误差表、运动多样性诊断。

更多脚本细节见 [scripts/README.md](../../scripts/README.md)。
