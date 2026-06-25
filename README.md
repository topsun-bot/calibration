# calibration

眼在手外（Eye-to-Hand）手眼标定 + YOLO 3D 目标识别 + Unitree D1 机械臂自动抓取放置。

仓库地址：<https://github.com/topsun-bot/calibration>

## 功能概览

| 模块 | 说明 |
|------|------|
| [calibate/](calibate/README.md) | 手眼标定（眼在手上 / 眼在手外） |
| [pick_place/](doc/user/pick_place.md) | 标定 + 抓取整合流水线 |
| [get_object/](get_object/yolo3d/run.py) | YOLO + RealSense 3D 目标检测 |
| [sim/](doc/user/simulation.md) | MuJoCo 仿真标定与测试 |
| [doc/](doc/README.md) | 设计文档与 ROS2 演进路线 |

## 项目结构

```
calibration/
├── calibate/          # 手眼标定（common / eye_to_hand / eye_in_hand / combined）
├── pick_place/        # 整合流水线（calibrate / pick / all）
├── get_object/        # YOLO 3D 检测
├── sim/               # MuJoCo 仿真
├── scripts/           # 自动化测试与演示录制
├── tests/             # pytest
├── doc/               # 设计文档与用户指南
└── output/            # 演示视频等本地输出（git 忽略）
```

## 快速开始

```bash
git clone https://github.com/topsun-bot/calibration.git
cd calibration

conda activate yolo
pip install -r get_object/requirements.txt
pip install -r requirements-dev.txt   # 仿真/测试：pytest、mujoco

# 编译 D1 桥接（真机）
./scripts/build_d1_bridge.sh
```

**真机标定 + 抓取：**

```bash
# 1. 眼在手外自标定（棋盘夹在夹爪，相机固定）
python pick_place/run.py calibrate --network eth0

# 2. 抓取指定类别物体
python pick_place/run.py pick --class bottle --network eth0

# 一步完成
python pick_place/run.py all --class cup --network eth0
```

**无真机仿真：**

```bash
python pick_place/run.py calibrate --sim
./scripts/run_sim_tests.sh
```

详细步骤见 [doc/user/quickstart.md](doc/user/quickstart.md)。

## 文档导航

### 用户指南

| 文档 | 内容 |
|------|------|
| [doc/user/quickstart.md](doc/user/quickstart.md) | 环境、硬件布置、快速上手 |
| [doc/user/pick_place.md](doc/user/pick_place.md) | 抓取流水线、配置项、标定结果说明 |
| [doc/user/simulation.md](doc/user/simulation.md) | MuJoCo 仿真、自动化测试、演示录制 |

### 标定模块

| 文档 | 内容 |
|------|------|
| [calibate/README.md](calibate/README.md) | 标定工具集总览 |
| [calibate/eye_to_hand/README.md](calibate/eye_to_hand/README.md) | 眼在手外标定（主流程） |
| [calibate/eye_in_hand/README.md](calibate/eye_in_hand/README.md) | 眼在手上标定 |
| [calibate/combined/README.md](calibate/combined/README.md) | 双相机融合定位 |
| [calibate/tools/README.md](calibate/tools/README.md) | D455 内参、URDF、D1 桥接 |
| [scripts/README.md](scripts/README.md) | 测试与演示脚本 |

### 设计文档

| 文档 | 内容 |
|------|------|
| [doc/hand_eye/dual_camera_setup.md](doc/hand_eye/dual_camera_setup.md) | 双相机配置与内参 |
| [doc/hand_eye/gap_analysis_vs_ros_tools.md](doc/hand_eye/gap_analysis_vs_ros_tools.md) | 与 ROS 手眼标定工具对比 |
| [doc/hand_eye/ros2_moveit_roadmap.md](doc/hand_eye/ros2_moveit_roadmap.md) | ROS2 + MoveIt 演进路线 |

## 典型工作流

```
自标定 → T_cam_base + 验证报告
观测位姿 → YOLO 3D 检测 → 相机坐标 → 基座坐标
    → 数值 IK → 预抓取 → 下降 → 夹紧 → 抬起
    → 移动到放置区 → 下降 → 松开 → 回 home
```

## 常见问题

| 问题 | 处理 |
|------|------|
| 未检测到目标 | 降低 `--conf`，确认 YOLO 模型含该类别 |
| 标定残差大 | 检查相机固定、棋盘夹持，增加 `--num-poses` |
| 换了 D455 后精度变差 | 重新采集标定数据 |
| 相机被占用 | `pkill -f 'python.*run.py'` 后重试 |
| USB 带宽不足 | 换 USB 3.0 口，或运行 `scripts/check_realsense.py` |

更多排障见 [doc/user/pick_place.md#常见问题](doc/user/pick_place.md#常见问题) 与 [calibate/eye_to_hand/README.md](calibate/eye_to_hand/README.md)。
