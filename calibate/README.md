# 手眼标定工具集

基于 OpenCV `calibrateHandEye`，在 `conda env yolo` 下提供三套独立手眼标定程序。

## 项目结构

```
calibate/
├── common/          # 公共模块（棋盘检测、RealSense 内参、坐标变换、标定报告）
├── tools/           # 辅助工具（D455 内参导出）
├── eye_in_hand/     # 眼在手上标定 → T_cam_gripper
├── eye_to_hand/     # 眼在手外标定 → T_cam_base
└── combined/        # 两路相机联合应用
```

上层整合项目见仓库根目录 [README.md](../README.md)（`pick_place/` 流水线）。

## 环境

```bash
conda activate yolo
cd calibration/calibate   # 仓库根目录下
```

依赖：`opencv-python`、`numpy`、`pyrealsense2`（D455 自动内参）。

## 推荐流程

```
1. 准备棋盘格 + 采集数据
2. （可选）导出 D455 内参，或标定时自动读取
3. 分别运行 eye_in_hand / eye_to_hand 标定
4. 查看 verify 报告与 calibration_report.txt
5. 在 pick_place/ 或 combined/ 中做抓取/融合定位
```

## 快速演示（合成数据）

```bash
cd eye_in_hand && python generate_demo_data.py && python calibrate.py && python verify.py
cd ../eye_to_hand && python generate_demo_data.py && python calibrate.py && python verify.py
cd ../combined && python hand_eye_fusion.py --demo
```

## Unitree D1 + D455 全自动流程

```
config/d1.urdf          ← 首次运行自动下载（或 python tools/get_d1_urdf.py）
config/d1_calibration_poses.json  ← 标定位姿（可编辑）
tools/d1_bridge/        ← 编译后与 D1 通信
tools/get_d1_urdf.py    ← 手动下载/更新 URDF

eye_in_hand/auto_calibrate.py   → 眼在手上 一键自标定
eye_to_hand/auto_calibrate.py   → 眼在手外 一键自标定（含结果报告）
pick_place/run.py calibrate     → 整合项目推荐入口
```

```bash
conda activate yolo
cd calibration   # 仓库根目录

# 整合入口（推荐）
python pick_place/run.py calibrate --network eth0

# 或仅标定模块
cd calibate/eye_to_hand
python auto_calibrate.py --network eth0
```

标定完成后查看 `eye_to_hand/output/calibration_report.txt`。

## 各目录说明

| 目录 | 说明 | 详细文档 |
|------|------|----------|
| `eye_in_hand/` | 相机装在末端，求相机→法兰变换 | [eye_in_hand/README.md](eye_in_hand/README.md) |
| `eye_to_hand/` | 相机固定在外部，求相机→基座变换 | [eye_to_hand/README.md](eye_to_hand/README.md) |
| `combined/` | 手外粗定位 + 手上精定位融合 | [combined/README.md](combined/README.md) |
| `tools/` | RealSense D455 内参工具 | [tools/README.md](tools/README.md) |
| `common/` | 底层公共库 | [common/README.md](common/README.md) |

## 数据格式约定

- 长度单位：**米**
- 角度单位：**度**（`euler_xyz`）
- 位姿含义：`robot_pose` 为 **base → gripper**（基座到末端）
- 棋盘：默认 9×6 内角点，方格 25 mm（可通过参数修改）

## RealSense D455 内参

D455 内参为出厂标定、按 **设备 SN** 存储。本项目默认在相机已连接时**动态读取**，避免换机后仍用旧 JSON：

| 场景 | 行为 |
|------|------|
| 标定采集 (`auto_collect`) | 启动 D455 时读取内参 + SN，写入 `camera_intrinsics.json` |
| 标定求解 / verify | 有 D455 时优先读当前设备；SN 变化则提示并更新 JSON |
| 抓取 3D 检测 (`get_object`) | 每帧从深度流 profile 读取 |
| 无相机 / 离线重算 | `calibrate.py --use-saved-intrinsics` |

手动导出（检查或备份）：

```bash
python tools/get_d455_intrinsics.py --list
python tools/get_d455_intrinsics.py --from-image eye_to_hand/data/images/0000.png \
  --output eye_to_hand/data/camera_intrinsics.json
```

详见 [eye_to_hand/README.md](eye_to_hand/README.md#相机内参d455-动态读取) 与 [tools/README.md](tools/README.md)。

设计文档见 [../doc/README.md](../doc/README.md)；用户指南见 [../doc/user/quickstart.md](../doc/user/quickstart.md)。
