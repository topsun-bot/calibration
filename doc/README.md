# 文档索引

本仓库文档按用途分为三层：**用户指南**、**标定模块**、**设计文档**。

## 用户指南

面向日常使用与真机/仿真操作。

| 文档 | 内容 |
|------|------|
| [user/quickstart.md](user/quickstart.md) | 环境安装、硬件布置、快速上手 |
| [user/pick_place.md](user/pick_place.md) | 抓取流水线、配置项、标定结果与排障 |
| [user/simulation.md](user/simulation.md) | MuJoCo 仿真、pytest、Docker、可视化 API |

## 标定模块

各子目录 README 与代码一一对应，修改功能时请同步更新。

| 文档 | 内容 |
|------|------|
| [../calibate/README.md](../calibate/README.md) | 标定工具集总览 |
| [../calibate/eye_to_hand/README.md](../calibate/eye_to_hand/README.md) | 眼在手外标定（主流程） |
| [../calibate/eye_in_hand/README.md](../calibate/eye_in_hand/README.md) | 眼在手上标定 |
| [../calibate/combined/README.md](../calibate/combined/README.md) | 双相机融合定位 |
| [../calibate/common/README.md](../calibate/common/README.md) | 公共库 API |
| [../calibate/tools/README.md](../calibate/tools/README.md) | D455 内参、URDF、D1 桥接 |
| [../calibate/config/patterns/README.md](../calibate/config/patterns/README.md) | 棋盘格打印 |
| [../scripts/README.md](../scripts/README.md) | 测试、检测与演示脚本 |

## 设计文档

手眼标定优化、双相机配置、ROS2/MoveIt 演进路线。

推荐阅读顺序：

1. [hand_eye/dual_camera_setup.md](hand_eye/dual_camera_setup.md) — 双相机硬件接入
2. [hand_eye/gap_analysis_vs_ros_tools.md](hand_eye/gap_analysis_vs_ros_tools.md) — 与 easy_handeye2 / MoveIt 差距分析（含已实现能力）
3. [hand_eye/ros2_moveit_roadmap.md](hand_eye/ros2_moveit_roadmap.md) — ROS2 + MoveIt 分阶段路线图

## 与代码模块映射

| 文档主题 | 相关代码 |
|----------|----------|
| 相机采集与工厂 | `calibate/common/calib_camera.py`、`usb_camera_capture.py`、`realsense_capture.py` |
| 双相机配置 | `calibate/config/cameras.json` |
| 自动采集 | `calibate/common/auto_collect.py` |
| 手眼求解（融合+精修） | `calibate/common/hand_eye_solve.py` |
| 验证与报告 | `calibate/common/calib_report.py`、`calib_visualizer.py` |
| 标准化 IO / ROS 导出 | `calibate/common/io_utils.py` |
| 标定板检测（棋盘+ChArUco） | `calibate/common/board.py` |
| 位姿多样性评分 | `calibate/common/pose_planner.py` |
| 调试显示 | `calibate/common/debug_display.py` |
| 双相机融合 | `calibate/combined/hand_eye_fusion.py` |
| 抓取流水线 | `pick_place/vision.py`、`pick_place/controller.py` |
| 仿真环境 | `sim/mujoco_env.py`、`sim/sim_calibration.py` |
| 相机工具 | `calibate/tools/list_cameras.py`、`calibrate_usb_intrinsics.py` |
| 摄像头检测 | `scripts/check_cameras.py` |
| Docker 打包 | `Dockerfile`、`docker-compose.yml` |

## 外部参考

- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) — ROS2 手眼标定中间件
- [MoveIt Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) — 运动规划、运动学
- [OpenCV calibrateHandEye](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — 手眼标定 API

## 文档维护约定

- 项目名统一为 **calibration**
- 命令示例使用仓库根目录相对路径，不写绝对路径
- 用户向操作说明放 `doc/user/`，模块 API 放各子目录 README，架构决策放 `doc/hand_eye/`
- 修改 CLI 参数、输出路径或配置字段时，同步更新对应 README
- Docker 镜像名：`calibration`
