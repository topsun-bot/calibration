# 设计文档

本目录收录手眼标定与 ROS2/MoveIt 演进相关的技术文档，与 [`calibate/`](../calibate/) 代码实现配套维护。

## 推荐阅读顺序

1. [双相机配置与内参](hand_eye/dual_camera_setup.md) — 当前硬件（RealSense D435I + 2M USB）如何接入标定流程
2. [与 easy_handeye2 / MoveIt 差距分析](hand_eye/gap_analysis_vs_ros_tools.md) — 现有实现的优势与不足、优化优先级
3. [ROS2 + MoveIt 演进路线图](hand_eye/ros2_moveit_roadmap.md) — 架构决策、分阶段落地计划

## 文档目录

| 文档 | 说明 |
|------|------|
| [hand_eye/dual_camera_setup.md](hand_eye/dual_camera_setup.md) | 双相机角色分工、`cameras.json` 配置、内参标定命令 |
| [hand_eye/gap_analysis_vs_ros_tools.md](hand_eye/gap_analysis_vs_ros_tools.md) | 对比 [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) 与 [MoveIt](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) |
| [hand_eye/ros2_moveit_roadmap.md](hand_eye/ros2_moveit_roadmap.md) | 推荐架构（calibate 核心 + ROS2 薄桥接）、Phase 0–3 |

## 与代码模块映射

| 文档主题 | 相关代码 |
|----------|----------|
| 相机采集与工厂 | [`calibate/common/calib_camera.py`](../calibate/common/calib_camera.py)、[`usb_camera_capture.py`](../calibate/common/usb_camera_capture.py)、[`realsense_capture.py`](../calibate/common/realsense_capture.py) |
| 双相机配置 | [`calibate/config/cameras.json`](../calibate/config/cameras.json) |
| 自动采集 | [`calibate/common/auto_collect.py`](../calibate/common/auto_collect.py) |
| 手眼求解 | [`calibate/common/hand_eye_solve.py`](../calibate/common/hand_eye_solve.py) |
| 验证与报告 | [`calibate/common/calib_report.py`](../calibate/common/calib_report.py) |
| 可视化报告 | [`calibate/common/calib_visualizer.py`](../calibate/common/calib_visualizer.py) |
| 标准化 IO / ROS 导出 | [`calibate/common/io_utils.py`](../calibate/common/io_utils.py) |
| 调试显示 | [`calibate/common/debug_display.py`](../calibate/common/debug_display.py) |
| 双相机融合 | [`calibate/combined/hand_eye_fusion.py`](../calibate/combined/hand_eye_fusion.py) |
| 抓取流水线 | [`pick_place/vision.py`](../pick_place/vision.py)、[`pick_place/controller.py`](../pick_place/controller.py) |
| 相机工具 | [`calibate/tools/list_cameras.py`](../calibate/tools/list_cameras.py)、[`calibate/tools/calibrate_usb_intrinsics.py`](../calibate/tools/calibrate_usb_intrinsics.py) |

## 外部参考

- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) — ROS2 手眼标定中间件（tf 采样 → OpenCV 求解 → tf 发布）
- [MoveIt Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) — 运动规划、运动学、场景监控
