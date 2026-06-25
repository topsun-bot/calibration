# calibration

眼在手外（Eye-to-Hand）自标定 + YOLO 3D 目标识别 + D1 机械臂自动抓取放置。

## 项目结构

```
calibration/
├── calibate/          # 手眼标定（眼在手上 / 眼在手外）
│   ├── common/        # 共享库：采集、求解、报告、可视化、IO
│   │   ├── hand_eye_solve.py    # 多算法融合 + 非线性精修 + 异常剔除
│   │   ├── calib_report.py      # 文本报告 + 离群检测 + 重投影误差 + LOO 交叉验证
│   │   ├── calib_visualizer.py  # matplotlib 可视化 + 坐标系3D图 + GT对比图
│   │   ├── io_utils.py          # 标准化 JSON v1.0 + ROS/tf2 导出 + YAML 静态变换
│   │   ├── board.py             # 棋盘格 + ChArUco 检测后端
│   │   ├── pose_planner.py      # 位姿规划 + 多样性评分
│   │   └── debug_display.py     # 实时调试窗口（坐标轴/边界/角度）
│   ├── config/        # URDF、相机配置、棋盘格 PDF
│   ├── eye_to_hand/   # 眼在手外标定脚本
│   └── eye_in_hand/   # 眼在手上标定脚本
├── doc/               # 设计文档（双相机、ROS2 路线图、差距分析）
├── get_object/        # YOLO + RealSense 3D 目标检测
├── pick_place/        # 整合流水线
│   ├── run.py         # 主入口：calibrate / pick / all
│   ├── controller.py  # 抓取放置控制器
│   ├── vision.py      # 相机→基座坐标变换 + 检测
│   ├── d1_ik.py       # 数值逆运动学
│   └── config/pick_place.json
├── sim/               # MuJoCo 眼在手外仿真（--sim）
│   ├── mujoco_env.py       # MuJoCo 场景 + 渲染
│   ├── sim_calibration.py  # 仿真标定（端到端 + GT 误差对比）
│   ├── backends.py         # SimD1Arm / SimCamera 接口
│   └── detection_stub.py   # 仿真 YOLO 替代（GT 检测框）
├── tests/             # pytest（全部 23 项自动化测试）
├── scripts/           # 自动化测试与演示录制
├── Dockerfile         # Docker 镜像构建
├── docker-compose.yml # 仿真/真机/测试模式
└── output/            # 输出文件
```

## 环境

### 本地安装

```bash
pip install -r requirements-dev.txt       # 仿真/测试：pytest、mujoco
pip install pyrealsense2 ultralytics      # 真机：RealSense + YOLO
```

### Docker

```bash
docker build -t calibration .
docker run --rm calibration -m pytest tests/ -q     # 运行测试
docker compose up calibration                        # 仿真标定
docker compose run --rm hw                           # 真机模式（需 USB 透传）
```

## 硬件布置

| 设备 | 用途 | 连接 |
|------|------|------|
| **RealSense D435i** | 眼在手外固定相机（RGB+深度） | USB 3.0（推荐）或 2.1 |
| **2M USB 相机** | 眼在手上末端相机 | USB |
| **Unitree D1** | 机械臂 | 网口 |

检查摄像头状态：
```bash
python scripts/check_cameras.py --save
```

## 快速开始

### 仿真标定（无硬件）

```bash
python pick_place/run.py calibrate --sim --no-show
python pick_place/run.py all --class bottle --sim --no-show
```

仿真标定精度：0.0000mm 平移 / 0.0000° 旋转（Excellent）。

### 真机标定

```bash
python pick_place/run.py calibrate --network eth0
```

### 抓取

```bash
python pick_place/run.py pick --class bottle --network eth0
```

### 一步完成

```bash
python pick_place/run.py all --class cup --network eth0
```

## 核心算法

### 多算法融合标定

支持 5 种 OpenCV hand-eye 算法（Tsai / Park / Horaud / Andreff / Daniilidis），特性：

- **Softmax 加权融合**：`w_i ∝ exp(-residual_i / τ)`，避免 winner-take-all
- **MAD-based 异常算法剔除**：自动移除残差远超中位的算法
- **Levenberg-Marquardt 非线性精修**：最小化标定板位置一致性残差
- **Eye-to-hand 正确处理**：自动取逆传入 OpenCV `calibrateHandEye`
- **运动多样性检测**：旋转轴跨度 + 平移范围 + 退化预警

### 标定板检测

- **棋盘格**（默认）：9×6 内角点，`findChessboardCornersSB` + 亚像素精化
- **ChArUco**：ArUco 字典 + 棋盘混合，抗遮挡能力更强

### 评估能力

| 指标 | 说明 |
|------|------|
| 位置一致性残差 | 各帧标定板在夹爪系下的位置离散度 |
| 重投影误差 | 通过标定链投影角点与检测角点的像素 RMS |
| LOO 交叉验证 | 逐一排除样本评估稳定性 |
| GT 对比（仿真） | 与已知真值的平移/旋转误差 |

### 可视化

| 可视化 | 类型 | 函数 |
|--------|------|------|
| 标定报告 PNG | 静态 | `generate_report_figure()` |
| 坐标系 3D 场景 | 静态 | `plot_coordinate_frames()` |
| 重投影误差分布 | 静态 | `plot_reprojection_errors()` |
| 仿真 GT 对比 | 静态 | `plot_sim_gt_comparison()` |
| 实时调试面板 | 动态 | `DebugDisplay` |

## 标定结果

### JSON 外参格式（v1.0.0）

```json
{
  "format_version": "1.0.0",
  "run_id": "a1b2c3d4e5f6",
  "parent_frame": "base_link",
  "child_frame": "camera_link",
  "transform": {
    "translation": { "x": 0.30, "y": 0.15, "z": 0.60 },
    "rotation": {
      "matrix_3x3": [[...]],
      "quaternion_xyzw": [0.02, 0.14, 0.01, 0.99]
    }
  }
}
```

### ROS2 互操作

```python
from common.io_utils import export_static_transform_yaml, export_tf2_transform, export_urdf_fragment

# 导出 ROS2 static_transform_publisher 兼容 YAML
export_static_transform_yaml("output/T_cam_base.json", "output/camera_tf.yaml")

# 导出 tf2 TransformStamped 字典
tf2_dict = export_tf2_transform("output/T_cam_base.json")

# 生成 URDF 固定关节
urdf_fragment = export_urdf_fragment("output/T_cam_base.json")
```

## 自动化测试

```bash
./scripts/run_sim_tests.sh            # 全部测试
python -m pytest tests/ -v            # 直接 pytest
docker compose run --rm test          # Docker 内测试
```

| 测试文件 | 覆盖内容 |
|----------|----------|
| `test_sim_pipeline.py` | MuJoCo 标定 + GT 对比 + 抓取放置 |
| `test_mock_calib.py` | Mock 臂+相机端到端标定 |
| `test_eye_to_hand.py` | T_cam_base 往返、cam→base、集成 |
| `test_offline_calib.py` | 离线合成数据标定 |
| `test_intrinsics.py` | 内参 JSON / 动态读取 |
| `test_ik.py` | 数值 IK |
| `test_calib_camera.py` | 相机工厂 + 配置解析 |

## Docker

### 构建

```bash
docker build -t calibration .
```

### 运行模式

```bash
# 仿真（无需硬件）
docker run --rm calibration pick_place/run.py calibrate --sim --no-show

# 真机（USB 透传）
docker run --rm -it --privileged \
  --device /dev/video8 --device /dev/video10 \
  -v /dev/bus/usb:/dev/bus/usb \
  -v $(pwd)/output:/app/output \
  calibration pick_place/run.py calibrate --network eth0

# 测试
docker run --rm calibration -m pytest tests/ -q
```

### Docker Compose

```bash
docker compose up calibration     # 仿真标定
docker compose run --rm hw        # 真机
docker compose run --rm test      # 测试套件
```

## 相对 MoveIt2 / easy_handeye2 的优势

| 能力 | 本项目 | easy_handeye2 | MoveIt2 |
|------|--------|---------------|---------|
| 多算法融合 + 精修 | ✅ 5种算法 + L-M | ❌ 单算法 | N/A |
| 仿真 GT 验证 | ✅ 0.0000mm | ❌ | ❌ |
| 异常算法/样本检测 | ✅ MAD + IQR | ❌ | ❌ |
| D1 全自动采集 | ✅ | ❌ 需手动确认 | 需 move_group |
| 双相机融合 | ✅ | ❌ | ❌ |
| ChArUco 支持 | ✅ | ✅ (ArUco) | N/A |
| ROS2 导出 | ✅ YAML/tf2/URDF | ✅ 原生 | ✅ 原生 |
| Docker 打包 | ✅ | ❌ | ✅ |
| 无 ROS 依赖 | ✅ | ❌ | ❌ |

## 文档

| 文档 | 说明 |
|------|------|
| [doc/user/quickstart.md](doc/user/quickstart.md) | 环境安装、快速上手 |
| [doc/user/simulation.md](doc/user/simulation.md) | MuJoCo 仿真与测试 |
| [doc/hand_eye/gap_analysis_vs_ros_tools.md](doc/hand_eye/gap_analysis_vs_ros_tools.md) | 与 ROS 工具对比 |
| [doc/hand_eye/ros2_moveit_roadmap.md](doc/hand_eye/ros2_moveit_roadmap.md) | ROS2 演进路线图 |
| [scripts/README.md](scripts/README.md) | 脚本说明 |

## 常见问题

1. **RealSense D435i USB 2.1 超时**：带宽不足，推荐接 USB 3.0 口；或用 V4L2 回退（`video8`）
2. **标定残差大**：确保相机固定、棋盘夹持无松动，增加样本数
3. **IK 误差大**：调整 `observe_joints_deg` 使 seed 更接近目标
4. **Docker 内无法访问相机**：添加 `--privileged --device /dev/videoN`
5. **pyrealsense2 未安装**：`pip install pyrealsense2`
