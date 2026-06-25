# goat_demo

眼在手外（Eye-to-Hand）自标定 + YOLO 3D 目标识别 + D1 机械臂自动抓取放置。

## 项目结构

```
goat_demo/
├── calibate/          # 手眼标定（眼在手上 / 眼在手外）
│   ├── common/        # 共享库：采集、求解、报告、可视化、IO
│   │   ├── hand_eye_solve.py    # 多算法融合 + 退化检测
│   │   ├── calib_report.py      # 文本报告 + 离群检测
│   │   ├── calib_visualizer.py  # matplotlib 可视化图表
│   │   ├── io_utils.py          # 标准化 JSON v1.0 + ROS/tf2 导出
│   │   └── debug_display.py     # 实时调试窗口（坐标轴/边界/角度）
│   ├── config/        # URDF、相机配置、棋盘格 PDF
│   ├── eye_to_hand/   # 眼在手外标定脚本
│   └── eye_in_hand/   # 眼在手上标定脚本
├── doc/               # 设计文档（双相机、ROS2 路线图、差距分析）
├── get_object/        # YOLO + RealSense 3D 目标检测
├── pick_place/        # 整合流水线（本 README 重点）
│   ├── run.py         # 主入口：calibrate / pick / all
│   ├── controller.py  # 抓取放置控制器
│   ├── vision.py      # 相机→基座坐标变换 + 检测
│   ├── d1_ik.py       # 数值逆运动学
│   └── config/pick_place.json
├── sim/               # MuJoCo 眼在手外仿真（--sim）
│   ├── mujoco_env.py       # MuJoCo 场景 + 渲染
│   ├── sim_calibration.py  # 仿真标定（端到端 + GT 误差对比）
│   ├── backends.py         # SimD1Arm / SimCamera 接口
│   ├── detection_stub.py   # 仿真 YOLO 替代（GT 检测框）
│   └── mjcf/               # D1 机器人 MJCF 模型
├── tests/             # pytest（眼在手外 / 标定 / IK / 内参）
├── scripts/           # 自动化测试与演示录制（见 scripts/README.md）
└── output/            # 演示视频等输出（eye_to_hand_demo.mp4）
```

## 环境

```bash
conda activate yolo
cd /home/gy/code/goat_demo
pip install -r get_object/requirements.txt
pip install -r requirements-dev.txt   # 仿真/测试：pytest、mujoco
# D1 真机还需编译 calibate/tools/d1_bridge（见 scripts/build_d1_bridge.sh）
```

## 硬件布置

1. **RealSense D455** 固定在工作空间外部（全程不动）
2. **Unitree D1** 机械臂，网口连接（如 `eth0`）
3. 工作台上放置待抓取物体；在配置中设定 **放置区域** `place_region`

## 快速开始

### 第一步：眼在手外自标定

将棋盘格置于夹爪可抓取位置，相机固定不动：

```bash
conda activate yolo
cd /home/gy/code/goat_demo

python pick_place/run.py calibrate --network eth0
```

标定完成后会自动输出：

- **终端报告**：外参 `t`、旋转欧拉角、4×4 矩阵 `T_cam_base`、验证残差、质量评估（优秀/良好/可用/较差）
- **结果文件**：`calibate/eye_to_hand/output/T_cam_base.json`
- **文本报告**：`calibate/eye_to_hand/output/calibration_report.txt`
- **结果窗口**：默认弹出摘要窗口（`--no-show-result` 可关闭）

> 也可直接使用 `calibate/eye_to_hand/auto_calibrate.py`，效果相同。

### 第二步：抓取指定类别物体并放到目标区域

修改 `pick_place/config/pick_place.json` 中的 `place_position_base` 与 `place_region`，然后：

```bash
python pick_place/run.py pick --class bottle --network eth0
```

### 一步完成（标定 + 抓取）

```bash
python pick_place/run.py all --class cup --network eth0
```

标定结束会显示结果报告；取下棋盘格后按 Enter 继续抓取（`--sim` 无交互）。

### 调试窗口

默认开启实时调试窗口（`--show`），左侧为相机画面，右侧为关节角、TCP 位置与状态信息。

**画面叠加内容**：
- 棋盘格角点 + RGB 坐标系轴（X 红、Y 绿、Z 蓝，板面法线方向）
- 板面法线与相机光轴夹角（> 60° 时 ⚠ 警告）
- 边缘安全边界（棋盘靠近画面边缘时红色边框提示）

```bash
# 默认：采集调试窗口 + 标定结果摘要窗口
python pick_place/run.py calibrate --network eth0

# 抓取时附加深度图
python pick_place/run.py pick --class bottle --network eth0 --show-depth

# 关闭实时窗口，但保留标定结果报告
python pick_place/run.py calibrate --network eth0 --no-show

# 关闭标定结果弹窗（终端仍有完整报告）
python pick_place/run.py calibrate --network eth0 --no-show-result
```

调试窗口中按 `q` 可关闭实时画面（任务仍会继续）。标定结果窗口按任意键关闭。

## 标定结果说明

| 输出 | 路径 |
|------|------|
| 外参 JSON（标准化 v1.0） | `calibate/eye_to_hand/output/T_cam_base.json` |
| 文本报告 | `calibate/eye_to_hand/output/calibration_report.txt` |
| 可视化图表（PNG） | `calibate/eye_to_hand/output/calibration_report.png` |
| 采集数据 | `calibate/eye_to_hand/data/` |

### JSON 外参格式（v1.0.0）

标定结果采用标准化格式，包含完整的元数据与互操作字段：

```json
{
  "format_version": "1.0.0",
  "run_id": "a1b2c3d4e5f6",
  "parent_frame": "base_link",
  "child_frame": "camera_link",
  "transform": {
    "translation": { "x": 0.30, "y": 0.15, "z": 0.60 },
    "rotation": {
      "matrix_3x3": [[0.956, -0.045, 0.290], ...],
      "quaternion_xyzw": [0.02, 0.14, 0.01, 0.99]
    }
  },
  "timestamp_utc": "2026-06-25T10:30:00Z",
  "software": { "pipeline": "goat_demo", "git_sha": "abc1234", "opencv_version": "4.11.0" },
  "board_config": { "cols": 9, "rows": 6, "square_size_m": 0.025 },
  "intrinsics_sha256": "a1b2c3d4e5f6a7b8",
  "_units": { "length": "meters", "angle": "radians", "quaternion_convention": "xyzw" }
}
```

新格式**向后兼容**旧版 JSON（仅有 `R`/`t`/`meta` 也可正常加载）。

**质量评估参考**（基于验证残差，含自动诊断提示）：

| 等级 | mean 残差 | 建议 |
|------|-----------|------|
| 优秀 | ≤ 5 mm | 可直接抓取 |
| 良好 | ≤ 15 mm | 可用，建议微调 `tcp_offset` |
| 可用 | ≤ 30 mm | 建议增加样本或重标 |
| 较差 | > 30 mm | 检查相机固定与棋盘夹持后重标 |

验证报告中新增：
- **离群样本检测**（IQR 箱线图法，自动标记 > Q3+1.5×IQR 的帧）
- **各样本重投影误差**（棋盘角点 px 级精度）
- **诊断提示**（如 "max/mean 比值大 → 可能存在运动模糊"）

### 可视化报告

标定完成后自动生成 `calibration_report.png`，包含：
- **各样本残差柱状图**（含 IQR 离群阈值线，异常样本红色高亮）
- **3D 散点图**：标定板原点在基座系下的离散度
- **算法对比图**（融合标定时各算法残差与融合权重）
- **质量仪表盘**（残差均值指针 + 绿/黄/橙/红四色等级）

单独查看已有标定结果：

```bash
cd calibate/eye_to_hand
python verify.py --data-dir data --result output/T_cam_base.json
# 离线重算（无相机，使用采集时的内参 JSON）
python verify.py --data-dir data --use-saved-intrinsics
```

### ROS / tf2 互操作

```python
from common.io_utils import export_tf2_transform, export_urdf_fragment

# 导出 ROS tf2 TransformStamped 兼容字典
tf2_dict = export_tf2_transform("output/T_cam_base.json")
# → {"header": {"frame_id": "base_link"}, "child_frame_id": "camera_link",
#    "transform": {"translation": {...}, "rotation": {"x","y","z","w"}}}

# 生成 URDF / xacro 固定关节片段
urdf_fragment = export_urdf_fragment("output/T_cam_base.json")
# → <joint name="camera_link_joint" type="fixed"> ... </joint>
```

校验已有标定文件：`validate_calibration_file(path)` 检查旋转矩阵正交性、平移有限性、四元数归一化。

## D455 相机内参（动态读取）

RealSense D455 内参由出厂标定、**按设备 SN 存储**，SDK 可随时从当前连接的设备读取。本项目默认在**有相机连接时动态获取**，避免更换 D455 后仍使用旧 JSON 造成棋盘/深度反投影偏差。

| 阶段 | 内参来源 |
|------|----------|
| **标定采集** | 打开 D455 时读取当前设备内参 + SN → 写入 `data/camera_intrinsics.json` |
| **标定求解 / verify** | 默认再次从当前 D455 读取；SN 与 JSON 不一致时提示并改用新设备内参 |
| **抓取 3D 检测** | 每帧从 RealSense 深度流 profile 读取（与当前分辨率一致） |

```bash
# 真机标定（默认动态读内参）
python pick_place/run.py calibrate --network eth0

# 离线重算历史数据（无相机 / 必须用采集时 JSON）
cd calibate/eye_to_hand
python calibrate.py --data-dir data --use-saved-intrinsics
python verify.py --data-dir data --use-saved-intrinsics

# 多台 RealSense 时指定 SN
python calibrate.py --realsense-serial 260722303031

# 手动导出内参（检查用）
python calibate/tools/get_d455_intrinsics.py --list
python calibate/tools/get_d455_intrinsics.py --from-image calibate/eye_to_hand/data/images/0000.png \
  --output calibate/eye_to_hand/data/camera_intrinsics.json
```

**换相机注意**：标定图像与内参必须来自**同一台 D455、同一分辨率**。更换相机后应重新采集并标定；仅更换抓取阶段的相机时，3D 检测会自动用新内参，但 `T_cam_base` 仍对应标定时的相机，需重新做手眼标定。

详见 [calibate/eye_to_hand/README.md](calibate/eye_to_hand/README.md#相机内参d455-动态读取)。

## 配置说明

编辑 `pick_place/config/pick_place.json`：

| 字段 | 说明 |
|------|------|
| `observe_joints_deg` | 观测位姿（7 关节角，度） |
| `home_joints_deg` | 任务结束后回 home |
| `place_position_base` | 放置点（基座坐标系，米） |
| `place_region` | 放置点合法范围（安全校验） |
| `workspace` | 可抓取/运动范围 |
| `approach_offset_z` | 预抓取/放置时 Z 方向抬高量 |
| `grasp_offset_z` | 相对检测点的抓取深度补偿 |
| `tcp_offset_xyz` | 夹爪 TCP 相对检测点的偏移 |

夹爪 j6 映射见 `calibate/config/d1_gripper.json`。

## 仿真与自动化测试

无真机时可用 **MuJoCo 仿真** 或 **mock 模式** 验证完整流程：

```bash
# 全部自动化测试（15 项：眼在手外 / 标定 / IK / 内参 / MuJoCo）
./scripts/run_sim_tests.sh

# 眼在手外专项：测试 + 录制演示视频
./scripts/run_eye_to_hand_demo.sh

# 仅录制视频 → output/eye_to_hand_demo.mp4
python scripts/record_eye_to_hand_demo.py
```

# MuJoCo 仿真标定
python pick_place/run.py calibrate --sim

# MuJoCo 一步完成（标定 + 抓取，无交互）
python pick_place/run.py all --class bottle --sim --no-show

# MuJoCo 仿真抓取（需先有标定结果，仿真检测使用 GT 位姿）
python pick_place/run.py pick --class bottle --sim

# 无 MuJoCo 时 mock 臂（不执行真实运动）
python pick_place/run.py pick --class bottle --mock-arm
```

### 自动化测试覆盖（眼在手外）

| 测试文件 | 覆盖内容 |
|----------|----------|
| `tests/test_eye_to_hand.py` | T_cam_base 往返、cam→base、`all --sim` 集成 |
| `tests/test_intrinsics.py` | 内参 JSON / 动态读取逻辑 |
| `tests/test_offline_calib.py` | 离线合成数据标定 |
| `tests/test_mock_calib.py` | Mock 臂+相机端到端标定 |
| `tests/test_sim_pipeline.py` | MuJoCo 标定 + 抓取放置 |
| `tests/test_ik.py` | 数值 IK |

### 脚本说明

| 脚本 | 作用 |
|------|------|
| `scripts/run_sim_tests.sh` | 安装依赖、下载 URDF、运行 pytest |
| `scripts/run_eye_to_hand_demo.sh` | 跑眼在手外相关测试 + 录制演示视频 |
| `scripts/record_eye_to_hand_demo.py` | 录制标定→报告→抓取全流程 MP4 |

演示视频：`output/eye_to_hand_demo.mp4`（侧视全景 + 固定相机双画面）。

仿真说明：
- 标定图像由解析模型合成（与 URDF FK 自洽），MuJoCo 驱动臂运动与场景渲染
- 抓取检测在 `--sim` 下使用 MuJoCo 物体 ground truth，避免渲染深度与标定外参混用误差
- 仿真标定残差可能偏大（解析模型 vs MuJoCo 几何差异），真机以实际报告为准
- 需 `conda activate yolo`，脚本会自动设置 `MUJOCO_GL=egl`

### 仿真标定 Python API

可直接调用仿真标定模块获取 GT 误差对比：

```python
from sim.mujoco_env import create_sim_env
from sim.sim_calibration import run_sim_calibration

env = create_sim_env(headless=True)
result = run_sim_calibration(env, num_poses=12)

# 结果字段
print(f"平移误差: {result.translation_error_mm:.2f} mm")
print(f"旋转误差: {result.rotation_error_deg:.3f}°")
print(f"准确度:   {result.quality_label}")

# 各算法 GT 误差
for alg in result.per_algorithm:
    print(f"  {alg.name}: {alg.translation_error_mm:.2f}mm, "
          f"{alg.rotation_error_deg:.3f}°, 残差={alg.residual_mean_mm:.2f}mm")

# 输出文件: result.report_json, result.report_txt, result.report_png
```

仿真报告在标准报告基础上额外包含：
- **GT 对比段**：平移/旋转误差、准确度评级（Excellent / Good / Fair / Poor）
- **各算法 GT 误差表**：逐算法对比估计值与真值的偏差
- **运动多样性诊断**：旋转跨度、平移跨度是否满足标定最低要求

## 流程概览

```
自标定 → T_cam_base + 验证报告
观测位姿 → YOLO 3D 检测 → 相机坐标 → 基座坐标
    → 数值 IK → 预抓取 → 下降 → 夹紧 → 抬起
    → 移动到放置区 → 下降 → 松开 → 回 home
```

## 常见问题

1. **未检测到目标**：确认 YOLO 模型含该类别，`--conf` 适当降低，或换 `--model`。
2. **IK 误差大**：调整 `observe_joints_deg` 使 seed 更接近目标；查看标定报告中的验证残差。
3. **标定残差大**：确保相机牢固固定、棋盘夹持无松动，增加 `--num-poses` 样本数。查看 `calibration_report.png` 中离群样本（红色柱），移除后重标。
4. **相机被占用**：`pkill -f 'python.*run.py'` 后重试。
5. **放置点校验失败**：确保 `place_position_base` 落在 `place_region` 内。
6. **换了 D455 后精度变差**：重新采集标定数据；或确认 `camera_intrinsics.json` 中 SN 与当前相机一致（默认会自动刷新）。
7. **多台 RealSense**：标定/抓取时用 `--realsense-serial` 或 `get_d455_intrinsics.py --list` 确认设备。
8. **退化标定（旋转多样性不足）**：系统自动检测并警告。增加机械臂多方向旋转位姿（不能仅平移），使旋转轴跨度 ≥ 10°。
9. **万向锁警告**：报告中 pitch≈±90° 时 roll/yaw 不可靠，请直接参考旋转矩阵。

## 设计文档

手眼标定优化建议、双相机配置、与 easy_handeye2/MoveIt 对比及 ROS2 演进路线见 [`doc/`](doc/README.md)：

| 文档 | 说明 |
|------|------|
| [doc/hand_eye/dual_camera_setup.md](doc/hand_eye/dual_camera_setup.md) | RealSense D435I + 2M USB 双相机配置 |
| [doc/hand_eye/gap_analysis_vs_ros_tools.md](doc/hand_eye/gap_analysis_vs_ros_tools.md) | 与 ROS 手眼标定工具差距分析 |
| [doc/hand_eye/ros2_moveit_roadmap.md](doc/hand_eye/ros2_moveit_roadmap.md) | ROS2 + MoveIt 分阶段路线图 |

## 单独运行子模块

```bash
# 仅 3D 检测预览
cd get_object && python yolo3d/run.py

# 仅手眼标定（含结果报告）
cd calibate/eye_to_hand && python auto_calibrate.py --network eth0
```
