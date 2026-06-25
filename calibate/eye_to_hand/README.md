# 眼在手外 (Eye-to-Hand) 手眼标定

相机固定安装在机器人工作空间外部，标定结果为 **T_cam_base**（相机坐标系 → 机器人基座坐标系）。

## 原理简述

```
相机固定不动
     ↓
标定板随末端运动（或固定在夹爪上）
     ↓
每组：图像 + 机器人位姿 → OpenCV calibrateHandEye
     ↓
输出 T_cam_base
```

验证思路：用标定结果将各帧标定板原点变换到基座系，多帧应重合。

## 目录结构

```
eye_to_hand/
├── calibrate.py
├── verify.py
├── auto_calibrate.py      # 一键自标定
├── generate_demo_data.py
├── data/
│   ├── images/
│   ├── poses.json
│   └── camera_intrinsics.json
└── output/
    ├── T_cam_base.json        # 标准化 v1.0.0 JSON
    ├── calibration_report.txt # 文本报告
    └── calibration_report.png # matplotlib 可视化图表（残差柱状图/3D散点/质量仪表）
```

## 全自动自标定（Unitree D1 + RealSense D455）

相机固定，**棋盘格夹在 D1 夹爪上**随臂运动。

```bash
conda activate yolo
cd /home/gy/code/goat_demo/calibate/eye_to_hand

# 1) 先确认 D455 能出图（推荐）
python3 ../../scripts/check_realsense.py

# 2) 真机标定（默认：回零 → 实时画面手动摆位 → Enter 开始）
python auto_calibrate.py
```

启动后机械臂会先**回零**，调试窗口显示相机画面；请**手动**将臂/棋盘摆到视野中心，按 **Enter** 开始自动采集。跳过该阶段：`--no-manual-prep`。

### 真机摆放（眼在手外）

| 组件 | 建议位置 |
|------|----------|
| D455 | 三脚架固定在工作区侧方，俯视桌面与机械臂，**USB 直插笔记本 USB 3.0 口** |
| D1 臂 | 基座稳固，夹爪可够到相机视野内的棋盘格 |
| 棋盘格 | 夹在夹爪上（或先放桌面做夹取准备），全程在相机视野内 |
| 网络 | 笔记本 `eth0` 接 D1（默认 IP `192.168.123.100`） |

> 若 `Frame didn't arrive within 5000`：多为 D455 接在 **USB 2.0** 口。程序会**自适应带宽**（自动选分辨率/帧率，运行中可再降档）；首帧可能需 15~30 秒。**最好换 USB 3.0 口**。

预检：`python3 ../../scripts/check_realsense.py` 会逐档探测并给出推荐配置。

**整合流水线入口**（标定 + 抓取，推荐）：

```bash
cd /home/gy/code/goat_demo
python pick_place/run.py calibrate --network eth0
```

详见 [eye_in_hand/README.md](../eye_in_hand/README.md) 中的自标定说明（含 **adaptive 视野保护** 机制）。

默认 `--pose-mode adaptive`：先找棋盘可见锚点，再小步扰动；失败自动回退，不会盲目执行大角度位姿。

**初始朝向**：config 种子位姿 j0 通常按“正前方”编写；若现场机械臂相对标定板已旋转约 90°（如臂在左、板在右），程序会**读取当前 j0 并自动修正**所有种子位姿，无需手改 JSON。禁用：`--no-align-base-yaw`。

### 实时调试与结果展示

| 参数 | 说明 | 默认 |
|------|------|------|
| `--show` / `--no-show` | 采集过程实时窗口（视频+关节角+棋盘） | 开启 |
| `--show-result` / `--no-show-result` | 标定完成后结果摘要弹窗 | 开启 |

标定结束后终端与 `output/calibration_report.txt` 中包含：

- 外参平移 `t_cam_base`、旋转欧拉角
- 4×4 齐次矩阵 `T_cam_base`
- 验证残差（mean / max / std，单位 mm）
- 各样本重投影误差（px）
- 离群样本标记（IQR 箱线图法，> Q3+1.5×IQR）
- 质量评估（优秀 / 良好 / 可用 / 较差）+ 诊断建议

同时生成 `calibration_report.png` 可视化报告：残差柱状图（含离群高亮）、3D 散点图、算法对比、质量仪表盘。

### 夹爪 j6 自动设置（眼在手外）

默认 **`--auto-gripper`**：采集前固定相机检测棋盘：

1. **距离**：`solvePnP` 估计相机到棋盘距离  
2. **尺寸**：角点像素跨度 + 已知棋盘格尺寸，估算抓取宽度  
3. **j6**：预开口 → 夹紧，标定全程 **锁定 j6**（仅动 6 轴臂关节）

```bash
# 默认已开启自动夹爪
python auto_calibrate.py --network eth0

# 关闭（已手动夹好棋盘）
python auto_calibrate.py --no-auto-gripper

# 微调夹爪映射（板厚、开口范围等）
python auto_calibrate.py --gripper-config ../config/d1_gripper.json
```

**现场流程**：臂大致对准棋盘 → 运行脚本 → 自动夹紧 → 小步扰动采集 → 查看标定报告。  
映射参数见 `config/d1_gripper.json`（j6 约 -40° 全开 ~ 20° 全闭）。

---

## 采集要求

1. **相机固定**在机器人外部（三脚架、支架等），全程不动。
2. **标定板随末端运动**——通常由夹爪夹持棋盘格，或用已知尺寸的标定板固定在夹爪上。
3. 至少 **10～15 组**位姿（最少 3 组），覆盖不同角度和位置。
4. 确保棋盘在相机视野内且清晰可辨。

## 与眼在手上的区别

| 项目 | 眼在手上 | 眼在手外 |
|------|----------|----------|
| 相机 | 装在末端 | 固定在外部 |
| 标定板 | 固定在工作台 | 随末端运动 |
| 输出 | T_cam_gripper | T_cam_base |
| OpenCV 输入 | gripper2base + target2cam | base2gripper + target2cam |

## 准备数据

### poses.json

格式与 `eye_in_hand` 相同，`robot_pose` 表示 **base → gripper**：

```json
{
  "samples": [
    {
      "image": "0000.png",
      "robot_pose": {
        "position": [0.30, 0.00, 0.20],
        "euler_xyz": [0, 0, 0]
      }
    }
  ]
}
```

### 相机内参（D455 动态读取）

RealSense D455 内参为**出厂标定、按设备 SN 存储**，SDK 可随时读取。本项目默认在**有相机连接时动态获取**，避免换机后仍用旧 JSON 导致偏差：

| 阶段 | 行为 |
|------|------|
| **标定采集** | 启动 D455 时从当前设备读取内参 + SN，写入 `data/camera_intrinsics.json` |
| **标定求解 / verify** | 默认再次从当前 D455 读取；若 SN 与 JSON 不同会提示并改用新设备内参 |
| **抓取 3D 检测** | 每帧从深度流 profile 读取内参（与当前相机、分辨率一致） |

```bash
# 默认：有 D455 则动态读内参（推荐）
python calibrate.py --data-dir data

# 离线重算历史数据（无相机 / 必须用采集时的 JSON）
python calibrate.py --data-dir data --use-saved-intrinsics

# 多台 RealSense 时指定 SN
python calibrate.py --realsense-serial 260722303031
```

**注意**：标定图像与内参必须来自**同一台相机、同一分辨率**。换相机后应重新采集标定数据，或至少重新跑 `calibrate`/`verify` 让内参与图像匹配。

也可手动导出：

```bash
python ../tools/get_d455_intrinsics.py --from-image data/images/0000.png \
  --output data/camera_intrinsics.json
```

## 运行标定

```bash
conda activate yolo
cd /home/gy/code/goat_demo/calibate/eye_to_hand

python calibrate.py --data-dir data
python calibrate.py --cols 9 --rows 6 --square-size 0.025
python calibrate.py --method tsai --auto-camera
```

### 常用参数

| 参数 | 说明 | 默认 |
|------|------|------|
| `--data-dir` | 数据目录 | `data` |
| `--output` | 结果路径 | `output/T_cam_base.json` |
| `--method` | 手眼算法 | `fused` |
| `--auto-camera` | 强制从 D455 读取内参 | 有相机时默认动态读取 |
| `--use-saved-intrinsics` | 仅使用 JSON，不读 RealSense | 关（离线重算时开启） |
| `--realsense-serial` | 指定 D455 序列号 | 自动选择 |
| `--realsense-width/height` | 彩色流分辨率 | 从首张图推断 |

## 验证标定

```bash
python verify.py --data-dir data --result output/T_cam_base.json

# 无相机、使用采集时的内参 JSON
python verify.py --data-dir data --use-saved-intrinsics
```

`verify.py` 会打印完整标定报告并写入 `output/calibration_report.txt`。

## 演示

```bash
python generate_demo_data.py
python calibrate.py
python verify.py
```

### MuJoCo 仿真测试（无需真机）

```bash
cd /home/gy/code/goat_demo
./scripts/run_sim_tests.sh          # 15 项自动化测试
./scripts/run_eye_to_hand_demo.sh   # 测试 + 录制 output/eye_to_hand_demo.mp4
python scripts/record_eye_to_hand_demo.py -o output/eye_to_hand_demo.mp4
```

仿真标定使用合成棋盘图（URDF FK + 固定 T_cam_base），与真机流程接口一致。  
仿真抓取使用 MuJoCo GT 位姿，验证 **T_cam_base → 基座坐标 → IK → 抓取放置** 全链路。

整合入口：

```bash
python pick_place/run.py calibrate --sim
python pick_place/run.py all --class bottle --sim --no-show
```

## 输出结果

`output/T_cam_base.json` 采用标准化 v1.0.0 格式，包含完整的元数据与互操作字段：

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

基础用法（向后兼容旧格式）：

```python
# p_base = R @ p_cam + t
```

ROS / tf2 互操作：

```python
from common.io_utils import export_tf2_transform, export_urdf_fragment

# ROS tf2 TransformStamped 兼容字典
tf2 = export_tf2_transform("output/T_cam_base.json")

# URDF / xacro 固定关节片段
urdf = export_urdf_fragment("output/T_cam_base.json")

# 校验文件完整性
from common.io_utils import validate_calibration_file
result = validate_calibration_file("output/T_cam_base.json")
```

典型用途：手外相机检测工件位置，直接得到工件在机器人基座系下的坐标，用于路径规划。

## 常见问题

| 现象 | 处理 |
|------|------|
| 棋盘检测失败 | 调整 `--cols/--rows/--square-size` |
| 标定板抖动 | 夹持牢固，每位姿停稳再拍照 |
| 内参不匹配 | 确保与采集时相同的彩色分辨率；有 D455 时默认动态刷新 |
| 换了 D455 | 重新采集标定数据，或确认 SN 一致（见内参 JSON `meta.serial`） |
| 验证残差大 | 增加 `--num-poses`；检查相机是否移动；重新夹紧棋盘 |
| 质量评估「较差」 | 查看 `calibration_report.txt` 各样本残差，剔除异常位姿后重标 |
| 离线重算旧数据 | `calibrate.py` / `verify.py` 加 `--use-saved-intrinsics` |
