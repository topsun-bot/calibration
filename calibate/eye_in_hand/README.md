# 眼在手上 (Eye-in-Hand) 手眼标定

相机安装在机器人末端法兰上，标定结果为 **T_cam_gripper**（相机坐标系 → 末端法兰坐标系）。

## 原理简述

```
棋盘格固定在工作台（不动）
     ↓
机器人带相机运动多个位姿
     ↓
每组：图像 + 机器人位姿 → OpenCV calibrateHandEye
     ↓
输出 T_cam_gripper
```

验证思路：用标定结果把各帧的标定板原点变换到基座系，多帧位置应重合（残差越小越好）。

## 目录结构

```
eye_in_hand/
├── calibrate.py           # 主标定程序
├── verify.py              # 标定结果验证
├── generate_demo_data.py  # 生成合成演示数据
├── data/
│   ├── images/            # 采集的图像
│   ├── poses.json         # 与图像对应的机器人位姿
│   └── camera_intrinsics.json  # 相机内参（可自动生成）
└── output/
    └── T_cam_gripper.json # 标定结果
```

## 全自动自标定（Unitree D1 + RealSense D455）

一条命令完成：**D455 内参 → D1 自动运动采集 → 手眼标定 → 验证**。

### 前置条件

1. 棋盘格**固定在工作台**（眼在手上）
2. D455 安装在 D1 末端
3. 已编译 [tools/d1_bridge](../tools/d1_bridge/README.md)
4. URDF 会在首次运行时**自动下载**到 `config/d1.urdf`（也可手动 `python ../tools/get_d1_urdf.py`）

### 一键自标定

```bash
conda activate yolo
cd /home/gy/code/calibate/eye_in_hand

export D1_NETWORK_INTERFACE=eth0

# 默认 adaptive：先找棋盘可见的锚点，再小步扰动（避免移出视野）
python auto_calibrate.py --network eth0
```

### 视野保护机制（重要）

旧逻辑会**盲目执行** `config/d1_calibration_poses.json` 中的大角度位姿，容易移出相机视野。

现已默认 **`--pose-mode adaptive`**：

1. **找锚点**：当前位置 → 种子位姿，直到棋盘出现在图像中
2. **小步扰动**：每次相对上一有效位姿，单关节变化 ≤ `--max-joint-delta`（默认 12°）
3. **插值移动**：分 `--interp-steps` 段运动，中途检测棋盘
4. **边缘检测**：棋盘太靠近图像边缘则拒绝（`--margin-ratio 0.06`）
5. **失败回退**：不可见则跳过，并回到上一有效位姿
6. **保存有效位姿** → `data/valid_poses.json`，下次可复用

```bash
# 更保守（更小步长）
python auto_calibrate.py --max-joint-delta 8 --num-poses 15

# 仍用旧版固定位姿列表（带插值+校验）
python auto_calibrate.py --pose-mode fixed

# 用上次采集成功的位姿作为种子
python auto_calibrate.py --poses-file data/valid_poses.json
```

### 分步 / 调试

```bash
# 仅采集数据
python auto_calibrate.py --collect-only --network eth0

# 无真机演示（mock）
python auto_calibrate.py --mock-arm --mock-camera

# 自定义标定位姿（编辑 config/d1_calibration_poses.json）
python auto_calibrate.py --poses-file ../config/d1_calibration_poses.json
```

文档：[D1Arm 服务接口](https://support.unitree.com/home/zh/developer/D1Arm_services)

---

## 采集要求

1. **棋盘格固定**在工作台或支架上，全程不动。
2. **相机固定**在末端法兰/夹爪上。
3. 至少采集 **10～15 组**（最少 3 组）不同位姿，每组包含：
   - 一张彩色图像
   - 对应的机器人 TCP 位姿（base → gripper）
4. 位姿需覆盖足够的 **旋转和平移**，避免共面或运动过小。

## 准备数据

### poses.json 格式

```json
{
  "samples": [
    {
      "image": "0000.png",
      "robot_pose": {
        "position": [0.50, 0.00, 0.40],
        "euler_xyz": [180, 0, 45]
      }
    },
    {
      "image": "0001.png",
      "robot_pose": {
        "position": [0.48, 0.05, 0.42],
        "quaternion": [0, 0.707, 0, 0.707]
      }
    }
  ]
}
```

- `position`：TCP 在基座系下的位置，单位 **米**
- `euler_xyz` 或 `quaternion` 二选一；欧拉角为 **度**，固定轴 X-Y-Z

### 相机内参

**方式 A：RealSense D455 自动读取（推荐）**

确保 D455 已连接，且采集分辨率与内参分辨率一致。无 JSON 时标定会自动读取：

```bash
python calibrate.py --data-dir data
```

**方式 B：手动导出**

```bash
python ../tools/get_d455_intrinsics.py \
  --from-image data/images/0000.png \
  --output data/camera_intrinsics.json
```

## 运行标定

```bash
conda activate yolo
cd /home/gy/code/calibate/eye_in_hand

# 正式标定
python calibrate.py --data-dir data

# 指定棋盘参数（与实物一致）
python calibrate.py --cols 9 --rows 6 --square-size 0.025

# 强制从 D455 重新读取内参
python calibrate.py --auto-camera --realsense-width 640 --realsense-height 480

# 更换手眼算法
python calibrate.py --method park
```

### 常用参数

| 参数 | 说明 | 默认 |
|------|------|------|
| `--data-dir` | 数据目录 | `data` |
| `--poses` | 位姿文件 | `data/poses.json` |
| `--camera` | 内参文件 | `data/camera_intrinsics.json` |
| `--auto-camera` | 强制从 RealSense 读内参 | 无 JSON 时自动 |
| `--output` | 结果输出路径 | `output/T_cam_gripper.json` |
| `--method` | tsai/park/horaud/andreff/daniilidis | `tsai` |
| `--cols / --rows` | 棋盘内角点数 | 9 / 6 |
| `--square-size` | 方格边长（米） | 0.025 |

## 验证标定

```bash
python verify.py --data-dir data --result output/T_cam_gripper.json
```

输出示例：

```
样本数: 12
标定板原点 in base 均值: [0.5  0.  0. ]
位置残差 (m): mean=0.001234, max=0.003456
```

残差在毫米级通常可接受；若 >1 cm，检查位姿单位、棋盘参数、内参分辨率。

## 演示（无需真实机器人）

```bash
python generate_demo_data.py
python calibrate.py
python verify.py
```

## 输出结果

`output/T_cam_gripper.json` 包含 4×4 变换矩阵 `T`、旋转 `R`、平移 `t` 及元信息。

在抓取程序中使用：

```python
# p_gripper = R @ p_cam + t
# 或齐次: p_gripper_h = T @ p_cam_h
```

## 常见问题

| 现象 | 可能原因 |
|------|----------|
| 未检测到棋盘 | 光照不足、棋盘参数错误、图像模糊 |
| 有效样本不足 | 图像或位姿文件缺失 |
| 残差很大 | 位姿单位错误（mm 当 m）、内参分辨率不匹配 |
| 找不到 RealSense | USB 未连接，或多台设备需 `--realsense-serial` |
