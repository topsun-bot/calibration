# 双相机配置与内参

本文档说明当前开发机上两台 USB 相机如何接入手眼标定流程。

## 硬件角色分工

| 标定类型 | 角色 | 设备 | 安装位置 |
|----------|------|------|----------|
| 眼在手外（eye_to_hand） | 固定监视相机 | Intel RealSense D435I（SN `346122070681`） | 工作空间外部三脚架 |
| 眼在手上（eye_in_hand） | 末端相机 | 2M USB（`/dev/video10`） | 机械臂末端 |

配置文件：[`calibate/config/cameras.json`](../../calibate/config/cameras.json)

```json
{
  "eye_to_hand": {
    "type": "realsense",
    "model_hint": "D435",
    "serial": "346122070681",
    "width": 640,
    "height": 480,
    "fps": 30,
    "auto_bandwidth": true
  },
  "eye_in_hand": {
    "type": "usb",
    "device": 10,
    "name_hint": "2M",
    "width": 1280,
    "height": 720,
    "fps": 30,
    "intrinsics_file": "eye_in_hand/data/camera_intrinsics.json",
    "warmup_frames": 8
  }
}
```

## 统一相机接口

标定采集通过 [`calibate/common/calib_camera.py`](../../calibate/common/calib_camera.py) 中的 `create_calib_camera()` 创建相机实例，屏蔽 RealSense 与 USB V4L2 的差异：

| 类型 | 实现类 | 内参来源 |
|------|--------|----------|
| `realsense` | `RealSenseCapture` | SDK 动态读取（含 SN） |
| `usb` / `v4l2` | `USBCameraCapture` | 预标定 JSON 文件 |

`auto_collect.py` 根据 `calibration_type`（`eye_in_hand` / `eye_to_hand`）自动读取 `cameras.json` 中对应配置，也可通过 CLI 覆盖。

## 设备枚举

```bash
conda activate yolo
cd calibration/calibate
python tools/list_cameras.py
```

输出示例：

- **RealSense**：设备名、SN、USB 路径
- **USB V4L2**（不含 RealSense 节点）：`videoN`、分辨率、设备名
- **推荐配置**：`cameras.json` 中的 `eye_to_hand` / `eye_in_hand` 映射

## 内参前置条件（P0）

### RealSense D435I（眼在手外）

- 内参由 SDK 出厂标定，采集时自动写入 `data/camera_intrinsics.json`
- 换机后 SN 变化会自动刷新
- 当前设备接在 USB 2.1 时首帧可能较慢，建议换 USB 3.0 口

### 2M USB（眼在手上）

- **无出厂内参**，必须先运行棋盘格内参标定
- 当前 [`eye_in_hand/data/camera_intrinsics.json`](../../calibate/eye_in_hand/data/camera_intrinsics.json) 仍为旧 D455 数据，**正式标定前必须更新**

```bash
conda activate yolo
cd calibration/calibate

python tools/calibrate_usb_intrinsics.py \
  --device 10 \
  --width 1280 \
  --height 720 \
  --output eye_in_hand/data/camera_intrinsics.json
```

标定过程中将 9×6 棋盘格置于相机视野内，程序采集约 20 张图像后输出内参 JSON。分辨率须与 `cameras.json` 中 `eye_in_hand` 一致。

## 标定命令

### 眼在手外（RealSense 固定）

```bash
cd calibate/eye_to_hand
python auto_calibrate.py --network eth0
```

默认从 `cameras.json` 读取 `eye_to_hand` 配置。显式指定：

```bash
python auto_calibrate.py --network eth0 \
  --camera-type realsense \
  --realsense-serial 346122070681
```

### 眼在手上（2M USB 末端）

```bash
cd calibate/eye_in_hand
python auto_calibrate.py --network eth0
```

默认从 `cameras.json` 读取 `eye_in_hand` 配置。显式指定：

```bash
python auto_calibrate.py --network eth0 \
  --camera-type usb \
  --usb-device 10 \
  --camera-width 1280 \
  --camera-height 720 \
  --intrinsics-file data/camera_intrinsics.json
```

## CLI 参数一览

| 参数 | 说明 | 默认 |
|------|------|------|
| `--camera-type` | `realsense` 或 `usb` | 读 `cameras.json` |
| `--camera-config` | 双相机配置文件路径 | `config/cameras.json` |
| `--realsense-serial` | RealSense SN | config 或自动选择 |
| `--usb-device` | USB 索引或 `/dev/videoN` | config 或名称匹配 |
| `--usb-name-hint` | sysfs 名称匹配（如 `2M`） | config |
| `--intrinsics-file` | USB 内参 JSON | config |
| `--camera-width/height/fps` | 采集分辨率 | config |

## 测试

自动化测试覆盖相机工厂与 USB 枚举，见 [`tests/test_calib_camera.py`](../../tests/test_calib_camera.py)：

```bash
conda activate yolo
cd calibration
python -m pytest tests/test_calib_camera.py -v
```

## 常见问题

1. **USB 相机打不开**：确认用户在 `video` 组；用 `list_cameras.py` 确认可用 `video` 索引（metadata 节点如 `video11` 不可采集）。
2. **内参分辨率不一致**：USB 内参 JSON 的 `width`/`height` 须与采集流一致，否则棋盘位姿估计会有系统误差。
3. **同时接两台相机**：RealSense 走 SDK，2M 走 V4L2，互不冲突；`cameras.json` 已按角色分开配置。
