# 辅助工具

## get_d455_intrinsics.py

从已连接的 **Intel RealSense D455**（或其它 RealSense）读取彩色相机内参，保存为标定程序使用的 JSON 格式。

### 环境

```bash
conda activate yolo
# 需要 pyrealsense2，D455 通过 USB 连接
```

### 列出设备

```bash
cd /home/gy/code/goat_demo/calibate
python tools/get_d455_intrinsics.py --list
```

输出示例：

```
Intel RealSense D455  SN=260722303031  USB=/sys/devices/...
```

### 导出内参

**指定分辨率：**

```bash
python tools/get_d455_intrinsics.py \
  --width 640 --height 480 \
  --output eye_in_hand/data/camera_intrinsics.json
```

**根据已有图像自动匹配分辨率（推荐）：**

```bash
python tools/get_d455_intrinsics.py \
  --from-image eye_in_hand/data/images/0000.png \
  --output eye_in_hand/data/camera_intrinsics.json
```

**多台 RealSense 时指定序列号：**

```bash
python tools/get_d455_intrinsics.py --serial 260722303031 --width 640 --height 480
```

### 参数说明

| 参数 | 说明 |
|------|------|
| `--list` | 列出已连接 RealSense 设备 |
| `--serial` | 设备序列号 |
| `--width / --height` | 彩色流分辨率 |
| `--fps` | 优先匹配的帧率 |
| `--from-image` | 从图像推断 width/height |
| `--output` | 输出 JSON 路径，默认 `camera_intrinsics.json` |

### 输出格式

```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [k1, k2, p1, p2, k3],
  "meta": {
    "source": "realsense",
    "device_name": "Intel RealSense D455",
    "serial": "...",
    "width": 640,
    "height": 480
  }
}
```

### 注意事项

1. **分辨率必须与采集图像一致**，否则棋盘位姿估计会有误差。
2. D455 常用彩色分辨率：640×480、848×480、1280×720、1280×800 等。
3. 标定脚本（`calibrate.py`）在无 JSON 时会自动调用相同逻辑；本工具用于提前导出或检查内参。

### 与标定脚本的关系

| 场景 | 做法 |
|------|------|
| 真机标定（默认） | 连接 D455 后直接 `python calibrate.py`，**自动动态读取**当前设备内参并写回 JSON |
| 提前导出 / 检查 | 运行本工具 → 生成 `data/camera_intrinsics.json`（含 SN） |
| 强制刷新 | `python calibrate.py --auto-camera` |
| 离线重算历史数据 | `python calibrate.py --use-saved-intrinsics`（不读当前相机） |
| 换机后 | 重新采集标定，或确认 JSON 中 `meta.serial` 与当前 D455 一致 |

动态读取逻辑见 `common/realsense.py` 中 `load_or_detect_intrinsics(prefer_live=True)`。

---

## get_d1_urdf.py

自动下载 Unitree D1 官方 URDF 到 `config/d1.urdf`。

来源：[Unitree 官方 URDF 包](https://oss-global-cdn.unitree.com/static/9b20252a26374d50aa369532657d0143.zip)（`d1_550_description`）

```bash
cd /home/gy/code/goat_demo/calibate
python tools/get_d1_urdf.py
python tools/get_d1_urdf.py --force   # 强制重新下载
```

也可设置 `D1_SDK=/path/to/d1_sdk`，优先从本地 SDK 复制 URDF。

**自动触发：** 运行 `auto_calibrate.py` 或 `load_d1_fk()` 时，若 `config/d1.urdf` 不存在会自动下载。

---

## build_d1_bridge.sh / d1_bridge

真机标定/抓取前需编译 D1 通信桥接：

```bash
sudo apt install -y cmake build-essential
export D1_SDK=/home/unitree/d1_sdk    # 官方 SDK 解压路径

cd /home/gy/code/goat_demo
./scripts/build_d1_bridge.sh
```

生成 `calibate/tools/d1_bridge/build/d1_send_joints` 等。详见 [d1_bridge/README.md](d1_bridge/README.md)。

无真机时可跳过编译，使用 `--mock-arm` 或 `--sim`。

