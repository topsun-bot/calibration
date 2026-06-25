# 快速开始

## 环境

```bash
conda activate yolo
cd calibration   # 仓库根目录

pip install -r get_object/requirements.txt
pip install -r requirements-dev.txt   # pytest、mujoco（仿真/测试）
```

真机使用 D1 机械臂时，还需编译通信桥接：

```bash
./scripts/build_d1_bridge.sh
```

详见 [calibate/tools/d1_bridge/README.md](../../calibate/tools/d1_bridge/README.md)。

## 硬件布置

1. **RealSense D455**（或 D435I）固定在工作空间外部，全程不动
2. **Unitree D1** 机械臂，网口连接（如 `eth0`，默认 IP `192.168.123.100`）
3. 工作台上放置待抓取物体；在 `pick_place/config/pick_place.json` 中设定放置区域

双相机配置（D435I + USB）见 [doc/hand_eye/dual_camera_setup.md](../hand_eye/dual_camera_setup.md)。

## 眼在手外自标定

棋盘格夹在 D1 夹爪上，相机固定不动：

```bash
python pick_place/run.py calibrate --network eth0
```

也可直接使用标定模块：

```bash
cd calibate/eye_to_hand
python auto_calibrate.py --network eth0
```

标定完成后输出：

| 输出 | 路径 |
|------|------|
| 外参 JSON（v1.0.0） | `calibate/eye_to_hand/output/T_cam_base.json` |
| 文本报告 | `calibate/eye_to_hand/output/calibration_report.txt` |
| 可视化图表 | `calibate/eye_to_hand/output/calibration_report.png` |

## 抓取放置

编辑 `pick_place/config/pick_place.json` 中的 `place_position_base` 与 `place_region`，然后：

```bash
python pick_place/run.py pick --class bottle --network eth0
```

一步完成（标定 + 抓取）：

```bash
python pick_place/run.py all --class cup --network eth0
```

## 调试窗口

默认开启实时调试窗口（`--show`），标定结束弹出结果摘要（`--show-result`）。

```bash
# 关闭实时窗口
python pick_place/run.py calibrate --network eth0 --no-show

# 关闭标定结果弹窗（终端仍有完整报告）
python pick_place/run.py calibrate --network eth0 --no-show-result

# 抓取时附加深度图
python pick_place/run.py pick --class bottle --network eth0 --show-depth
```

调试窗口中按 `q` 关闭实时画面；标定结果窗口按任意键关闭。

## 仿真模式

无真机时可用 MuJoCo 验证完整流程：

```bash
python pick_place/run.py calibrate --sim
python pick_place/run.py all --class bottle --sim --no-show
```

详见 [simulation.md](simulation.md)。

## 单独运行子模块

```bash
# 仅 3D 检测预览
cd get_object && python yolo3d/run.py

# 仅手眼标定
cd calibate/eye_to_hand && python auto_calibrate.py --network eth0
```
