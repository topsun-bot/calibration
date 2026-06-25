# 抓取流水线

`pick_place/` 整合手眼标定与 YOLO 3D 检测，提供 `calibrate` / `pick` / `all` 三个子命令。

入口：`pick_place/run.py`

## 命令

```bash
# 眼在手外自标定
python pick_place/run.py calibrate --network eth0

# 抓取指定类别
python pick_place/run.py pick --class bottle --network eth0

# 标定 + 抓取
python pick_place/run.py all --class cup --network eth0
```

常用参数：

| 参数 | 说明 |
|------|------|
| `--network` | D1 网口（如 `eth0`） |
| `--class` | YOLO 目标类别 |
| `--model` | YOLO 权重路径（默认 `yolo11n.pt`，需自行下载） |
| `--sim` | MuJoCo 仿真模式 |
| `--mock-arm` | Mock 臂（不执行真实运动） |
| `--show` / `--no-show` | 实时调试窗口 |
| `--show-result` / `--no-show-result` | 标定结果弹窗 |

## 配置

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

## 标定结果

### 输出文件

| 输出 | 路径 |
|------|------|
| 外参 JSON | `calibate/eye_to_hand/output/T_cam_base.json` |
| 文本报告 | `calibate/eye_to_hand/output/calibration_report.txt` |
| 可视化图表 | `calibate/eye_to_hand/output/calibration_report.png` |
| 采集数据 | `calibate/eye_to_hand/data/` |

### JSON 格式（v1.0.0）

```json
{
  "format_version": "1.0.0",
  "parent_frame": "base_link",
  "child_frame": "camera_link",
  "transform": {
    "translation": { "x": 0.30, "y": 0.15, "z": 0.60 },
    "rotation": {
      "matrix_3x3": [[0.956, -0.045, 0.290], "..."],
      "quaternion_xyzw": [0.02, 0.14, 0.01, 0.99]
    }
  },
  "board_config": { "cols": 9, "rows": 6, "square_size_m": 0.025 },
  "_units": { "length": "meters", "angle": "radians", "quaternion_convention": "xyzw" }
}
```

新格式向后兼容旧版（仅有 `R`/`t`/`meta` 也可加载）。

### 质量评估

| 等级 | mean 残差 | 建议 |
|------|-----------|------|
| 优秀 | ≤ 5 mm | 可直接抓取 |
| 良好 | ≤ 15 mm | 可用，建议微调 `tcp_offset` |
| 可用 | ≤ 30 mm | 建议增加样本或重标 |
| 较差 | > 30 mm | 检查相机固定与棋盘夹持后重标 |

验证报告包含离群样本检测（IQR）、各样本重投影误差、诊断提示。

### 离线验证

```bash
cd calibate/eye_to_hand
python verify.py --data-dir data --result output/T_cam_base.json
python verify.py --data-dir data --use-saved-intrinsics   # 无相机
```

### ROS / tf2 互操作

```python
from common.io_utils import export_tf2_transform, export_urdf_fragment

tf2_dict = export_tf2_transform("output/T_cam_base.json")
urdf_fragment = export_urdf_fragment("output/T_cam_base.json")
```

## D455 相机内参

RealSense 内参按设备 SN 存储，本项目默认在相机连接时**动态读取**：

| 阶段 | 内参来源 |
|------|----------|
| 标定采集 | 打开 D455 时读取 → 写入 `data/camera_intrinsics.json` |
| 标定求解 / verify | 默认再次从当前 D455 读取 |
| 抓取 3D 检测 | 每帧从深度流 profile 读取 |

```bash
# 离线重算历史数据
cd calibate/eye_to_hand
python calibrate.py --data-dir data --use-saved-intrinsics

# 多台 RealSense 时指定 SN
python calibrate.py --realsense-serial 260722303031

# 手动导出内参
python calibate/tools/get_d455_intrinsics.py --list
```

**换相机注意**：标定图像与内参必须来自同一台相机、同一分辨率。更换相机后需重新标定。

详见 [calibate/eye_to_hand/README.md](../../calibate/eye_to_hand/README.md#相机内参d455-动态读取)。

## 常见问题

1. **未检测到目标**：确认 YOLO 模型含该类别，降低 `--conf`，或换 `--model`。
2. **IK 误差大**：调整 `observe_joints_deg` 使 seed 更接近目标；查看标定验证残差。
3. **标定残差大**：确保相机牢固固定、棋盘夹持无松动，增加 `--num-poses`。查看 `calibration_report.png` 中离群样本（红色柱），移除后重标。
4. **相机被占用**：`pkill -f 'python.*run.py'` 后重试。
5. **放置点校验失败**：确保 `place_position_base` 落在 `place_region` 内。
6. **换了 D455 后精度变差**：重新采集标定数据。
7. **退化标定（旋转多样性不足）**：增加多方向旋转位姿，使旋转轴跨度 ≥ 10°。
8. **万向锁警告**：报告中 pitch≈±90° 时直接参考旋转矩阵，勿依赖欧拉角。

更多眼在手外标定排障见 [calibate/eye_to_hand/README.md](../../calibate/eye_to_hand/README.md)。
