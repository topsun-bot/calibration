# 公共模块 (common)

标定三个项目共用的底层库，一般无需直接运行，供 `eye_in_hand`、`eye_to_hand`、`combined` 调用。

## 模块说明

| 文件 | 功能 |
|------|------|
| `board.py` | 棋盘格配置、角点检测、`target→camera` 位姿估计 |
| `realsense.py` | RealSense D455 内参动态读取、SN 校验、分辨率推断 |
| `transforms.py` | 位姿格式转换（欧拉角/四元数 ↔ R,t）、齐次变换 |
| `io_utils.py` | `poses.json` 读取、标定结果 JSON 读写 |
| `calib_report.py` | 标定验证、结果报告格式化与保存 |
| `debug_display.py` | 实时调试窗口（视频+关节角+状态） |
| `auto_collect.py` | D1 自动采集（adaptive 视野保护） |
| `gripper_auto.py` | 眼在手外自动夹爪 j6 |

## 主要 API

### 棋盘检测

```python
from common.board import BoardConfig, detect_board_pose

board = BoardConfig(cols=9, rows=6, square_size=0.025)
R, t = detect_board_pose("image.png", K, dist_coeffs, board)
# R, t: target → camera
```

### RealSense 内参

```python
from pathlib import Path
from common.realsense import (
    get_d455_color_intrinsics,
    is_realsense_available,
    load_or_detect_intrinsics,
)

# 默认：有 D455 时从当前设备动态读取（含 SN），并写回 JSON
K, dist, meta = load_or_detect_intrinsics(
    camera_file=Path("data/camera_intrinsics.json"),
    images_dir=Path("data/images"),
    prefer_live=True,
    use_file_only=False,
)

# 离线重算：仅用采集时的 JSON
K, dist, meta = load_or_detect_intrinsics(
    camera_file=Path("data/camera_intrinsics.json"),
    use_file_only=True,
    prefer_live=False,
)
```

| 参数 | 说明 |
|------|------|
| `prefer_live` | 有 RealSense 时优先读当前设备（默认 `True`） |
| `use_file_only` | 仅读 JSON，对应 CLI `--use-saved-intrinsics` |
| `serial` | 多台设备时指定 SN |

### 位姿转换

```python
from common.transforms import pose_to_rt, rt_to_homogeneous

R, t = pose_to_rt([0.5, 0, 0.4], euler_xyz=[180, 0, 45])  # base→gripper
T = rt_to_homogeneous(R, t)
```

### 标定结果报告

```python
from common.calib_report import print_calibration_report

report = print_calibration_report(
    result_path=Path("eye_to_hand/output/T_cam_base.json"),
    data_dir=Path("eye_to_hand/data"),
    save=True,
    use_file_only=False,  # True：离线，仅用 JSON 内参
)
# report.quality: 优秀 / 良好 / 可用 / 较差
```

## 数据约定

- `robot_pose.position`：米
- `robot_pose.euler_xyz`：度（固定轴 X-Y-Z）
- `robot_pose.quaternion`：`[x, y, z, w]`
- 标定板 `square_size`：米

## 扩展

若需支持 ArUco 板或其它标定板，可在 `board.py` 中扩展 `detect_board_pose`，或在各项目的 `calibrate.py` 中替换检测逻辑，手眼求解部分无需改动。
