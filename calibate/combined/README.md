# 眼在手上 + 眼在手外 联合应用

在完成两个标定项目后，本目录提供 **双相机融合定位** 与 **抓取补偿** 能力。

## 适用场景

```
┌─────────────────┐     粗定位      ┌─────────────────┐
│  眼在手外相机    │ ──────────────→ │ 目标位置 (base) │
│  (固定监视)      │                 └────────┬────────┘
└─────────────────┘                          │
                                               ↓ 机器人移动
┌─────────────────┐     精定位      ┌─────────────────┐
│  眼在手上相机    │ ──────────────→ │ 融合 / 抓取修正  │
│  (末端相机)      │                 └─────────────────┘
└─────────────────┘
```

- **手外相机**：全局视野，工件粗定位
- **手上相机**：末端近距离，精定位与抓取修正
- **融合**：加权合并两路估计，提高鲁棒性

## 前置条件

先完成两个标定并生成结果文件：

```
../eye_in_hand/output/T_cam_gripper.json
../eye_to_hand/output/T_cam_base.json
```

## 目录结构

```
combined/
├── hand_eye_fusion.py   # 联合应用主程序 / API
└── cross_check.py       # 标定一致性检查（可选）
```

## 快速演示

使用两个项目的合成演示数据：

```bash
conda activate yolo
cd /home/gy/code/calibate

# 确保已运行过 demo 标定
cd eye_in_hand && python generate_demo_data.py && python calibrate.py && cd ..
cd eye_to_hand && python generate_demo_data.py && python calibrate.py && cd ..

# 联合演示
cd combined
python hand_eye_fusion.py --demo --sample-index 0
```

输出示例：

```
=== 联合应用演示 ===
样本索引: 0
融合后目标位置 (base): [...]
手外相机估计 (base):   [...]
手上相机估计 (base):   [...]
两路分歧 (m): 0.00xxxx
抓取修正量 (gripper): [...]
```

## 命令行参数

```bash
python hand_eye_fusion.py --demo [选项]
```

| 参数 | 说明 | 默认 |
|------|------|------|
| `--demo` | 运行内置演示 | — |
| `--eye-in-hand-result` | 眼在手上标定结果 | `../eye_in_hand/output/T_cam_gripper.json` |
| `--eye-to-hand-result` | 眼在手外标定结果 | `../eye_to_hand/output/T_cam_base.json` |
| `--eye-in-hand-data` | 手上相机数据目录 | `../eye_in_hand/data` |
| `--eye-to-hand-data` | 手外相机数据目录 | `../eye_to_hand/data` |
| `--sample-index` | 演示样本索引 | `0` |

## 集成到抓取程序

```python
import sys
sys.path.insert(0, "/home/gy/code/calibate")

from pathlib import Path
from combined.hand_eye_fusion import HandEyeFusion

fusion = HandEyeFusion(
    Path("eye_in_hand/output/T_cam_gripper.json"),
    Path("eye_to_hand/output/T_cam_base.json"),
)

# robot_pose: 当前 TCP 位姿 (base→gripper)，来自机器人控制器
# R_*, t_*: 棋盘检测得到的 target→cam（可用 common.board.detect_board_pose）

# 1. 手外相机粗定位 → 目标在基座系
T_target_base = fusion.target_in_base_from_eye_to_hand(R_fixed, t_fixed)

# 2. 手上相机精定位 → 目标在基座系
T_target_base = fusion.target_in_base_from_eye_in_hand(robot_pose, R_hand, t_hand)

# 3. 双路融合
result = fusion.fuse_target_position(
    robot_pose,
    R_fixed, t_fixed,   # 手外相机观测
    R_hand, t_hand,     # 手上相机观测
    weight_fixed=0.3,   # 手外权重 30%，手上 70%
)
target_pos = result["position_base"]

# 4. 计算末端抓取补偿量（gripper 系下位移）
offset = fusion.grasp_offset_in_gripper(robot_pose, R_fixed, t_fixed)
```

## 交叉验证（可选）

```bash
python cross_check.py \
  --eye-in-hand ../eye_in_hand/output/T_cam_gripper.json \
  --eye-to-hand ../eye_to_hand/output/T_cam_base.json \
  --poses ../eye_in_hand/data/poses.json
```

说明：该检查验证 `T_base_gripper × T_cam_gripper ≈ T_cam_base` 是否成立，**仅当手外/手上为同一物理相机或同一坐标系迁移时**才有意义。双相机独立安装时，请使用 `fuse_target_position` 做联合定位。

## 典型工作流

1. 手外相机检测目标 → `target_in_base_from_eye_to_hand`
2. 机器人移动到预抓取位
3. 手上相机精拍 → `fuse_target_position` 融合
4. `grasp_offset_in_gripper` 计算末端微调量
5. 执行抓取

## 注意事项

- 两路相机的 **内参、分辨率** 各自独立，需分别标定或使用各自的 D455 设备
- `weight_fixed` 可根据场景调整：手外视野大但精度低时降低其权重
- 融合前确保两路 `target2cam` 来自同一物理目标（同一棋盘角点或同一工件特征）
