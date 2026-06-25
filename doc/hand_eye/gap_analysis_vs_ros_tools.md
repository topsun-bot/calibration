# 与 easy_handeye2 / MoveIt 差距分析

本文对比本项目 [`calibate/`](../../calibate/) 手眼标定实现与两类外部工具的定位差异，并给出优化优先级。

## 三者定位

| 工具 | 本质 |
|------|------|
| [MoveIt](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) | 运动规划框架：运动学、规划、场景监控、轨迹处理，**不是**手眼标定工具 |
| [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) | ROS2 手眼标定中间件：通过 `tf` 采样 → OpenCV 求解 → 发布 `tf` |
| 本项目 `calibate` | D1 专用离线标定流水线：采集 → 求解 → JSON 结果 → `pick_place` 直接读取 |

主要对标对象是 **easy_handeye2**；MoveIt 的价值在于标定位姿如何安全、多样地采集，以及标定结果如何进入规划链。

## 当前实现的优势（应保留）

### 1. D1 全流程自动化

[`auto_collect.py`](../../calibate/common/auto_collect.py) 提供真机专用流程，超出通用 handeye 包范围：

- 折叠位姿 → 倒计时卸力 → 手动摆位 → Enter 开始
- adaptive 模式：锚点 + 小步关节扰动，避免棋盘出视野
- 眼在手外：根据棋盘距离自动设置 j6 并夹紧
- 插值运动 + 逐段可见性校验
- 退出时安全回折叠位姿

### 2. 多算法融合求解

[`hand_eye_solve.py`](../../calibate/common/hand_eye_solve.py) 支持 Tsai、Park、Horaud、Andreff、Daniilidis 五种 OpenCV 算法，并可按残差加权融合（`fused` 模式）。easy_handeye2 默认使用单一算法。

### 3. 双相机联合应用

[`combined/hand_eye_fusion.py`](../../calibate/combined/hand_eye_fusion.py) 支持手外粗定位 + 手上精定位融合，是项目差异化能力，不在 easy_handeye2 设计范围内。

### 4. 无 ROS 依赖

conda + OpenCV + D1 SDK 即可运行，与 [`pick_place/`](../../pick_place/) JSON 流水线直接对接，部署成本低。

### 5. 采集质量控制与验证报告

- 棋盘边缘裕量检测、跳过出视野样本
- [`calib_report.py`](../../calibate/common/calib_report.py) 提供残差统计与质量评级（优秀/良好/可用/较差）

## 相对 easy_handeye2 的主要不足

| 维度 | easy_handeye2 | 本项目 |
|------|---------------|--------|
| 坐标系集成 | `tf` 树，`robot_base` / `ee` / `optical` / `marker` 四帧 | 静态 JSON（`T_cam_base`、`T_cam_gripper`） |
| 在线热更新 | `publish.launch` 启动即发布，重标定后重启节点 | 需手动替换 JSON 或重启应用 |
| 机器人位姿 | `robot_state_publisher` 发布的 `tf` | URDF FK + D1 关节读数 |
| 视觉跟踪 | 任意能发 `tf` 的系统（常配合 ArUco） | 仅棋盘格 OpenCV 角点 |
| 标定 GUI | 逐样本确认/丢弃、RViz 可视化 | `debug_display` 调试面板，非标准标定 GUI |
| 多标定管理 | 命名空间（`my_eob_calib` / `my_eih_calib`） | 结果散落在 `eye_*/output/` |
| 时间同步 | 同一时刻采 robot `tf` + marker `tf` | 到位 → 等待 → 拍照 → 读关节（无时间戳对齐） |

### 影响

- 无法与 ROS 感知/规划节点直接拼 `tf` 链路
- URDF 与真机偏差、关节零点误差会直接进入手外参（easy_handeye2 文档同样强调需标定机器人）
- 大视角、光照差场景下棋盘不如 ArUco 鲁棒

## 相对 MoveIt 生态的不足

MoveIt 提供运动学、碰撞感知规划、Planning Scene Monitor 等能力。本项目当前：

- 采姿仅在关节空间小步扰动（[`pose_planner.py`](../../calibate/common/pose_planner.py)），**不考虑碰撞**（桌面、相机架、自身）
- **不保证旋转充分**（Tsai 算法建议各轴尽量大角度旋转）
- 标定结果不进 `tf` 树，MoveIt 无法直接用于「相机系 → 基座系」变换
- 无 `move_group` 规划的无碰撞多样化采样本

## 架构差异示意

```mermaid
flowchart LR
    subgraph current [当前实现]
        A1[D1 SDK] --> B1[URDF FK]
        C1[OpenCV 棋盘] --> D1[JSON 标定结果]
        D1 --> E1[pick_place 直接读]
    end

    subgraph ros_stack [MoveIt + easy_handeye2 典型链路]
        F1[joint_states] --> G1[robot_state_publisher]
        G1 --> H1[tf tree]
        I1[相机/ArUco] --> H1
        H1 --> J1[easy_handeye2]
        J1 --> K1[tf 发布标定]
        K1 --> L1[MoveIt Planning Scene]
    end
```

## 优化优先级（P0–P3）

| 级别 | 改进项 | 说明 | 状态 |
|------|--------|------|------|
| **P0** | USB 内参标定 | 2M 相机正式标定前必须完成；见 [dual_camera_setup.md](dual_camera_setup.md) | 待完成 |
| **P0** | URDF/TCP 校验 | 验证 [`config/d1.urdf`](../../calibate/config/d1.urdf) 末端与真实夹爪一致；FK 误差比换算法影响更大 | 待验证 |
| **P1** | 采姿旋转多样性 | 增大各轴旋转幅度，记录被跳过样本的关节分布 | ✅ 已实现 `score_pose_diversity` |
| **P1** | 标定结果版本化 | SN、时间戳、残差、采集 meta 写入结果 JSON | ✅ 已实现 (format v1.0.0) |
| **P1** | 非线性精修 | Levenberg-Marquardt 优化融合结果 | ✅ 已实现 `_refine_nonlinear` |
| **P1** | eye-to-hand 算法修正 | OpenCV `calibrateHandEye` 正确处理 eye-to-hand 取逆 | ✅ 已修复 |
| **P1** | 异常算法剔除 | MAD-based 离群值检测，自动移除残差异常算法 | ✅ 已实现 |
| **P2** | ArUco/ChArUco 后端 | 在 [`board.py`](../../calibate/common/board.py) 扩展第二检测后端 | ✅ 已实现 |
| **P2** | 轻量 tf 发布桥 | YAML 导出兼容 ROS2 `static_transform_publisher` | ✅ 已实现 `export_static_transform_yaml` |
| **P2** | 重投影误差评估 | 逐样本重投影误差 + LOO 交叉验证 | ✅ 已实现 |
| **P3** | MoveIt 自动采姿 | 上 ROS 后用 `move_group` 规划无碰撞位姿 | 见路线图 Phase 2 |

## 借鉴 MoveIt2 / easy_handeye2 后的新增能力

### 来自 MoveIt2 的借鉴

| MoveIt2 能力 | 本项目对应实现 |
|-------------|---------------|
| 运动规划多样性保证 | `score_pose_diversity()` 评分函数 + 采集时自动偏好高多样性位姿 |
| 碰撞感知路径约束 | 工作空间包围盒约束 + 关节限位校验（轻量替代） |
| 轨迹时间参数化 | 插值步数 + settle 等待 |
| Planning Scene Monitor | 调试窗口实时坐标系轴 + 工作空间可视化 |
| 约束类型（位置/方向/可见性） | 棋盘边缘裕量 + 法线夹角检测 + Z>0 可见性约束 |

### 来自 easy_handeye2 的借鉴

| easy_handeye2 能力 | 本项目对应实现 |
|-------------------|---------------|
| tf2 发布标定结果 | `export_static_transform_yaml()` 输出兼容 ROS2 YAML |
| 四帧 tf 语义 | JSON v1.0 含 `parent_frame`/`child_frame` + `export_urdf_fragment()` |
| 标定评估器 (rqt) | `evaluate_reprojection_error()` + `leave_one_out_cv()` |
| 多标定命名空间 | `run_id` 唯一标识 + 结果 JSON 含完整采集元数据 |
| ArUco 跟踪支持 | ChArUco 检测后端（`board_type="charuco"`） |
| 逐样本确认/丢弃 | `debug_display` 实时画面 + 插值可见性校验自动丢弃 |
| Tsai 建议最佳实践 | 运动多样性检测 + 退化警告 + 采姿多样性评分 |

### 超越两者的独有能力

| 能力 | 说明 |
|------|------|
| **多算法融合 + 非线性精修** | 5 种 OpenCV 算法 softmax 加权融合 + L-M 优化，优于 easy_handeye2 单算法 |
| **仿真验证 (GT 对比)** | MuJoCo 仿真标定精度达 0.001°/0.00mm，可回归验证算法正确性 |
| **D1 全自动采集** | 锚点自适应 + 夹爪自动 + 折叠安全位姿，无需人工干预 |
| **双相机融合** | 手外粗定位 + 手上精定位，不在 MoveIt/easy_handeye2 设计范围内 |
| **异常算法/样本检测** | MAD-based 异常剔除 + 离群帧标记 + IQR 箱线图 |
| **离线无 ROS 部署** | conda + OpenCV + D1 SDK 即运行，零 ROS 基础设施开销 |

## 结论

本项目在 **D1 真机自动化、双相机融合、多算法融合精修、离线批处理、仿真验证** 上优于 easy_handeye2；
在 **标准化互操作（tf/ROS）、在线热更新、MoveIt 碰撞感知采姿** 上仍有不足，但已通过 YAML 导出和多样性评分部分弥补。

当前实现已覆盖 easy_handeye2 的核心技术（多算法 + 评估 + 导出），且在精度（非线性精修）和鲁棒性（异常检测 + 融合）上超越。
若继续走无 ROS 的 `pick_place` 路线，P0/P1 已基本完成；若未来上 ROS2 + MoveIt，保留 `calibate` 核心仅加薄桥接层，详见 [ros2_moveit_roadmap.md](ros2_moveit_roadmap.md)。
