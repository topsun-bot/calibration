# D1 通信桥接工具

Python 通过编译好的可执行文件与 Unitree D1 机械臂通信（DDS）。

接口文档：[D1Arm 服务接口](https://support.unitree.com/home/zh/developer/D1Arm_services)

## 依赖

1. **unitree_sdk2** + CycloneDDS（可自动下载安装，见下）
2. **DDS 消息文件** `ArmString_` / `PubServoInfo_`（自动从 GitHub 下载）
3. PC 与 D1 在同一网段（默认臂 IP `192.168.123.100`）

官方完整 `d1_sdk` 包需向 Unitree 获取；本项目用开源 **unitree_sdk2** + 社区消息定义即可编译 bridge。

## 编译

**推荐（一键，缺 SDK 时自动下载）：**

```bash
sudo apt install -y cmake build-essential git   # 首次需安装

cd calibration
./scripts/build_d1_bridge.sh
```

脚本会：
1. 若 `third_party/d1_sdk` 不存在 → 运行 `calibate/tools/get_d1_sdk.py`
2. 从 GitHub 下载 DDS msg 文件到 `d1_bridge/msg/`
3. clone 并编译 [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) 到 `third_party/d1_sdk`
4. 编译 `d1_send_joints` / `d1_read_joints` / `d1_arm_zero` / `d1_joint_enable`

也可单独安装 SDK：

```bash
python3 calibate/tools/get_d1_sdk.py
export D1_SDK=calibration/third_party/d1_sdk   # 或绝对路径
./scripts/build_d1_bridge.sh
```

若已有官方 SDK，指定路径即可：

```bash
export D1_SDK=/home/unitree/d1_sdk
./scripts/build_d1_bridge.sh
```

**手动编译：**

```bash
export D1_SDK=/path/to/d1_sdk

cd calibration/calibate/tools/d1_bridge
mkdir -p build && cd build
cmake .. -DD1_SDK=$D1_SDK
make -j
```

生成：
- `d1_send_joints` — 发送 7 关节角（度）
- `d1_read_joints` — 读取当前关节角 JSON
- `d1_arm_zero` — 回零
- `d1_joint_enable` — 关节使能/卸力（0=手拖，1=位置保持）
- `d1_arm_release` — 退出 SDK 控制（funcode=7）

## 测试

```bash
# 指定连接 D1 的网卡
export D1_NETWORK_INTERFACE=eth0

./d1_read_joints
./d1_send_joints 0 -30 60 0 30 0 0
./d1_arm_zero
./d1_joint_enable 0   # 卸力，可手拖
./d1_joint_enable 1   # 恢复位置保持
./d1_arm_release      # 退出 SDK 控制
```

## 话题说明

| 话题 | 方向 | 说明 |
|------|------|------|
| `rt/arm_Command` | PC → D1 | JSON 命令，`funcode=2` 多关节，`funcode=5` 使能/卸力 |
| `current_servo_angle` | D1 → PC | 7 个关节当前角度（度） |

`funcode=5` 示例（手动拖拽 / 恢复保持）：

```json
{"seq":4,"address":1,"funcode":5,"data":{"mode":0}}
{"seq":4,"address":1,"funcode":5,"data":{"mode":1}}
```

命令 JSON 示例（与官方 SDK 一致）：

```json
{"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":-60,"angle2":60,"angle3":0,"angle4":30,"angle5":0,"angle6":0}}
```

## 故障排查

1. **ping 192.168.123.100 失败** → 检查网线/网段
2. **d1_read_joints 超时** → 确认 D1 已上电，网卡正确
3. **臂不动** → 在 D1 上运行 `./multiple_joint_angle_control` 唤醒（见官方文档）
4. **编译找不到 msg/** → 确认 `D1_SDK` 路径，或从 [Grasp-with-the-Unitree-D1](https://github.com/chen37058/Grasp-with-the-Unitree-D1) 复制 `src/msg/` 到本目录
5. **路径错误** → 项目在 `calibration/calibate/tools/d1_bridge`
6. **cmake not found** → `sudo apt install -y cmake build-essential`
7. **D1_SDK 目录不存在** → 运行 `./scripts/build_d1_bridge.sh` 会自动下载；或 `python3 calibate/tools/get_d1_sdk.py`
