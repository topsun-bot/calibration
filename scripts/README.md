# 脚本说明

眼在手外流水线相关的自动化脚本。仿真与测试详情见 [doc/user/simulation.md](../doc/user/simulation.md)。

## run_sim_tests.sh

安装依赖、确保 D1 URDF 存在、设置 `MUJOCO_GL=egl`，运行全部 pytest。

```bash
./scripts/run_sim_tests.sh
./scripts/run_sim_tests.sh tests/test_eye_to_hand.py   # 仅跑指定测试
```

## run_eye_to_hand_demo.sh

依次执行：眼在手外相关 pytest → 录制演示视频。

```bash
./scripts/run_eye_to_hand_demo.sh
```

输出：`output/eye_to_hand_demo.mp4`

## record_eye_to_hand_demo.py

MuJoCo 仿真下录制完整流程视频（标定采集 → 报告 → 抓取放置）。

```bash
python scripts/record_eye_to_hand_demo.py
python scripts/record_eye_to_hand_demo.py -o output/my_demo.mp4 --poses 8 --class bottle
```

| 参数 | 说明 |
|------|------|
| `-o / --output` | 输出 MP4 路径 |
| `--poses` | 演示用标定采集帧数 |
| `--class` | 抓取目标类别 |

视频为双画面：侧视全景（overview_cam）+ 固定相机（眼在手外 fixed_cam）。

Phase 3 抓取阶段：瓶子为 mocap 运动学体，夹爪闭合（j6 > -20°）后自动附着并随臂移动至放置区，松开夹爪后释放。录制脚本会校验 `bottle attached OK` 后才输出视频。
