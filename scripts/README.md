# 脚本说明

自动化测试、摄像头检查与演示录制脚本。

## check_cameras.py

检测所有可用摄像头（USB V4L2 + RealSense），验证取帧能力。

```bash
python scripts/check_cameras.py            # 检测报告
python scripts/check_cameras.py --save     # 保存测试帧到 output/
```

输出示例：
```
[1] USB V4L2 相机:
   video10: 2M: 2M          640x480@30fps
[2] Intel RealSense:
   Intel RealSense D435I  SN=346122070681  USB=2.1
   → V4L2 回退 (video8): ✅

✅ 双摄像头就绪，可执行手眼标定
```

## run_sim_tests.sh

安装依赖、确保 D1 URDF 存在、设置 `MUJOCO_GL=egl`，运行全部 pytest。

```bash
./scripts/run_sim_tests.sh
./scripts/run_sim_tests.sh tests/test_sim_pipeline.py   # 仅跑指定测试
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

## check_realsense.py

快速验证 RealSense 连接状态（需 pyrealsense2）。

```bash
python scripts/check_realsense.py
```

## Docker 内运行

```bash
docker compose run --rm test                    # 全部测试
docker run --rm calibration -m pytest tests/ -q # 等效
```
