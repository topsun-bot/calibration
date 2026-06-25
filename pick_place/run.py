#!/usr/bin/env python3
"""
眼在手外自标定 + 指定类别物体抓取放置 一体化入口。

典型用法:
  # 1. 仅自标定
  python pick_place/run.py calibrate --network eth0

  # 2. 仅抓取放置（需已有 T_cam_base.json）
  python pick_place/run.py pick --class bottle --network eth0

  # 3. 标定完成后立即抓取
  python pick_place/run.py all --class cup --network eth0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))

from pick_place.controller import (  # noqa: E402
    DEFAULT_CALIB,
    PickPlaceConfig,
    PickPlaceController,
    run_eye_to_hand_calibration,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="眼在手外自标定 + YOLO 3D 抓取放置",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--network", type=str, default=None, help="D1 网卡，如 eth0")
    common.add_argument("--mock-arm", action="store_true", help="仿真机械臂（无真机）")
    common.add_argument("--mock-camera", action="store_true", help="标定时使用 mock 相机")
    common.add_argument(
        "--calib",
        type=Path,
        default=DEFAULT_CALIB,
        help=f"手眼标定结果路径 (默认 {DEFAULT_CALIB})",
    )
    common.add_argument(
        "--config",
        type=Path,
        default=None,
        help="pick_place 配置文件",
    )
    common.add_argument("--urdf", type=Path, default=None, help="D1 URDF 路径")
    common.add_argument("--model", default="yolo11n.pt", help="YOLO 权重")
    common.add_argument("--conf", type=float, default=0.5, help="YOLO 置信度阈值")
    common.add_argument("--device", default=None, help="YOLO 推理设备 cpu/cuda/0")
    common.add_argument(
        "--show",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="显示实时调试窗口（视频+关节角+状态）",
    )
    common.add_argument(
        "--show-depth",
        action="store_true",
        help="调试窗口附加深度图",
    )
    common.add_argument(
        "--show-result",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="标定完成后显示结果摘要窗口",
    )
    common.add_argument(
        "--sim",
        action="store_true",
        help="MuJoCo 仿真模式（无需真机/RealSense）",
    )

    p_cal = sub.add_parser("calibrate", parents=[common], help="眼在手外自标定")
    p_cal.add_argument("--data-dir", type=Path, default=None, help="标定数据目录")

    p_pick = sub.add_parser("pick", parents=[common], help="检测并抓取放置")
    p_pick.add_argument(
        "--class",
        dest="target_class",
        required=True,
        help="目标类别名（YOLO class，如 bottle / cup）",
    )

    p_all = sub.add_parser("all", parents=[common], help="自标定 + 抓取放置")
    p_all.add_argument(
        "--class",
        dest="target_class",
        required=True,
        help="目标类别名",
    )
    p_all.add_argument("--data-dir", type=Path, default=None, help="标定数据目录")

    return parser


def main() -> int:
    args = build_parser().parse_args()
    calib_path = args.calib.resolve()

    if args.command in ("calibrate", "all"):
        calib_path = run_eye_to_hand_calibration(
            network=args.network,
            mock_arm=args.mock_arm,
            mock_camera=args.mock_camera,
            data_dir=args.data_dir,
            show_debug=args.show,
            show_result=args.show_result,
            sim=args.sim,
        )
        print(f"[标定] 结果: {calib_path}")
        if args.command == "all" and not args.mock_arm and not args.sim:
            print("\n[提示] 标定完成后请取下夹爪上的棋盘格，按 Enter 继续抓取...")
            try:
                input()
            except EOFError:
                pass

    if args.command == "calibrate":
        return 0

    if not calib_path.exists():
        print(
            f"未找到标定文件: {calib_path}\n"
            "请先运行: python pick_place/run.py calibrate --network eth0",
            file=sys.stderr,
        )
        return 1

    cfg = PickPlaceConfig.load(args.config) if args.config else None
    sim_env = None
    if args.sim:
        sys.path.insert(0, str(ROOT))
        from sim.mujoco_env import create_sim_env

        sim_env = create_sim_env(headless=True)
    controller = PickPlaceController(
        calib_path=calib_path,
        cfg=cfg,
        network=args.network,
        mock_arm=args.mock_arm or args.sim,
        urdf_path=args.urdf,
        yolo_model=args.model,
        yolo_conf=args.conf,
        yolo_device=args.device,
        show_debug=args.show and not args.sim,
        show_depth=args.show_depth,
        sim_env=sim_env,
    )
    try:
        controller.pick_and_place(args.target_class)
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
