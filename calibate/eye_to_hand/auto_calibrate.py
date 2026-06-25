#!/usr/bin/env python3
"""
眼在手外 全自动自标定：相机固定，标定板随 D1 末端运动。

参考: https://support.unitree.com/home/zh/developer/D1Arm_services
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.auto_collect import collect_hand_eye_samples
from common.board import BoardConfig
from common.calib_report import print_calibration_report, show_calibration_result_window


def main() -> None:
    parser = argparse.ArgumentParser(description="D1 + 相机 眼在手外 全自动自标定")
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--poses-file", type=Path, default=ROOT / "config/d1_calibration_poses.json")
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--network", type=str, default=None, help="D1 DDS 网卡；省略则自动检测 192.168.123.x")
    parser.add_argument("--mock-arm", action="store_true")
    parser.add_argument("--mock-camera", action="store_true")
    parser.add_argument("--collect-only", action="store_true")
    parser.add_argument("--skip-board-check", action="store_true")
    parser.add_argument("--pose-mode", choices=["adaptive", "fixed"], default="adaptive")
    parser.add_argument("--num-poses", type=int, default=12)
    parser.add_argument("--max-attempts", type=int, default=40)
    parser.add_argument("--max-joint-delta", type=float, default=12.0)
    parser.add_argument("--interp-steps", type=int, default=3)
    parser.add_argument("--margin-ratio", type=float, default=0.06)
    parser.add_argument("--go-zero-first", action="store_true", help="开始前回零（manual-prep 默认已回零）")
    parser.add_argument(
        "--no-manual-prep",
        action="store_true",
        help="跳过手动摆位阶段（默认：回零→实时画面→Enter 后开始）",
    )
    parser.add_argument(
        "--manual-require-board",
        action="store_true",
        help="手动摆位阶段必须检测到棋盘才允许按 Enter",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument(
        "--camera-type",
        choices=["realsense", "usb"],
        default=None,
        help="相机类型；默认读 config/cameras.json 中 eye_to_hand",
    )
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=ROOT / "config/cameras.json",
        help="双相机配置文件",
    )
    parser.add_argument("--usb-device", type=str, default=None, help="USB 相机索引或 /dev/videoN")
    parser.add_argument("--usb-name-hint", type=str, default=None, help="USB 相机名称匹配")
    parser.add_argument(
        "--intrinsics-file",
        type=Path,
        default=None,
        help="USB 相机内参 JSON",
    )
    parser.add_argument("--realsense-serial", type=str, default=None, help="RealSense 序列号")
    parser.add_argument(
        "--no-auto-bandwidth",
        action="store_true",
        help="禁用 D455 自适应带宽（固定使用 --camera-width/height/fps）",
    )
    parser.add_argument(
        "--no-align-base-yaw",
        action="store_true",
        help="禁用按当前 j0 自动修正 config 种子位姿（默认开启，用于消除现场约 90° 朝向偏差）",
    )
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    parser.add_argument("--method", default="tsai")
    parser.add_argument(
        "--auto-gripper",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="根据棋盘距离与尺寸自动设置 j6 并夹紧（眼在手外默认开启）",
    )
    parser.add_argument(
        "--gripper-config",
        type=Path,
        default=ROOT / "config/d1_gripper.json",
        help="夹爪 j6 与开口宽度映射配置",
    )
    parser.add_argument(
        "--show",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="显示实时调试窗口（视频+关节角）",
    )
    parser.add_argument(
        "--show-result",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="标定完成后显示结果摘要窗口",
    )
    args = parser.parse_args()

    data_dir = args.data_dir.resolve()
    board = BoardConfig(cols=args.cols, rows=args.rows, square_size=args.square_size)

    print("=" * 60)
    print("眼在手外：请将棋盘格置于夹爪可抓取位置（或已夹持）")
    print("默认将根据相机检测的距离与尺寸自动设置 j6")
    pattern = ROOT / "config/patterns/chessboard_9x6_25mm_a4.pdf"
    if pattern.exists():
        print(f"棋盘格模板（9×6，25mm）: {pattern}")
    if not args.no_manual_prep and not args.mock_arm:
        print("流程: 折叠位姿 → 倒计时卸力 → 手动摆位 → Enter 开始（不再跳回 config 种子角）")
    print("=" * 60)

    manual_prep = not args.no_manual_prep and not args.mock_arm and not args.mock_camera
    result = collect_hand_eye_samples(
        data_dir=data_dir,
        calibration_type="eye_to_hand",
        poses_file=args.poses_file,
        mock_arm=args.mock_arm,
        mock_camera=args.mock_camera,
        network_interface=args.network,
        urdf_path=args.urdf,
        board=board,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        camera_fps=args.camera_fps,
        camera_type=args.camera_type,
        camera_config=args.camera_config,
        realsense_serial=args.realsense_serial,
        usb_device=args.usb_device,
        usb_name_hint=args.usb_name_hint,
        intrinsics_file=args.intrinsics_file,
        auto_bandwidth=not args.no_auto_bandwidth,
        align_base_yaw=not args.no_align_base_yaw,
        skip_board_check=args.skip_board_check or args.mock_camera,
        go_zero_first=args.go_zero_first or manual_prep,
        manual_prep=manual_prep,
        manual_require_board=args.manual_require_board,
        pose_mode=args.pose_mode,
        num_poses=args.num_poses,
        max_attempts=args.max_attempts,
        max_joint_delta_deg=args.max_joint_delta,
        interp_steps=args.interp_steps,
        margin_ratio=args.margin_ratio,
        auto_gripper=args.auto_gripper and not args.mock_camera,
        gripper_config=args.gripper_config,
        show_debug=(args.show or manual_prep) and not args.mock_camera,
    )

    if result["meta"]["num_collected"] < 3:
        raise RuntimeError("有效样本不足 3 组")

    if args.collect_only:
        return

    cal_cmd = [
        sys.executable,
        str(Path(__file__).parent / "calibrate.py"),
        "--data-dir",
        str(data_dir),
        "--method",
        args.method,
        "--cols",
        str(args.cols),
        "--rows",
        str(args.rows),
        "--square-size",
        str(args.square_size),
    ]
    subprocess.run(cal_cmd, check=True, cwd=Path(__file__).parent)

    verify_cmd = [
        sys.executable,
        str(Path(__file__).parent / "verify.py"),
        "--data-dir",
        str(data_dir),
    ]
    subprocess.run(verify_cmd, check=True, cwd=Path(__file__).parent)

    result_path = Path(__file__).parent / "output" / "T_cam_base.json"
    report = print_calibration_report(
        result_path=result_path,
        data_dir=data_dir,
        board=board,
        collect_meta=result.get("meta"),
        save=True,
    )
    if args.show_result:
        show_calibration_result_window(report, title="眼在手外 标定结果")
    print(f"\n自标定完成 -> {result_path}")


if __name__ == "__main__":
    main()
