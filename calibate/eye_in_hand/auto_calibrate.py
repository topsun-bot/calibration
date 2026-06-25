#!/usr/bin/env python3
"""
眼在手上 全自动自标定：D455 内参 + D1 自动采集 + 标定 + 验证。

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


def main() -> None:
    parser = argparse.ArgumentParser(description="D1 + 相机 眼在手上 全自动自标定")
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--poses-file", type=Path, default=ROOT / "config/d1_calibration_poses.json")
    parser.add_argument("--urdf", type=Path, default=None, help="D1 URDF 路径，默认 config/d1.urdf")
    parser.add_argument("--network", type=str, default=None, help="D1 DDS 网卡，如 eth0")
    parser.add_argument("--mock-arm", action="store_true", help="模拟机械臂（无 D1）")
    parser.add_argument("--mock-camera", action="store_true", help="模拟相机")
    parser.add_argument(
        "--camera-type",
        choices=["realsense", "usb"],
        default=None,
        help="相机类型；默认读 config/cameras.json 中 eye_in_hand",
    )
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=ROOT / "config/cameras.json",
        help="双相机配置文件",
    )
    parser.add_argument("--usb-device", type=str, default=None, help="USB 相机索引或 /dev/videoN")
    parser.add_argument("--usb-name-hint", type=str, default=None, help="USB 相机名称匹配，如 2M")
    parser.add_argument(
        "--intrinsics-file",
        type=Path,
        default=None,
        help="USB 相机内参 JSON（默认 config 中 eye_in_hand.intrinsics_file）",
    )
    parser.add_argument("--realsense-serial", type=str, default=None, help="RealSense 序列号")
    parser.add_argument("--collect-only", action="store_true", help="仅采集，不标定")
    parser.add_argument("--skip-board-check", action="store_true")
    parser.add_argument(
        "--pose-mode",
        choices=["adaptive", "fixed"],
        default="adaptive",
        help="adaptive=锚点+小步扰动(默认); fixed=按 poses 文件顺序",
    )
    parser.add_argument("--num-poses", type=int, default=12, help="adaptive 模式目标样本数")
    parser.add_argument("--max-attempts", type=int, default=40, help="adaptive 模式最大尝试次数")
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=12.0,
        help="相对上一有效位姿的单关节最大变化(度)",
    )
    parser.add_argument("--interp-steps", type=int, default=3, help="关节插值分段数")
    parser.add_argument("--margin-ratio", type=float, default=0.06, help="棋盘距图像边缘最小比例")
    parser.add_argument("--go-zero-first", action="store_true", help="采集前先回零")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    parser.add_argument("--method", default="tsai")
    args = parser.parse_args()

    data_dir = args.data_dir.resolve()
    board = BoardConfig(cols=args.cols, rows=args.rows, square_size=args.square_size)

    print("=" * 60)
    print("步骤 1/3: 自动采集 (相机内参 + D1 位姿 + 图像)")
    print("=" * 60)
    result = collect_hand_eye_samples(
        data_dir=data_dir,
        calibration_type="eye_in_hand",
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
        skip_board_check=args.skip_board_check or args.mock_camera,
        go_zero_first=args.go_zero_first,
        pose_mode=args.pose_mode,
        num_poses=args.num_poses,
        max_attempts=args.max_attempts,
        max_joint_delta_deg=args.max_joint_delta,
        interp_steps=args.interp_steps,
        margin_ratio=args.margin_ratio,
    )

    if result["meta"]["num_collected"] < 3:
        raise RuntimeError("有效样本不足 3 组，请检查棋盘可见性或 poses 配置")

    if args.collect_only:
        print("采集完成 (--collect-only)")
        return

    print("\n" + "=" * 60)
    print("步骤 2/3: 手眼标定")
    print("=" * 60)
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

    print("\n" + "=" * 60)
    print("步骤 3/3: 验证标定")
    print("=" * 60)
    verify_cmd = [
        sys.executable,
        str(Path(__file__).parent / "verify.py"),
        "--data-dir",
        str(data_dir),
    ]
    subprocess.run(verify_cmd, check=True, cwd=Path(__file__).parent)
    print("\n自标定完成 -> output/T_cam_gripper.json")


if __name__ == "__main__":
    main()
