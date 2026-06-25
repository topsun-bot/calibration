#!/usr/bin/env python3
"""
眼在手上 (Eye-in-Hand) 手眼标定。

求解相机相对于末端执行器（法兰/夹爪）的变换 T_cam_gripper。
OpenCV 输入：gripper2base 与 target2cam。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig, detect_board_pose
from common.hand_eye_solve import (
    METHOD_MAP,
    calibrate_fused,
    calibrate_single,
    print_method_comparison,
)
from common.io_utils import load_pose_list, save_calibration_result
from common.realsense import load_or_detect_intrinsics
from common.transforms import pose_to_rt


def _parse_methods(method: str) -> list[str] | None:
    if method == "fused":
        return None
    if "," in method:
        return [m.strip() for m in method.split(",") if m.strip()]
    return None


def sample_to_gripper2base(sample: dict) -> tuple[np.ndarray, np.ndarray]:
    """JSON 中 robot_pose 表示 base->gripper，需取逆得到 gripper->base。"""
    if "gripper2base" in sample:
        gb = sample["gripper2base"]
        R, t = pose_to_rt(gb["position"], quaternion=gb.get("quaternion"), euler_xyz=gb.get("euler_xyz"))
        return R, t
    rp = sample["robot_pose"]
    R_bg, t_bg = pose_to_rt(
        rp["position"],
        quaternion=rp.get("quaternion"),
        euler_xyz=rp.get("euler_xyz"),
    )
    R = R_bg.T
    t = -R @ t_bg
    return R, t


def collect_observations(
    data_dir: Path,
    poses_file: Path,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
) -> tuple[list, list, list, list]:
    samples = load_pose_list(poses_file)
    R_g2b_list, t_g2b_list = [], []
    R_t2c_list, t_t2c_list = [], []
    used = []

    for i, sample in enumerate(samples):
        img_name = sample.get("image", f"{i:04d}.png")
        img_path = data_dir / "images" / img_name
        if not img_path.exists():
            print(f"[跳过] 无图像: {img_path}")
            continue

        pose = detect_board_pose(img_path, K, dist, board)
        if pose is None:
            print(f"[跳过] 未检测到棋盘: {img_path}")
            continue

        R_t2c, t_t2c = pose
        R_g2b, t_g2b = sample_to_gripper2base(sample)

        R_g2b_list.append(R_g2b)
        t_g2b_list.append(t_g2b)
        R_t2c_list.append(R_t2c)
        t_t2c_list.append(t_t2c)
        used.append(img_name)
        print(f"[OK] {img_name}")

    return R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, used


def main() -> None:
    parser = argparse.ArgumentParser(description="眼在手上 手眼标定")
    parser.add_argument("--data-dir", type=Path, default=Path("data"), help="数据目录")
    parser.add_argument("--poses", type=Path, default=None, help="位姿 JSON，默认 data/poses.json")
    parser.add_argument("--camera", type=Path, default=None, help="相机内参 JSON")
    parser.add_argument(
        "--auto-camera",
        action="store_true",
        help="从 RealSense D455 自动读取内参（无 JSON 时默认启用）",
    )
    parser.add_argument("--realsense-serial", type=str, default=None, help="RealSense 序列号")
    parser.add_argument("--realsense-width", type=int, default=None, help="彩色流宽度")
    parser.add_argument("--realsense-height", type=int, default=None, help="彩色流高度")
    parser.add_argument("--realsense-fps", type=int, default=None, help="彩色流帧率")
    parser.add_argument("--output", type=Path, default=Path("output/T_cam_gripper.json"))
    parser.add_argument(
        "--method",
        default="fused",
        help="标定算法: tsai/park/... 或 fused(多算法融合,默认) 或 tsai,park,horaud",
    )
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    args = parser.parse_args()

    data_dir = args.data_dir.resolve()
    poses_file = args.poses or (data_dir / "poses.json")
    camera_file = args.camera or (data_dir / "camera_intrinsics.json")

    if not poses_file.exists():
        raise FileNotFoundError(f"位姿文件不存在: {poses_file}")

    auto_camera = args.auto_camera or not camera_file.exists()
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=auto_camera,
        images_dir=data_dir / "images",
        width=args.realsense_width,
        height=args.realsense_height,
        serial=args.realsense_serial,
        fps=args.realsense_fps,
        save_to=camera_file if auto_camera else None,
    )
    board = BoardConfig(cols=args.cols, rows=args.rows, square_size=args.square_size)

    obs = collect_observations(data_dir, poses_file, K, dist, board)
    R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, used = obs

    if len(R_g2b_list) < 3:
        raise RuntimeError(f"有效样本不足 ({len(R_g2b_list)}), 至少需要 3 组")

    method_arg = args.method.lower()
    use_fused = method_arg == "fused" or "," in method_arg
    method_list = _parse_methods(method_arg)

    if use_fused:
        fused = calibrate_fused(
            R_g2b_list,
            t_g2b_list,
            R_t2c_list,
            t_t2c_list,
            mode="eye_in_hand",
            methods=method_list,
        )
        R_cam2gripper, t_cam2gripper = fused.R, fused.t
        print_method_comparison(fused)
        meta_method = "fused"
        meta_extra = {
            "fusion": {
                "residual_mean_m": fused.residual_mean,
                "residual_max_m": fused.residual_max,
                "methods": [
                    {
                        "name": m.name,
                        "residual_mean_m": m.residual_mean,
                        "residual_max_m": m.residual_max,
                        "weight": m.weight,
                    }
                    for m in fused.methods
                ],
            }
        }
    else:
        if method_arg not in METHOD_MAP:
            raise ValueError(f"未知算法: {method_arg}，可选: {list(METHOD_MAP)} 或 fused")
        R_cam2gripper, t_cam2gripper = calibrate_single(
            R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, method_arg
        )
        meta_method = method_arg
        meta_extra = {}

    out = args.output.resolve()
    save_calibration_result(
        out,
        R_cam2gripper,
        t_cam2gripper,
        meta={
            "type": "eye_in_hand",
            "description": "T_cam_gripper: 相机坐标系到末端法兰坐标系",
            "method": meta_method,
            "num_samples": len(used),
            "images": used,
            **meta_extra,
        },
    )
    print(f"\n标定完成 -> {out}")
    print("T_cam_gripper (4x4):")
    T = np.eye(4)
    T[:3, :3] = R_cam2gripper
    T[:3, 3] = t_cam2gripper.reshape(3)
    print(T)


if __name__ == "__main__":
    main()
