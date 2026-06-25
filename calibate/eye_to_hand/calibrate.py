#!/usr/bin/env python3
"""
眼在手外 (Eye-to-Hand) 手眼标定。

相机固定，求解 T_cam_base（相机相对于机器人基座）。
OpenCV 眼在手外：输入 base2gripper 与 target2cam。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
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
from common.io_utils import (
    compute_intrinsics_hash,
    generate_run_id,
    load_pose_list,
    save_calibration_result,
)
from common.realsense import load_or_detect_intrinsics
from common.transforms import pose_to_rt


def _parse_methods(method: str) -> list[str] | None:
    if method == "fused":
        return None
    if "," in method:
        return [m.strip() for m in method.split(",") if m.strip()]
    return None


def sample_to_base2gripper(sample: dict) -> tuple[np.ndarray, np.ndarray]:
    if "base2gripper" in sample:
        bg = sample["base2gripper"]
        return pose_to_rt(bg["position"], quaternion=bg.get("quaternion"), euler_xyz=bg.get("euler_xyz"))
    rp = sample["robot_pose"]
    return pose_to_rt(
        rp["position"],
        quaternion=rp.get("quaternion"),
        euler_xyz=rp.get("euler_xyz"),
    )


def collect_observations(
    data_dir: Path,
    poses_file: Path,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
):
    samples = load_pose_list(poses_file)
    R_b2g_list, t_b2g_list = [], []
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
        R_b2g, t_b2g = sample_to_base2gripper(sample)

        R_b2g_list.append(R_b2g)
        t_b2g_list.append(t_b2g)
        R_t2c_list.append(R_t2c)
        t_t2c_list.append(t_t2c)
        used.append(img_name)
        print(f"[OK] {img_name}")

    return R_b2g_list, t_b2g_list, R_t2c_list, t_t2c_list, used


def main() -> None:
    parser = argparse.ArgumentParser(description="眼在手外 手眼标定")
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--poses", type=Path, default=None)
    parser.add_argument("--camera", type=Path, default=None)
    parser.add_argument("--auto-camera", action="store_true", help="强制从 RealSense D455 自动读取内参")
    parser.add_argument(
        "--use-saved-intrinsics",
        action="store_true",
        help="仅使用 data/camera_intrinsics.json（离线重算，不读当前相机）",
    )
    parser.add_argument("--realsense-serial", type=str, default=None)
    parser.add_argument("--realsense-width", type=int, default=None)
    parser.add_argument("--realsense-height", type=int, default=None)
    parser.add_argument("--realsense-fps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=Path("output/T_cam_base.json"))
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

    auto_camera = args.auto_camera or not camera_file.exists()
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=auto_camera,
        images_dir=data_dir / "images",
        width=args.realsense_width,
        height=args.realsense_height,
        serial=args.realsense_serial,
        fps=args.realsense_fps,
        save_to=camera_file,
        prefer_live=not args.use_saved_intrinsics,
        use_file_only=args.use_saved_intrinsics,
    )
    board = BoardConfig(cols=args.cols, rows=args.rows, square_size=args.square_size)

    R_b2g_list, t_b2g_list, R_t2c_list, t_t2c_list, used = collect_observations(
        data_dir, poses_file, K, dist, board
    )

    if len(R_b2g_list) < 3:
        raise RuntimeError(f"有效样本不足 ({len(R_b2g_list)}), 至少需要 3 组")

    method_arg = args.method.lower()
    use_fused = method_arg == "fused" or "," in method_arg
    method_list = _parse_methods(method_arg)

    if use_fused:
        fused = calibrate_fused(
            R_b2g_list,
            t_b2g_list,
            R_t2c_list,
            t_t2c_list,
            mode="eye_to_hand",
            methods=method_list,
        )
        R_cam2base, t_cam2base = fused.R, fused.t
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
        try:
            R_cam2base, t_cam2base = calibrate_single(
                R_b2g_list, t_b2g_list, R_t2c_list, t_t2c_list, method_arg
            )
        except cv2.error as exc:
            raise RuntimeError(
                f"标定算法 {method_arg} 求解失败: {exc}。"
                f"建议使用 --method fused 尝试多算法融合。"
            ) from exc
        meta_method = method_arg
        meta_extra = {}

    out = args.output.resolve()
    run_id = generate_run_id()
    intrinsics_hash = compute_intrinsics_hash(camera_file)
    save_calibration_result(
        out,
        R_cam2base,
        t_cam2base,
        meta={
            "type": "eye_to_hand",
            "description": "T_cam_base: 相机坐标系到机器人基座坐标系",
            "method": meta_method,
            "num_samples": len(used),
            "images": used,
            **meta_extra,
        },
        parent_frame="base_link",
        child_frame="camera_link",
        run_id=run_id,
        intrinsics_sha256=intrinsics_hash,
        board_config={
            "cols": board.cols,
            "rows": board.rows,
            "square_size_m": board.square_size,
        },
        hardware_info={
            "robot_model": "Unitree D1",
        },
    )
    print(f"\n标定完成 -> {out}")
    T = np.eye(4)
    T[:3, :3] = R_cam2base
    T[:3, 3] = t_cam2base.reshape(3)
    print("T_cam_base (4x4):")
    print(T)


if __name__ == "__main__":
    main()
