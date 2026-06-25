#!/usr/bin/env python3
"""生成眼在手外标定演示数据（与 D1 FK + 固定 T_cam_base 自洽）。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.auto_collect import DEFAULT_MOCK_T_CAM_BASE
from common.board import BoardConfig, detect_board_pose_from_image
from common.d1_fk import joint_angles_to_robot_pose, load_d1_fk
from common.synthetic_board import render_board_for_robot_pose


def main() -> None:
    out = Path(__file__).parent / "data"
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    board = BoardConfig()
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5)
    fk = load_d1_fk()
    T_cam = DEFAULT_MOCK_T_CAM_BASE

    seed_poses = [
        [0, -30, 60, 0, 30, 0, -40],
        [5, -35, 65, 5, 25, 5, -40],
        [-5, -25, 55, -5, 35, -5, -40],
        [8, -40, 70, 8, 20, 8, -40],
        [-8, -28, 58, -8, 38, -8, -40],
        [3, -32, 62, 2, 28, 3, -40],
        [-3, -33, 63, -2, 32, -3, -40],
        [6, -38, 68, 4, 22, 6, -40],
        [-6, -27, 57, -4, 36, -6, -40],
        [2, -36, 64, 1, 26, 2, -40],
        [-2, -31, 61, -1, 34, -2, -40],
        [4, -34, 66, 3, 24, 4, -40],
    ]

    samples = []
    for i, q in enumerate(seed_poses):
        rp = joint_angles_to_robot_pose(np.array(q, dtype=float), fk)
        img = render_board_for_robot_pose(rp, T_cam, np.eye(4), K, dist, board)
        if detect_board_pose_from_image(img, K, dist, board) is None:
            raise RuntimeError(f"样本 {i} 棋盘不可检测，请调整 seed_poses")

        name = f"{i:04d}.png"
        cv2.imwrite(str(img_dir / name), img)
        samples.append(
            {
                "image": name,
                "robot_pose": rp,
                "joint_angles_deg": q,
            }
        )

    with (out / "camera_intrinsics.json").open("w") as f:
        json.dump({"camera_matrix": K.tolist(), "dist_coeffs": dist.tolist()}, f, indent=2)
    with (out / "poses.json").open("w") as f:
        json.dump({"samples": samples}, f, indent=2)

    print(f"演示数据已写入 {out} ({len(samples)} 组，全部可检测)")


if __name__ == "__main__":
    main()
