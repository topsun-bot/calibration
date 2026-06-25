#!/usr/bin/env python3
"""生成眼在手上标定的演示数据（合成图像 + 位姿）。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig, make_object_points
from common.transforms import rt_to_homogeneous


def main() -> None:
    out = Path(__file__).parent / "data"
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    board = BoardConfig()
    K = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5)

    # 真值：相机相对法兰
    T_c2g_true = np.eye(4)
    T_c2g_true[:3, :3] = cv2.Rodrigues(np.array([0.1, -0.2, 0.05]))[0]
    T_c2g_true[:3, 3] = [0.05, 0.02, 0.12]

    T_t2b = np.eye(4)
    T_t2b[:3, :3] = cv2.Rodrigues(np.array([0, 0, 0.3]))[0]
    T_t2b[:3, 3] = [0.5, 0.0, 0.0]

    objp = make_object_points(board)
    samples = []

    for i in range(12):
        angle = i * 0.35
        T_b2g = np.eye(4)
        R, _ = cv2.Rodrigues(np.array([0, 0, angle]))
        T_b2g[:3, :3] = R
        T_b2g[:3, 3] = [0.4 * np.cos(angle), 0.4 * np.sin(angle), 0.5 + 0.05 * i]

        T_b2c = T_b2g @ T_c2g_true
        T_c2t = np.linalg.inv(T_b2c) @ T_t2b
        R_c2t = T_c2t[:3, :3]
        t_c2t = T_c2t[:3, 3]
        rvec, _ = cv2.Rodrigues(R_c2t)
        img_pts, _ = cv2.projectPoints(objp, rvec, t_c2t, K, dist)
        img = np.ones((480, 640, 3), dtype=np.uint8) * 40
        for p in img_pts.reshape(-1, 2).astype(int):
            cv2.circle(img, tuple(p), 4, (0, 255, 0), -1)

        name = f"{i:04d}.png"
        cv2.imwrite(str(img_dir / name), img)
        samples.append(
            {
                "image": name,
                "robot_pose": {
                    "position": T_b2g[:3, 3].tolist(),
                    "euler_xyz": [0, 0, np.rad2deg(angle)],
                },
            }
        )

    with (out / "camera_intrinsics.json").open("w") as f:
        json.dump({"camera_matrix": K.tolist(), "dist_coeffs": dist.tolist()}, f, indent=2)
    with (out / "poses.json").open("w") as f:
        json.dump({"samples": samples}, f, indent=2)

    print(f"演示数据已写入 {out}")


if __name__ == "__main__":
    main()
