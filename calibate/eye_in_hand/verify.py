#!/usr/bin/env python3
"""眼在手上标定结果一致性验证：标定板在基座系下应近似恒定。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig, detect_board_pose
from common.io_utils import load_calibration_result, load_pose_list
from common.realsense import load_or_detect_intrinsics
from common.transforms import invert_rt, pose_to_rt, rt_to_homogeneous


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--result", type=Path, default=Path("output/T_cam_gripper.json"))
    parser.add_argument("--auto-camera", action="store_true")
    args = parser.parse_args()

    data_dir = args.data_dir.resolve()
    camera_file = data_dir / "camera_intrinsics.json"
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=args.auto_camera or not camera_file.exists(),
        images_dir=data_dir / "images",
    )
    board = BoardConfig()

    R_c2g, t_c2g, _ = load_calibration_result(args.result)
    T_c2g = rt_to_homogeneous(R_c2g, t_c2g)

    positions = []
    for i, sample in enumerate(load_pose_list(data_dir / "poses.json")):
        img = data_dir / "images" / sample.get("image", f"{i:04d}.png")
        det = detect_board_pose(img, K, dist, board)
        if det is None:
            continue
        R_t2c, t_t2c = det
        T_t2c = rt_to_homogeneous(R_t2c, t_t2c)

        rp = sample["robot_pose"]
        R_b2g, t_b2g = pose_to_rt(
            rp["position"], quaternion=rp.get("quaternion"), euler_xyz=rp.get("euler_xyz")
        )
        T_b2g = rt_to_homogeneous(R_b2g, t_b2g)

        # target in base: T_base_target = T_base_gripper * T_gripper_cam * T_cam_target
        T_g2c = invert_rt(R_c2g, t_c2g)
        T_g2c_h = rt_to_homogeneous(*T_g2c)
        T_b2t = T_b2g @ T_g2c_h @ np.linalg.inv(T_t2c)
        positions.append(T_b2t[:3, 3])

    if len(positions) < 2:
        print("样本不足，无法验证")
        return

    pts = np.array(positions)
    spread = np.linalg.norm(pts - pts.mean(axis=0), axis=1)
    print(f"样本数: {len(positions)}")
    print(f"标定板原点 in base 均值: {pts.mean(axis=0)}")
    print(f"位置残差 (m): mean={spread.mean():.6f}, max={spread.max():.6f}")


if __name__ == "__main__":
    main()
