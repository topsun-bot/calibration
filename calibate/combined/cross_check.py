#!/usr/bin/env python3
"""
交叉验证：理论上 T_base_gripper * T_cam_gripper ≈ T_cam_base_fixed（需同一相机坐标系约定）。

用于检查两份标定是否与当前机器人位姿一致。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.io_utils import load_calibration_result, load_pose_list
from common.transforms import pose_to_rt, rt_to_homogeneous


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--eye-in-hand", type=Path, required=True)
    parser.add_argument("--eye-to-hand", type=Path, required=True)
    parser.add_argument("--poses", type=Path, required=True, help="eye_in_hand 的 poses.json")
    args = parser.parse_args()

    R_cg, t_cg, _ = load_calibration_result(args.eye_in_hand)
    R_cf, t_cf, _ = load_calibration_result(args.eye_to_hand)
    T_c2g = rt_to_homogeneous(R_cg, t_cg)
    T_c2f = rt_to_homogeneous(R_cf, t_cf)

    errors = []
    for sample in load_pose_list(args.poses):
        rp = sample["robot_pose"]
        R_b2g, t_b2g = pose_to_rt(
            rp["position"], quaternion=rp.get("quaternion"), euler_xyz=rp.get("euler_xyz")
        )
        T_b2g = rt_to_homogeneous(R_b2g, t_b2g)
        T_c2b_via_arm = T_b2g @ T_c2g
        delta = T_c2b_via_arm - T_c2f
        errors.append(np.linalg.norm(delta))

    err = np.array(errors)
    print(f"样本数: {len(err)}")
    print(f"T_cam_base 链式 vs 手外标定 矩阵差 Frobenius: mean={err.mean():.6f}, max={err.max():.6f}")
    print("注：两相机物理上不同，仅当手外/手上为同一相机迁移标定时该指标才有意义。")
    print("      双相机联合应用请用 hand_eye_fusion.py 的 fuse 接口。")


if __name__ == "__main__":
    main()
