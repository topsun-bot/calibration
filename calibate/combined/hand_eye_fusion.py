#!/usr/bin/env python3
"""
眼在手上 + 眼在手外 联合应用。

典型场景：
  - 手外相机：全局监视、工件粗定位（目标在 base 系）
  - 手上相机：末端精定位、抓取修正（目标在 gripper 系）

通过机器人当前位姿与两份标定结果，统一点云/目标到 base 或 gripper 系。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig, detect_board_pose
from common.io_utils import load_calibration_result
from common.transforms import (
    homogeneous_to_rt,
    invert_rt,
    pose_to_rt,
    rt_to_homogeneous,
    transform_points,
)


class HandEyeFusion:
    """联合手眼标定变换链。"""

    def __init__(
        self,
        T_cam_gripper_path: Path,
        T_cam_base_fixed_path: Path,
    ) -> None:
        R_cg, t_cg, _ = load_calibration_result(T_cam_gripper_path)
        R_cf, t_cf, _ = load_calibration_result(T_cam_base_fixed_path)
        self.T_cam_gripper = rt_to_homogeneous(R_cg, t_cg)
        self.T_cam_base_fixed = rt_to_homogeneous(R_cf, t_cf)

    def target_in_base_from_eye_to_hand(
        self,
        R_target2cam: np.ndarray,
        t_target2cam: np.ndarray,
    ) -> np.ndarray:
        """手外相机观测 -> 目标在基座系 4x4。"""
        T_t2c = rt_to_homogeneous(R_target2cam, t_target2cam)
        T_t2b = self.T_cam_base_fixed @ np.linalg.inv(T_t2c)
        return T_t2b

    def target_in_gripper_from_eye_in_hand(
        self,
        R_target2cam: np.ndarray,
        t_target2cam: np.ndarray,
    ) -> np.ndarray:
        """手上相机观测 -> 目标在法兰系 4x4。"""
        T_t2c = rt_to_homogeneous(R_target2cam, t_target2cam)
        T_t2g = self.T_cam_gripper @ np.linalg.inv(T_t2c)
        return T_t2g

    def target_in_base_from_eye_in_hand(
        self,
        robot_pose: dict,
        R_target2cam: np.ndarray,
        t_target2cam: np.ndarray,
    ) -> np.ndarray:
        """手上相机观测 + 机器人位姿 -> 目标在基座系。"""
        R_b2g, t_b2g = pose_to_rt(
            robot_pose["position"],
            quaternion=robot_pose.get("quaternion"),
            euler_xyz=robot_pose.get("euler_xyz"),
        )
        T_b2g = rt_to_homogeneous(R_b2g, t_b2g)
        T_t2g = self.target_in_gripper_from_eye_in_hand(R_target2cam, t_target2cam)
        return T_b2g @ T_t2g

    def fuse_target_position(
        self,
        robot_pose: dict,
        R_t2c_fixed: np.ndarray,
        t_t2c_fixed: np.ndarray,
        R_t2c_hand: np.ndarray,
        t_t2c_hand: np.ndarray,
        weight_fixed: float = 0.4,
    ) -> dict:
        """
        融合两路目标位置估计（加权平均）。
        weight_fixed: 手外相机权重，其余给手上相机。
        """
        T_fixed = self.target_in_base_from_eye_to_hand(R_t2c_fixed, t_t2c_fixed)
        T_hand = self.target_in_base_from_eye_in_hand(robot_pose, R_t2c_hand, t_t2c_hand)
        p_fixed = T_fixed[:3, 3]
        p_hand = T_hand[:3, 3]
        w = np.clip(weight_fixed, 0.0, 1.0)
        p_fused = w * p_fixed + (1 - w) * p_hand
        return {
            "position_base": p_fused.tolist(),
            "position_eye_to_hand": p_fixed.tolist(),
            "position_eye_in_hand": p_hand.tolist(),
            "disagreement_m": float(np.linalg.norm(p_fixed - p_hand)),
        }

    def grasp_offset_in_gripper(
        self,
        robot_pose: dict,
        R_t2c_fixed: np.ndarray,
        t_t2c_fixed: np.ndarray,
    ) -> np.ndarray:
        """
        粗定位(手外) -> 抓取修正量 in gripper 系。
        返回: 目标原点相对当前法兰的位移向量 (3,)，用于 MoveL 补偿。
        """
        T_target_base = self.target_in_base_from_eye_to_hand(R_t2c_fixed, t_t2c_fixed)
        p_target = T_target_base[:3, 3]

        R_b2g, t_b2g = pose_to_rt(
            robot_pose["position"],
            quaternion=robot_pose.get("quaternion"),
            euler_xyz=robot_pose.get("euler_xyz"),
        )
        R_g2b, t_g2b = invert_rt(R_b2g, t_b2g)
        offset_base = p_target - t_b2g.reshape(3)
        offset_gripper = transform_points(R_g2b, t_g2b, offset_base)
        return offset_gripper


def run_demo(args: argparse.Namespace) -> None:
    fusion = HandEyeFusion(
        Path(args.eye_in_hand_result),
        Path(args.eye_to_hand_result),
    )

    with (Path(args.eye_to_hand_data) / "camera_intrinsics.json").open() as f:
        cam_fixed = json.load(f)
    with (Path(args.eye_in_hand_data) / "camera_intrinsics.json").open() as f:
        cam_hand = json.load(f)

    Kf = np.asarray(cam_fixed["camera_matrix"])
    df = np.asarray(cam_fixed.get("dist_coeffs", [0] * 5))
    Kh = np.asarray(cam_hand["camera_matrix"])
    dh = np.asarray(cam_hand.get("dist_coeffs", [0] * 5))
    board = BoardConfig()

    idx = args.sample_index
    img_f = Path(args.eye_to_hand_data) / "images" / f"{idx:04d}.png"
    img_h = Path(args.eye_in_hand_data) / "images" / f"{idx:04d}.png"

    det_f = detect_board_pose(img_f, Kf, df, board)
    det_h = detect_board_pose(img_h, Kh, dh, board)
    if det_f is None or det_h is None:
        raise RuntimeError("演示样本检测失败，请确认已生成 demo 数据")

    import json as _json

    poses_f = _json.load(open(Path(args.eye_to_hand_data) / "poses.json"))["samples"]
    poses_h = _json.load(open(Path(args.eye_in_hand_data) / "poses.json"))["samples"]
    robot_pose = poses_h[idx]["robot_pose"]

    result = fusion.fuse_target_position(robot_pose, det_f[0], det_f[1], det_h[0], det_h[1])
    offset = fusion.grasp_offset_in_gripper(robot_pose, det_f[0], det_f[1])

    print("=== 联合应用演示 ===")
    print(f"样本索引: {idx}")
    print(f"融合后目标位置 (base): {result['position_base']}")
    print(f"手外相机估计 (base):   {result['position_eye_to_hand']}")
    print(f"手上相机估计 (base):   {result['position_eye_in_hand']}")
    print(f"两路分歧 (m): {result['disagreement_m']:.6f}")
    print(f"抓取修正量 (gripper): {offset.tolist()}")


def main() -> None:
    parser = argparse.ArgumentParser(description="眼在手上+眼在手外 联合应用")
    parser.add_argument(
        "--eye-in-hand-result",
        default="../eye_in_hand/output/T_cam_gripper.json",
    )
    parser.add_argument(
        "--eye-to-hand-result",
        default="../eye_to_hand/output/T_cam_base.json",
    )
    parser.add_argument("--eye-in-hand-data", default="../eye_in_hand/data")
    parser.add_argument("--eye-to-hand-data", default="../eye_to_hand/data")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--demo", action="store_true", help="运行内置演示")
    args = parser.parse_args()

    if args.demo:
        run_demo(args)
        return

    print("请使用 --demo 运行演示，或在代码中 import HandEyeFusion 集成到您的抓取流程。")
    print("示例:")
    print("  from hand_eye_fusion import HandEyeFusion")
    print("  fusion = HandEyeFusion('T_cam_gripper.json', 'T_cam_base.json')")
    print("  fused = fusion.fuse_target_position(robot_pose, R1,t1, R2,t2)")


if __name__ == "__main__":
    main()
