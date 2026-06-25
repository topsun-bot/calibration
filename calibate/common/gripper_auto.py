"""根据视觉检测的距离与物体尺寸，自动计算 D1 夹爪 j6。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .board import BoardConfig, detect_board_pose_from_image

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRIPPER_CONFIG = ROOT / "config" / "d1_gripper.json"


@dataclass
class GripperAutoConfig:
    j6_open_deg: float = -40.0
    j6_close_deg: float = 20.0
    max_opening_m: float = 0.066
    board_thickness_m: float = 0.003
    grasp_margin_m: float = 0.010
    grasp_compress_m: float = 0.002
    min_grasp_width_m: float = 0.002
    preopen_extra_m: float = 0.008
    settle_s: float = 1.5

    @classmethod
    def load(cls, path: Path | None = None) -> GripperAutoConfig:
        p = path or DEFAULT_GRIPPER_CONFIG
        if not p.exists():
            return cls()
        with p.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(
            j6_open_deg=float(data.get("j6_open_deg", -40.0)),
            j6_close_deg=float(data.get("j6_close_deg", 20.0)),
            max_opening_m=float(data.get("max_opening_m", 0.066)),
            board_thickness_m=float(data.get("board_thickness_m", 0.003)),
            grasp_margin_m=float(data.get("grasp_margin_m", 0.010)),
            grasp_compress_m=float(data.get("grasp_compress_m", 0.002)),
            min_grasp_width_m=float(data.get("min_grasp_width_m", 0.002)),
            preopen_extra_m=float(data.get("preopen_extra_m", 0.008)),
            settle_s=float(data.get("settle_s", 1.5)),
        )


def board_physical_extent_m(board: BoardConfig) -> tuple[float, float]:
    """棋盘格物理宽、高 (米)，按内角点跨度估算。"""
    w = (board.cols - 1) * board.square_size
    h = (board.rows - 1) * board.square_size
    return w, h


def opening_m_to_j6(opening_m: float, cfg: GripperAutoConfig) -> float:
    """开口宽度(米) -> j6 角度。j6_open=全开，j6_close=全闭。"""
    opening_m = float(np.clip(opening_m, 0.0, cfg.max_opening_m))
    span = cfg.j6_close_deg - cfg.j6_open_deg
    if cfg.max_opening_m <= 1e-9 or abs(span) < 1e-9:
        return cfg.j6_close_deg
    ratio = opening_m / cfg.max_opening_m
    return float(cfg.j6_close_deg - ratio * span)


def j6_to_opening_m(j6_deg: float, cfg: GripperAutoConfig) -> float:
    span = cfg.j6_close_deg - cfg.j6_open_deg
    if abs(span) < 1e-9:
        return 0.0
    ratio = (cfg.j6_close_deg - j6_deg) / span
    return float(np.clip(ratio, 0.0, 1.0) * cfg.max_opening_m)


def _vision_span_m(
    corners: np.ndarray,
    K: np.ndarray,
    distance_m: float,
) -> tuple[float, float, float]:
    """由角点像素跨度与距离估算物体在图像中的宽、高(米)。"""
    pts = corners.reshape(-1, 2)
    min_x, min_y = pts.min(axis=0)
    max_x, max_y = pts.max(axis=0)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    width_m = (max_x - min_x) / fx * distance_m
    height_m = (max_y - min_y) / fy * distance_m
    return width_m, height_m, max(width_m, height_m)


def estimate_grasp_from_board(
    image: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
    cfg: GripperAutoConfig | None = None,
) -> dict[str, Any] | None:
    """
    从固定相机图像估计抓取参数。

    结合 solvePnP 距离、角点像素尺寸与棋盘已知物理尺寸，
    计算夹爪开口与 j6。眼在手外场景：相机看到棋盘正面时，
    夹爪侧向抓取通常对应标定板厚度；若视觉估算的短边更可靠则取较小值。
    """
    cfg = cfg or GripperAutoConfig.load()
    det = detect_board_pose_from_image(image, K, dist, board)
    if det is None:
        return None

    R, tvec, corners = det
    distance_m = float(np.linalg.norm(tvec))
    depth_m = float(abs(tvec[2, 0])) if tvec.size >= 3 else distance_m

    vis_w, vis_h, vis_max = _vision_span_m(corners, K, depth_m)
    known_w, known_h = board_physical_extent_m(board)
    known_min, known_max = min(known_w, known_h), max(known_w, known_h)

    aspect_known = known_w / max(known_h, 1e-6)
    aspect_vis = vis_w / max(vis_h, 1e-6)
    face_on = abs(aspect_known - aspect_vis) < 0.35 and vis_max > known_min * 0.6

    if face_on:
        # 棋盘正对相机：夹爪侧向夹持，宽度≈板厚
        grasp_width_m = cfg.board_thickness_m
        grasp_mode = "thickness"
    else:
        # 棋盘侧对相机或倾斜：用视觉短边估算
        grasp_width_m = min(vis_w, vis_h)
        grasp_mode = "vision_min"

    grasp_width_m = float(
        np.clip(
            grasp_width_m,
            cfg.min_grasp_width_m,
            min(known_max, cfg.max_opening_m * 0.95),
        )
    )

    preopen_m = float(
        np.clip(
            grasp_width_m + cfg.grasp_margin_m + cfg.preopen_extra_m,
            cfg.min_grasp_width_m,
            cfg.max_opening_m,
        )
    )
    close_m = float(
        np.clip(
            grasp_width_m - cfg.grasp_compress_m,
            cfg.min_grasp_width_m,
            preopen_m,
        )
    )

    j6_preopen = opening_m_to_j6(preopen_m, cfg)
    j6_grasp = opening_m_to_j6(close_m, cfg)

    return {
        "distance_m": distance_m,
        "depth_m": depth_m,
        "vision_width_m": vis_w,
        "vision_height_m": vis_h,
        "known_board_m": [known_w, known_h],
        "grasp_width_m": grasp_width_m,
        "preopen_m": preopen_m,
        "close_m": close_m,
        "j6_preopen": j6_preopen,
        "j6_grasp": j6_grasp,
        "grasp_mode": grasp_mode,
        "face_on": face_on,
        "board_detected": True,
    }


def apply_locked_gripper(q: np.ndarray, locked_j6: float | None) -> np.ndarray:
    out = np.asarray(q, dtype=np.float64).reshape(-1).copy()
    if out.size < 7:
        out = np.pad(out, (0, 7 - out.size))
    if locked_j6 is not None:
        out[6] = locked_j6
    return out


def run_auto_gripper_grasp(
    arm,
    image: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
    cfg: GripperAutoConfig | None = None,
    mock_arm: bool = False,
) -> tuple[float | None, dict[str, Any]]:
    """
    根据当前图像自动：预开口 -> 夹紧，返回锁定 j6。

    仅改变 j6，6 个臂关节保持当前值（适合眼在手外、臂已大致对准棋盘）。
    """
    import time

    cfg = cfg or GripperAutoConfig.load()
    est = estimate_grasp_from_board(image, K, dist, board, cfg)
    if est is None:
        return None, {"error": "未检测到棋盘，无法自动设置 j6"}

    q = apply_locked_gripper(arm.read_joints(), None)
    j6_pre = est["j6_preopen"]
    j6_close = est["j6_grasp"]

    print(
        f"[夹爪] 距离≈{est['distance_m']:.3f}m, "
        f"抓取宽度≈{est['grasp_width_m']*1000:.1f}mm "
        f"(视觉 {est['vision_width_m']*1000:.0f}×{est['vision_height_m']*1000:.0f}mm)"
    )
    print(
        f"[夹爪] 预开口 j6={j6_pre:.1f}° ({est['preopen_m']*1000:.1f}mm) -> "
        f"夹紧 j6={j6_close:.1f}° ({est['close_m']*1000:.1f}mm)"
    )

    q_pre = q.copy()
    q_pre[6] = j6_pre
    arm.move_joints(q_pre.tolist(), wait=True)
    if not mock_arm:
        time.sleep(cfg.settle_s)

    q_close = q.copy()
    q_close[6] = j6_close
    arm.move_joints(q_close.tolist(), wait=True)
    if not mock_arm:
        time.sleep(cfg.settle_s * 0.5)

    actual = arm.read_joints()
    locked = float(actual[6])
    est["j6_locked"] = locked
    print(f"[夹爪] 已夹紧并锁定 j6={locked:.1f}°")
    return locked, est
