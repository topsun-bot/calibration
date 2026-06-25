"""标定板检测与相机位姿估计。"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class BoardConfig:
    """棋盘格参数（内角点数量，不含边框格）。"""

    cols: int = 9
    rows: int = 6
    square_size: float = 0.025  # 米

    @property
    def pattern_size(self) -> Tuple[int, int]:
        return (self.cols, self.rows)


def make_object_points(board: BoardConfig) -> np.ndarray:
    objp = np.zeros((board.rows * board.cols, 3), np.float32)
    grid = np.mgrid[0 : board.cols, 0 : board.rows].T.reshape(-1, 2)
    objp[:, :2] = grid * board.square_size
    return objp


def _find_chessboard_corners(
    gray: np.ndarray, pattern_size: tuple[int, int]
) -> tuple[bool, np.ndarray | None]:
    """优先 findChessboardCornersSB，回退经典算法（兼容合成图）。"""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        if found:
            return True, corners.reshape(-1, 1, 2).astype(np.float32)
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    return found, corners


def detect_board_pose_from_image(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """从 BGR 图像估计 target->camera 的 R, t 及角点。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = _find_chessboard_corners(gray, board.pattern_size)
    if not found or corners is None:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    objp = make_object_points(board)

    ok, rvec, tvec = cv2.solvePnP(
        objp, corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return (
        R.astype(np.float64),
        tvec.reshape(3, 1).astype(np.float64),
        corners.astype(np.float64),
    )


def detect_board_pose(
    image_path: str | Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    从图像估计 target->camera 的 R, t。
    标定板固定在环境中，相机看到板子即得到 T_target_cam。
    """
    img = cv2.imread(str(image_path))
    if img is None:
        return None

    det = detect_board_pose_from_image(img, camera_matrix, dist_coeffs, board)
    if det is None:
        return None
    R, tvec, _ = det
    return R, tvec
