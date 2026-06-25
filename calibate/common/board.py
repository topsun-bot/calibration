"""标定板检测与相机位姿估计。"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

# 尝试导入 aruco 模块，不可用时优雅降级
try:
    import cv2.aruco as aruco

    _ARUCO_AVAILABLE = True
except ImportError:
    _ARUCO_AVAILABLE = False


@dataclass
class BoardConfig:
    """棋盘格参数（内角点数量，不含边框格）。"""

    cols: int = 9
    rows: int = 6
    square_size: float = 0.025  # 米

    @property
    def pattern_size(self) -> Tuple[int, int]:
        return (self.cols, self.rows)


@dataclass
class CharucoBoardConfig:
    """ChArUco 标定板参数。"""

    cols: int = 7
    rows: int = 5
    square_size: float = 0.04  # 米，棋盘格边长
    marker_size: float = 0.02  # 米，ArUco 标记边长
    aruco_dict_name: str = "DICT_4X4_50"

    @property
    def pattern_size(self) -> Tuple[int, int]:
        """内角点数量 (cols-1, rows-1)。"""
        return (self.cols - 1, self.rows - 1)


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


def _get_aruco_dict(dict_name: str):
    """根据名称获取 ArUco 字典对象。"""
    if not _ARUCO_AVAILABLE:
        raise RuntimeError("cv2.aruco 模块不可用，无法使用 ChArUco 检测")
    dict_id = getattr(aruco, dict_name, None)
    if dict_id is None:
        raise ValueError(f"未知的 ArUco 字典名称: {dict_name}")
    return aruco.getPredefinedDictionary(dict_id)


def _create_charuco_board(config: CharucoBoardConfig):
    """创建 ChArUco 标定板对象。"""
    dictionary = _get_aruco_dict(config.aruco_dict_name)
    board = aruco.CharucoBoard(
        (config.cols, config.rows),
        config.square_size,
        config.marker_size,
        dictionary,
    )
    return board, dictionary


def _find_charuco_corners(
    gray: np.ndarray, config: CharucoBoardConfig
) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
    """检测 ChArUco 角点，返回 (found, corners, ids)。"""
    if not _ARUCO_AVAILABLE:
        return False, None, None

    board, dictionary = _create_charuco_board(config)
    detector_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, detector_params)

    marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    if marker_ids is None or len(marker_ids) == 0:
        return False, None, None

    charuco_retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    if charuco_retval < 4:
        return False, None, None

    return True, charuco_corners, charuco_ids


def detect_board_pose_from_image(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig | CharucoBoardConfig,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """从 BGR 图像估计 target->camera 的 R, t 及角点。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if isinstance(board, CharucoBoardConfig):
        return _detect_charuco_pose(gray, camera_matrix, dist_coeffs, board)

    # 经典棋盘格路径
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


def _detect_charuco_pose(
    gray: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    config: CharucoBoardConfig,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """从灰度图像检测 ChArUco 角点并估计位姿。"""
    found, corners, ids = _find_charuco_corners(gray, config)
    if not found or corners is None or ids is None:
        return None

    board, _ = _create_charuco_board(config)

    # 获取角点对应的 3D 坐标
    obj_points, img_points = board.matchImagePoints(corners, ids)
    if obj_points is None or len(obj_points) < 4:
        return None

    ok, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return (
        R.astype(np.float64),
        tvec.reshape(3, 1).astype(np.float64),
        corners.reshape(-1, 1, 2).astype(np.float64),
    )


def detect_board_pose(
    image_path: str | Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig | CharucoBoardConfig,
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
