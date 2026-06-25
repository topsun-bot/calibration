"""棋盘在图像中的可见性检测。"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .board import BoardConfig, _find_chessboard_corners


def is_board_visible(
    image: np.ndarray | str | Path,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
    min_corners_ratio: float = 1.0,
) -> bool:
    """检测棋盘是否完整出现在图像中。"""
    if isinstance(image, (str, Path)):
        img = cv2.imread(str(image))
        if img is None:
            return False
    else:
        img = image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = _find_chessboard_corners(gray, board.pattern_size)
    if not found or corners is None:
        return False
    if min_corners_ratio >= 1.0:
        return True
    expected = board.cols * board.rows
    return len(corners) >= int(expected * min_corners_ratio)


def board_coverage_in_image(
    image: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
    margin_ratio: float = 0.08,
) -> dict:
    """
    评估棋盘在图像中的占比与边距。
    margin_ratio: 角点距图像边缘的最小比例，低于则视为“太靠边/out of view 风险”。
    """
    h, w = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = _find_chessboard_corners(gray, board.pattern_size)
    if not found or corners is None:
        return {"visible": False, "margin_ok": False}

    pts = corners.reshape(-1, 2)
    min_x, min_y = pts.min(axis=0)
    max_x, max_y = pts.max(axis=0)
    mx, my = w * margin_ratio, h * margin_ratio
    margin_ok = (
        min_x >= mx
        and min_y >= my
        and max_x <= w - mx
        and max_y <= h - my
    )
    cx = (min_x + max_x) / 2 / w
    cy = (min_y + max_y) / 2 / h
    return {
        "visible": True,
        "margin_ok": margin_ok,
        "center_norm": [float(cx), float(cy)],
        "bbox_px": [float(min_x), float(min_y), float(max_x), float(max_y)],
    }
