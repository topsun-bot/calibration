"""合成可检测棋盘格图像（OpenCV findChessboardCorners 兼容）。"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .board import BoardConfig, make_object_points
from .transforms import rt_to_homogeneous


def _draw_flat_board(board: BoardConfig, sq_px: int = 64) -> np.ndarray:
    """生成平面棋盘纹理 (rows+1 × cols+1 格)。"""
    cols, rows = board.cols, board.rows
    h = (rows + 1) * sq_px
    w = (cols + 1) * sq_px
    canvas = np.zeros((h, w), dtype=np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                canvas[r * sq_px : (r + 1) * sq_px, c * sq_px : (c + 1) * sq_px] = 220
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def render_checkerboard_image(
    R_target2cam: np.ndarray,
    t_target2cam: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig | None = None,
    image_size: tuple[int, int] = (640, 480),
    background: int = 40,
) -> np.ndarray:
    """用单应性变换渲染 target→camera 位姿下的棋盘格。"""
    board = board or BoardConfig()
    objp = make_object_points(board)
    rvec, _ = cv2.Rodrigues(R_target2cam)
    tvec = np.asarray(t_target2cam, dtype=np.float64).reshape(3, 1)

    img_pts, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
    pts = img_pts.reshape(-1, 2)

    # 用四角内点估计单应性（板面 Z=0）
    obj_corners = np.array(
        [
            [0, 0],
            [(board.cols - 1) * board.square_size, 0],
            [(board.cols - 1) * board.square_size, (board.rows - 1) * board.square_size],
            [0, (board.rows - 1) * board.square_size],
        ],
        dtype=np.float32,
    )
    img_corners = np.array(
        [pts[0], pts[board.cols - 1], pts[-1], pts[-board.cols]], dtype=np.float32
    )

    w, h = image_size
    canvas = np.ones((h, w, 3), dtype=np.uint8) * background
    flat = _draw_flat_board(board)
    H, _ = cv2.findHomography(
        np.array(
            [[0, 0], [flat.shape[1], 0], [flat.shape[1], flat.shape[0]], [0, flat.shape[0]]],
            dtype=np.float32,
        ),
        img_corners,
    )
    if H is None:
        return canvas
    warped = cv2.warpPerspective(flat, H, (w, h), flags=cv2.INTER_LINEAR)
    mask = warped.sum(axis=2) > 0
    canvas[mask] = warped[mask]
    return canvas


def render_board_for_robot_pose(
    robot_pose: dict,
    T_cam_base: np.ndarray,
    T_target_gripper: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig | None = None,
    image_size: tuple[int, int] = (640, 480),
) -> np.ndarray:
    """眼在手外：标定板固连夹爪，渲染相机图像。

    参数:
        robot_pose: FK 输出的末端位姿（位置 + 四元数/欧拉角），表示夹爪在基座系下的位姿。
        T_cam_base: 4x4 仿真 GT 变换矩阵。
        T_target_gripper: 4x4，标定板原点相对夹爪 TCP 的偏移（通常为 I）。
    """
    board = board or BoardConfig()
    from .transforms import pose_to_rt

    R_g2b, t_g2b = pose_to_rt(
        robot_pose["position"],
        quaternion=robot_pose.get("quaternion"),
        euler_xyz=robot_pose.get("euler_xyz"),
    )
    T_g2b = rt_to_homogeneous(R_g2b, t_g2b)
    T_t2c = np.linalg.inv(T_cam_base) @ T_g2b @ T_target_gripper
    return render_checkerboard_image(
        T_t2c[:3, :3],
        T_t2c[:3, 3],
        camera_matrix,
        dist_coeffs,
        board,
        image_size,
    )


def write_checkerboard_image(
    path: Path,
    R_target2cam: np.ndarray,
    t_target2cam: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: BoardConfig | None = None,
) -> np.ndarray:
    img = render_checkerboard_image(
        R_target2cam, t_target2cam, camera_matrix, dist_coeffs, board
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)
    return img
