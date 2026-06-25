"""齐次变换与位姿格式转换。"""

import numpy as np


def pose_to_rt(
    position: list | np.ndarray,
    quaternion: list | np.ndarray | None = None,
    euler_xyz: list | np.ndarray | None = None,
    degrees: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """
    将位姿转为 R, t。
    支持四元数 [x,y,z,w] 或欧拉角 [rx,ry,rz]（绕固定轴 X-Y-Z）。
    """
    t = np.asarray(position, dtype=np.float64).reshape(3, 1)
    if quaternion is not None:
        q = np.asarray(quaternion, dtype=np.float64).reshape(4)
        x, y, z, w = q
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
    elif euler_xyz is not None:
        e = np.asarray(euler_xyz, dtype=np.float64)
        if degrees:
            e = np.deg2rad(e)
        rx, ry, rz = e
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
    else:
        raise ValueError("必须提供 quaternion 或 euler_xyz")
    return R, t


def rt_to_pose(R: np.ndarray, t: np.ndarray) -> dict:
    t = t.reshape(3)
    return {
        "position": t.tolist(),
        "rotation_matrix": R.tolist(),
    }


def invert_rt(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv


def rt_to_homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def homogeneous_to_rt(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].reshape(3, 1).copy()


def transform_points(R: np.ndarray, t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64)
    if pts.ndim == 1:
        return (R @ pts.reshape(3, 1) + t).reshape(3)
    return (R @ pts.T + t).T
