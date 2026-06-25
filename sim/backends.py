"""仿真机械臂与相机后端。"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))

from sim.mujoco_env import (  # noqa: E402
    SIM_CAMERA_MATRIX,
    SIM_DEPTH_SCALE,
    SIM_DIST_COEFFS,
    MuJoCoSimEnv,
    create_sim_env,
)
from sim.detection_stub import SimDetectionStub as _DetectionStub  # noqa: E402


class SimD1Arm:
    """MuJoCo 驱动的 D1 接口兼容臂。"""

    def __init__(self, env: MuJoCoSimEnv) -> None:
        self.env = env

    def read_joints(self) -> np.ndarray:
        return self.env.read_joints_deg()

    def move_joints(self, angles_deg, wait: bool = True) -> None:
        steps = 22 if wait else 1
        self.env.move_joints_deg(np.asarray(angles_deg), steps=steps, settle=wait)

    def go_zero(self) -> None:
        self.move_joints([0, 0, 0, 0, 0, 0, -40])

    def close(self) -> None:
        pass


class SimCalibCamera:
    """兼容 RealSenseCapture 的标定相机。"""

    def __init__(self, env: MuJoCoSimEnv) -> None:
        self.env = env
        self.width = env.width
        self.height = env.height

    @property
    def intrinsics(self) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        meta = {"width": self.width, "height": self.height, "source": "mujoco_sim"}
        return SIM_CAMERA_MATRIX.copy(), SIM_DIST_COEFFS.copy(), meta

    def grab(self) -> np.ndarray:
        return self.env.render_calib_rgb()

    def save_frame(self, path: str | Path) -> np.ndarray:
        img = self.grab()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), img)
        return img

    def save_intrinsics(self, path: str | Path) -> None:
        from common.realsense import save_camera_intrinsics

        K, dist, meta = self.intrinsics
        save_camera_intrinsics(path, K, dist, meta)

    def close(self) -> None:
        pass


def _make_rs_intrinsics(width: int, height: int):
    try:
        import pyrealsense2 as rs

        intr = rs.intrinsics()
        intr.width = width
        intr.height = height
        intr.fx = float(SIM_CAMERA_MATRIX[0, 0])
        intr.fy = float(SIM_CAMERA_MATRIX[1, 1])
        intr.ppx = float(SIM_CAMERA_MATRIX[0, 2])
        intr.ppy = float(SIM_CAMERA_MATRIX[1, 2])
        intr.model = rs.distortion.none
        intr.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        return intr
    except ImportError:
        from types import SimpleNamespace

        return SimpleNamespace(
            width=width,
            height=height,
            fx=float(SIM_CAMERA_MATRIX[0, 0]),
            fy=float(SIM_CAMERA_MATRIX[1, 1]),
            ppx=float(SIM_CAMERA_MATRIX[0, 2]),
            ppy=float(SIM_CAMERA_MATRIX[1, 2]),
            model=None,
            coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
        )


class SimPickCamera:
    """兼容 get_object RealSenseCamera 的抓取相机。"""

    def __init__(self, env: MuJoCoSimEnv) -> None:
        self.env = env
        self.width = env.width
        self.height = env.height
        self._started = True
        self._depth_scale = SIM_DEPTH_SCALE
        self._intrinsics = _make_rs_intrinsics(env.width, env.height)

    def start(self) -> None:
        self._started = True

    def read(self, timeout_ms: int = 10000):
        from yolo3d.camera import FrameBundle

        if not self._started:
            self.start()
        color = self.env.render_rgb()
        depth = self.env.render_depth()
        self._inject_gt_depth(depth, self._stub_default_class())
        return FrameBundle(
            color=color,
            depth=depth,
            depth_scale=self._depth_scale,
            intrinsics=self._intrinsics,
        )

    def _stub_default_class(self) -> str:
        return "bottle"

    def _inject_gt_depth(self, depth: np.ndarray, class_name: str) -> None:
        """在物体投影区域写入 GT 深度，保证 3D 反投影与仿真一致。"""
        p_cam = self.env.object_cam_position(class_name)
        uv = self.env.project_cam_to_pixel(p_cam)
        if uv is None:
            return
        u, v = uv
        z_raw = int(np.clip(p_cam[2] / SIM_DEPTH_SCALE, 1, 65535))
        h, w = depth.shape
        r = 12
        y1, y2 = max(0, v - r), min(h, v + r + 1)
        x1, x2 = max(0, u - r), min(w, u + r + 1)
        depth[y1:y2, x1:x2] = z_raw

    def stop(self) -> None:
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


class SimDetectionStub:
    def __init__(self, env: MuJoCoSimEnv, default_class: str = "bottle"):
        self._stub = _DetectionStub(env, default_class)

    def predict(self, *args, **kwargs):
        return self._stub.predict(*args, **kwargs)

    @property
    def names(self):
        return self._stub.names
