"""眼在手外视觉：YOLO 3D 检测 + T_cam_base 坐标变换。"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))

from common.io_utils import load_calibration_result  # noqa: E402
from common.transforms import rt_to_homogeneous, transform_points  # noqa: E402
from yolo3d.camera import FrameBundle, RealSenseCamera  # noqa: E402
from yolo3d.detector import Detection3D, YOLO3DDetector  # noqa: E402


@dataclass
class ObjectInBase:
    class_name: str
    confidence: float
    position_base: np.ndarray
    position_cam: np.ndarray
    size_m: tuple[float, float, float] | None
    detection: Detection3D


class EyeToHandVision:
    """固定相机观测，将目标变换到机器人基座坐标系。"""

    def __init__(
        self,
        calib_path: Path,
        model_path: str = "yolo11n.pt",
        conf: float = 0.5,
        device: str | None = None,
        camera_width: int = 640,
        camera_height: int = 480,
        camera_fps: int = 30,
        sim_env=None,
    ) -> None:
        R, t, meta = load_calibration_result(calib_path)
        self.T_cam_base = rt_to_homogeneous(R, t)
        self.calib_meta = meta
        self.sim_env = sim_env
        self.detector = YOLO3DDetector(
            model_path=model_path,
            conf=conf,
            device=device,
        )
        if sim_env is not None:
            from sim.backends import SimDetectionStub, SimPickCamera

            self.camera = SimPickCamera(sim_env)
            self.detector.model = SimDetectionStub(sim_env)
            self.T_cam_base = sim_env.T_cam_base_render.copy()
        else:
            self.camera = RealSenseCamera(
                width=camera_width,
                height=camera_height,
                fps=camera_fps,
            )

    def cam_to_base(self, point_cam: np.ndarray | list | tuple) -> np.ndarray:
        p = np.asarray(point_cam, dtype=np.float64).reshape(3)
        R = self.T_cam_base[:3, :3]
        t = self.T_cam_base[:3, 3].reshape(3, 1)
        return transform_points(R, t, p)

    def detect_in_base(
        self,
        frame: FrameBundle | None = None,
        target_class: str | None = None,
        settle_frames: int = 5,
    ) -> tuple[list[ObjectInBase], FrameBundle]:
        if frame is None:
            for _ in range(settle_frames):
                frame = self.camera.read()
            if frame is None:
                raise RuntimeError("相机读帧失败")

        if self.sim_env is not None and target_class:
            return self._detect_in_base_sim(frame, target_class)

        detections = self.detector.detect(frame)
        if target_class:
            target = target_class.lower()
            detections = [d for d in detections if d.class_name.lower() == target]

        objects: list[ObjectInBase] = []
        for det in detections:
            p_cam = np.asarray(det.position_xyz, dtype=np.float64)
            p_base = self.cam_to_base(p_cam)
            objects.append(
                ObjectInBase(
                    class_name=det.class_name,
                    confidence=det.confidence,
                    position_base=p_base,
                    position_cam=p_cam,
                    size_m=det.size_xyz,
                    detection=det,
                )
            )
        objects.sort(key=lambda o: o.confidence, reverse=True)
        return objects, frame

    def _detect_in_base_sim(
        self, frame: FrameBundle, target_class: str
    ) -> tuple[list[ObjectInBase], FrameBundle]:
        """仿真模式：使用 MuJoCo ground truth，避免渲染/深度与标定外参不一致。"""
        sim_obj = self.sim_env.get_object_in_cam(target_class)
        p_base = np.asarray(sim_obj.position_base, dtype=np.float64)
        p_cam = self.sim_env.object_cam_position(target_class)
        uv = self.sim_env.project_cam_to_pixel(p_cam) or (320, 240)
        u, v = uv
        det = Detection3D(
            class_id=39 if target_class.lower() == "bottle" else 41,
            class_name=target_class,
            confidence=0.93,
            bbox_xyxy=(u - 40, v - 50, u + 40, v + 50),
            center_uv=(u, v),
            depth_m=float(p_cam[2]),
            position_xyz=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            size_xyz=sim_obj.size_m,
        )
        obj = ObjectInBase(
            class_name=target_class,
            confidence=0.93,
            position_base=p_base,
            position_cam=p_cam,
            size_m=sim_obj.size_m,
            detection=det,
        )
        return [obj], frame

    def select_best(
        self,
        objects: list[ObjectInBase],
        strategy: str = "highest_confidence",
    ) -> ObjectInBase | None:
        if not objects:
            return None
        if strategy == "highest_confidence":
            return objects[0]
        if strategy == "nearest":
            return min(objects, key=lambda o: o.position_cam[2])
        raise ValueError(f"未知选择策略: {strategy}")

    def close(self) -> None:
        self.camera.stop()
