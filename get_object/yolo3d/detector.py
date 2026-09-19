"""YOLO 2D 检测 + 深度图融合，输出目标 3D 位置。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    import pyrealsense2 as rs

from .camera import FrameBundle


@dataclass
class Detection3D:
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    center_uv: tuple[int, int]
    depth_m: float
    position_xyz: tuple[float, float, float]
    size_xyz: tuple[float, float, float] | None = None


class YOLO3DDetector:
    def __init__(
        self,
        model_path: str = "yolo11n.pt",
        conf: float = 0.5,
        device: str | None = None,
        depth_percentile: float = 20.0,
        load_model: bool = True,
    ) -> None:
        self.model = None
        self.conf = conf
        self.depth_percentile = depth_percentile
        self.device = device or ("cuda" if self._has_cuda() else "cpu")
        if load_model:
            from ultralytics import YOLO

            self.model = YOLO(model_path)

    @staticmethod
    def _has_cuda() -> bool:
        try:
            import torch

            return torch.cuda.is_available()
        except Exception:
            return False

    def detect(self, frame: FrameBundle) -> list[Detection3D]:
        results = self.model.predict(
            source=frame.color,
            conf=self.conf,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        names = result.names
        detections: list[Detection3D] = []

        if result.boxes is None or len(result.boxes) == 0:
            return detections

        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            depth_m = self._median_depth_in_bbox(frame, x1, y1, x2, y2)
            if depth_m is None:
                continue

            position = self._deproject(frame.intrinsics, cx, cy, depth_m)
            size_xyz = self._estimate_size(frame.intrinsics, x1, y1, x2, y2, depth_m)

            detections.append(
                Detection3D(
                    class_id=class_id,
                    class_name=names[class_id],
                    confidence=confidence,
                    bbox_xyxy=(x1, y1, x2, y2),
                    center_uv=(cx, cy),
                    depth_m=depth_m,
                    position_xyz=position,
                    size_xyz=size_xyz,
                )
            )

        return detections

    def _median_depth_in_bbox(
        self,
        frame: FrameBundle,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> float | None:
        h, w = frame.depth.shape
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))
        if x2 <= x1 or y2 <= y1:
            return None

        region = frame.depth[y1:y2, x1:x2].astype(np.float32)
        valid = region[region > 0]
        if valid.size == 0:
            return None

        depth_raw = float(np.percentile(valid, self.depth_percentile))
        return depth_raw * frame.depth_scale

    @staticmethod
    def _deproject(intrinsics, u: int, v: int, depth_m: float) -> tuple[float, float, float]:
        """反投影像素到 3D 点。支持 pyrealsense2.intrinsics 和纯数值 intrinsics。"""
        import pyrealsense2 as rs

        point = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)
        return float(point[0]), float(point[1]), float(point[2])

    @staticmethod
    def _estimate_size(
        intrinsics,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        depth_m: float,
    ) -> tuple[float, float, float]:
        import pyrealsense2 as rs

        corners = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]
        points = [rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m) for u, v in corners]
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        return float(width), float(height), float(depth_m)

    @staticmethod
    def depth_colormap(depth: np.ndarray, depth_scale: float, max_m: float = 5.0) -> np.ndarray:
        depth_m = depth.astype(np.float32) * depth_scale
        depth_m[depth_m <= 0] = max_m
        depth_m = np.clip(depth_m, 0, max_m)
        depth_u8 = (depth_m / max_m * 255).astype(np.uint8)
        return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

    @staticmethod
    def draw(image: np.ndarray, detections: list[Detection3D]) -> np.ndarray:
        canvas = image.copy()
        for det in detections:
            x1, y1, x2, y2 = det.bbox_xyxy
            cx, cy = det.center_uv
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(canvas, (cx, cy), 4, (0, 0, 255), -1)
            x, y, z = det.position_xyz
            label = f"{det.class_name} {det.confidence:.2f}  Z:{z:.2f}m"
            detail = f"X:{x:.2f} Y:{y:.2f}"
            cv2.putText(canvas, label, (x1, max(20, y1 - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(canvas, detail, (x1, max(38, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        return canvas

    @staticmethod
    def build_display(
        frame: FrameBundle,
        detections: list[Detection3D],
        fps: float,
        show_depth: bool = True,
    ) -> np.ndarray:
        rgb_vis = YOLO3DDetector.draw(frame.color, detections)
        h, w = rgb_vis.shape[:2]

        cv2.putText(rgb_vis, "RGB + 3D Detection", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(rgb_vis, f"FPS: {fps:.1f}  Objects: {len(detections)}", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

        if not show_depth:
            return rgb_vis

        depth_vis = YOLO3DDetector.depth_colormap(frame.depth, frame.depth_scale)
        for det in detections:
            x1, y1, x2, y2 = det.bbox_xyxy
            cv2.rectangle(depth_vis, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(depth_vis, "Depth", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        return np.hstack([rgb_vis, depth_vis])
