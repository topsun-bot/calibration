"""Intel RealSense RGB-D 相机封装。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class FrameBundle:
    """RGB-D 帧数据。intrinsics 可以是 pyrealsense2.intrinsics 或任何兼容对象。"""
    color: np.ndarray
    depth: np.ndarray
    depth_scale: float
    intrinsics: Any


class RealSenseCamera:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        warmup_frames: int = 10,
    ) -> None:
        import pyrealsense2 as rs

        self.width = width
        self.height = height
        self.fps = fps
        self.warmup_frames = warmup_frames

        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._align = rs.align(rs.stream.color)
        self._started = False

    def start(self) -> None:
        import pyrealsense2 as rs

        if self._started:
            return

        self._config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self._config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = self._pipeline.start(self._config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        for _ in range(self.warmup_frames):
            self._pipeline.wait_for_frames(timeout_ms=10000)

        self._started = True

    def read(self, timeout_ms: int = 10000) -> FrameBundle | None:
        if not self._started:
            self.start()

        frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)
        aligned = self._align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        return FrameBundle(color=color, depth=depth, depth_scale=self._depth_scale, intrinsics=intrinsics)

    def stop(self) -> None:
        if self._started:
            self._pipeline.stop()
            self._started = False

    def __enter__(self) -> "RealSenseCamera":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
