"""RealSense D455 彩色流采集（含自适应带宽，兼容 get_object RGB-D 开流方式）。"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .realsense import find_realsense_device, save_camera_intrinsics
from .realsense_bandwidth import (
    AdaptiveBandwidthState,
    ColorStreamProfile,
    default_frame_timeout_ms,
    ensure_device_ready,
    format_adaptive_summary,
    is_usb2,
    profile_candidates,
    recover_busy_device,
    start_pipeline,
    wait_color_frame,
    _is_device_busy,
)


class RealSenseCapture:
    """
    标定用 RealSense 采集。

    与 get_object/yolo3d/camera.py 一致：默认同时开启 depth + color，
    在 USB 2.1 下比仅开 color 流更稳定。
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial: str | None = None,
        model_hint: str = "D435",
        warmup_frames: int | None = None,
        frame_timeout_ms: int | None = None,
        auto_bandwidth: bool = True,
    ) -> None:
        import pyrealsense2 as rs

        self.rs = rs
        self.serial = serial
        self.model_hint = model_hint
        self.frame_timeout_ms = frame_timeout_ms
        self.auto_bandwidth = auto_bandwidth
        self._requested = ColorStreamProfile(width, height, fps)
        self._align = rs.align(rs.stream.color)

        self.pipeline = rs.pipeline()
        self._bandwidth: AdaptiveBandwidthState
        self.profile = self._open_pipeline()
        self.width = self._bandwidth.active.width
        self.height = self._bandwidth.active.height
        self.fps = self._bandwidth.active.fps
        if warmup_frames is None:
            warmup_frames = 10 if not is_usb2(self._bandwidth.usb_type) else 5
        self._warmup(warmup_frames)

    def _timeout_ms(self) -> int:
        if self.frame_timeout_ms is not None:
            return self.frame_timeout_ms
        # 与 get_object 一致默认 10s；USB2 放宽到 30s
        return 30000 if is_usb2(self._bandwidth.usb_type) else 10000

    def _device_usb_type(self, dev: "rs.device") -> str:
        try:
            return dev.get_info(self.rs.camera_info.usb_type_descriptor)
        except Exception:
            return "unknown"

    def _profiles_to_try(self, usb_type: str) -> list[ColorStreamProfile]:
        if not self.auto_bandwidth:
            return [self._requested]
        return profile_candidates(
            (self._requested.width, self._requested.height, self._requested.fps),
            usb_type,
        )

    def _open_pipeline(self) -> Any:
        rs = self.rs
        dev = find_realsense_device(serial=self.serial, model_hint=self.model_hint)
        self.serial = dev.get_info(rs.camera_info.serial_number)
        usb_type = self._device_usb_type(dev)
        device_name = dev.get_info(rs.camera_info.name)
        candidates = self._profiles_to_try(usb_type)

        ensure_device_ready(rs, dev, self.serial)

        if is_usb2(usb_type):
            print(
                f"[RealSense] 警告: {device_name} SN={self.serial} 为 USB {usb_type}。"
                "使用 RGB-D 开流（与 get_object 相同）。建议改插 USB 3.0 口。"
            )

        self._bandwidth = AdaptiveBandwidthState(
            usb_type=usb_type,
            requested=self._requested,
            candidates=candidates,
            active=candidates[0],
        )

        timeout_ms = default_frame_timeout_ms(usb_type, self.frame_timeout_ms)
        last_error: Exception | None = None

        for profile_cfg in candidates:
            profile = self._try_start(profile_cfg, timeout_ms)
            if profile is not None:
                note = ""
                if profile_cfg != self._requested:
                    note = f"（请求 {self._requested.label()}，已自适应）"
                print(
                    f"[RealSense] 已打开 {device_name} SN={self.serial} "
                    f"RGB-D {profile_cfg.label()}{note}\n"
                    f"           {format_adaptive_summary(self._bandwidth)}"
                )
                return profile
            last_error = RuntimeError(f"{profile_cfg.label()} 无法出图")

        print("[RealSense] RGB-D 开流失败，尝试 hardware_reset ...")
        recover_busy_device(dev)
        find_realsense_device(serial=self.serial, model_hint="D455")
        time.sleep(1.0)

        for profile_cfg in candidates[:4]:
            profile = self._try_start(profile_cfg, timeout_ms * 2)
            if profile is not None:
                print(
                    f"[RealSense] 复位后已打开 RGB-D {profile_cfg.label()} "
                    f"(USB {usb_type})"
                )
                return profile

        raise RuntimeError(
            f"RealSense 无法出图 (SN={self.serial}, USB {usb_type})。\n"
            "get_object 用法: depth+color @ 640x480@30；请确认相机未被占用。\n"
            "运行: python3 scripts/check_realsense.py"
        ) from last_error

    def _try_start(
        self,
        profile_cfg: ColorStreamProfile,
        timeout_ms: int,
    ) -> Any | None:
        rs = self.rs
        for attempt in range(2):
            try:
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
                time.sleep(0.3)
                self.pipeline = rs.pipeline()
                profile = start_pipeline(
                    rs, self.pipeline, self.serial, profile_cfg, rgbd=True
                )
                if wait_color_frame(
                    self.pipeline, timeout_ms, align=self._align
                ):
                    self._bandwidth.active = profile_cfg
                    return profile
                self.pipeline.stop()
            except Exception as exc:
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
                self.pipeline = rs.pipeline()
                if _is_device_busy(exc) and attempt == 0:
                    time.sleep(1.0)
                    continue
                return None
        return None

    def _warmup(self, frames: int) -> None:
        timeout_ms = self._timeout_ms()
        for i in range(frames):
            try:
                self.pipeline.wait_for_frames(timeout_ms)
            except RuntimeError as exc:
                if i == 0:
                    raise RuntimeError(
                        f"RealSense 预热失败: {exc}。"
                        "请关闭占用相机的程序或改插 USB 3.0 口。"
                    ) from exc

    def _try_runtime_downgrade(self) -> bool:
        from .realsense_bandwidth import select_downgrade

        nxt = select_downgrade(self._bandwidth.candidates, self._bandwidth.active)
        if nxt is None:
            return False
        self._bandwidth.downgrade_count += 1
        print(
            f"[RealSense] 运行时降档 → {nxt.label()} "
            f"({self._bandwidth.downgrade_count} 次)"
        )
        profile = self._try_start(nxt, self._timeout_ms())
        if profile is None:
            return False
        self.profile = profile
        self.width = nxt.width
        self.height = nxt.height
        self.fps = nxt.fps
        self._bandwidth.active = nxt
        return True

    @property
    def intrinsics(self) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        import pyrealsense2 as rs

        from .realsense import intrinsics_from_profile

        profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        K, dist, meta = intrinsics_from_profile(profile)
        meta["serial"] = self.serial
        meta["usb_type"] = self._bandwidth.usb_type
        meta["adaptive_bandwidth"] = self.auto_bandwidth
        meta["requested"] = self._requested.label()
        meta["stream_mode"] = "rgbd"
        return K, dist, meta

    def grab(self, retries: int = 3) -> np.ndarray:
        timeout_ms = self._timeout_ms()
        last_error: Exception | None = None
        for attempt in range(retries):
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms)
                aligned = self._align.process(frames)
                color = aligned.get_color_frame()
                if color:
                    return np.asanyarray(color.get_data())
                last_error = RuntimeError("未获取到彩色帧")
            except RuntimeError as exc:
                last_error = exc
                if attempt + 1 < retries:
                    time.sleep(0.2)
                    continue
                if self.auto_bandwidth and self._try_runtime_downgrade():
                    return self.grab(retries=retries)
        raise last_error or RuntimeError("未获取到彩色帧")

    def save_frame(self, path: str | Path) -> np.ndarray:
        img = self.grab()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), img)
        return img

    def save_intrinsics(self, path: str | Path) -> None:
        K, dist, meta = self.intrinsics
        save_camera_intrinsics(path, K, dist, meta)

    def close(self) -> None:
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def __enter__(self) -> "RealSenseCapture":
        return self

    def __exit__(self, *args) -> None:
        self.close()
