"""标定用相机统一接口与工厂（RealSense / USB V4L2）。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMERAS_CONFIG = ROOT / "config" / "cameras.json"


@runtime_checkable
class CalibCamera(Protocol):
    width: int
    height: int
    fps: int

    def grab(self, retries: int = 3) -> np.ndarray: ...

    def save_frame(self, path: str | Path) -> np.ndarray: ...

    @property
    def intrinsics(self) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]: ...

    def save_intrinsics(self, path: str | Path) -> None: ...

    def close(self) -> None: ...


def load_cameras_config(path: Path | None = None) -> dict[str, Any]:
    cfg_path = path or DEFAULT_CAMERAS_CONFIG
    if not cfg_path.exists():
        return {}
    with cfg_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def resolve_camera_profile(
    calibration_type: str | None = None,
    camera_type: str | None = None,
    config_file: Path | None = None,
) -> dict[str, Any]:
    """
    解析相机配置。

    优先级：显式 camera_type / CLI 参数 > config/cameras.json[calibration_type] > 默认 realsense。
    """
    cfg = load_cameras_config(config_file)
    profile: dict[str, Any] = {}

    if calibration_type and calibration_type in cfg:
        entry = cfg[calibration_type]
        if isinstance(entry, dict):
            profile.update(entry)

    if camera_type:
        profile["type"] = camera_type

    if "type" not in profile:
        profile["type"] = "realsense"

    return profile


def create_calib_camera(
    *,
    calibration_type: str | None = None,
    camera_type: str | None = None,
    config_file: Path | None = None,
    width: int = 640,
    height: int = 480,
    fps: int = 30,
    realsense_serial: str | None = None,
    realsense_model_hint: str | None = None,
    auto_bandwidth: bool = True,
    usb_device: int | str | None = None,
    usb_name_hint: str | None = None,
    intrinsics_file: Path | None = None,
    warmup_frames: int | None = None,
) -> CalibCamera:
    profile = resolve_camera_profile(calibration_type, camera_type, config_file)

    cam_type = (camera_type or profile.get("type", "realsense")).lower()
    w = int(profile.get("width", width))
    h = int(profile.get("height", height))
    f = int(profile.get("fps", fps))

    if cam_type == "realsense":
        from .realsense_capture import RealSenseCapture

        return RealSenseCapture(
            width=w,
            height=h,
            fps=f,
            serial=realsense_serial or profile.get("serial"),
            model_hint=realsense_model_hint or profile.get("model_hint", "D435"),
            warmup_frames=warmup_frames,
            auto_bandwidth=profile.get("auto_bandwidth", auto_bandwidth),
        )

    if cam_type in ("usb", "v4l2"):
        from .usb_camera_capture import USBCameraCapture

        intr_path = intrinsics_file or profile.get("intrinsics_file")
        if intr_path is not None:
            intr_path = Path(intr_path)
            if not intr_path.is_absolute():
                intr_path = (ROOT / intr_path).resolve()

        device = usb_device if usb_device is not None else profile.get("device")
        name_hint = usb_name_hint or profile.get("name_hint")

        return USBCameraCapture(
            device=device,
            name_hint=name_hint,
            width=w,
            height=h,
            fps=f,
            intrinsics_file=intr_path,
            warmup_frames=warmup_frames or profile.get("warmup_frames", 5),
        )

    raise ValueError(f"未知相机类型: {cam_type}，可选 realsense / usb")
