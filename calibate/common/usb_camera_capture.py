"""USB V4L2 相机采集（OpenCV VideoCapture）。"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .realsense import load_camera_intrinsics_from_file, save_camera_intrinsics


def list_v4l2_devices() -> list[dict[str, Any]]:
    """枚举可打开的 V4L2 视频节点（跳过 metadata 节点）。"""
    sysfs = Path("/sys/class/video4linux")
    if not sysfs.is_dir():
        return []

    devices: list[dict[str, Any]] = []
    for dev_dir in sorted(sysfs.iterdir(), key=lambda p: int(p.name.replace("video", ""))):
        name_file = dev_dir / "name"
        if not name_file.exists():
            continue
        name = name_file.read_text(encoding="utf-8", errors="replace").strip()
        if "realsense" in name.lower():
            continue
        index = int(dev_dir.name.replace("video", ""))
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        ret, _ = cap.read()
        cap.release()
        if not ret:
            continue
        if "realsense" in name.lower():
            continue
        devices.append(
            {
                "index": index,
                "path": f"/dev/video{index}",
                "name": name,
                "width": w,
                "height": h,
                "fps": fps,
            }
        )
    return devices


def find_v4l2_device(
    device: int | str | None = None,
    name_hint: str | None = None,
) -> dict[str, Any]:
    devices = list_v4l2_devices()
    if not devices:
        raise RuntimeError("未检测到可采集的 USB 相机，请检查连接与权限（video 组）")

    if device is not None:
        if isinstance(device, str) and device.startswith("/dev/video"):
            index = int(device.replace("/dev/video", ""))
        else:
            index = int(device)
        for d in devices:
            if d["index"] == index:
                return d
        raise RuntimeError(f"无法打开 USB 相机 {device}，可用: {_format_device_list(devices)}")

    if name_hint:
        hint = name_hint.lower()
        matched = [d for d in devices if hint in d["name"].lower()]
        if len(matched) == 1:
            return matched[0]
        if len(matched) > 1:
            raise RuntimeError(
                f"名称含 '{name_hint}' 的相机有多台: {_format_device_list(matched)}，"
                "请用 --usb-device 指定"
            )

    externals = [
        d
        for d in devices
        if "realsense" not in d["name"].lower()
        and "integrated" not in d["name"].lower()
    ]
    if len(externals) == 1:
        return externals[0]

    raise RuntimeError(
        "无法自动选择 USB 相机，请用 --usb-device 或 config/cameras.json 指定。\n"
        f"已检测到:\n{_format_device_list(devices)}"
    )


def _format_device_list(devices: list[dict[str, Any]]) -> str:
    return "\n".join(
        f"  video{d['index']}: {d['name']} ({d['width']}x{d['height']})"
        for d in devices
    )


class USBCameraCapture:
    """标定用 USB 相机（需预先标定内参 JSON）。"""

    def __init__(
        self,
        device: int | str | None = None,
        name_hint: str | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        intrinsics_file: Path | None = None,
        warmup_frames: int = 5,
        fourcc: str | None = None,
    ) -> None:
        self._device_info = find_v4l2_device(device=device, name_hint=name_hint)
        self.device_index = self._device_info["index"]
        self.device_path = self._device_info["path"]
        self.device_name = self._device_info["name"]
        self.intrinsics_file = intrinsics_file

        self._cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开 {self.device_path}")

        if fourcc:
            fc = cv2.VideoWriter_fourcc(*fourcc[:4])
            self._cap.set(cv2.CAP_PROP_FOURCC, fc)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = int(self._cap.get(cv2.CAP_PROP_FPS)) or fps

        self._K: np.ndarray | None = None
        self._dist: np.ndarray | None = None
        self._meta: dict[str, Any] | None = None
        self._load_intrinsics()

        self._warmup(warmup_frames)
        print(
            f"[USB] 已打开 {self.device_name} {self.device_path} "
            f"{self.width}x{self.height}@{self.fps}fps "
            f"fx={self._K[0, 0]:.1f}"
        )

    def _load_intrinsics(self) -> None:
        if self.intrinsics_file is not None and self.intrinsics_file.exists():
            K, dist, meta = load_camera_intrinsics_from_file(self.intrinsics_file)
            self._validate_intrinsics_resolution(meta)
            self._K, self._dist, self._meta = K, dist, meta
            return

        raise RuntimeError(
            f"USB 相机需要内参文件，未找到: {self.intrinsics_file}\n"
            "请先运行:\n"
            f"  python tools/calibrate_usb_intrinsics.py "
            f"--device {self.device_index} --width {self.width} --height {self.height} "
            f"--output {self.intrinsics_file or 'data/camera_intrinsics.json'}"
        )

    def _validate_intrinsics_resolution(self, meta: dict[str, Any]) -> None:
        iw = meta.get("width")
        ih = meta.get("height")
        if iw and ih and (int(iw) != self.width or int(ih) != self.height):
            print(
                f"[USB] 警告: 内参分辨率 {iw}x{ih} 与当前流 {self.width}x{self.height} 不一致，"
                "请用相同分辨率重新标定内参"
            )

    def _warmup(self, frames: int) -> None:
        for _ in range(frames):
            self._cap.read()
            time.sleep(0.03)

    @property
    def intrinsics(self) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        assert self._K is not None and self._dist is not None and self._meta is not None
        meta = dict(self._meta)
        meta.update(
            {
                "source": "usb",
                "device_index": self.device_index,
                "device_path": self.device_path,
                "device_name": self.device_name,
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
            }
        )
        return self._K.copy(), self._dist.copy(), meta

    def grab(self, retries: int = 3) -> np.ndarray:
        last_error: Exception | None = None
        for attempt in range(retries):
            ret, frame = self._cap.read()
            if ret and frame is not None:
                return frame
            last_error = RuntimeError(f"{self.device_path} 未获取到帧")
            time.sleep(0.1 * (attempt + 1))
        raise last_error or RuntimeError("未获取到帧")

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
        if self._cap is not None:
            self._cap.release()
            self._cap = None  # type: ignore[assignment]

    def __enter__(self) -> "USBCameraCapture":
        return self

    def __exit__(self, *args) -> None:
        self.close()
