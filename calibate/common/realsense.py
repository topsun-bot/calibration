"""Intel RealSense（D455 等）彩色相机内参自动读取。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None  # type: ignore
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def _require_rs() -> None:
    if rs is None:
        raise ImportError(
            "未安装 pyrealsense2，请执行: pip install pyrealsense2"
        ) from _IMPORT_ERROR


def list_realsense_devices() -> list[dict[str, str]]:
    _require_rs()
    ctx = rs.context()
    devices = []
    for dev in ctx.query_devices():
        devices.append(
            {
                "name": dev.get_info(rs.camera_info.name),
                "serial": dev.get_info(rs.camera_info.serial_number),
                "usb": dev.get_info(rs.camera_info.physical_port),
            }
        )
    return devices


def find_realsense_device(
    serial: str | None = None,
    model_hint: str = "D435",
) -> "rs.device":
    _require_rs()
    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        raise RuntimeError("未检测到 RealSense 相机，请检查 USB 连接")

    if serial:
        for dev in devs:
            if dev.get_info(rs.camera_info.serial_number) == serial:
                return dev
        raise RuntimeError(f"未找到序列号为 {serial} 的 RealSense 设备")

    if len(devs) == 1:
        return devs[0]

    for dev in devs:
        name = dev.get_info(rs.camera_info.name)
        if model_hint.lower() in name.lower():
            return dev

    names = [d.get_info(rs.camera_info.name) for d in devs]
    raise RuntimeError(
        f"检测到多台 RealSense: {names}，请用 --realsense-serial 指定"
    )


def _rgb_sensor(device: "rs.device") -> "rs.sensor":
    for sensor in device.query_sensors():
        if sensor.get_info(rs.camera_info.name) == "RGB Camera":
            return sensor
    raise RuntimeError("该设备无 RGB Camera 传感器")


def _match_color_profile(
    sensor: "rs.sensor",
    width: int,
    height: int,
    fps: int | None = None,
) -> "rs.video_stream_profile":
    candidates = []
    for profile in sensor.get_stream_profiles():
        if profile.stream_type() != rs.stream.color:
            continue
        vp = profile.as_video_stream_profile()
        if vp.format() not in (rs.format.bgr8, rs.format.rgb8):
            continue
        if vp.width() == width and vp.height() == height:
            candidates.append(vp)

    if not candidates:
        available = sorted(
            {
                (p.as_video_stream_profile().width(), p.as_video_stream_profile().height())
                for p in sensor.get_stream_profiles()
                if p.stream_type() == rs.stream.color
            }
        )
        raise RuntimeError(
            f"未找到 {width}x{height} 彩色流内参，可用分辨率: {available}"
        )

    if fps is not None:
        for vp in candidates:
            if vp.fps() == fps:
                return vp

    return max(candidates, key=lambda p: p.fps())


def intrinsics_from_profile(profile: "rs.video_stream_profile") -> tuple[np.ndarray, np.ndarray, dict]:
    intr = profile.get_intrinsics()
    K = np.array(
        [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
        dtype=np.float64,
    )
    dist = np.asarray(intr.coeffs[:5], dtype=np.float64)
    meta = {
        "source": "realsense",
        "model": str(intr.model),
        "width": profile.width(),
        "height": profile.height(),
        "fps": profile.fps(),
        "format": str(profile.format()),
    }
    return K, dist, meta


def get_d455_color_intrinsics(
    width: int | None = None,
    height: int | None = None,
    serial: str | None = None,
    fps: int | None = None,
    model_hint: str = "D435",
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """
    从已连接的 RealSense D455（或其它型号）读取彩色相机内参。
    若未指定 width/height，默认 640x480。
    """
    _require_rs()
    if width is None:
        width = 640
    if height is None:
        height = 480

    device = find_realsense_device(serial=serial, model_hint=model_hint)
    sensor = _rgb_sensor(device)
    profile = _match_color_profile(sensor, width, height, fps=fps)

    K, dist, meta = intrinsics_from_profile(profile)
    meta.update(
        {
            "device_name": device.get_info(rs.camera_info.name),
            "serial": device.get_info(rs.camera_info.serial_number),
        }
    )
    return K, dist, meta


def infer_image_size(image_path: str | Path) -> tuple[int, int]:
    img = cv2.imread(str(image_path))
    if img is None:
        raise ValueError(f"无法读取图像: {image_path}")
    h, w = img.shape[:2]
    return w, h


def infer_size_from_images_dir(images_dir: str | Path) -> tuple[int, int] | None:
    images_dir = Path(images_dir)
    if not images_dir.is_dir():
        return None
    for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        files = sorted(images_dir.glob(ext))
        if files:
            return infer_image_size(files[0])
    return None


def save_camera_intrinsics(
    path: str | Path,
    K: np.ndarray,
    dist: np.ndarray,
    meta: dict[str, Any] | None = None,
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.reshape(-1).tolist(),
    }
    if meta:
        payload["meta"] = meta
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def is_realsense_available() -> bool:
    """是否已连接 RealSense（可用于动态读内参）。"""
    try:
        _require_rs()
    except ImportError:
        return False
    try:
        return len(list_realsense_devices()) > 0
    except Exception:
        return False


def load_camera_intrinsics_from_file(
    camera_file: Path,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    with camera_file.open("r", encoding="utf-8") as f:
        cam = json.load(f)
    K = np.asarray(cam["camera_matrix"], dtype=np.float64)
    dist = np.asarray(cam.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64)
    meta = cam.get("meta", {"source": "file", "path": str(camera_file)})
    return K, dist, meta


def load_or_detect_intrinsics(
    camera_file: Path | None = None,
    auto_realsense: bool = False,
    images_dir: Path | None = None,
    width: int | None = None,
    height: int | None = None,
    serial: str | None = None,
    fps: int | None = None,
    save_to: Path | None = None,
    prefer_live: bool = True,
    use_file_only: bool = False,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """
    加载相机内参。

    优先级（默认 prefer_live=True，推荐真机流程）：
    1. 已连接 RealSense 且未指定 use_file_only → 从当前设备动态读取（含序列号 SN）
    2. 否则若 camera_file 存在 → 读取 JSON
    3. auto_realsense=True 或文件不存在 → 从 RealSense 读取

    换机场景：若 JSON 中 SN 与当前设备不一致，会提示并改用当前设备内参后写回 JSON。
    离线重算历史数据（无相机）请设 use_file_only=True 或 --use-saved-intrinsics。
    """
    outfile = save_to or camera_file

    if camera_file is not None and camera_file.exists() and not use_file_only:
        _, _, file_meta = load_camera_intrinsics_from_file(camera_file)
        file_source = str(file_meta.get("source", "")).lower()
        if file_source.startswith("usb"):
            K, dist, meta = load_camera_intrinsics_from_file(camera_file)
            print(
                f"[USB] 使用已保存内参 {camera_file} "
                f"({meta.get('device_name', '?')}, {meta.get('width', '?')}x{meta.get('height', '?')})"
            )
            return K, dist, meta

    live_ok = prefer_live and not use_file_only and is_realsense_available()
    use_auto = auto_realsense or camera_file is None or not camera_file.exists()

    if live_ok or use_auto:
        if width is None or height is None:
            if images_dir is not None:
                inferred = infer_size_from_images_dir(images_dir)
                if inferred is not None:
                    width, height = inferred
        K, dist, meta = get_d455_color_intrinsics(
            width=width, height=height, serial=serial, fps=fps
        )
        if (
            live_ok
            and camera_file is not None
            and camera_file.exists()
            and not use_auto
        ):
            _, _, old_meta = load_camera_intrinsics_from_file(camera_file)
            old_sn = old_meta.get("serial")
            new_sn = meta.get("serial")
            if old_sn and new_sn and old_sn != new_sn:
                print(
                    f"[RealSense] 检测到相机更换: SN {old_sn} → {new_sn}，"
                    "已改用当前设备内参（避免不同 D455 偏差）"
                )
            elif old_sn and new_sn and old_sn == new_sn:
                print(
                    f"[RealSense] 动态刷新内参 SN={new_sn} "
                    f"{meta['width']}x{meta['height']} fx={K[0,0]:.2f}"
                )
        else:
            print(
                f"[RealSense] {meta['device_name']} SN={meta['serial']} "
                f"{meta['width']}x{meta['height']} fx={K[0,0]:.2f} fy={K[1,1]:.2f}"
            )
        if outfile is not None:
            save_camera_intrinsics(outfile, K, dist, meta)
        return K, dist, meta

    if camera_file is not None and camera_file.exists():
        K, dist, meta = load_camera_intrinsics_from_file(camera_file)
        print(
            f"[内参] 使用已保存文件 {camera_file} "
            f"(SN={meta.get('serial', '?')}, {meta.get('width', '?')}x{meta.get('height', '?')})"
        )
        return K, dist, meta

    raise RuntimeError(
        "未找到 camera_intrinsics.json 且未连接 RealSense。"
        "请连接相机、提供 --use-saved-intrinsics，或对 USB 相机先运行 tools/calibrate_usb_intrinsics.py"
    )
