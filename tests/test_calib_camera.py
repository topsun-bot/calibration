"""标定相机工厂与 USB 枚举测试。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest


def test_resolve_camera_profile_from_config(tmp_path):
    from common.calib_camera import resolve_camera_profile

    cfg = tmp_path / "cameras.json"
    cfg.write_text(
        json.dumps(
            {
                "eye_to_hand": {"type": "realsense", "serial": "ABC"},
                "eye_in_hand": {"type": "usb", "device": 10},
            }
        ),
        encoding="utf-8",
    )
    eth = resolve_camera_profile("eye_to_hand", config_file=cfg)
    eih = resolve_camera_profile("eye_in_hand", config_file=cfg)
    assert eth["type"] == "realsense"
    assert eth["serial"] == "ABC"
    assert eih["type"] == "usb"
    assert eih["device"] == 10


def test_camera_type_override():
    from common.calib_camera import resolve_camera_profile

    profile = resolve_camera_profile("eye_in_hand", camera_type="realsense")
    assert profile["type"] == "realsense"


def test_usb_intrinsics_file_only(tmp_path):
    from common.realsense import load_or_detect_intrinsics, save_camera_intrinsics

    cam_file = tmp_path / "camera_intrinsics.json"
    K = np.array([[900, 0, 640], [0, 900, 360], [0, 0, 1]], dtype=np.float64)
    save_camera_intrinsics(
        cam_file,
        K,
        np.zeros(5),
        {
            "source": "usb_calibrated",
            "device_name": "2M",
            "width": 1280,
            "height": 720,
        },
    )
    K2, _, meta = load_or_detect_intrinsics(
        camera_file=cam_file,
        prefer_live=True,
        use_file_only=False,
    )
    assert meta["source"] == "usb_calibrated"
    assert np.allclose(K, K2)


def test_yolo_detector_skips_ultralytics_when_load_model_false():
    from yolo3d.detector import YOLO3DDetector

    det = YOLO3DDetector(load_model=False)
    assert det.model is None
    assert det.conf == 0.5


def test_list_v4l2_devices():
    from common.usb_camera_capture import list_v4l2_devices

    devices = list_v4l2_devices()
    assert isinstance(devices, list)
    for d in devices:
        assert "index" in d
        assert "name" in d
        assert "width" in d


@pytest.mark.hardware
def test_create_realsense_eye_to_hand():
    """Needs a connected RealSense (or V4L2 fallback). Skipped on GitHub Actions."""
    pytest.importorskip("pyrealsense2")
    from common.realsense import is_realsense_available
    from common.calib_camera import create_calib_camera

    if not is_realsense_available():
        pytest.skip("RealSense camera not connected")

    cam = create_calib_camera(
        calibration_type="eye_to_hand",
        config_file=Path(__file__).resolve().parents[1] / "calibate/config/cameras.json",
    )
    try:
        assert cam.width > 0 and cam.height > 0
        frame = cam.grab()
        assert frame.ndim == 3 and frame.shape[2] == 3
        K, dist, meta = cam.intrinsics
        assert K.shape == (3, 3)
        assert dist.size >= 5
        assert meta.get("serial")
    finally:
        cam.close()


@pytest.mark.hardware
def test_create_usb_eye_in_hand_with_saved_intrinsics(tmp_path):
    """Needs the 2M USB wrist camera. Skipped on GitHub Actions."""
    from common.calib_camera import create_calib_camera
    from common.realsense import save_camera_intrinsics

    intr = tmp_path / "usb_intrinsics.json"
    K = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float64)
    save_camera_intrinsics(
        intr,
        K,
        np.zeros(5),
        {"source": "usb_calibrated", "width": 1280, "height": 720, "device_name": "2M"},
    )

    try:
        from common.usb_camera_capture import find_v4l2_device

        find_v4l2_device(device=10, name_hint="2M")
    except RuntimeError as exc:
        pytest.skip(f"2M USB 相机不可用: {exc}")

    cam = create_calib_camera(
        calibration_type="eye_in_hand",
        camera_type="usb",
        width=1280,
        height=720,
        usb_device=10,
        intrinsics_file=intr,
    )
    try:
        frame = cam.grab()
        assert frame.shape[1] == cam.width
        K2, _, meta = cam.intrinsics
        assert np.allclose(K, K2)
        assert meta["source"] == "usb"
    finally:
        cam.close()
