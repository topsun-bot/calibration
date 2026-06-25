"""内参加载逻辑测试（无需 RealSense 硬件）。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest


def test_load_saved_intrinsics_when_file_only(tmp_path):
    from common.realsense import load_or_detect_intrinsics, save_camera_intrinsics

    cam_file = tmp_path / "camera_intrinsics.json"
    K = np.array([[610, 0, 320], [0, 610, 240], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5)
    save_camera_intrinsics(
        cam_file, K, dist, {"serial": "TEST123", "width": 640, "height": 480}
    )

    K2, dist2, meta = load_or_detect_intrinsics(
        camera_file=cam_file,
        prefer_live=False,
        use_file_only=True,
    )
    assert np.allclose(K, K2)
    assert meta["serial"] == "TEST123"


def test_prefer_live_without_device_falls_back_to_file(tmp_path):
    from common.realsense import is_realsense_available, load_or_detect_intrinsics, save_camera_intrinsics

    if is_realsense_available():
        pytest.skip("本机已接 RealSense，跳过无设备回退测试")

    cam_file = tmp_path / "camera_intrinsics.json"
    K = np.eye(3)
    save_camera_intrinsics(cam_file, K, np.zeros(5), {"serial": "OFFLINE"})

    K2, _, meta = load_or_detect_intrinsics(
        camera_file=cam_file,
        prefer_live=True,
        use_file_only=False,
    )
    assert meta["serial"] == "OFFLINE"
    assert np.allclose(K, K2)
