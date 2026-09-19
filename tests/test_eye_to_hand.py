"""眼在手外 (Eye-to-Hand) 专项测试。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
CALIB_DIR = ROOT / "calibate" / "eye_to_hand"
PY = sys.executable


def test_t_cam_base_roundtrip_sim():
    """仿真 GT 外参：相机↔基座坐标往返误差应极小。"""
    pytest.importorskip("mujoco")
    from sim.mujoco_env import create_sim_env

    env = create_sim_env(headless=True)
    try:
        p_base = env.data.xpos[env.model.body("bottle").id].copy()
        p_cam = env.object_cam_position("bottle")
        T = env.T_cam_base_render
        R, t = T[:3, :3], T[:3, 3]
        recovered = R @ p_cam + t
        err_mm = float(np.linalg.norm(recovered - p_base) * 1000)
        assert err_mm < 1.0, f"往返误差 {err_mm:.2f} mm"
    finally:
        env.close()


def test_cam_to_base_matches_gt():
    """EyeToHandVision 在仿真下应输出与 GT 一致的基座坐标。"""
    pytest.importorskip("mujoco")
    from pick_place.vision import EyeToHandVision
    from sim.mujoco_env import create_sim_env

    env = create_sim_env(headless=True)
    calib = CALIB_DIR / "output" / "T_cam_base.json"
    if not calib.exists():
        pytest.skip("无标定文件，请先运行标定或仿真测试")
    try:
        vision = EyeToHandVision(calib_path=calib, sim_env=env)
        objects, _ = vision.detect_in_base(target_class="bottle", settle_frames=1)
        assert objects, "仿真应检测到 bottle"
        gt = env.get_object_in_cam("bottle").position_base
        err_mm = float(np.linalg.norm(objects[0].position_base - gt) * 1000)
        assert err_mm < 5.0, f"检测与 GT 偏差 {err_mm:.2f} mm"
        vision.close()
    finally:
        env.close()


def test_verify_report_fields(demo_data_dir):
    """verify / build_calibration_report 应产出完整眼在手外验证字段。"""
    from common.calib_report import build_calibration_report

    calib_dir = CALIB_DIR
    out = calib_dir / "output" / "T_cam_base.json"
    if not out.exists():
        subprocess.run(
            [PY, str(calib_dir / "generate_demo_data.py")],
            check=True,
            cwd=calib_dir,
        )
        subprocess.run(
            [PY, str(calib_dir / "calibrate.py"), "--data-dir", "data"],
            check=True,
            cwd=calib_dir,
        )
    report = build_calibration_report(out, calib_dir / "data")
    assert report.calib_type == "eye_to_hand"
    assert report.T_cam_base.shape == (4, 4)
    assert report.verification is not None
    assert report.verification.num_verified >= 3
    assert report.quality in ("优秀", "良好", "可用", "较差")
    assert report.result_path.name == "T_cam_base.json"


@pytest.fixture(scope="module")
def demo_data_dir():
    subprocess.run([PY, str(CALIB_DIR / "generate_demo_data.py")], check=True, cwd=CALIB_DIR)
    return CALIB_DIR / "data"


def test_run_all_sim_integration(tmp_path):
    """pick_place/run.py all --sim 应无交互完成标定+抓取。"""
    pytest.importorskip("mujoco")
    import os

    # Keep the runner backend (osmesa on CI). Do not force EGL.
    env = {"MUJOCO_GL": os.environ.get("MUJOCO_GL", "osmesa")}
    proc = subprocess.run(
        [
            PY,
            str(ROOT / "pick_place" / "run.py"),
            "all",
            "--class",
            "bottle",
            "--sim",
            "--no-show",
            "--no-show-result",
        ],
        cwd=ROOT,
        env={**dict(**__import__("os").environ), **env},
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert proc.returncode == 0, proc.stderr or proc.stdout
    assert "抓取放置成功" in proc.stdout or "[完成]" in proc.stdout


def test_cam_to_base_transform():
    """眼在手外核心公式：p_base = R @ p_cam + t。"""
    from common.transforms import transform_points

    R = np.eye(3)
    t = np.array([[0.3], [0.1], [0.5]])
    p_cam = np.array([0.0, 0.0, 0.2])
    p_base = transform_points(R, t, p_cam)
    assert np.allclose(p_base, [0.3, 0.1, 0.7], atol=1e-9)
