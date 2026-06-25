"""MuJoCo 仿真测试。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def sim_env():
    from sim.mujoco_env import create_sim_env

    env = create_sim_env(headless=True)
    yield env
    env.close()


def test_mujoco_env_loads(sim_env):
    assert sim_env.model is not None
    assert sim_env.T_cam_base_gt.shape == (4, 4)


def test_sim_arm_move(sim_env):
    from sim.backends import SimD1Arm

    arm = SimD1Arm(sim_env)
    q0 = arm.read_joints()
    target = q0.copy()
    target[0] += 10.0
    arm.move_joints(target)
    q1 = arm.read_joints()
    assert abs(q1[0] - target[0]) < 2.0


def test_sim_calib_camera_detectable(sim_env):
    from sim.backends import SimCalibCamera
    from common.board import BoardConfig, detect_board_pose_from_image

    cam = SimCalibCamera(sim_env)
    img = cam.grab()
    K, dist, _ = cam.intrinsics
    det = detect_board_pose_from_image(img, K, dist, BoardConfig())
    assert det is not None


def test_sim_calibration_pipeline(tmp_path):
    from pick_place.controller import run_eye_to_hand_calibration
    from common.calib_report import build_calibration_report

    data_dir = tmp_path / "sim_calib_data"
    calib_path = run_eye_to_hand_calibration(
        sim=True,
        data_dir=data_dir,
        show_debug=False,
        show_result=False,
    )
    assert calib_path.exists()
    report = build_calibration_report(calib_path, data_dir)
    assert report.verification.num_verified >= 3


def test_sim_calibration_with_gt_error(tmp_path):
    """仿真标定应输出 GT 误差对比。"""
    from sim.mujoco_env import create_sim_env
    from sim.sim_calibration import run_sim_calibration
    import json

    env = create_sim_env(headless=True)
    try:
        result = run_sim_calibration(
            env,
            num_poses=8,
            data_dir=tmp_path / "sim_gt",
            save_report=True,
        )
        # 基本检查
        assert result.num_samples >= 3
        assert result.translation_error_mm >= 0
        assert result.rotation_error_deg >= 0
        assert result.R_est.shape == (3, 3)
        assert result.t_est.shape == (3, 1)

        # 各算法 GT 误差
        assert len(result.per_algorithm) >= 1
        for a in result.per_algorithm:
            assert a.translation_error_mm >= 0
            assert a.rotation_error_deg >= 0

        # 运动多样性
        assert "rotation_span_deg" in result.motion_diversity
        assert "translation_span_m" in result.motion_diversity

        # JSON 报告含 sim_gt 字段
        if result.report_json:
            with open(result.report_json) as f:
                data = json.load(f)
            assert "sim_gt" in data["meta"]
            assert "translation_error_mm" in data["meta"]["sim_gt"]
            assert "rotation_error_deg" in data["meta"]["sim_gt"]
            # per_algorithm_gt_error
            assert "per_algorithm_gt_error" in data["meta"]
            assert len(data["meta"]["per_algorithm_gt_error"]) >= 1

        # TXT 报告含仿真对比段
        if result.report_txt:
            txt = result.report_txt.read_text()
            assert "仿真 GT 对比" in txt
            assert "各算法 GT 误差" in txt

        print(f"[OK] GT error: {result.translation_error_mm:.2f}mm, "
              f"{result.rotation_error_deg:.3f}°")
    finally:
        env.close()


def test_sim_motion_diversity_check(sim_env):
    """仿真采集应有足够的运动多样性。"""
    from sim.sim_calibration import _collect_sim_observations
    from common.board import BoardConfig
    from common.hand_eye_solve import check_motion_diversity
    import tempfile, shutil

    tmp = tempfile.mkdtemp()
    try:
        data_dir = Path(tmp) / "diversity_test"
        data_dir.mkdir()
        (data_dir / "images").mkdir()
        board = BoardConfig()

        R_g2b, t_g2b, R_t2c, t_t2c, used, meta = _collect_sim_observations(
            sim_env, data_dir, board, num_poses=6, max_attempts=20
        )
        assert len(R_g2b) >= 3, f"Only {len(R_g2b)} valid samples"

        div = check_motion_diversity(R_g2b, t_g2b)
        print(f"Diversity: rot={div['rotation_span_deg']:.1f}°, "
              f"trans={div['translation_span_m']*1000:.1f}mm")
        # 仿真自适应采集应有足够多样性
        assert div["rotation_span_deg"] > 0
        assert div["translation_span_m"] > 0.005
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_sim_calib_rotation_error_identity():
    """旋转误差计算：相同矩阵 → 0°。"""
    from sim.sim_calibration import _compute_rotation_error_deg
    import numpy as np

    R = np.eye(3)
    assert _compute_rotation_error_deg(R, R) == pytest.approx(0.0, abs=1e-6)

    R2 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)  # 90° Z
    deg = _compute_rotation_error_deg(R, R2)
    assert deg == pytest.approx(90.0, abs=0.1)


def test_sim_pick_place(tmp_path):
    from pick_place.controller import PickPlaceController, run_eye_to_hand_calibration
    from sim.mujoco_env import create_sim_env

    data_dir = tmp_path / "sim_all"
    calib_path = run_eye_to_hand_calibration(
        sim=True,
        data_dir=data_dir,
        show_debug=False,
        show_result=False,
    )
    env = create_sim_env(headless=True)
    ctrl = PickPlaceController(
        calib_path=calib_path,
        mock_arm=True,
        show_debug=False,
        sim_env=env,
    )
    try:
        result = ctrl.pick_and_place("bottle")
        assert result["target_class"] == "bottle"
    finally:
        ctrl.close()
