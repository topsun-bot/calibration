"""Mock 标定端到端（无需 MuJoCo / 真机）。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PY = sys.executable
ROOT = Path(__file__).resolve().parents[1]


def test_mock_arm_camera_calibrate(tmp_path):
    data_dir = tmp_path / "mock_data"
    out_dir = tmp_path / "output"
    out_dir.mkdir()
    cal_dir = ROOT / "calibate" / "eye_to_hand"

    from common.auto_collect import collect_hand_eye_samples

    result = collect_hand_eye_samples(
        data_dir=data_dir,
        calibration_type="eye_to_hand",
        mock_arm=True,
        mock_camera=True,
        skip_board_check=False,
        num_poses=6,
        max_attempts=15,
        max_joint_delta_deg=8.0,
        pose_mode="adaptive",
        show_debug=False,
    )
    assert result["meta"]["num_collected"] >= 3

    out_json = out_dir / "T_cam_base.json"
    subprocess.run(
        [
            PY,
            str(cal_dir / "calibrate.py"),
            "--data-dir",
            str(data_dir),
            "--output",
            str(out_json),
            "--method",
            "tsai",
        ],
        check=True,
        cwd=cal_dir,
    )
    assert out_json.exists()

    from common.calib_report import build_calibration_report

    report = build_calibration_report(out_json, data_dir)
    assert report.verification.num_verified >= 3
    assert report.verification.residual_mean_m < 0.30
