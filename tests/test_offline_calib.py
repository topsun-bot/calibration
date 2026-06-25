"""离线标定测试（合成数据，无需 MuJoCo）。"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

CALIB_DIR = Path(__file__).resolve().parents[1] / "calibate" / "eye_to_hand"
PY = sys.executable


@pytest.fixture(scope="module")
def demo_data_dir(tmp_path_factory):
    work = tmp_path_factory.mktemp("demo_calib")
    subprocess.run([PY, str(CALIB_DIR / "generate_demo_data.py")], check=True, cwd=CALIB_DIR)
    src = CALIB_DIR / "data"
    shutil.copytree(src, work, dirs_exist_ok=True)
    return work


def test_generate_and_calibrate(demo_data_dir, tmp_path):
    out = tmp_path / "T_cam_base.json"
    subprocess.run(
        [
            PY,
            str(CALIB_DIR / "calibrate.py"),
            "--data-dir",
            str(demo_data_dir),
            "--output",
            str(out),
            "--method",
            "tsai",
        ],
        check=True,
        cwd=CALIB_DIR,
    )
    assert out.exists()

    from common.calib_report import build_calibration_report

    report = build_calibration_report(out, demo_data_dir)
    assert report.verification is not None
    assert report.verification.num_verified >= 3
    assert report.verification.residual_mean_m < 0.30
