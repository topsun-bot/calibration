"""pytest 配置。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers",
        "hardware: requires RealSense, USB camera, D1 arm, or GPU/YOLO weights",
    )
