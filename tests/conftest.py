"""pytest 配置。"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))
