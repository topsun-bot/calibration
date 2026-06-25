#!/usr/bin/env python3
"""下载 Unitree D1 官方 URDF 到 config/d1.urdf。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.d1_urdf import DEFAULT_URDF, download_d1_urdf, ensure_d1_urdf


def main() -> None:
    parser = argparse.ArgumentParser(description="下载 D1 URDF 到 config/")
    parser.add_argument("--output", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--force", action="store_true", help="强制重新下载")
    args = parser.parse_args()

    path = download_d1_urdf(output=args.output, force=args.force)
    print(f"完成: {path.resolve()}")


if __name__ == "__main__":
    main()
