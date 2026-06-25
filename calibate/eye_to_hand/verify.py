#!/usr/bin/env python3
"""眼在手外验证：标定板在基座系下位置应一致。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig
from common.calib_report import print_calibration_report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--result", type=Path, default=Path("output/T_cam_base.json"))
    parser.add_argument("--auto-camera", action="store_true")
    parser.add_argument(
        "--use-saved-intrinsics",
        action="store_true",
        help="仅使用 data 目录 JSON，不读当前 RealSense",
    )
    parser.add_argument("--no-save", action="store_true", help="不写入 calibration_report.txt")
    args = parser.parse_args()

    print_calibration_report(
        result_path=args.result.resolve(),
        data_dir=args.data_dir.resolve(),
        board=BoardConfig(),
        save=not args.no_save,
        auto_camera=args.auto_camera,
        use_file_only=args.use_saved_intrinsics,
    )


if __name__ == "__main__":
    main()
