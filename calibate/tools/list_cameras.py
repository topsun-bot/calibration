#!/usr/bin/env python3
"""列出本机 RealSense 与 USB V4L2 相机。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main() -> None:
    parser = argparse.ArgumentParser(description="列出标定可用相机")
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config/cameras.json",
        help="显示 config/cameras.json 中的推荐映射",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("Intel RealSense")
    print("=" * 60)
    try:
        from common.realsense import list_realsense_devices

        rs_devs = list_realsense_devices()
        if not rs_devs:
            print("  (无)")
        else:
            for d in rs_devs:
                print(f"  {d['name']}  SN={d['serial']}  {d['usb']}")
    except ImportError:
        print("  pyrealsense2 未安装")

    print("\n" + "=" * 60)
    print("USB V4L2 相机（可采集，不含 RealSense）")
    print("=" * 60)
    from common.usb_camera_capture import list_v4l2_devices

    usb = list_v4l2_devices()
    if not usb:
        print("  (无)")
    else:
        for d in usb:
            print(
                f"  video{d['index']:2d}  {d['name']:<40} "
                f"{d['width']}x{d['height']} @ {d['fps']:.0f}fps  {d['path']}"
            )

    if args.config.exists():
        print("\n" + "=" * 60)
        print(f"推荐配置 ({args.config.name})")
        print("=" * 60)
        with args.config.open("r", encoding="utf-8") as f:
            cfg = json.load(f)
        for role in ("eye_to_hand", "eye_in_hand"):
            if role in cfg:
                print(f"\n  [{role}]")
                for k, v in cfg[role].items():
                    if k.startswith("_"):
                        continue
                    print(f"    {k}: {v}")


if __name__ == "__main__":
    main()
