#!/usr/bin/env python3
"""RealSense D455 连接与 RGB-D 开流预检（与 get_object 一致）。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "calibate"))

from common.realsense import find_realsense_device, list_realsense_devices
from common.realsense_bandwidth import (
    ColorStreamProfile,
    default_frame_timeout_ms,
    is_usb2,
    profile_candidates,
    start_pipeline,
    wait_color_frame,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="检查 RealSense D455 RGB-D 开流")
    parser.add_argument("--serial", type=str, default=None)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("错误: 未安装 pyrealsense2，请执行 pip install pyrealsense2")
        return 1

    devs = list_realsense_devices()
    if not devs:
        print("错误: 未检测到 RealSense。请检查 USB 连接。")
        return 1

    print("已检测到 RealSense:")
    for d in devs:
        print(f"  - {d['name']}  SN={d['serial']}")

    dev = find_realsense_device(serial=args.serial, model_hint="D455")
    usb = dev.get_info(rs.camera_info.usb_type_descriptor)
    serial = dev.get_info(rs.camera_info.serial_number)
    requested = ColorStreamProfile(args.width, args.height, args.fps)
    timeout_ms = default_frame_timeout_ms(usb, None)
    candidates = profile_candidates(
        (requested.width, requested.height, requested.fps), usb
    )
    align = rs.align(rs.stream.color)

    print(f"\nSN={serial}  USB={usb}  模式=RGB-D（depth+color）")
    if is_usb2(usb):
        print("⚠ USB 2.x：与 get_object 相同开流，首帧可能较慢。")

    for profile in candidates:
        pipe = rs.pipeline()
        try:
            start_pipeline(rs, pipe, serial, profile, rgbd=True)
        except Exception as exc:
            print(f"  {profile.label():18s}  启动失败: {exc}")
            continue
        t0 = time.time()
        ok = wait_color_frame(pipe, timeout_ms, align=align)
        elapsed = time.time() - t0
        try:
            pipe.stop()
        except Exception:
            pass
        if ok:
            print(f"  {profile.label():18s}  OK  首帧 {elapsed:.1f}s")
            print("\n预检通过，可运行 auto_calibrate.py")
            return 0
        print(f"  {profile.label():18s}  超时 ({timeout_ms}ms)")

    print("\n预检失败。请关闭占用相机的程序，或换 USB 3.0 口。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
