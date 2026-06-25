#!/usr/bin/env python3
"""从已连接的 Intel RealSense D455 读取彩色内参并保存为 JSON。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.realsense import (
    get_d455_color_intrinsics,
    infer_image_size,
    list_realsense_devices,
    save_camera_intrinsics,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="读取 RealSense D455 彩色相机内参")
    parser.add_argument("--list", action="store_true", help="列出已连接设备")
    parser.add_argument("--serial", type=str, default=None, help="设备序列号")
    parser.add_argument("--width", type=int, default=None, help="彩色流宽度")
    parser.add_argument("--height", type=int, default=None, help="彩色流高度")
    parser.add_argument("--fps", type=int, default=None, help="优先匹配的帧率")
    parser.add_argument(
        "--from-image",
        type=Path,
        default=None,
        help="根据图像尺寸自动匹配内参分辨率",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("camera_intrinsics.json"),
        help="输出 JSON 路径",
    )
    args = parser.parse_args()

    if args.list:
        devs = list_realsense_devices()
        if not devs:
            print("未检测到 RealSense 设备")
            return
        for d in devs:
            print(f"{d['name']}  SN={d['serial']}  USB={d['usb']}")
        return

    width, height = args.width, args.height
    if args.from_image is not None:
        width, height = infer_image_size(args.from_image)
        print(f"从图像推断分辨率: {width}x{height}")

    K, dist, meta = get_d455_color_intrinsics(
        width=width,
        height=height,
        serial=args.serial,
        fps=args.fps,
    )
    save_camera_intrinsics(args.output, K, dist, meta)

    print(f"设备: {meta['device_name']} ({meta['serial']})")
    print(f"分辨率: {meta['width']}x{meta['height']} @ {meta['fps']}fps")
    print(f"camera_matrix:\n{K}")
    print(f"dist_coeffs: {dist.tolist()}")
    print(f"已保存 -> {args.output.resolve()}")


if __name__ == "__main__":
    main()
