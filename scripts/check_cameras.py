#!/usr/bin/env python3
"""检查所有可用摄像头并验证取帧能力。

Usage:
    python scripts/check_cameras.py
    python scripts/check_cameras.py --save  # 保存测试帧到 output/
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "calibate"))


def check_usb_cameras() -> list[dict]:
    """检测 USB V4L2 相机。"""
    from common.usb_camera_capture import list_v4l2_devices

    return list_v4l2_devices()


def check_realsense() -> list[dict]:
    """检测 RealSense 设备。"""
    devices = []
    try:
        import pyrealsense2 as rs

        ctx = rs.context()
        for dev in ctx.query_devices():
            devices.append(
                {
                    "name": dev.get_info(rs.camera_info.name),
                    "serial": dev.get_info(rs.camera_info.serial_number),
                    "fw": dev.get_info(rs.camera_info.firmware_version),
                    "usb": dev.get_info(rs.camera_info.usb_type_descriptor),
                }
            )
    except ImportError:
        pass
    return devices


def try_realsense_pipeline(serial: str | None = None) -> tuple[bool, str]:
    """尝试用 pyrealsense2 pipeline 取帧。"""
    try:
        import pyrealsense2 as rs

        pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        profile = pipeline.start(config)
        for _ in range(5):
            pipeline.wait_for_frames(timeout_ms=15000)
        frames = pipeline.wait_for_frames(timeout_ms=15000)
        pipeline.stop()
        return True, "pipeline OK"
    except Exception as e:
        return False, str(e)


def try_v4l2_realsense() -> tuple[int | None, np.ndarray | None]:
    """尝试用 V4L2 后端直接访问 RealSense RGB 节点。"""
    sysfs = Path("/sys/class/video4linux")
    if not sysfs.is_dir():
        return None, None

    for dev_dir in sorted(sysfs.iterdir()):
        name_file = dev_dir / "name"
        if not name_file.exists():
            continue
        name = name_file.read_text(errors="replace").strip()
        if "realsense" not in name.lower() and "depth" not in name.lower():
            continue
        index = int(dev_dir.name.replace("video", ""))
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None and frame.mean() > 5:
            return index, frame
    return None, None


def main():
    parser = argparse.ArgumentParser(description="检查所有可用摄像头")
    parser.add_argument("--save", action="store_true", help="保存测试帧到 output/")
    args = parser.parse_args()

    output_dir = ROOT / "output"
    if args.save:
        output_dir.mkdir(exist_ok=True)

    print("=" * 60)
    print(" 摄像头检测报告")
    print("=" * 60)

    # 1. USB V4L2 相机
    print("\n[1] USB V4L2 相机:")
    usb_devs = check_usb_cameras()
    if not usb_devs:
        print("   (无)")
    for d in usb_devs:
        print(f"   video{d['index']:2d}: {d['name']:<36} {d['width']}x{d['height']}@{d['fps']:.0f}fps")
        if args.save:
            cap = cv2.VideoCapture(d["index"], cv2.CAP_V4L2)
            ret, frame = cap.read()
            cap.release()
            if ret:
                path = output_dir / f"test_video{d['index']}.jpg"
                cv2.imwrite(str(path), frame)
                print(f"         → 已保存 {path.name}")

    # 2. RealSense
    print("\n[2] Intel RealSense:")
    rs_devs = check_realsense()
    if not rs_devs:
        print("   (未检测到，或 pyrealsense2 未安装)")
    for d in rs_devs:
        print(f"   {d['name']}  SN={d['serial']}  FW={d['fw']}  USB={d['usb']}")

        # 尝试 pipeline
        ok, msg = try_realsense_pipeline(d["serial"])
        if ok:
            print(f"   → pyrealsense2 pipeline: ✅")
        else:
            print(f"   → pyrealsense2 pipeline: ❌ ({msg[:60]})")
            # 尝试 V4L2 回退
            idx, frame = try_v4l2_realsense()
            if idx is not None:
                print(f"   → V4L2 回退 (video{idx}): ✅ ({frame.shape})")
                if args.save:
                    path = output_dir / f"test_d435i_v4l2.jpg"
                    cv2.imwrite(str(path), frame)
                    print(f"         → 已保存 {path.name}")
            else:
                print(f"   → V4L2 回退: ❌")

    # 3. 总结
    print("\n" + "=" * 60)
    print(" 总结")
    print("=" * 60)
    n_usb = len([d for d in usb_devs if "integrated" not in d["name"].lower()])
    n_rs = len(rs_devs)
    print(f"   外接 USB 相机: {n_usb} 台")
    print(f"   RealSense:     {n_rs} 台")
    print(f"   总可用:        {n_usb + n_rs} 台")

    if n_usb + n_rs >= 2:
        print("\n   ✅ 双摄像头就绪，可执行手眼标定")
        print("      眼在手外 (固定): RealSense D435i")
        print("      眼在手上 (末端): 2M USB 相机")
    elif n_usb + n_rs == 1:
        print("\n   ⚠️  仅检测到 1 台相机，只能做单相机标定")
    else:
        print("\n   ❌ 未检测到可用相机")


if __name__ == "__main__":
    main()
