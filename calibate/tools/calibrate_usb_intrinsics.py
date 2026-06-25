#!/usr/bin/env python3
"""用棋盘格标定 USB 相机内参，输出 camera_intrinsics.json。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.board import BoardConfig
from common.realsense import save_camera_intrinsics
from common.usb_camera_capture import find_v4l2_device, list_v4l2_devices


def collect_corners(
    cap: cv2.VideoCapture,
    board: BoardConfig,
    num_images: int,
    interval_s: float,
) -> tuple[list[np.ndarray], list[np.ndarray], tuple[int, int]]:
    objp = board.object_points()
    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    print(f"采集 {num_images} 张，每 {interval_s:.1f}s 一张；按 q 提前结束")
    collected = 0
    while collected < num_images:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, board.pattern_size, None)
        vis = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            cv2.drawChessboardCorners(vis, board.pattern_size, corners, found)
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            collected += 1
            print(f"  [{collected}/{num_images}] OK")
            time.sleep(interval_s)
        else:
            cv2.putText(
                vis,
                "show chessboard",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        cv2.imshow("USB intrinsics", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    if image_size is None or len(objpoints) < 5:
        raise RuntimeError(f"有效样本不足 ({len(objpoints)}), 至少需要 5 张")
    return objpoints, imgpoints, image_size


def main() -> None:
    parser = argparse.ArgumentParser(description="USB 相机棋盘格内参标定")
    parser.add_argument("--device", type=str, default=None, help="video 索引或 /dev/videoN")
    parser.add_argument("--name-hint", type=str, default=None, help="按 sysfs 名称匹配，如 2M")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-images", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.8)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    parser.add_argument("--output", type=Path, default=Path("camera_intrinsics.json"))
    parser.add_argument("--list", action="store_true", help="列出 USB 相机后退出")
    args = parser.parse_args()

    if args.list:
        for d in list_v4l2_devices():
            print(f"video{d['index']}: {d['name']} {d['width']}x{d['height']}")
        return

    dev = find_v4l2_device(device=args.device, name_hint=args.name_hint)
    cap = cv2.VideoCapture(dev["index"], cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"打开 {dev['name']} video{dev['index']} -> {w}x{h}")
    board = BoardConfig(cols=args.cols, rows=args.rows, square_size=args.square_size)
    objpoints, imgpoints, image_size = collect_corners(
        cap, board, args.num_images, args.interval
    )
    cap.release()

    flags = cv2.CALIB_RATIONAL_MODEL
    ret, K, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags
    )
    print(f"重投影误差 RMS: {ret:.4f} px")
    print(f"K=\n{K}")
    print(f"dist={dist.ravel()}")

    meta = {
        "source": "usb_calibrated",
        "device_index": dev["index"],
        "device_path": dev["path"],
        "device_name": dev["name"],
        "width": w,
        "height": h,
        "fps": args.fps,
        "reprojection_rms_px": float(ret),
        "num_images": len(objpoints),
        "board": f"{args.cols}x{args.rows}",
    }
    save_camera_intrinsics(args.output, K, dist, meta)
    print(f"已保存 -> {args.output.resolve()}")


if __name__ == "__main__":
    main()
