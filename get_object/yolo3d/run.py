#!/usr/bin/env python3
"""YOLO + RealSense 3D 目标检测入口。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from yolo3d.camera import RealSenseCamera
from yolo3d.detector import YOLO3DDetector

WINDOW_NAME = "YOLO 3D Detection"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 3D 目标检测 (RealSense RGB-D)")
    parser.add_argument("--model", default="yolo11n.pt", help="YOLO 模型权重")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--width", type=int, default=640, help="相机宽度")
    parser.add_argument("--height", type=int, default=480, help="相机高度")
    parser.add_argument("--fps", type=int, default=30, help="相机帧率")
    parser.add_argument("--device", default=None, help="推理设备: cpu / cuda / 0")
    parser.add_argument("--save", default="", help="保存可视化视频路径，例如 output.mp4")
    parser.add_argument("--no-show", action="store_true", help="不弹出窗口")
    parser.add_argument("--no-depth", action="store_true", help="不显示深度图面板")
    parser.add_argument("--print-json", action="store_true", help="每帧打印 JSON 结果")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    detector = YOLO3DDetector(model_path=args.model, conf=args.conf, device=args.device)

    writer = None
    show_depth = not args.no_depth
    prev_time = time.perf_counter()
    fps = 0.0

    if not args.no_show:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280 if show_depth else 640, 480)

    try:
        with RealSenseCamera(width=args.width, height=args.height, fps=args.fps) as camera:
            print("RealSense 已启动，实时视频窗口已打开，按 q 退出")
            while True:
                frame = camera.read()
                if frame is None:
                    continue

                detections = detector.detect(frame)

                now = time.perf_counter()
                fps = 0.9 * fps + 0.1 / max(now - prev_time, 1e-6)
                prev_time = now

                vis = detector.build_display(frame, detections, fps, show_depth=show_depth)

                if args.print_json:
                    payload = [
                        {
                            "class": d.class_name,
                            "confidence": d.confidence,
                            "bbox": d.bbox_xyxy,
                            "position_m": {
                                "x": d.position_xyz[0],
                                "y": d.position_xyz[1],
                                "z": d.position_xyz[2],
                            },
                            "size_m": d.size_xyz,
                        }
                        for d in detections
                    ]
                    print(json.dumps(payload, ensure_ascii=False))

                if args.save:
                    if writer is None:
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        writer = cv2.VideoWriter(
                            args.save,
                            fourcc,
                            args.fps,
                            (vis.shape[1], vis.shape[0]),
                        )
                    writer.write(vis)

                if not args.no_show:
                    cv2.imshow(WINDOW_NAME, vis)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
    except KeyboardInterrupt:
        print("\n已停止")
    except RuntimeError as exc:
        msg = str(exc)
        print(f"相机启动失败: {msg}", file=sys.stderr)
        if "busy" in msg.lower() or "errno=16" in msg.lower():
            print("相机被其他进程占用。常见原因：之前用 Ctrl+Z 暂停了程序。", file=sys.stderr)
            print("请执行: pkill -f 'python yolo3d/run.py'  然后重新运行。", file=sys.stderr)
        else:
            print("请确认 RealSense 已连接，且当前用户在 video 组内。", file=sys.stderr)
        return 1
    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
