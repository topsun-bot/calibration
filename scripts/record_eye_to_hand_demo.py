#!/usr/bin/env python3
"""
录制眼在手外完整仿真演示视频：标定采集 → 验证报告 → 抓取放置。

输出：output/eye_to_hand_demo.mp4
"""

from __future__ import annotations

import argparse
import os
import sys
import textwrap
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "calibate"))
sys.path.insert(0, str(ROOT / "get_object"))

DEFAULT_OUT = ROOT / "output" / "eye_to_hand_demo.mp4"
FPS = 10
HOLD_FRAMES = FPS  # 1 秒


class VideoBuilder:
    def __init__(self, path: Path, fps: int = FPS, size: tuple[int, int] = (1280, 480)) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.fps = fps
        self.size = size
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(path), fourcc, fps, size)
        if not self.writer.isOpened():
            raise RuntimeError(f"无法创建视频: {path}")

    def add(self, frame: np.ndarray, repeat: int = 1) -> None:
        img = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        for _ in range(repeat):
            self.writer.write(img)

    def add_title(self, title: str, subtitle: str = "", repeat: int = HOLD_FRAMES) -> None:
        canvas = np.zeros((self.size[1], self.size[0], 3), dtype=np.uint8)
        canvas[:] = (28, 32, 38)
        cv2.putText(
            canvas, title, (40, self.size[1] // 2 - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (240, 240, 240), 2, cv2.LINE_AA,
        )
        if subtitle:
            cv2.putText(
                canvas, subtitle, (40, self.size[1] // 2 + 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 200, 220), 1, cv2.LINE_AA,
            )
        self.add(canvas, repeat=repeat)

    def close(self) -> None:
        self.writer.release()


def _overlay_bar(img: np.ndarray, lines: list[str], phase: str) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]
    bar_h = 28 + 22 * min(len(lines), 4)
    cv2.rectangle(out, (0, 0), (w, bar_h), (20, 24, 30), -1)
    cv2.putText(out, phase, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 220, 255), 2, cv2.LINE_AA)
    for i, line in enumerate(lines[:4]):
        cv2.putText(
            out, line, (8, 46 + i * 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA,
        )
    return out


def _compose_dual(left: np.ndarray, right: np.ndarray, label_l: str, label_r: str) -> np.ndarray:
    h = max(left.shape[0], right.shape[0])
    def pad(x):
        if x.shape[0] == h:
            return x
        pad_h = h - x.shape[0]
        return cv2.copyMakeBorder(x, 0, pad_h, 0, 0, cv2.BORDER_CONSTANT, value=(30, 30, 30))
    left, right = pad(left), pad(right)
    w = left.shape[1] + right.shape[1]
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:, : left.shape[1]] = left
    canvas[:, left.shape[1] :] = right
    cv2.putText(canvas, label_l, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    cv2.putText(canvas, label_r, (left.shape[1] + 8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return canvas


def _report_slide(report_text: str, size: tuple[int, int]) -> np.ndarray:
    canvas = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    canvas[:] = (24, 28, 34)
    y = 36
    for line in report_text.splitlines()[:22]:
        cv2.putText(canvas, line[:95], (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 215, 220), 1, cv2.LINE_AA)
        y += 20
    return canvas


class RecordingController:
    """在抓取流程中插入视频帧采集。"""

    def __init__(self, controller, env, video: VideoBuilder) -> None:
        self.ctrl = controller
        self.env = env
        self.video = video
        self._orig_move = controller._move_to_position
        self._orig_detect = controller.detect_target
        self._orig_set_gripper = controller._set_gripper
        self._orig_env_move = env.move_joints_deg
        self.grasp_verified = False
        controller._move_to_position = self._move_with_capture  # type: ignore[method-assign]
        controller.detect_target = self._detect_with_capture  # type: ignore[method-assign]
        controller._set_gripper = self._set_gripper_with_capture  # type: ignore[method-assign]
        env.move_joints_deg = self._env_move_with_frames  # type: ignore[method-assign]

    def _grasp_status(self) -> str:
        return "GRASPED" if self.env._grasp_attached else "open"

    def _env_move_with_frames(self, target_deg, steps=15, settle=True):
        import mujoco

        target = np.asarray(target_deg, dtype=np.float64).reshape(-1)
        start = self.env.read_joints_deg()
        for s in range(1, steps + 1):
            alpha = s / steps
            q = start + alpha * (target - start)
            self.env.set_joints_deg(q)
            for _ in range(4):
                mujoco.mj_step(self.env.model, self.env.data)
                self.env._update_grasp_physics()
            if s % 2 == 0 or s == steps:
                self._capture("Moving...", [f"step {s}/{steps}", self._grasp_status()])

    def _set_gripper_with_capture(self, j6: float, q):
        label = "Gripper CLOSE" if j6 > -20 else "Gripper OPEN"
        q = self._orig_set_gripper(j6, q)
        self.env._update_grasp_physics()
        extra = [f"j6={j6:.1f}°", self._grasp_status()]
        if "CLOSE" in label and self.env._grasp_attached:
            extra.append("bottle attached OK")
            self.grasp_verified = True
        self._capture(label, extra)
        return q

    def _detect_with_capture(self, target_class: str):
        obj = self._orig_detect(target_class)
        self._capture(
            "Detection OK",
            [
                f"target {obj.class_name} conf={obj.confidence:.2f}",
                f"base {obj.position_base.round(3).tolist()}",
            ],
        )
        return obj

    def _capture(self, phase: str, extra: list[str]) -> None:
        overview = self.env.render_overview()
        cam = self.env.render_rgb()
        q = self.ctrl.arm.read_joints()
        lines = [f"joints(deg) {q.round(1).tolist()}"] + extra
        dual = _compose_dual(overview, cam, "Overview", "Fixed Cam (Eye-to-Hand)")
        frame = _overlay_bar(dual, lines, phase)
        self.video.add(frame, repeat=max(2, FPS // 4))

    def _move_with_capture(self, target_base, seed, label, j6=None):
        q = self._orig_move(target_base, seed, label, j6=j6)
        self._capture(f"Pick: {label}", [f"TCP target {np.asarray(target_base).round(3).tolist()}"])
        return q


def record_demo(
    output: Path,
    num_calib_poses: int = 6,
    target_class: str = "bottle",
) -> Path:
    os.environ.setdefault("MUJOCO_GL", "egl")
    from common.calib_report import build_calibration_report
    from pick_place.controller import PickPlaceController, run_eye_to_hand_calibration
    from sim.mujoco_env import create_sim_env

    video = VideoBuilder(output)
    data_dir = ROOT / "output" / "demo_calib_data"
    data_dir.mkdir(parents=True, exist_ok=True)

    video.add_title(
        "Eye-to-Hand Demo",
        "MuJoCo Sim: Calibrate + Pick & Place",
        repeat=HOLD_FRAMES,
    )

    env = create_sim_env(headless=True)

    # --- Phase 1: 标定采集预览 ---
    video.add_title("Phase 1/3", "Eye-to-Hand Calibration Collection", repeat=HOLD_FRAMES // 2)

    calib_path = run_eye_to_hand_calibration(
        sim=True,
        data_dir=data_dir,
        show_debug=False,
        show_result=False,
        num_poses=num_calib_poses,
    )

    img_dir = data_dir / "images"
    if img_dir.exists():
        for i, img_path in enumerate(sorted(img_dir.glob("*.png"))[:num_calib_poses]):
            calib_img = cv2.imread(str(img_path))
            if calib_img is None:
                continue
            overview = env.render_overview()
            dual = _compose_dual(overview, calib_img, "Sim Scene", "Calib Camera (Synthetic Board)")
            frame = _overlay_bar(dual, [f"Sample {i+1}/{num_calib_poses}", str(img_path.name)], "Calibration")
            video.add(frame, repeat=max(4, FPS // 2))

    # --- Phase 2: 标定报告 ---
    report = build_calibration_report(calib_path, data_dir)
    report_path = calib_path.parent / "calibration_report.txt"
    report_text = report_path.read_text(encoding="utf-8") if report_path.exists() else str(report)
    summary = (
        f"Quality: {report.quality} | verified {report.verification.num_verified if report.verification else 0} poses"
    )
    if report.verification:
        summary += f" | mean residual {report.verification.residual_mean_m*1000:.1f} mm"
    video.add_title("Phase 2/3", "Calibration Report", repeat=HOLD_FRAMES // 2)
    slide = _report_slide(report_text, video.size)
    frame = _overlay_bar(slide, textwrap.wrap(summary, 70), "T_cam_base Verification")
    video.add(frame, repeat=HOLD_FRAMES * 2)

    # --- Phase 3: 抓取放置 ---
    video.add_title("Phase 3/3", "YOLO 3D Detection + Pick & Place", repeat=HOLD_FRAMES // 2)
    env.reset()
    env.reset_bottle_home()
    ctrl = PickPlaceController(
        calib_path=calib_path,
        mock_arm=True,
        show_debug=False,
        sim_env=env,
    )
    recorder = RecordingController(ctrl, env, video)
    try:
        recorder._capture("Observe", ["Moving to observe pose"])
        result = ctrl.pick_and_place(target_class)
        if not recorder.grasp_verified:
            raise RuntimeError("演示抓取失败：夹爪闭合后瓶子未附着")
    finally:
        env.move_joints_deg = recorder._orig_env_move  # type: ignore[method-assign]
        ctrl.close()

    video.add_title(
        "All Tests Passed",
        f"Picked {result['target_class']} -> placed at {result['place_position_base']}",
        repeat=HOLD_FRAMES * 2,
    )
    video.close()
    env.close()
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="录制眼在手外 MuJoCo 演示视频")
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--poses", type=int, default=6, help="标定采集帧数（演示用）")
    parser.add_argument("--class", dest="target_class", default="bottle")
    args = parser.parse_args()

    t0 = time.time()
    out = record_demo(args.output.resolve(), num_calib_poses=args.poses, target_class=args.target_class)
    print(f"[OK] 视频已保存 -> {out} ({time.time()-t0:.1f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
