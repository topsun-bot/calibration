"""实时调试窗口：视频 + 关节角 + 状态信息。"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np

JOINT_LABELS = ["j0", "j1", "j2", "j3", "j4", "j5", "j6(夹爪)"]


@dataclass
class DebugState:
    phase: str = "就绪"
    joints_deg: np.ndarray | None = None
    tcp_position: list[float] | None = None
    sample_index: str = ""
    extra_lines: list[str] = field(default_factory=list)
    hint: str = "q=退出调试窗口"


class DebugDisplay:
    """OpenCV 调试面板：左侧视频、右侧关节与状态。"""

    def __init__(
        self,
        title: str = "Goat Demo Debug",
        enabled: bool = True,
        panel_width: int = 300,
        show_depth: bool = False,
    ) -> None:
        self.enabled = enabled
        self.title = title
        self.panel_width = panel_width
        self.show_depth = show_depth
        self.state = DebugState()
        self._last_frame: np.ndarray | None = None
        self._window_open = False
        self._fps = 0.0
        self._last_t = time.perf_counter()

    def set_phase(self, phase: str) -> None:
        self.state.phase = phase

    def set_joints(self, joints_deg: np.ndarray | list) -> None:
        self.state.joints_deg = np.asarray(joints_deg, dtype=np.float64).reshape(-1)

    def set_tcp(self, position: list | np.ndarray | None) -> None:
        if position is None:
            self.state.tcp_position = None
        else:
            p = np.asarray(position, dtype=np.float64).reshape(3)
            self.state.tcp_position = p.tolist()

    def set_sample_index(self, current: int, total: int | None = None) -> None:
        if total is not None:
            self.state.sample_index = f"{current}/{total}"
        else:
            self.state.sample_index = str(current)

    def set_extra_lines(self, lines: list[str]) -> None:
        self.state.extra_lines = lines

    def add_extra_line(self, line: str) -> None:
        self.state.extra_lines.append(line)

    def update(
        self,
        image: np.ndarray | None = None,
        *,
        phase: str | None = None,
        joints_deg: np.ndarray | list | None = None,
        tcp_position: list | np.ndarray | None = None,
        extra_lines: list[str] | None = None,
        sample_index: tuple[int, int | None] | None = None,
        poll_ms: int = 1,
    ) -> int:
        if not self.enabled:
            return -1
        if image is not None:
            self._last_frame = image.copy()
        if phase is not None:
            self.state.phase = phase
        if joints_deg is not None:
            self.set_joints(joints_deg)
        if tcp_position is not None:
            self.set_tcp(tcp_position)
        if extra_lines is not None:
            self.state.extra_lines = extra_lines
        if sample_index is not None:
            cur, total = sample_index
            self.set_sample_index(cur, total)
        return self.show(poll_ms=poll_ms)

    def show(self, poll_ms: int = 1) -> int:
        if not self.enabled:
            return -1
        if self._last_frame is None:
            self._last_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        now = time.perf_counter()
        self._fps = 0.85 * self._fps + 0.15 / max(now - self._last_t, 1e-6)
        self._last_t = now

        canvas = self._compose(self._last_frame)
        if not self._window_open:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, canvas.shape[1], canvas.shape[0])
            self._window_open = True
        cv2.imshow(self.title, canvas)
        key = cv2.waitKey(max(1, poll_ms))
        if key in (ord("q"), 27):
            self.enabled = False
        return key

    def close(self) -> None:
        if self._window_open:
            cv2.destroyWindow(self.title)
            self._window_open = False

    def _compose(self, video: np.ndarray) -> np.ndarray:
        h, w = video.shape[:2]
        panel = np.zeros((h, self.panel_width, 3), dtype=np.uint8)
        panel[:] = (32, 32, 32)

        y = 28
        cv2.putText(panel, self.state.phase, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 220, 255), 2, cv2.LINE_AA)
        y += 26
        if self.state.sample_index:
            cv2.putText(panel, f"样本 {self.state.sample_index}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)
            y += 22

        cv2.putText(panel, f"FPS {self._fps:.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)
        y += 24
        cv2.line(panel, (8, y), (self.panel_width - 8, y), (80, 80, 80), 1)
        y += 18

        cv2.putText(panel, "关节角 (deg)", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)
        y += 20
        joints = self.state.joints_deg
        for i, label in enumerate(JOINT_LABELS):
            val = "--"
            color = (220, 220, 220)
            if joints is not None and i < len(joints):
                val = f"{joints[i]:7.1f}"
                if i == 6:
                    color = (100, 200, 255)
            cv2.putText(panel, f"{label:8s} {val}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)
            y += 18

        y += 6
        cv2.line(panel, (8, y), (self.panel_width - 8, y), (80, 80, 80), 1)
        y += 18
        cv2.putText(panel, "TCP (base, m)", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)
        y += 20
        if self.state.tcp_position:
            x, yy, z = self.state.tcp_position
            cv2.putText(panel, f"X {x:+.4f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)
            y += 16
            cv2.putText(panel, f"Y {yy:+.4f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)
            y += 16
            cv2.putText(panel, f"Z {z:+.4f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)
            y += 20
        else:
            cv2.putText(panel, "  --", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 120, 120), 1, cv2.LINE_AA)
            y += 20

        if self.state.extra_lines:
            cv2.line(panel, (8, y), (self.panel_width - 8, y), (80, 80, 80), 1)
            y += 18
            cv2.putText(panel, "调试信息", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)
            y += 18
            for line in self.state.extra_lines[:12]:
                if y > h - 24:
                    break
                cv2.putText(panel, line[:38], (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 220, 160), 1, cv2.LINE_AA)
                y += 15

        cv2.putText(panel, self.state.hint, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)

        cv2.putText(video, self.state.phase, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        return np.hstack([video, panel])

    @staticmethod
    def draw_board(
        image: np.ndarray,
        corners: np.ndarray | None,
        detected: bool,
        board_size: tuple[int, int] | None = None,
    ) -> np.ndarray:
        vis = image.copy()
        if detected and corners is not None and board_size is not None:
            cv2.drawChessboardCorners(vis, board_size, corners, True)
            cv2.putText(vis, "Board OK", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(vis, "Board NOT found", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        return vis

    @staticmethod
    def attach_depth(rgb: np.ndarray, depth_u8: np.ndarray) -> np.ndarray:
        if depth_u8.shape[:2] != rgb.shape[:2]:
            depth_u8 = cv2.resize(depth_u8, (rgb.shape[1], rgb.shape[0]))
        return np.hstack([rgb, depth_u8])


class MotionPreview:
    """机械臂运动期间后台刷新关节/画面。"""

    def __init__(
        self,
        debug: DebugDisplay,
        arm,
        frame_reader: Callable[[], np.ndarray | None] | None = None,
        overlay_fn: Callable[[np.ndarray], np.ndarray] | None = None,
        interval_s: float = 0.12,
    ) -> None:
        self.debug = debug
        self.arm = arm
        self.frame_reader = frame_reader
        self.overlay_fn = overlay_fn
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def __enter__(self) -> MotionPreview:
        if not self.debug.enabled:
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *args) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                joints = self.arm.read_joints()
                frame = self.frame_reader() if self.frame_reader else None
                if frame is not None and self.overlay_fn:
                    frame = self.overlay_fn(frame)
                self.debug.update(image=frame, joints_deg=joints)
            except Exception:
                pass
            time.sleep(self.interval_s)


def board_debug_overlay(
    image: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board,
) -> tuple[np.ndarray, list[str]]:
    """标定板检测可视化，返回 (overlay图, 调试行)。

    叠加内容：
    - 棋盘格角点
    - 标定板坐标系轴 (RGB: XYZ)
    - 板面法线与相机光轴夹角
    - 边缘安全边界 (绿/黄/红)
    """
    from .board import detect_board_pose_from_image
    from .board_visibility import board_coverage_in_image

    det = detect_board_pose_from_image(image, K, dist, board)
    lines: list[str] = []
    if det is None:
        vis = DebugDisplay.draw_board(image, None, False)
        lines.append("棋盘: 未检测到")
        return vis, lines

    R, tvec, corners = det
    dist_m = float(np.linalg.norm(tvec))
    depth_m = float(abs(tvec[2, 0]))

    # 基础可视化：角点 + 检测状态
    vis = DebugDisplay.draw_board(image, corners, True, board.pattern_size)

    # 1. 绘制标定板坐标系轴 (ARUCO 风格)
    try:
        rvec, _ = cv2.Rodrigues(R)
        axis_length = float(board.square_size * board.cols * 0.7)
        axis_pts = np.float32([
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, -axis_length],  # Z 指向相机 (板面向外)
        ]).reshape(-1, 3)
        img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, K, dist)
        origin = tuple(int(x) for x in img_pts[0].ravel())
        # 过滤 NaN/Inf (OpenCV 在边缘处可能返回极值)
        if all(-1e6 < c < 1e6 for pt in img_pts for c in pt.ravel()):
            cv2.line(vis, origin, tuple(int(x) for x in img_pts[1].ravel()),
                     (0, 0, 255), 2, cv2.LINE_AA)  # X 红
            cv2.line(vis, origin, tuple(int(x) for x in img_pts[2].ravel()),
                     (0, 255, 0), 2, cv2.LINE_AA)  # Y 绿
            cv2.line(vis, origin, tuple(int(x) for x in img_pts[3].ravel()),
                     (255, 0, 0), 2, cv2.LINE_AA)  # Z 蓝
    except Exception:
        pass

    # 2. 板面法线角度
    cam_z = np.array([0.0, 0.0, 1.0])
    board_normal_cam = R[:, 2]
    cos_angle = np.abs(np.dot(board_normal_cam, cam_z))
    angle_deg = float(np.rad2deg(np.arccos(np.clip(cos_angle, 0.0, 1.0))))

    # 3. 边缘安全边界
    coverage = board_coverage_in_image(image, K, dist, board, margin_ratio=0.06)
    margin_ok = coverage.get("margin_ok", True)
    margin_color = (0, 255, 0) if margin_ok else (0, 165, 255)
    if not margin_ok:
        cv2.rectangle(vis, (4, 4), (vis.shape[1] - 5, vis.shape[0] - 5),
                      margin_color, 4)

    lines.extend(
        [
            "棋盘: 已检测",
            f"距离 {dist_m*1000:.0f} mm  深度 {depth_m*1000:.0f} mm",
            f"法线角 {angle_deg:.0f}° {'✓' if angle_deg < 60 else '⚠ 太斜'}",
            f"t_cam [{tvec[0,0]:+.3f}, {tvec[1,0]:+.3f}, {tvec[2,0]:+.3f}]",
            f"边缘 {'OK' if margin_ok else '⚠ 靠近边缘'}",
        ]
    )
    return vis, lines
