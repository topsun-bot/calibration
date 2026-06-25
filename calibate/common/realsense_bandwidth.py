"""RealSense D455 自适应带宽：探测可用分辨率并在运行时降档。"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import pyrealsense2 as rs

# 按带宽从高到低排列的彩色流候选（仅 color / bgr8）
_ALL_PROFILES: tuple[tuple[int, int, int], ...] = (
    (1280, 720, 30),
    (1280, 720, 15),
    (1280, 720, 6),
    (640, 480, 30),
    (640, 480, 15),
    (640, 480, 6),
    (424, 240, 30),
    (424, 240, 15),
    (424, 240, 6),
)

# USB 2.x 仅探测这些（带宽受限）
_USB2_PROBE: tuple[tuple[int, int, int], ...] = (
    (1280, 720, 15),
    (640, 480, 15),
    (640, 480, 30),
    (424, 240, 15),
    (424, 240, 6),
)


@dataclass(frozen=True)
class ColorStreamProfile:
    width: int
    height: int
    fps: int

    @property
    def bandwidth(self) -> int:
        return self.width * self.height * self.fps

    def label(self) -> str:
        return f"{self.width}x{self.height}@{self.fps}fps"


@dataclass
class ProbeStats:
    profile: ColorStreamProfile
    success: int
    attempts: int
    latencies_ms: list[float]

    @property
    def success_rate(self) -> float:
        return self.success / max(1, self.attempts)

    @property
    def median_latency_ms(self) -> float:
        if not self.latencies_ms:
            return float("inf")
        return float(statistics.median(self.latencies_ms))

    @property
    def score(self) -> float:
        """成功率优先，其次带宽，再次延迟。"""
        if self.success == 0:
            return -1.0
        return (
            self.success_rate * 1_000_000
            + self.profile.bandwidth * self.success_rate
            - self.median_latency_ms
        )


def parse_usb_tier(usb_type: str) -> str:
    if usb_type.startswith("2."):
        return "usb2"
    if usb_type.startswith("3."):
        return "usb3"
    return "unknown"


def is_usb2(usb_type: str) -> bool:
    return parse_usb_tier(usb_type) == "usb2"


def default_frame_timeout_ms(usb_type: str, override: int | None = None) -> int:
    if override is not None:
        return override
    return 35000 if is_usb2(usb_type) else 8000


def profile_candidates(
    requested: tuple[int, int, int],
    usb_type: str,
) -> list[ColorStreamProfile]:
    """返回按探测顺序排列的候选流配置（后续可逐级降档）。"""
    rw, rh, rf = requested
    preferred = ColorStreamProfile(rw, rh, rf)

    if is_usb2(usb_type):
        pool = [ColorStreamProfile(*p) for p in _USB2_PROBE]
    else:
        pool = [ColorStreamProfile(*p) for p in _ALL_PROFILES]

    seen: set[tuple[int, int, int]] = set()
    ordered: list[ColorStreamProfile] = []

    def add(p: ColorStreamProfile) -> None:
        key = (p.width, p.height, p.fps)
        if key not in seen:
            seen.add(key)
            ordered.append(p)

    if not is_usb2(usb_type):
        add(preferred)
    for p in pool:
        add(p)
    if is_usb2(usb_type):
        # USB2 从低带宽试到高带宽，探测时反向遍历；降档列表仍保持高→低
        ordered.sort(key=lambda p: p.bandwidth, reverse=True)
    return ordered


def _probe_one_profile(
    rs: Any,
    serial: str,
    profile: ColorStreamProfile,
    *,
    probe_frames: int,
    timeout_ms: int,
) -> ProbeStats:
    stats = ProbeStats(profile=profile, success=0, attempts=0, latencies_ms=[])
    for attempt in range(2):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(
            rs.stream.depth,
            profile.width,
            profile.height,
            rs.format.z16,
            profile.fps,
        )
        cfg.enable_stream(
            rs.stream.color,
            profile.width,
            profile.height,
            rs.format.bgr8,
            profile.fps,
        )
        try:
            pipe.start(cfg)
        except Exception as exc:
            if attempt == 0 and _is_device_busy(exc):
                time.sleep(1.0)
                continue
            return stats

        try:
            for _ in range(probe_frames):
                stats.attempts += 1
                t0 = time.perf_counter()
                try:
                    frames = pipe.wait_for_frames(timeout_ms)
                    color = frames.get_color_frame()
                    elapsed_ms = (time.perf_counter() - t0) * 1000.0
                    if color:
                        stats.success += 1
                        stats.latencies_ms.append(elapsed_ms)
                except RuntimeError:
                    pass
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
            time.sleep(0.2)
        return stats
    return stats


def probe_best_profile(
    rs: Any,
    serial: str,
    candidates: list[ColorStreamProfile],
    *,
    probe_frames: int = 3,
    timeout_ms: int = 8000,
    min_success_rate: float = 0.34,
    pick_first_stable: bool = True,
) -> ProbeStats | None:
    """
    按候选顺序探测彩色流配置。

    pick_first_stable=True（默认）：命中即返回，候选已按 USB 带宽优先级排序。
    pick_first_stable=False：全量探测后取得分最高者。
    """
    results: list[ProbeStats] = []
    for profile in candidates:
        stats = _probe_one_profile(
            rs,
            serial,
            profile,
            probe_frames=probe_frames,
            timeout_ms=timeout_ms,
        )
        if stats.success_rate >= min_success_rate:
            if pick_first_stable:
                return stats
            results.append(stats)

    if not results:
        return None
    return max(results, key=lambda s: s.score)


def select_downgrade(
    candidates: list[ColorStreamProfile],
    current: ColorStreamProfile,
) -> ColorStreamProfile | None:
    """返回比当前更低带宽的下一档配置。"""
    try:
        idx = next(
            i
            for i, p in enumerate(candidates)
            if (p.width, p.height, p.fps) == (current.width, current.height, current.fps)
        )
    except StopIteration:
        return None
    if idx + 1 >= len(candidates):
        return None
    return candidates[idx + 1]


@dataclass
class AdaptiveBandwidthState:
    """记录当前自适应带宽状态，供采集类读写。"""

    usb_type: str
    requested: ColorStreamProfile
    candidates: list[ColorStreamProfile]
    active: ColorStreamProfile
    probe_stats: ProbeStats | None = None
    fail_streak: int = 0
    downgrade_count: int = 0

    @property
    def frame_timeout_ms(self) -> int:
        return default_frame_timeout_ms(self.usb_type)

    def note_success(self) -> None:
        self.fail_streak = 0

    def note_failure(self, threshold: int = 2) -> ColorStreamProfile | None:
        self.fail_streak += 1
        if self.fail_streak < threshold:
            return None
        self.fail_streak = 0
        nxt = select_downgrade(self.candidates, self.active)
        if nxt is None:
            return None
        self.active = nxt
        self.downgrade_count += 1
        return nxt


def recover_busy_device(dev: Any, wait_s: float = 4.0) -> None:
    """设备被占用时尝试 hardware_reset。"""
    try:
        dev.hardware_reset()
        time.sleep(wait_s)
    except Exception:
        pass


def _is_device_busy(exc: BaseException) -> bool:
    msg = str(exc).lower()
    return "busy" in msg or "errno=16" in msg


def ensure_device_ready(
    rs: Any,
    dev: Any,
    serial: str,
    *,
    wait_s: float = 5.0,
) -> None:
    """若设备被占用则 hardware_reset，并短暂等待 V4L2 释放。"""
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    try:
        pipe.start(cfg)
        pipe.stop()
        time.sleep(0.3)
        return
    except Exception as exc:
        try:
            pipe.stop()
        except Exception:
            pass
        if _is_device_busy(exc):
            print("[RealSense] 设备被占用，正在 hardware_reset ...")
            recover_busy_device(dev, wait_s=wait_s)
            find = None
            for _ in range(20):
                ctx = rs.context()
                for d in ctx.query_devices():
                    if d.get_info(rs.camera_info.serial_number) == serial:
                        find = d
                        break
                if find is not None:
                    break
                time.sleep(0.5)
            time.sleep(0.5)


def start_pipeline(
    rs: Any,
    pipeline: "rs.pipeline",
    serial: str,
    profile: ColorStreamProfile,
    *,
    rgbd: bool = True,
) -> Any:
    """启动 pipeline。默认与 get_object 一致：同时开 depth + color。"""
    cfg = rs.config()
    cfg.enable_device(serial)
    if rgbd:
        cfg.enable_stream(
            rs.stream.depth,
            profile.width,
            profile.height,
            rs.format.z16,
            profile.fps,
        )
    cfg.enable_stream(
        rs.stream.color,
        profile.width,
        profile.height,
        rs.format.bgr8,
        profile.fps,
    )
    return pipeline.start(cfg)


def wait_color_frame(
    pipeline: "rs.pipeline",
    timeout_ms: int,
    align: Any | None = None,
) -> bool:
    try:
        frames = pipeline.wait_for_frames(timeout_ms)
        if align is not None:
            frames = align.process(frames)
        return frames.get_color_frame() is not None
    except RuntimeError:
        return False


def format_adaptive_summary(state: AdaptiveBandwidthState) -> str:
    req = state.requested.label()
    act = state.active.label()
    tier = parse_usb_tier(state.usb_type).upper()
    parts = [f"USB {state.usb_type} ({tier})", f"活动流 {act}"]
    if (state.active.width, state.active.height, state.active.fps) != (
        state.requested.width,
        state.requested.height,
        state.requested.fps,
    ):
        parts.append(f"请求 {req} 已自适应调整")
    if state.probe_stats:
        ps = state.probe_stats
        parts.append(
            f"探测成功率 {ps.success}/{ps.attempts} "
            f"中位延迟 {ps.median_latency_ms:.0f}ms"
        )
    if state.downgrade_count:
        parts.append(f"运行时降档 {state.downgrade_count} 次")
    return " | ".join(parts)
