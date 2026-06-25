"""自动采集：D1 运动 + RealSense 拍照 + poses.json（视野保护）。"""

from __future__ import annotations

import json
import random
import threading
import time
from pathlib import Path
from typing import Any, Literal

import cv2
import numpy as np

from .board import BoardConfig, detect_board_pose
from .board_visibility import board_coverage_in_image, is_board_visible
from .gripper_auto import (
    GripperAutoConfig,
    apply_locked_gripper,
    run_auto_gripper_grasp,
)
from .d1_arm import D1Config, create_d1_arm
from .d1_fk import joint_angles_to_robot_pose, load_d1_fk
from .synthetic_board import render_board_for_robot_pose
from .pose_planner import (
    align_seed_poses_base_yaw,
    build_manual_prep_seeds,
    clamp_joints,
    interpolate_joints,
    load_manual_prep_fold_pose,
    load_seed_poses,
    random_pose_near,
    resolve_manual_prep_fold,
    save_poses_config,
)
from .calib_camera import CalibCamera, create_calib_camera

try:
    from .debug_display import DebugDisplay, board_debug_overlay
except ImportError:
    DebugDisplay = None  # type: ignore
    board_debug_overlay = None  # type: ignore

ROOT = Path(__file__).resolve().parents[1]

# mock/仿真 标定 GT（与 D1 URDF FK + 种子位姿自洽）
_DEFAULT_R = np.array(
    [
        [0.956, -0.045, 0.290],
        [0.0, 0.988, 0.154],
        [-0.294, -0.147, 0.944],
    ],
    dtype=np.float64,
)
DEFAULT_MOCK_T_CAM_BASE = np.eye(4)
DEFAULT_MOCK_T_CAM_BASE[:3, :3] = _DEFAULT_R
DEFAULT_MOCK_T_CAM_BASE[:3, 3] = [0.30, 0.15, 0.60]


class AsyncCameraPreview:
    """后台取流，避免手动摆位界面被 wait_for_frames 卡住。"""

    def __init__(self, camera: CalibCamera) -> None:
        self._camera = camera
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="rs-preview")
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._camera.grab()
                with self._lock:
                    self._frame = frame
            except Exception:
                time.sleep(0.05)

    def latest(self) -> np.ndarray | None:
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


class CachedJointReader:
    """降低 read_joints 子进程调用频率。"""

    def __init__(self, arm, interval_s: float = 0.25) -> None:
        self._arm = arm
        self._interval_s = interval_s
        self._lock = threading.Lock()
        self._joints: np.ndarray | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="joint-cache")
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                joints = np.asarray(self._arm.read_joints(), dtype=np.float64)
                with self._lock:
                    self._joints = joints
            except Exception:
                pass
            time.sleep(self._interval_s)

    def latest(self) -> np.ndarray:
        with self._lock:
            if self._joints is not None:
                return self._joints.copy()
        return np.asarray(self._arm.read_joints(), dtype=np.float64)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


PoseMode = Literal["fixed", "adaptive"]


def load_calibration_poses(path: Path) -> list[list[float]]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data["poses"]


def _write_mock_chess_image(
    path: Path,
    index: int,
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    arm=None,
    fk=None,
    T_cam_base: np.ndarray | None = None,
) -> None:
    """生成 OpenCV 可检测的棋盘图（基于当前关节 FK）。"""
    if arm is not None and fk is not None:
        robot_pose = joint_angles_to_robot_pose(arm.read_joints(), fk)
        T = T_cam_base if T_cam_base is not None else DEFAULT_MOCK_T_CAM_BASE
        img = render_board_for_robot_pose(
            robot_pose, T, np.eye(4), K, dist, board
        )
    else:
        # 退化：固定姿态
        angle = index * 0.35
        robot_pose = {
            "position": [0.3 + 0.08 * np.cos(angle), 0.08 * np.sin(angle), 0.2],
            "euler_xyz": [10.0, 0.0, np.rad2deg(angle)],
        }
        img = render_board_for_robot_pose(
            robot_pose,
            T_cam_base or DEFAULT_MOCK_T_CAM_BASE,
            np.eye(4),
            K,
            dist,
            board,
        )
    cv2.imwrite(str(path), img)


def _capture_image(
    camera: CalibCamera | None,
    img_path: Path,
    index: int,
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    mock_camera: bool,
    arm=None,
    fk=None,
    T_cam_base: np.ndarray | None = None,
) -> np.ndarray:
    if mock_camera:
        _write_mock_chess_image(
            img_path, index, board, K, dist, arm=arm, fk=fk, T_cam_base=T_cam_base
        )
        return cv2.imread(str(img_path))
    camera.save_frame(img_path)
    return cv2.imread(str(img_path))


def _verify_frame(
    img: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    board: BoardConfig,
    require_margin: bool,
    margin_ratio: float,
) -> tuple[bool, str]:
    if not is_board_visible(img, K, dist, board):
        return False, "棋盘未检测到"
    if require_margin:
        cov = board_coverage_in_image(img, K, dist, board, margin_ratio)
        if not cov.get("margin_ok", False):
            return False, "棋盘过于靠近图像边缘（可能即将出视野）"
    return True, "OK"


def _move_and_capture(
    arm,
    camera: CalibCamera | None,
    target_joints: np.ndarray,
    from_joints: np.ndarray | None,
    img_path: Path,
    index: int,
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    mock_arm: bool,
    mock_camera: bool,
    interp_steps: int,
    require_margin: bool,
    margin_ratio: float,
    locked_gripper_j6: float | None = None,
    debug: DebugDisplay | None = None,
    fk=None,
    T_cam_base_mock: np.ndarray | None = None,
) -> tuple[bool, np.ndarray | None, np.ndarray | None, str]:
    """
    插值移动到目标关节角，逐段验证；返回 (成功, 实际关节角, 图像, 原因)。
    """
    target_joints = apply_locked_gripper(target_joints, locked_gripper_j6)
    path = [clamp_joints(target_joints)]
    if from_joints is not None and interp_steps > 1:
        from_joints = apply_locked_gripper(from_joints, locked_gripper_j6)
        path = interpolate_joints(from_joints, target_joints, interp_steps)

    last_actual = None
    last_img = None
    for step_idx, q in enumerate(path):
        arm.move_joints(q.tolist(), wait=True)
        actual = arm.read_joints()
        img = _capture_image(
            camera, img_path, index, board, K, dist, mock_camera,
            arm=arm, fk=fk, T_cam_base=T_cam_base_mock,
        )
        ok, reason = _verify_frame(img, K, dist, board, require_margin, margin_ratio)
        last_actual, last_img = actual, img
        if debug is not None and debug.enabled and img is not None:
            vis, dbg_lines = board_debug_overlay(img, K, dist, board)
            debug.update(
                vis,
                phase=f"标定采集 #{index}",
                joints_deg=actual,
                extra_lines=dbg_lines + ([reason] if not ok else ["可见性: OK"]),
                sample_index=(index + 1, None),
            )
        if not ok:
            is_last = step_idx == len(path) - 1
            return False, actual, img, reason if is_last else f"插值第{step_idx+1}步: {reason}"

    return True, last_actual, last_img, "OK"


def _capture_at_current(
    arm,
    camera: CalibCamera | None,
    img_path: Path,
    index: int,
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    mock_camera: bool,
    require_margin: bool,
    margin_ratio: float,
    *,
    fk=None,
    T_cam_base_mock: np.ndarray | None = None,
) -> tuple[bool, np.ndarray, np.ndarray | None, str]:
    """在当前关节角拍照校验，不移动机械臂。"""
    actual = np.asarray(arm.read_joints(), dtype=np.float64)
    img = _capture_image(
        camera, img_path, index, board, K, dist, mock_camera,
        arm=arm, fk=fk, T_cam_base=T_cam_base_mock,
    )
    ok, reason = _verify_frame(img, K, dist, board, require_margin, margin_ratio)
    return ok, actual, img, reason


def _find_anchor_pose(
    arm,
    camera: CalibCamera | None,
    seed_poses: list[np.ndarray],
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    mock_arm: bool,
    mock_camera: bool,
    img_dir: Path,
    try_current_first: bool = True,
    locked_gripper_j6: float | None = None,
    debug: DebugDisplay | None = None,
    fk=None,
    T_cam_base_mock: np.ndarray | None = None,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """寻找第一个棋盘可见的关节配置作为后续扰动中心。"""
    candidates: list[np.ndarray] = []
    if try_current_first:
        candidates.append(apply_locked_gripper(arm.read_joints(), locked_gripper_j6))
    candidates.extend(apply_locked_gripper(p, locked_gripper_j6) for p in seed_poses)

    for i, q in enumerate(candidates):
        print(f"[锚点] 尝试候选 {i + 1}/{len(candidates)}: {q[:6].round(1).tolist()}")
        ok, actual, _, reason = _move_and_capture(
            arm,
            camera,
            q,
            None,
            img_dir / "_anchor_probe.png",
            0,
            board,
            K,
            dist,
            mock_arm,
            mock_camera,
            interp_steps=1,
            require_margin=False,
            margin_ratio=0.05,
            locked_gripper_j6=locked_gripper_j6,
            debug=debug,
            fk=fk,
            T_cam_base_mock=T_cam_base_mock,
        )
        if ok and actual is not None:
            print(f"[锚点] 找到有效位姿: {actual[:6].round(1).tolist()}")
            return actual, actual
        print(f"[锚点] 失败: {reason}")

    return None, None


def _normalize_wait_key(key: int) -> int:
    """OpenCV/Qt 下 Enter 可能是 10/13，也可能带高位标志。"""
    if key < 0:
        return -1
    low = key & 0xFF
    if low in (10, 13) or key in (10, 13):
        return 13
    return low


def _is_confirm_key(key: int) -> bool:
    key = _normalize_wait_key(key)
    return key in (13, ord("s"), ord("S"), ord(" "))


def _start_terminal_confirm_listener(confirm_event) -> None:
    """调试窗口打开时，终端按 Enter 也可确认开始。"""
    import sys
    import threading

    if not sys.stdin.isatty():
        return

    def _listen() -> None:
        try:
            input(">>> 终端按 Enter 也可开始标定: ")
            confirm_event.set()
        except EOFError:
            pass

    threading.Thread(target=_listen, daemon=True).start()


def wait_for_manual_arm_position(
    arm,
    camera: CalibCamera | None,
    board: BoardConfig,
    K: np.ndarray,
    dist: np.ndarray,
    fk,
    *,
    debug: DebugDisplay | None,
    mock_arm: bool,
    mock_camera: bool,
    img_dir: Path,
    T_cam_base_mock: np.ndarray | None = None,
    require_board: bool = False,
    manual_drag: bool = True,
) -> np.ndarray:
    """
    回零后显示实时画面，等待操作者手动摆臂，按 Enter 开始自动标定。
    真机默认开启 funcode=5 卸力拖拽；也可用 [ ] 选关节、+/- 微调。
    """
    import cv2

    from .pose_planner import clamp_joints

    print("\n" + "=" * 60)
    print("[准备] 机械臂已在折叠位姿")
    print("  1. 在调试窗口中查看相机画面")
    print("  2. 倒计时结束后卸力，请扶住再手拖摆位")
    print("  3. [ ] 选关节、+/- 微调（2°）；Enter/Space/s 开始标定")
    print("=" * 60 + "\n")

    if debug is not None:
        debug.set_phase("手动摆位")
        debug.state.hint = "Enter/Space/s=开始  [ ]关节  +/-微调  q=取消"

    import threading

    confirm_event = threading.Event()
    _start_terminal_confirm_listener(confirm_event)

    drag_active = False
    preview: AsyncCameraPreview | None = None
    joint_cache: CachedJointReader | None = None
    if not mock_camera and camera is not None:
        preview = AsyncCameraPreview(camera)
    if not mock_arm:
        joint_cache = CachedJointReader(arm, interval_s=0.2)

    def _read_joints() -> np.ndarray:
        if joint_cache is not None:
            return joint_cache.latest()
        return np.asarray(arm.read_joints(), dtype=np.float64)

    if manual_drag and not mock_arm and hasattr(arm, "set_manual_mode"):
        try:
            arm.set_manual_mode(True, countdown_s=3.0)
            drag_active = True
        except Exception as exc:
            print(f"[D1] 警告: 无法开启手动拖拽 ({exc})")
            print("       请重新编译: ./scripts/build_d1_bridge.sh")
            print("       仍可用键盘 +/- 微调关节")

    selected_joint = 0
    nudge_step = 2.0

    def _nudge_joint(delta: float) -> None:
        nonlocal drag_active
        joints = clamp_joints(_read_joints())
        joints[selected_joint] += delta
        joints = clamp_joints(joints)
        was_drag = drag_active
        try:
            if was_drag:
                arm.set_manual_mode(False)
                drag_active = False
            arm.move_joints(joints, wait=True, quick=True)
        finally:
            if was_drag:
                try:
                    arm.set_manual_mode(True, countdown_s=0.0)
                    drag_active = True
                except Exception as exc:
                    print(f"[D1] 警告: 微调后无法恢复拖拽模式 ({exc})")

    frame_idx = 0
    lines: list[str] = []
    try:
        if debug is None and not mock_camera:
            print("提示: 未开启调试窗口，摆位完成后在本终端按 Enter")
            input(">>> 按 Enter 开始自动标定...")
            return _read_joints()

        vis = None
        joints = np.zeros(7, dtype=np.float64)
        tcp = None
        board_ok = False
        extra = ["init camera..."]

        while True:
            frame_idx += 1
            key = -1
            if debug is not None and debug.enabled:
                key = debug.update(
                    vis,
                    phase="手动摆位",
                    joints_deg=joints,
                    tcp_position=tcp,
                    extra_lines=extra,
                    poll_ms=30,
                )
            elif vis is not None:
                cv2.imshow("标定准备", vis)
                key = cv2.waitKey(30)

            key = _normalize_wait_key(key)

            if key == ord("["):
                selected_joint = (selected_joint - 1) % 6
            elif key == ord("]"):
                selected_joint = (selected_joint + 1) % 6
            elif key in (ord("+"), ord("=")):
                _nudge_joint(nudge_step)
            elif key in (ord("-"), ord("_")):
                _nudge_joint(-nudge_step)
            elif (_is_confirm_key(key) or confirm_event.is_set()) and vis is not None:
                if require_board and not board_ok:
                    print("[准备] 仍未检测到棋盘，请继续调整或更换标定板")
                    confirm_event.clear()
                else:
                    cv2.imwrite(str(img_dir / "_manual_prep.png"), vis)
                    joints = _read_joints()
                    print(f"[准备] 当前关节角: {joints[:6].round(1).tolist()}")
                    if not board_ok:
                        print("[准备] 警告: 开始时棋盘未检测到，后续锚点搜索可能失败")
                    break
            elif key in (ord("q"), ord("Q"), 27):
                raise RuntimeError("用户取消标定")

            if mock_camera:
                prep_path = img_dir / "_manual_prep.png"
                _write_mock_chess_image(
                    prep_path, 0, board, K, dist, arm=arm, fk=fk, T_cam_base=T_cam_base_mock
                )
                img = cv2.imread(str(prep_path))
            elif preview is not None:
                img = preview.latest()
                if img is None:
                    time.sleep(0.02)
                    continue
            else:
                img = camera.grab()

            joints = _read_joints()
            if frame_idx % 4 == 0 or vis is None:
                vis, lines = board_debug_overlay(img, K, dist, board)
                board_ok = bool(lines and lines[0].startswith("棋盘: 已检测"))
            elif vis is not None:
                vis = img.copy()

            tcp = None
            if fk is not None and not mock_arm:
                tcp = joint_angles_to_robot_pose(joints, fk)["position"]

            drag_hint = "drag:on" if drag_active else "drag:off"
            board_line = lines[0] if lines else ("board:ok" if board_ok else "board:no")
            extra = [
                board_line,
                drag_hint,
                f"j{selected_joint}={joints[selected_joint]:.1f}  [ ] +/-",
                "Enter/Space/s start",
            ]
            if not board_ok:
                extra.append("WARN: no chessboard")
    finally:
        if preview is not None:
            preview.close()
        if joint_cache is not None:
            joint_cache.close()
        if drag_active:
            try:
                arm.set_manual_mode(False)
            except Exception as exc:
                print(f"[D1] 警告: 关闭手动拖拽失败 ({exc})")

    if debug is None:
        cv2.destroyWindow("标定准备")
    return joints


def collect_hand_eye_samples(
    data_dir: Path,
    calibration_type: str = "eye_in_hand",
    poses_file: Path | None = None,
    mock_arm: bool = False,
    mock_camera: bool = False,
    network_interface: str | None = None,
    urdf_path: Path | None = None,
    board: BoardConfig | None = None,
    camera_width: int = 640,
    camera_height: int = 480,
    camera_fps: int = 30,
    camera_type: str | None = None,
    camera_config: Path | None = None,
    realsense_serial: str | None = None,
    realsense_model_hint: str | None = None,
    usb_device: int | str | None = None,
    usb_name_hint: str | None = None,
    intrinsics_file: Path | None = None,
    auto_bandwidth: bool = True,
    skip_board_check: bool = False,
    go_zero_first: bool = False,
    pose_mode: PoseMode = "adaptive",
    num_poses: int = 12,
    max_attempts: int = 40,
    max_joint_delta_deg: float = 12.0,
    interp_steps: int = 3,
    require_margin: bool = True,
    margin_ratio: float = 0.06,
    save_valid_poses: bool = True,
    seed: int | None = None,
    auto_gripper: bool = False,
    gripper_config: Path | None = None,
    show_debug: bool = False,
    sim: bool = False,
    sim_env=None,
    align_base_yaw: bool = True,
    manual_prep: bool = False,
    manual_require_board: bool = False,
) -> dict[str, Any]:
    """
    自动采集手眼标定数据。

    pose_mode:
      - adaptive: 以首个可见位姿为锚点，小步随机扰动（推荐，避免出视野）
      - fixed: 按 poses 文件顺序运动，插值+可见性校验，失败则跳过

    auto_gripper (眼在手外):
      采集前用固定相机检测棋盘距离与尺寸，自动预开口并夹紧，标定全程锁定 j6。
    """
    data_dir = data_dir.resolve()
    img_dir = data_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    poses_path = poses_file or (ROOT / "config" / "d1_calibration_poses.json")
    seed_poses = load_seed_poses(poses_path) if poses_path.exists() else []
    board = board or BoardConfig()
    fk = load_d1_fk(urdf_path)
    rng = random.Random(seed)

    cfg = D1Config(network_interface=network_interface)
    samples: list[dict] = []
    valid_joint_poses: list[np.ndarray] = []
    last_success: np.ndarray | None = None
    meta: dict[str, Any] = {
        "calibration_type": calibration_type,
        "pose_mode": pose_mode,
        "num_requested": num_poses if pose_mode == "adaptive" else len(seed_poses),
        "num_collected": 0,
        "skipped_out_of_view": 0,
        "auto_gripper": auto_gripper,
        "gripper_j6_locked": None,
        "sim": sim,
    }

    T_cam_base_mock: np.ndarray | None = None
    _sim_env = sim_env
    camera = None

    if sim:
        import sys

        sys.path.insert(0, str(ROOT.parent))
        from sim.backends import SimCalibCamera, SimD1Arm
        from sim.mujoco_env import create_sim_env

        _sim_env = _sim_env or create_sim_env(headless=True)
        arm = SimD1Arm(_sim_env)
        camera = SimCalibCamera(_sim_env)
        K, dist, cam_meta = camera.intrinsics
        from .realsense import save_camera_intrinsics

        save_camera_intrinsics(
            data_dir / "camera_intrinsics.json", K, dist, cam_meta
        )
        T_cam_base_mock = _sim_env.T_cam_base_gt
        print("[Sim] MuJoCo 仿真采集, GT T_cam_base 已就绪")
    else:
        arm = create_d1_arm(mock=mock_arm, cfg=cfg)
        if mock_camera:
            T_cam_base_mock = DEFAULT_MOCK_T_CAM_BASE

    effective_mock_camera = mock_camera and not sim

    locked_gripper_j6: float | None = None
    manual_prep_joints: np.ndarray | None = None
    gripper_cfg = GripperAutoConfig.load(gripper_config)

    debug: DebugDisplay | None = None
    if (show_debug or manual_prep) and DebugDisplay is not None:
        debug = DebugDisplay(title="标定调试", enabled=True)
        debug.set_phase("标定采集准备")
        debug.state.hint = "Enter=开始标定  q=退出"

    try:
        if sim:
            pass  # camera 已在上面初始化
        elif mock_camera:
            K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float64)
            dist = np.zeros(5)
            camera = None
            from .realsense import save_camera_intrinsics

            save_camera_intrinsics(data_dir / "camera_intrinsics.json", K, dist, {"source": "mock"})
        elif not sim:
            camera = create_calib_camera(
                calibration_type=calibration_type,
                camera_type=camera_type,
                config_file=camera_config,
                width=camera_width,
                height=camera_height,
                fps=camera_fps,
                realsense_serial=realsense_serial,
                realsense_model_hint=realsense_model_hint,
                auto_bandwidth=auto_bandwidth,
                usb_device=usb_device,
                usb_name_hint=usb_name_hint,
                intrinsics_file=intrinsics_file,
            )
            K, dist, cam_meta = camera.intrinsics
            camera.save_intrinsics(data_dir / "camera_intrinsics.json")
            src = cam_meta.get("source", "camera")
            label = cam_meta.get("serial") or cam_meta.get("device_path") or src
            print(
                f"[{src}] 内参 {label} "
                f"{cam_meta.get('width', camera_width)}x{cam_meta.get('height', camera_height)} "
                f"fx={K[0,0]:.2f}"
            )

        if manual_prep and not mock_arm and not sim:
            if go_zero_first:
                fold = resolve_manual_prep_fold(
                    load_manual_prep_fold_pose(poses_path),
                    arm.read_joints(),
                )
                print(
                    f"[D1] 移到折叠准备位姿: {fold[:6].round(1).tolist()} "
                    f"（收拢后再卸力，避免全零伸展下坠）"
                )
                arm.move_joints(fold.tolist(), wait=True)
                time.sleep(0.3)
            manual_joints = wait_for_manual_arm_position(
                arm,
                camera,
                board,
                K,
                dist,
                fk,
                debug=debug,
                mock_arm=mock_arm,
                mock_camera=effective_mock_camera,
                img_dir=img_dir,
                T_cam_base_mock=T_cam_base_mock,
                require_board=manual_require_board,
            )
            manual_prep_joints = np.asarray(manual_joints, dtype=np.float64)
            meta["manual_prep_joints_deg"] = manual_prep_joints.tolist()
            if seed_poses:
                seed_poses = build_manual_prep_seeds(manual_prep_joints, seed_poses)
                meta["manual_prep_seeds"] = True
                print(
                    f"[标定] 以手动摆位为锚点，后续小步扰动不再跳回 config 绝对角"
                )
                print(f"       参考中心: {manual_prep_joints[:6].round(1).tolist()}")
        else:
            if align_base_yaw and not mock_arm and not sim and seed_poses:
                try:
                    current_joints = arm.read_joints()
                    seed_poses, delta_j0 = align_seed_poses_base_yaw(seed_poses, current_joints)
                    meta["base_yaw_align_deg"] = delta_j0
                    if abs(delta_j0) > 1.0:
                        print(
                            f"[标定] 种子位姿 j0 已修正 {delta_j0:+.1f}° "
                            f"(当前 j0={current_joints[0]:.1f}°，对齐现场机械臂与标定板相对朝向)"
                        )
                        print(
                            f"       修正后首个种子: {seed_poses[0][:6].round(1).tolist()}"
                        )
                except Exception as exc:
                    print(f"[标定] 警告: 无法读取关节以修正 j0，沿用 config 种子 ({exc})")

            if go_zero_first and not mock_arm and not sim and seed_poses:
                print("[D1] 回零...")
                arm.go_zero()
                time.sleep(1.0)

        if skip_board_check:
            require_margin = False

        if auto_gripper and calibration_type == "eye_to_hand" and not skip_board_check and not sim:
            print("\n[夹爪] 眼在手外：根据棋盘距离与尺寸自动设置 j6...")
            prep_img_path = img_dir / "_gripper_prep.png"
            prep_img = _capture_image(
                camera, prep_img_path, 0, board, K, dist, effective_mock_camera,
                arm=arm, fk=fk, T_cam_base=T_cam_base_mock,
            )
            board_visible, _ = _verify_frame(
                prep_img, K, dist, board, require_margin=False, margin_ratio=margin_ratio
            )
            if not board_visible:
                print("[夹爪] 跳过: 当前画面未检测到棋盘，沿用当前 j6")
                locked_gripper_j6 = float(arm.read_joints()[6])
            else:
                locked, gripper_meta = run_auto_gripper_grasp(
                    arm,
                    prep_img,
                    K,
                    dist,
                    board,
                    gripper_cfg,
                    mock_arm=mock_arm,
                )
                if locked is None:
                    print(f"[夹爪] 警告: {gripper_meta.get('error', '自动夹爪失败')}，沿用当前 j6")
                    locked_gripper_j6 = float(arm.read_joints()[6])
                else:
                    locked_gripper_j6 = locked
                meta["gripper_estimate"] = gripper_meta
            meta["gripper_j6_locked"] = locked_gripper_j6
            if debug is not None and prep_img is not None and board_visible:
                vis, dbg_lines = board_debug_overlay(prep_img, K, dist, board)
                debug.update(
                    vis,
                    phase="自动夹爪",
                    joints_deg=arm.read_joints(),
                    extra_lines=dbg_lines
                    + [
                        f"j6锁定 {locked_gripper_j6:.1f}°" if locked_gripper_j6 else "j6未锁定",
                    ],
                )

        # ---------- adaptive：锚点 + 小步扰动 ----------
        if pose_mode == "adaptive":
            if manual_prep_joints is not None:
                if debug is not None:
                    debug.set_phase("使用手动摆位锚点")
                anchor = apply_locked_gripper(manual_prep_joints, locked_gripper_j6)
                ok, actual, _, reason = _capture_at_current(
                    arm,
                    camera,
                    img_dir / "_anchor_probe.png",
                    0,
                    board,
                    K,
                    dist,
                    effective_mock_camera,
                    require_margin,
                    margin_ratio,
                    fk=fk,
                    T_cam_base_mock=T_cam_base_mock,
                )
                last_success = actual if actual is not None else anchor
                if not ok and not skip_board_check:
                    raise RuntimeError(
                        f"手动摆位处棋盘不可见: {reason}。"
                        "请换 9x6 棋盘格或加 --skip-board-check 调试"
                    )
                if not ok:
                    print(f"[锚点] 警告: {reason}，仍用手动关节角作为中心")
                else:
                    anchor = last_success
                    print(f"[锚点] 手动摆位 OK: {anchor[:6].round(1).tolist()}")
            else:
                if debug is not None:
                    debug.set_phase("寻找锚点位姿")
                anchor, last_success = _find_anchor_pose(
                    arm,
                    camera,
                    seed_poses,
                    board,
                    K,
                    dist,
                    mock_arm,
                    effective_mock_camera,
                    img_dir,
                    try_current_first=True,
                    locked_gripper_j6=locked_gripper_j6,
                    debug=debug,
                    fk=fk,
                    T_cam_base_mock=T_cam_base_mock,
                )
                if anchor is None:
                    raise RuntimeError(
                        "未找到棋盘可见的初始位姿。请手动调整机械臂或编辑 config/d1_calibration_poses.json"
                    )

            # 保存锚点样本
            img_name = f"{0:04d}.png"
            img_path = img_dir / img_name
            _capture_image(
                camera, img_path, 0, board, K, dist, effective_mock_camera,
                arm=arm, fk=fk, T_cam_base=T_cam_base_mock,
            )
            robot_pose = joint_angles_to_robot_pose(anchor, fk)
            samples.append(
                {
                    "image": img_name,
                    "robot_pose": robot_pose,
                    "joint_angles_deg": anchor.tolist(),
                }
            )
            valid_joint_poses.append(anchor.copy())
            print(f"[1/{num_poses}] 锚点位姿 OK, TCP={robot_pose['position']}")
            if debug is not None:
                debug.update(
                    phase="锚点 OK",
                    joints_deg=anchor,
                    tcp_position=robot_pose["position"],
                    sample_index=(1, num_poses),
                    extra_lines=[f"已采集 {len(samples)}/{num_poses}"],
                )

            attempts = 0
            while len(samples) < num_poses and attempts < max_attempts:
                attempts += 1
                target = apply_locked_gripper(
                    random_pose_near(last_success, max_joint_delta_deg, rng),
                    locked_gripper_j6,
                )
                idx = len(samples)
                img_name = f"{idx:04d}.png"
                img_path = img_dir / img_name
                print(
                    f"\n[{len(samples)+1}/{num_poses}] 尝试扰动 (Δ≤{max_joint_delta_deg}°): "
                    f"{target[:6].round(1).tolist()}"
                )

                ok, actual, _, reason = _move_and_capture(
                    arm,
                    camera,
                    target,
                    last_success,
                    img_path,
                    idx,
                    board,
                    K,
                    dist,
                    mock_arm,
                    effective_mock_camera,
                    interp_steps=interp_steps,
                    require_margin=require_margin and not skip_board_check,
                    margin_ratio=margin_ratio,
                    locked_gripper_j6=locked_gripper_j6,
                    debug=debug,
                    fk=fk,
                    T_cam_base_mock=T_cam_base_mock,
                )
                if not ok:
                    meta["skipped_out_of_view"] += 1
                    print(f"  [跳过] {reason}，回退到上一有效位姿")
                    if debug is not None:
                        debug.set_phase(f"跳过: {reason[:30]}")
                    if last_success is not None:
                        arm.move_joints(last_success.tolist(), wait=True)
                    continue

                robot_pose = joint_angles_to_robot_pose(actual, fk)
                samples.append(
                    {
                        "image": img_name,
                        "robot_pose": robot_pose,
                        "joint_angles_deg": actual.tolist(),
                    }
                )
                valid_joint_poses.append(actual.copy())
                last_success = actual.copy()
                print(f"  [OK] TCP={robot_pose['position']}")
                if debug is not None:
                    debug.update(
                        phase="样本 OK",
                        joints_deg=actual,
                        tcp_position=robot_pose["position"],
                        sample_index=(len(samples), num_poses),
                        extra_lines=[
                            f"TCP {np.round(robot_pose['position'], 4).tolist()}",
                            f"跳过次数 {meta['skipped_out_of_view']}",
                        ],
                    )

        # ---------- fixed：按预设位姿，插值+校验 ----------
        else:
            if not seed_poses:
                raise FileNotFoundError(f"fixed 模式需要位姿文件: {poses_path}")

            for i, waypoint in enumerate(seed_poses):
                print(f"\n[{i + 1}/{len(seed_poses)}] 目标关节角: {waypoint[:6].round(1).tolist()}")
                img_name = f"{i:04d}.png"
                img_path = img_dir / img_name

                if skip_board_check:
                    q = apply_locked_gripper(waypoint, locked_gripper_j6)
                    arm.move_joints(q.tolist(), wait=True)
                    actual = arm.read_joints()
                    _capture_image(
                        camera, img_path, i, board, K, dist, effective_mock_camera,
                        arm=arm, fk=fk, T_cam_base=T_cam_base_mock,
                    )
                    ok, reason = True, "OK"
                else:
                    ok, actual, _, reason = _move_and_capture(
                        arm,
                        camera,
                        waypoint,
                        last_success,
                        img_path,
                        i,
                        board,
                        K,
                        dist,
                        mock_arm,
                        effective_mock_camera,
                        interp_steps=interp_steps,
                        require_margin=require_margin,
                        margin_ratio=margin_ratio,
                        locked_gripper_j6=locked_gripper_j6,
                        debug=debug,
                        fk=fk,
                        T_cam_base_mock=T_cam_base_mock,
                    )

                if not ok:
                    meta["skipped_out_of_view"] += 1
                    print(f"  [跳过] {reason}")
                    if debug is not None:
                        debug.set_phase(f"跳过: {reason[:30]}")
                    if last_success is not None:
                        arm.move_joints(last_success.tolist(), wait=True)
                    continue

                robot_pose = joint_angles_to_robot_pose(actual, fk)
                samples.append(
                    {
                        "image": img_name,
                        "robot_pose": robot_pose,
                        "joint_angles_deg": actual.tolist(),
                    }
                )
                valid_joint_poses.append(actual.copy())
                last_success = actual.copy()
                print(f"  [OK] TCP={robot_pose['position']}")
                if debug is not None:
                    debug.update(
                        phase="样本 OK",
                        joints_deg=actual,
                        tcp_position=robot_pose["position"],
                        sample_index=(len(samples), len(seed_poses)),
                    )

        if save_valid_poses and valid_joint_poses:
            out_poses = data_dir / "valid_poses.json"
            save_poses_config(
                out_poses,
                valid_joint_poses,
                description="本次采集验证通过的有效关节角，可复用于下次标定",
            )
            print(f"\n有效位姿已保存 -> {out_poses}")

        payload = {"samples": samples, "meta": meta}
        with (data_dir / "poses.json").open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

        meta["num_collected"] = len(samples)
        print(
            f"\n采集完成: {len(samples)}/{meta['num_requested']} "
            f"(跳过视野外 {meta['skipped_out_of_view']} 次) -> {data_dir}"
        )
        if debug is not None:
            debug.set_phase(f"采集完成 {len(samples)}/{meta['num_requested']}")
            debug.update(extra_lines=[f"数据目录 {data_dir.name}"])
        return payload

    finally:
        if not mock_arm and not sim and arm is not None and hasattr(arm, "shutdown_safely"):
            try:
                current = arm.read_joints()
                fold = resolve_manual_prep_fold(
                    load_manual_prep_fold_pose(poses_path),
                    current,
                )
                arm.shutdown_safely(fold)
            except Exception as exc:
                print(f"[D1] 退出清理异常: {exc}")
        elif arm is not None:
            arm.close()
        if camera is not None and not sim and hasattr(camera, "close"):
            camera.close()
        if _sim_env is not None:
            _sim_env.close()
        if debug is not None:
            debug.close()
