"""D1 抓取放置控制器。"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CALIB_ROOT = ROOT / "calibate"
sys.path.insert(0, str(CALIB_ROOT))

from common.d1_arm import D1Config, create_d1_arm  # noqa: E402
from common.debug_display import DebugDisplay, MotionPreview  # noqa: E402
from common.d1_fk import joint_angles_to_robot_pose, load_d1_fk  # noqa: E402
from common.gripper_auto import GripperAutoConfig, opening_m_to_j6  # noqa: E402
from yolo3d.detector import YOLO3DDetector  # noqa: E402

from .d1_ik import D1InverseKinematics  # noqa: E402
from .vision import EyeToHandVision, ObjectInBase  # noqa: E402

DEFAULT_CONFIG = Path(__file__).parent / "config" / "pick_place.json"
DEFAULT_CALIB = CALIB_ROOT / "eye_to_hand" / "output" / "T_cam_base.json"


@dataclass
class PickPlaceConfig:
    home_joints_deg: list[float]
    observe_joints_deg: list[float]
    place_position_base: list[float]
    place_region: dict[str, float]
    workspace: dict[str, float]
    approach_offset_z: float
    grasp_offset_z: float
    lift_offset_z: float
    tcp_offset_xyz: list[float]
    grasp_margin_m: float
    grasp_compress_m: float
    min_grasp_width_m: float
    max_grasp_width_m: float
    ik_tol_m: float
    settle_s: float
    detection_strategy: str
    detection_frames: int

    @classmethod
    def load(cls, path: Path | None = None) -> PickPlaceConfig:
        p = path or DEFAULT_CONFIG
        with p.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(
            home_joints_deg=data["home_joints_deg"],
            observe_joints_deg=data.get("observe_joints_deg", data["home_joints_deg"]),
            place_position_base=data["place_position_base"],
            place_region=data["place_region"],
            workspace=data["workspace"],
            approach_offset_z=float(data["approach_offset_z"]),
            grasp_offset_z=float(data["grasp_offset_z"]),
            lift_offset_z=float(data["lift_offset_z"]),
            tcp_offset_xyz=data.get("tcp_offset_xyz", [0, 0, -0.04]),
            grasp_margin_m=float(data.get("grasp_margin_m", 0.012)),
            grasp_compress_m=float(data.get("grasp_compress_m", 0.003)),
            min_grasp_width_m=float(data.get("min_grasp_width_m", 0.008)),
            max_grasp_width_m=float(data.get("max_grasp_width_m", 0.060)),
            ik_tol_m=float(data.get("ik_tol_m", 0.004)),
            settle_s=float(data.get("settle_s", 1.5)),
            detection_strategy=data.get("detection_strategy", "highest_confidence"),
            detection_frames=int(data.get("detection_frames", 8)),
        )


def _in_box(point: np.ndarray, box: dict[str, float], eps: float = 1e-3) -> bool:
    x, y, z = point
    return (
        box["x_min"] - eps <= x <= box["x_max"] + eps
        and box["y_min"] - eps <= y <= box["y_max"] + eps
        and box["z_min"] - eps <= z <= box["z_max"] + eps
    )


def run_eye_to_hand_calibration(
    network: str | None = None,
    mock_arm: bool = False,
    mock_camera: bool = False,
    data_dir: Path | None = None,
    show_debug: bool = True,
    show_result: bool = True,
    sim: bool = False,
    num_poses: int | None = None,
) -> Path:
    """眼在手外自标定：采集 + 求解 + 验证。

    仿真模式使用 sim.sim_calibration 模块，直接进行 Python 调用
    （无 subprocess），并自动对比 GT 误差。
    """
    from common.board import BoardConfig

    cal_dir = CALIB_ROOT / "eye_to_hand"

    # ---------- 仿真路径 ----------
    if sim:
        import sys
        sys.path.insert(0, str(ROOT))
        from sim.mujoco_env import create_sim_env
        from sim.sim_calibration import run_sim_calibration

        env = create_sim_env(headless=True)
        try:
            sim_data_dir = data_dir or (cal_dir / "data" / "sim_test")
            result = run_sim_calibration(
                env,
                num_poses=num_poses or 8,
                data_dir=sim_data_dir,
                output_dir=cal_dir / "output",
                save_report=True,
            )
            out = result.report_json
            if out is None or not out.exists():
                raise RuntimeError("仿真标定完成但未生成结果文件")
            return out.resolve()
        finally:
            env.close()

    # ---------- 真机 / Mock 路径 ----------
    import subprocess

    from common.auto_collect import collect_hand_eye_samples
    from common.calib_report import print_calibration_report, show_calibration_result_window

    work_data = (data_dir or cal_dir / "data").resolve()
    board = BoardConfig()

    print("[标定] 启动眼在手外自标定...")
    result = collect_hand_eye_samples(
        data_dir=work_data,
        calibration_type="eye_to_hand",
        mock_arm=mock_arm,
        mock_camera=mock_camera,
        network_interface=network,
        show_debug=show_debug and not mock_camera,
        sim=False,
        num_poses=num_poses or 12,
        max_attempts=40,
        skip_board_check=mock_camera,
    )
    if result["meta"]["num_collected"] < 3:
        raise RuntimeError("有效样本不足 3 组")

    cal_cmd = [
        sys.executable,
        str(cal_dir / "calibrate.py"),
        "--data-dir",
        str(work_data),
    ]
    subprocess.run(cal_cmd, check=True, cwd=cal_dir)

    out = cal_dir / "output" / "T_cam_base.json"
    if not out.exists():
        raise RuntimeError(f"标定求解完成但未找到结果: {out}")

    report = print_calibration_report(
        result_path=out,
        data_dir=work_data,
        board=board,
        collect_meta=result.get("meta"),
        save=True,
    )
    if show_result and not mock_camera:
        show_calibration_result_window(report, title="眼在手外 标定结果")

    return out


class PickPlaceController:
    def __init__(
        self,
        calib_path: Path,
        cfg: PickPlaceConfig | None = None,
        gripper_cfg: GripperAutoConfig | None = None,
        network: str | None = None,
        mock_arm: bool = False,
        urdf_path: Path | None = None,
        yolo_model: str = "yolo11n.pt",
        yolo_conf: float = 0.5,
        yolo_device: str | None = None,
        show_debug: bool = True,
        show_depth: bool = False,
        sim_env=None,
    ) -> None:
        self.cfg = cfg or PickPlaceConfig.load()
        self.gripper_cfg = gripper_cfg or GripperAutoConfig.load()
        if sim_env is not None:
            from sim.backends import SimD1Arm

            self.arm = SimD1Arm(sim_env)
            self.mock_arm = True
        else:
            self.arm = create_d1_arm(mock=mock_arm, cfg=D1Config(network_interface=network))
            self.mock_arm = mock_arm
        self.fk = load_d1_fk(urdf_path=urdf_path)
        self.ik = D1InverseKinematics(urdf_path=urdf_path)
        self.vision = EyeToHandVision(
            calib_path=calib_path,
            model_path=yolo_model,
            conf=yolo_conf,
            device=yolo_device,
            sim_env=sim_env,
        )
        self.sim_env = sim_env
        self.show_depth = show_depth
        self.debug: DebugDisplay | None = None
        if show_debug and sim_env is None:
            self.debug = DebugDisplay(title="抓取调试", enabled=True, show_depth=show_depth)
            self.debug.state.hint = "q=关闭窗口(任务继续)"
        self._last_ik_err_mm: float | None = None
        self._target_class: str | None = None

    def close(self) -> None:
        if self.debug is not None:
            self.debug.close()
        self.vision.close()
        self.arm.close()
        if self.sim_env is not None:
            self.sim_env.close()

    def _tcp_from_joints(self, joints: np.ndarray) -> list[float]:
        pose = joint_angles_to_robot_pose(joints, self.fk)
        return pose["position"]

    def _read_debug_frame(self):
        bundle = self.vision.camera.read()
        if bundle is None:
            return None, None
        return bundle, bundle.color

    def _overlay_detection(self, color: np.ndarray, objects: list[ObjectInBase]) -> np.ndarray:
        dets = [o.detection for o in objects]
        vis = YOLO3DDetector.draw(color, dets)
        for obj in objects:
            pb = obj.position_base
            x1, y1, x2, y2 = obj.detection.bbox_xyxy
            label = f"base [{pb[0]:+.2f},{pb[1]:+.2f},{pb[2]:+.2f}]"
            cv2.putText(
                vis, label, (x1, min(y2 + 18, vis.shape[0] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA,
            )
        return vis

    def _debug_show(
        self,
        phase: str,
        *,
        frame=None,
        objects: list[ObjectInBase] | None = None,
        joints: np.ndarray | None = None,
        extra_lines: list[str] | None = None,
        sample_index: tuple[int, int | None] | None = None,
    ) -> None:
        if self.debug is None or not self.debug.enabled:
            return
        image = None
        if frame is not None:
            color = frame.color if hasattr(frame, "color") else frame
            image = self._overlay_detection(color, objects or [])
            if self.show_depth and hasattr(frame, "depth"):
                depth_vis = YOLO3DDetector.depth_colormap(frame.depth, frame.depth_scale)
                image = DebugDisplay.attach_depth(image, depth_vis)
        if joints is None:
            try:
                joints = self.arm.read_joints()
            except Exception:
                joints = None
        tcp = self._tcp_from_joints(joints) if joints is not None else None
        lines = list(extra_lines or [])
        if self._last_ik_err_mm is not None:
            lines.append(f"IK误差 {self._last_ik_err_mm:.1f} mm")
        if self._target_class:
            lines.append(f"目标类别 {self._target_class}")
        self.debug.update(
            image=image,
            phase=phase,
            joints_deg=joints,
            tcp_position=tcp,
            extra_lines=lines,
            sample_index=sample_index,
        )

    def _apply_tcp_offset(self, target_base: np.ndarray) -> np.ndarray:
        offset = np.asarray(self.cfg.tcp_offset_xyz, dtype=np.float64)
        return target_base + offset

    def _estimate_grasp_j6(self, obj: ObjectInBase) -> tuple[float, float]:
        width_m = self.cfg.min_grasp_width_m
        if obj.size_m:
            width_m = float(min(obj.size_m[0], obj.size_m[1]))
        width_m = float(
            np.clip(width_m, self.cfg.min_grasp_width_m, self.cfg.max_grasp_width_m)
        )
        preopen_m = float(
            np.clip(
                width_m + self.cfg.grasp_margin_m,
                self.cfg.min_grasp_width_m,
                self.gripper_cfg.max_opening_m,
            )
        )
        close_m = float(
            np.clip(
                width_m - self.cfg.grasp_compress_m,
                self.cfg.min_grasp_width_m,
                preopen_m,
            )
        )
        j6_open = opening_m_to_j6(preopen_m, self.gripper_cfg)
        j6_close = opening_m_to_j6(close_m, self.gripper_cfg)
        return j6_open, j6_close

    def _move_to_position(
        self,
        target_base: np.ndarray,
        seed: np.ndarray,
        label: str,
        j6: float | None = None,
    ) -> np.ndarray:
        target = self._apply_tcp_offset(target_base)
        if not _in_box(target, self.cfg.workspace):
            raise RuntimeError(
                f"{label} 目标超出工作空间: {target.tolist()} "
                f"(workspace={self.cfg.workspace})"
            )
        q, err = self.ik.solve_position(target, seed, tol_m=self.cfg.ik_tol_m)
        if j6 is not None:
            q[6] = j6
        self._last_ik_err_mm = err * 1000
        solved_tcp = self.ik.fk_position(q)
        print(f"[运动] {label}: base={target.round(4).tolist()}, IK误差={err*1000:.1f}mm")
        if err > self.cfg.ik_tol_m * 3:
            print(f"[警告] {label} IK 误差偏大 ({err*1000:.1f}mm)，请检查标定或 seed 位姿")

        def _overlay(color: np.ndarray) -> np.ndarray:
            return color

        frame_reader = None
        if self.debug is not None and self.debug.enabled:
            def _overlay(color: np.ndarray) -> np.ndarray:
                vis = color.copy()
                cv2.putText(
                    vis, label, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA,
                )
                cv2.putText(
                    vis, f"IK err {err*1000:.1f}mm", (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA,
                )
                return vis

            def frame_reader() -> np.ndarray | None:
                bundle, color = self._read_debug_frame()
                return _overlay(color) if color is not None else None

            self._debug_show(
                f"运动中: {label}",
                joints=seed,
                extra_lines=[
                    f"目标 {target.round(3).tolist()}",
                    f"IK解 {solved_tcp.round(3).tolist()}",
                    f"j6={q[6]:.1f}°",
                ],
            )

        ctx = (
            MotionPreview(
                self.debug,
                self.arm,
                frame_reader=frame_reader if self.debug else None,
                overlay_fn=_overlay if self.debug else None,
            )
            if self.debug
            else nullcontext()
        )
        with ctx:
            self.arm.move_joints(q.tolist(), wait=True)

        if not self.mock_arm:
            time.sleep(self.cfg.settle_s * 0.3)
        self._debug_show(
            f"到位: {label}",
            joints=q,
            extra_lines=[
                f"目标 {target.round(3).tolist()}",
                f"FK验证 {solved_tcp.round(3).tolist()}",
            ],
        )
        return q

    def _set_gripper(self, j6: float, seed: np.ndarray) -> np.ndarray:
        q = seed.copy()
        q[6] = j6
        self._debug_show(
            f"夹爪 j6={j6:.1f}°",
            joints=seed,
            extra_lines=[f"目标 j6 {j6:.1f}°"],
        )
        with (
            MotionPreview(self.debug, self.arm, frame_reader=lambda: self._read_debug_frame()[1])
            if self.debug
            else nullcontext()
        ):
            self.arm.move_joints(q.tolist(), wait=True)
        if self.sim_env is not None:
            self.sim_env.on_gripper_j6(j6)
        if not self.mock_arm:
            time.sleep(self.cfg.settle_s * 0.5)
        self._debug_show("夹爪到位", joints=q)
        return q

    def detect_target(self, target_class: str) -> ObjectInBase:
        accum: dict[str, list[np.ndarray]] = {}
        best_det: ObjectInBase | None = None
        self._target_class = target_class

        for i in range(self.cfg.detection_frames):
            objects, frame = self.vision.detect_in_base(target_class=target_class, settle_frames=1)
            obj = self.vision.select_best(objects, self.cfg.detection_strategy)
            extra = [f"帧 {i+1}/{self.cfg.detection_frames}"]
            if obj is None:
                extra.append("未检测到目标")
                self._debug_show("检测中...", frame=frame, objects=[], extra_lines=extra, sample_index=(i + 1, self.cfg.detection_frames))
                continue
            key = obj.class_name
            accum.setdefault(key, []).append(obj.position_base)
            if best_det is None or obj.confidence > best_det.confidence:
                best_det = obj
            extra.extend(
                [
                    f"{obj.class_name} conf={obj.confidence:.2f}",
                    f"cam {obj.position_cam.round(3).tolist()}",
                    f"base {obj.position_base.round(3).tolist()}",
                ]
            )
            if obj.size_m:
                extra.append(f"尺寸 {tuple(round(s*1000,1) for s in obj.size_m[:2])} mm")
            print(
                f"[检测] 帧 {i+1}/{self.cfg.detection_frames}: "
                f"{obj.class_name} conf={obj.confidence:.2f} "
                f"base={obj.position_base.round(3).tolist()}"
            )
            self._debug_show(
                "检测中",
                frame=frame,
                objects=objects,
                extra_lines=extra,
                sample_index=(i + 1, self.cfg.detection_frames),
            )

        if best_det is None:
            raise RuntimeError(f"未检测到类别 '{target_class}'，请调整相机/光照/模型")

        if best_det.class_name in accum and len(accum[best_det.class_name]) >= 2:
            mean_pos = np.mean(accum[best_det.class_name], axis=0)
            best_det = ObjectInBase(
                class_name=best_det.class_name,
                confidence=best_det.confidence,
                position_base=mean_pos,
                position_cam=best_det.position_cam,
                size_m=best_det.size_m,
                detection=best_det.detection,
            )
            print(f"[检测] 多帧平均位置 base={mean_pos.round(4).tolist()}")

        self._debug_show(
            "检测完成",
            objects=[best_det] if best_det else [],
            extra_lines=[
                f"最终 base {best_det.position_base.round(4).tolist()}",
                f"置信度 {best_det.confidence:.2f}",
            ],
        )

        if not _in_box(best_det.position_base, self.cfg.workspace):
            raise RuntimeError(
                f"目标在工作空间外: {best_det.position_base.tolist()}"
            )
        return best_det

    def pick_and_place(self, target_class: str) -> dict[str, Any]:
        """完整流程：观测 → 检测 → 抓取 → 放置 → 回 home。"""
        cfg = self.cfg
        place = np.asarray(cfg.place_position_base, dtype=np.float64)
        if not _in_box(place, cfg.place_region):
            raise RuntimeError(
                f"放置点 {place.tolist()} 不在 place_region {cfg.place_region} 内，请修改配置"
            )

        print("[流程] 移动到观测位姿...")
        q = np.asarray(cfg.observe_joints_deg, dtype=np.float64)
        self._debug_show("移动到观测位姿", joints=q)
        with (
            MotionPreview(self.debug, self.arm, frame_reader=lambda: self._read_debug_frame()[1])
            if self.debug
            else nullcontext()
        ):
            self.arm.move_joints(q.tolist(), wait=True)
        if not self.mock_arm:
            time.sleep(cfg.settle_s)
        self._debug_show("观测位姿就绪", joints=q)

        obj = self.detect_target(target_class)
        j6_open, j6_close = self._estimate_grasp_j6(obj)
        print(
            f"[抓取] 目标={obj.class_name}, base={obj.position_base.round(4).tolist()}, "
            f"j6 开={j6_open:.1f}° 闭={j6_close:.1f}°"
        )

        p = obj.position_base.copy()
        q = self.arm.read_joints()

        # 1. 预抓取高度
        pre_grasp = p.copy()
        pre_grasp[2] += cfg.approach_offset_z
        q = self._set_gripper(j6_open, q)
        q = self._move_to_position(pre_grasp, q, "预抓取", j6=j6_open)

        # 2. 下降抓取
        grasp = p.copy()
        grasp[2] += cfg.grasp_offset_z
        q = self._move_to_position(grasp, q, "抓取点", j6=j6_open)

        # 3. 夹紧
        q = self._set_gripper(j6_close, q)

        # 4. 抬起
        lift = grasp.copy()
        lift[2] += cfg.lift_offset_z
        q = self._move_to_position(lift, q, "抬起", j6=j6_close)

        # 5. 移动到放置区上方
        place_pre = place.copy()
        place_pre[2] += cfg.approach_offset_z
        q = self._move_to_position(place_pre, q, "放置预备", j6=j6_close)

        # 6. 下降放置
        q = self._move_to_position(place, q, "放置点", j6=j6_close)

        # 7. 松开
        q = self._set_gripper(j6_open, q)

        # 8. 抬起并回 home
        place_lift = place.copy()
        place_lift[2] += cfg.lift_offset_z * 0.5
        q = self._move_to_position(place_lift, q, "放置后抬起", j6=j6_open)

        home = np.asarray(cfg.home_joints_deg, dtype=np.float64)
        home[6] = j6_open
        print("[流程] 回 home...")
        self._debug_show("回 home", joints=home)
        self.arm.move_joints(home.tolist(), wait=True)
        self._debug_show("任务完成", joints=home, extra_lines=[f"已放置 {obj.class_name}"])

        result = {
            "target_class": obj.class_name,
            "confidence": obj.confidence,
            "pick_position_base": obj.position_base.tolist(),
            "place_position_base": place.tolist(),
            "j6_open": j6_open,
            "j6_close": j6_close,
        }
        print(f"[完成] 抓取放置成功: {result}")
        return result
