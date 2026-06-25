"""仿真标定：端到端 MuJoCo 标定 + GT 对比 + 增强可视化报告。

Usage:
    from sim.sim_calibration import run_sim_calibration, SimCalibrationResult

    env = create_sim_env(headless=True)
    result = run_sim_calibration(env, num_poses=12)
    print(f"平移误差: {result.translation_error_mm:.1f} mm")
    print(f"旋转误差: {result.rotation_error_deg:.2f}°")
    # 报告: result.report_path (JSON), result.report_txt, result.report_png
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

SIM_ROOT = Path(__file__).resolve().parent
PROJ_ROOT = SIM_ROOT.parent

# 确保 calibate 可导入
sys.path.insert(0, str(PROJ_ROOT / "calibate"))
sys.path.insert(0, str(PROJ_ROOT))

from common.board import BoardConfig  # noqa: E402
from common.hand_eye_solve import (  # noqa: E402
    METHOD_MAP,
    calibrate_fused,
    calibrate_single,
    check_motion_diversity,
    evaluate_residual,
)
from common.io_utils import (  # noqa: E402
    generate_run_id,
    save_calibration_result,
)
from common.transforms import rt_to_homogeneous  # noqa: E402

from sim.mujoco_env import MuJoCoSimEnv, create_sim_env  # noqa: E402


@dataclass
class AlgorithmGTError:
    """单个算法的 GT 误差。"""
    name: str
    translation_error_mm: float
    rotation_error_deg: float
    residual_mean_mm: float
    residual_max_mm: float
    weight: float = 0.0


@dataclass
class SimCalibrationResult:
    """仿真标定完整结果。"""
    # 估计值
    R_est: np.ndarray
    t_est: np.ndarray
    T_est: np.ndarray

    # GT
    R_gt: np.ndarray
    t_gt: np.ndarray
    T_gt: np.ndarray

    # 误差
    translation_error_mm: float
    rotation_error_deg: float

    # 各算法
    per_algorithm: list[AlgorithmGTError] = field(default_factory=list)
    fused_method: str = "fused"

    # 元数据
    num_samples: int = 0
    motion_diversity: dict[str, Any] = field(default_factory=dict)
    collect_meta: dict[str, Any] = field(default_factory=dict)
    run_id: str = ""

    # 报告路径
    report_json: Path | None = None
    report_txt: Path | None = None
    report_png: Path | None = None

    @property
    def is_accurate(self) -> bool:
        """平移 < 5mm 且旋转 < 0.5° 视为准确。"""
        return self.translation_error_mm < 5.0 and self.rotation_error_deg < 0.5

    @property
    def quality_label(self) -> str:
        if self.translation_error_mm < 3.0 and self.rotation_error_deg < 0.3:
            return "Excellent"
        if self.translation_error_mm < 10.0 and self.rotation_error_deg < 1.0:
            return "Good"
        if self.translation_error_mm < 30.0 and self.rotation_error_deg < 3.0:
            return "Fair"
        return "Poor"


def _compute_rotation_error_deg(R_est: np.ndarray, R_gt: np.ndarray) -> float:
    """旋转矩阵 → 旋转误差角度 (度)。R_err = R_est @ R_gt^T。"""
    R_err = R_est @ R_gt.T
    trace = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(trace)))


def _compute_translation_error_mm(t_est: np.ndarray, t_gt: np.ndarray) -> float:
    return float(np.linalg.norm(t_est.reshape(3) - t_gt.reshape(3)) * 1000)


def _collect_sim_observations(
    env: MuJoCoSimEnv,
    data_dir: Path,
    board: BoardConfig,
    num_poses: int = 12,
    max_attempts: int = 30,
) -> tuple[list, list, list, list, list, dict]:
    """仿真采集：使用 MuJoCo FK + 解析 T_t2c，返回 OpenCV 直接可用的 R/t 列表。

    返回 (R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, used_images, collect_meta)

    注意：为保证几何一致性，T_t2c 由解析公式直接计算
    （T_t2c = T_base2cam @ T_g2b @ T_t2g），不经过渲染+检测环节。
    图像仍会渲染保存（用于可视化/报告），但不用于标定求解。
    """
    import cv2

    from common.auto_collect import collect_hand_eye_samples
    from sim.backends import SimCalibCamera, SimD1Arm

    arm = SimD1Arm(env)
    camera = SimCalibCamera(env)
    K, dist, _ = camera.intrinsics

    # 保存内参
    from common.realsense import save_camera_intrinsics
    save_camera_intrinsics(data_dir / "camera_intrinsics.json", K, dist,
                          {"source": "mujoco_sim", "width": env.width, "height": env.height})

    # 使用 auto_collect 采集样本（同时保存图像用于报告）
    result = collect_hand_eye_samples(
        data_dir=data_dir,
        calibration_type="eye_to_hand",
        mock_arm=False,
        mock_camera=False,
        sim=True,
        sim_env=env,
        board=board,
        num_poses=num_poses,
        max_attempts=max_attempts,
        show_debug=False,
        skip_board_check=False,
    )

    samples = result["samples"]
    collect_meta = result["meta"]

    # 解析计算 T_t2c = T_base2cam @ T_g2b @ T_t2g
    # T_cam_base_gt = T_cam2base, inv = T_base2cam
    T_b2c = np.linalg.inv(env.T_cam_base_gt)

    R_g2b_list, t_g2b_list = [], []
    R_t2c_list, t_t2c_list = [], []
    used = []

    from common.transforms import pose_to_rt

    for i, sample in enumerate(samples):
        img_name = sample.get("image", f"{i:04d}.png")
        rp = sample["robot_pose"]
        R_g2b, t_g2b = pose_to_rt(
            rp["position"],
            quaternion=rp.get("quaternion"),
            euler_xyz=rp.get("euler_xyz"),
        )
        T_g2b = rt_to_homogeneous(R_g2b, t_g2b)

        # 解析 T_t2c（T_target_gripper = I）
        T_t2c = T_b2c @ T_g2b
        R_t2c = T_t2c[:3, :3].copy()
        t_t2c = T_t2c[:3, 3].reshape(3, 1).copy()

        # 验证 Z > 0（标定板在相机前方）
        if t_t2c[2, 0] <= 0.01:
            continue

        R_g2b_list.append(R_g2b)
        t_g2b_list.append(t_g2b)
        R_t2c_list.append(R_t2c)
        t_t2c_list.append(t_t2c)
        used.append(img_name)

    return R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, used, collect_meta


def run_sim_calibration(
    env: MuJoCoSimEnv | None = None,
    *,
    num_poses: int = 12,
    max_attempts: int = 30,
    data_dir: Path | None = None,
    output_dir: Path | None = None,
    board: BoardConfig | None = None,
    methods: list[str] | None = None,
    headless: bool = True,
    save_report: bool = True,
) -> SimCalibrationResult:
    """运行完整的仿真标定流程：采集 → 求解 → GT 对比 → 报告。

    参数
    ----
    env : 可选的已创建 MuJoCoSimEnv；若为 None 则自动创建。
    num_poses : 采集样本数。
    data_dir : 数据存储目录。
    output_dir : 结果输出目录。
    methods : 算法列表，默认全部 5 种。
    headless : 是否 headless 渲染。
    save_report : 是否保存 JSON/TXT/PNG 报告。

    返回 SimCalibrationResult。
    """
    board = board or BoardConfig()
    own_env = env is None
    if own_env:
        env = create_sim_env(headless=headless)

    # 目录
    cal_dir = PROJ_ROOT / "calibate" / "eye_to_hand"
    work_data = (data_dir or cal_dir / "data" / "sim_test").resolve()
    work_data.mkdir(parents=True, exist_ok=True)
    (work_data / "images").mkdir(parents=True, exist_ok=True)

    run_id = generate_run_id()
    T_gt = env.T_cam_base_gt.copy()
    R_gt = T_gt[:3, :3].copy()
    t_gt = T_gt[:3, 3].reshape(3, 1).copy()

    try:
        # ---------- Phase 1: 采集 ----------
        print(f"[SimCalib] 采集 {num_poses} 组仿真样本...")
        t0 = time.perf_counter()
        R_g2b, t_g2b, R_t2c, t_t2c, used, collect_meta = _collect_sim_observations(
            env, work_data, board, num_poses=num_poses, max_attempts=max_attempts
        )
        t_collect = time.perf_counter() - t0
        print(f"[SimCalib] 采集完成: {len(used)}/{num_poses} 组, 耗时 {t_collect:.1f}s")

        if len(R_g2b) < 3:
            raise RuntimeError(f"有效样本不足: {len(R_g2b)} < 3")

        # ---------- Phase 2: 运动多样性检测 ----------
        diversity = check_motion_diversity(R_g2b, t_g2b)
        print(
            f"[SimCalib] 运动多样性: "
            f"旋转跨度 {diversity['rotation_span_deg']:.1f}°, "
            f"平移跨度 {diversity['translation_span_m']*1000:.1f}mm"
        )

        # ---------- Phase 3: 求解（多算法 + 融合）----------
        names = methods or list(METHOD_MAP.keys())
        per_alg: list[AlgorithmGTError] = []

        t1 = time.perf_counter()
        fused = calibrate_fused(
            R_g2b, t_g2b, R_t2c, t_t2c,
            mode="eye_to_hand",
            methods=names,
        )
        t_solve = time.perf_counter() - t1
        R_est, t_est = fused.R, fused.t
        print(f"[SimCalib] 求解完成: 融合残差 mean={fused.residual_mean*1000:.2f}mm, 耗时 {t_solve:.2f}s")

        # 各算法 GT 误差
        for m in fused.methods:
            t_err = _compute_translation_error_mm(m.t, t_gt)
            r_err = _compute_rotation_error_deg(m.R, R_gt)
            per_alg.append(AlgorithmGTError(
                name=m.name,
                translation_error_mm=t_err,
                rotation_error_deg=r_err,
                residual_mean_mm=m.residual_mean * 1000,
                residual_max_mm=m.residual_max * 1000,
                weight=m.weight,
            ))

        # 融合后 GT 误差
        t_err_fused = _compute_translation_error_mm(t_est, t_gt)
        r_err_fused = _compute_rotation_error_deg(R_est, R_gt)

        result = SimCalibrationResult(
            R_est=R_est,
            t_est=t_est.reshape(3, 1),
            T_est=rt_to_homogeneous(R_est, t_est.reshape(3, 1)),
            R_gt=R_gt,
            t_gt=t_gt,
            T_gt=T_gt,
            translation_error_mm=t_err_fused,
            rotation_error_deg=r_err_fused,
            per_algorithm=per_alg,
            fused_method="fused",
            num_samples=len(used),
            motion_diversity=diversity,
            collect_meta=collect_meta,
            run_id=run_id,
        )

        # ---------- Phase 4: 保存报告 ----------
        if save_report:
            out_dir = output_dir or (cal_dir / "output").resolve()
            out_dir.mkdir(parents=True, exist_ok=True)

            # JSON
            json_path = out_dir / "T_cam_base.json"
            from common.realsense import load_or_detect_intrinsics
            intrinsics_file = work_data / "camera_intrinsics.json"
            K, dist, _ = load_or_detect_intrinsics(
                camera_file=intrinsics_file,
                auto_realsense=False,
                use_file_only=True,
            )

            from common.io_utils import compute_intrinsics_hash
            save_calibration_result(
                json_path, R_est, t_est,
                meta={
                    "type": "eye_to_hand",
                    "description": "T_cam_base (simulation)",
                    "method": "fused",
                    "num_samples": len(used),
                    "images": used,
                    "sim_gt": {
                        "translation_error_mm": t_err_fused,
                        "rotation_error_deg": r_err_fused,
                        "t_gt": t_gt.reshape(3).tolist(),
                        "R_gt": R_gt.tolist(),
                    },
                    "fusion": {
                        "residual_mean_m": fused.residual_mean,
                        "residual_max_m": fused.residual_max,
                        "methods": [
                            {
                                "name": m.name,
                                "residual_mean_m": m.residual_mean,
                                "residual_max_m": m.residual_max,
                                "weight": m.weight,
                            }
                            for m in fused.methods
                        ],
                    },
                    "motion_diversity": diversity,
                    "per_algorithm_gt_error": [
                        {
                            "name": a.name,
                            "translation_error_mm": a.translation_error_mm,
                            "rotation_error_deg": a.rotation_error_deg,
                        }
                        for a in per_alg
                    ],
                },
                parent_frame="base_link",
                child_frame="camera_link",
                run_id=run_id,
                intrinsics_sha256=compute_intrinsics_hash(intrinsics_file),
                board_config={
                    "cols": board.cols,
                    "rows": board.rows,
                    "square_size_m": board.square_size,
                },
            )
            result.report_json = json_path

            # TXT + PNG 报告
            try:
                from common.calib_report import print_calibration_report
                report = print_calibration_report(
                    result_path=json_path,
                    data_dir=work_data,
                    board=board,
                    collect_meta=collect_meta,
                    save=True,
                )
                # 注入 sim GT 信息
                _append_sim_gt_to_txt(json_path, result)
                result.report_txt = json_path.parent / "calibration_report.txt"
                result.report_png = json_path.parent / "calibration_report.png"
            except Exception as exc:
                print(f"[SimCalib] 报告生成警告: {exc}")

        # ---------- Phase 5: 结果摘要 ----------
        _print_sim_summary(result)

        return result

    finally:
        if own_env and env is not None:
            env.close()


def _append_sim_gt_to_txt(json_path: Path, result: SimCalibrationResult) -> None:
    """在文本报告中追加仿真 GT 对比段。"""
    txt_path = json_path.parent / "calibration_report.txt"
    if not txt_path.exists():
        return

    lines = txt_path.read_text("utf-8").rstrip().split("\n")
    lines.append("")
    lines.append("  [仿真 GT 对比]")
    lines.append(f"    平移误差: {result.translation_error_mm:.2f} mm")
    lines.append(f"    旋转误差: {result.rotation_error_deg:.3f}°")
    lines.append(f"    准确度:   {result.quality_label}")
    lines.append("")
    lines.append("  [各算法 GT 误差]")
    lines.append(f"    {'算法':<12} {'平移mm':<10} {'旋转°':<10} {'残差mean mm':<13} {'权重':<8}")
    for a in result.per_algorithm:
        lines.append(
            f"    {a.name:<12} {a.translation_error_mm:<10.2f} "
            f"{a.rotation_error_deg:<10.3f} {a.residual_mean_mm:<13.2f} {a.weight:<8.3f}"
        )
    lines.append("")
    lines.append(f"  运动多样性: 旋转 {result.motion_diversity.get('rotation_span_deg', 0):.1f}°  "
                 f"平移 {result.motion_diversity.get('translation_span_m', 0)*1000:.1f}mm")
    lines.append(f"  run_id: {result.run_id}")
    lines.append("=" * 62)
    txt_path.write_text("\n".join(lines) + "\n", "utf-8")


def _print_sim_summary(result: SimCalibrationResult) -> None:
    """打印仿真标定摘要。"""
    sep = "=" * 62
    print(f"\n{sep}")
    print("  仿真标定结果 (Sim Calibration)")
    print(f"{sep}")
    print(f"  样本数:     {result.num_samples}")
    print(f"  平移误差:   {result.translation_error_mm:.2f} mm")
    print(f"  旋转误差:   {result.rotation_error_deg:.3f}°")
    print(f"  准确度:     {result.quality_label}")
    print(f"  运动多样性: 旋转 {result.motion_diversity.get('rotation_span_deg', 0):.1f}°  "
          f"平移 {result.motion_diversity.get('translation_span_m', 0)*1000:.1f}mm")
    print()
    print("  [各算法 GT 误差]")
    print(f"  {'算法':<12} {'平移mm':<10} {'旋转°':<10} {'残差mm':<10} {'权重':<8}")
    for a in sorted(result.per_algorithm, key=lambda x: x.translation_error_mm):
        print(f"  {a.name:<12} {a.translation_error_mm:<10.2f} "
              f"{a.rotation_error_deg:<10.3f} {a.residual_mean_mm:<10.2f} {a.weight:<8.3f}")
    print(f"{sep}")
    if result.report_json:
        print(f"  JSON:  {result.report_json}")
    if result.report_txt and result.report_txt.exists():
        print(f"  TXT:   {result.report_txt}")
    if result.report_png and result.report_png.exists():
        print(f"  PNG:   {result.report_png}")
    print(f"{sep}\n")


# ===================================================================
# 便捷入口
# ===================================================================


def quick_sim_test(num_poses: int = 8) -> SimCalibrationResult:
    """快速仿真标定测试（headless）。"""
    env = create_sim_env(headless=True)
    return run_sim_calibration(env, num_poses=num_poses, headless=True)
