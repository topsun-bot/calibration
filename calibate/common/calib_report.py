"""手眼标定结果汇总与验证报告。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .board import BoardConfig, detect_board_pose, detect_board_pose_from_image, make_object_points
from .calib_config import DEFAULT_THRESHOLDS, QualityThresholds
from .io_utils import load_calibration_result, load_pose_list
from .realsense import load_or_detect_intrinsics
from .transforms import rt_to_homogeneous


@dataclass
class CalibVerification:
    num_poses: int
    num_verified: int
    board_mean_base: np.ndarray
    residual_mean_m: float
    residual_max_m: float
    residual_std_m: float
    per_sample_mm: list[float] = field(default_factory=list)
    outlier_indices: list[int] = field(default_factory=list)
    rotation_residuals_deg: list[float] = field(default_factory=list)
    per_sample_reproj_px: list[float] = field(default_factory=list)
    # 新增：用于可视化的扩展字段
    board_positions: np.ndarray | None = None  # 各帧标定板原点在 base 系 (N, 3)
    board_centers_norm: list[tuple[float, float]] = field(default_factory=list)  # 归一化图像坐标 (cx, cy)


def _detect_outliers(values: list[float], method: str = "iqr") -> list[int]:
    """检测离群样本索引。

    method='iqr': Tukey 箱线图法 (Q3 + 1.5*IQR)
    method='sigma': 均值 + 2.5σ
    """
    if len(values) < 3:
        return []
    arr = np.array(values)
    if method == "sigma":
        mean = arr.mean()
        std = arr.std()
        if std < 1e-9:
            return []
        threshold = mean + 2.5 * std
        return [i for i, v in enumerate(values) if v > threshold]
    else:
        q1 = np.percentile(arr, 25)
        q3 = np.percentile(arr, 75)
        iqr = q3 - q1
        if iqr < 1e-9:
            return []
        upper = q3 + 1.5 * iqr
        return [i for i, v in enumerate(values) if v > upper]


@dataclass
class CalibReport:
    result_path: Path
    calib_type: str
    T_cam_base: np.ndarray
    t_cam_base: np.ndarray
    R_cam_base: np.ndarray
    euler_xyz_deg: list[float]
    meta: dict[str, Any]
    verification: CalibVerification | None = None
    quality: str = ""
    quality_hint: str = ""
    collect_meta: dict[str, Any] | None = None


def _R_to_euler_xyz_deg(R: np.ndarray) -> tuple[list[float], bool]:
    """旋转矩阵 -> 固定轴 XYZ 欧拉角 (度)。返回 (角度列表, 是否接近万向锁)。"""
    sy = float(np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    gimbal_lock = sy < 1e-4
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.rad2deg([roll, pitch, yaw]).tolist(), gimbal_lock


def _assess_quality(mean_mm: float, max_mm: float, num_verified: int = 0,
                    std_mm: float = 0.0, outlier_count: int = 0,
                    thresholds: QualityThresholds | None = None) -> tuple[str, str]:
    thresholds = thresholds or DEFAULT_THRESHOLDS
    hints: list[str] = []
    quality, _, _ = thresholds.classify(mean_mm, max_mm)
    if quality == "优秀":
        hints.append("残差小，可直接用于抓取定位")
    elif quality == "良好":
        hints.append("可用于抓取，建议现场微调 tcp_offset")
    elif quality == "可用":
        hints.append("定位误差偏大")
    else:
        hints.append("残差过大")

    # 诊断附加信息
    if outlier_count > 0:
        hints.append(f"检测到 {outlier_count} 个离群样本（>3σ），建议移除后重新标定")
    if std_mm > 10.0 and mean_mm > 5.0:
        hints.append("样本间一致性差（std 大），请确认标定板夹持稳固")
    if max_mm > 3 * mean_mm and mean_mm > 3.0:
        hints.append("max/mean 比值大，可能存在个别异常帧（运动模糊/棋盘部分出视野）")
    if num_verified < 5:
        hints.append("有效样本偏少，建议增加到 8-15 组以提高鲁棒性")
    if mean_mm <= 30.0 and max_mm > 60.0:
        hints.append("平均值可接受但个别帧偏差大，检查是否有帧采集时机械臂未完全停止")
    if not hints:
        hints.append("标定质量良好，无需额外操作")

    return quality, "；".join(hints)


def verify_eye_to_hand(
    data_dir: Path,
    result_path: Path,
    board: BoardConfig | None = None,
    auto_camera: bool = False,
    use_file_only: bool = False,
) -> CalibVerification:
    """验证眼在手外标定：各帧标定板原点在基座系应重合。"""
    data_dir = data_dir.resolve()
    board = board or BoardConfig()
    camera_file = data_dir / "camera_intrinsics.json"
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=auto_camera or not camera_file.exists(),
        images_dir=data_dir / "images",
        save_to=camera_file,
        prefer_live=not use_file_only,
        use_file_only=use_file_only,
    )

    R_c2b, t_c2b, _ = load_calibration_result(result_path)
    T_c2b = rt_to_homogeneous(R_c2b, t_c2b)

    samples = load_pose_list(data_dir / "poses.json")
    positions: list[np.ndarray] = []
    per_sample: list[float] = []
    board_centers_norm: list[tuple[float, float]] = []
    img_h, img_w = 480, 640  # 默认尺寸，首帧检测后更新

    reproj_errors_px: list[float] = []
    for i, sample in enumerate(samples):
        img = data_dir / "images" / sample.get("image", f"{i:04d}.png")
        det = detect_board_pose(img, K, dist, board)
        if det is None:
            continue
        R_t2c, t_t2c = det
        T_t2c = rt_to_homogeneous(R_t2c, t_t2c)
        T_b2t = T_c2b @ np.linalg.inv(T_t2c)
        positions.append(T_b2t[:3, 3])

        # 计算标定板中心在图像中的归一化坐标（供覆盖热力图）
        try:
            from .board import detect_board_pose_from_image
            img_data = cv2.imread(str(img))
            if img_data is not None:
                img_h, img_w = img_data.shape[:2]
                full_det = detect_board_pose_from_image(img_data, K, dist, board)
                if full_det is not None and len(full_det) > 2:
                    corners = full_det[2]
                    # 角点中心归一化坐标
                    cx = float(corners[:, 0, 0].mean()) / img_w
                    cy = float(corners[:, 0, 1].mean()) / img_h
                    board_centers_norm.append((cx, cy))

                    # 计算重投影误差
                    objp = np.zeros((board.rows * board.cols, 3), np.float32)
                    grid = np.mgrid[0:board.cols, 0:board.rows].T.reshape(-1, 2)
                    objp[:, :2] = grid * board.square_size
                    rvec, _ = cv2.Rodrigues(R_t2c)
                    projected, _ = cv2.projectPoints(objp, rvec, t_t2c, K, dist)
                    err = float(np.linalg.norm(corners - projected) / np.sqrt(len(corners)))
                    reproj_errors_px.append(err)
                else:
                    board_centers_norm.append((0.5, 0.5))
            else:
                board_centers_norm.append((0.5, 0.5))
        except Exception:
            board_centers_norm.append((0.5, 0.5))

    if not positions:
        raise RuntimeError("验证失败：无有效棋盘检测样本")

    pts = np.array(positions)
    mean = pts.mean(axis=0)
    spread = np.linalg.norm(pts - mean, axis=1)
    per_sample = (spread * 1000).tolist()
    outlier_indices = _detect_outliers(per_sample, method="iqr")

    return CalibVerification(
        num_poses=len(samples),
        num_verified=len(positions),
        board_mean_base=mean,
        residual_mean_m=float(spread.mean()),
        residual_max_m=float(spread.max()),
        residual_std_m=float(spread.std()),
        per_sample_mm=per_sample,
        outlier_indices=outlier_indices,
        per_sample_reproj_px=reproj_errors_px,
        board_positions=pts,
        board_centers_norm=board_centers_norm,
    )


def build_calibration_report(
    result_path: Path,
    data_dir: Path | None = None,
    board: BoardConfig | None = None,
    collect_meta: dict[str, Any] | None = None,
    auto_camera: bool = False,
    use_file_only: bool = False,
) -> CalibReport:
    result_path = result_path.resolve()
    R, t, meta = load_calibration_result(result_path)
    T = rt_to_homogeneous(R, t)
    t_vec = t.reshape(3)

    verification = None
    quality, hint = "未验证", "未运行 verify，可执行 verify.py 查看残差"
    if data_dir is not None and (data_dir / "poses.json").exists():
        verification = verify_eye_to_hand(
            data_dir, result_path, board, auto_camera, use_file_only
        )
        quality, hint = _assess_quality(
            verification.residual_mean_m * 1000,
            verification.residual_max_m * 1000,
            num_verified=verification.num_verified,
            std_mm=verification.residual_std_m * 1000,
            outlier_count=len(verification.outlier_indices),
        )

    euler_deg, gimbal_lock = _R_to_euler_xyz_deg(R)
    return CalibReport(
        result_path=result_path,
        calib_type=str(meta.get("type", "eye_to_hand")),
        T_cam_base=T,
        t_cam_base=t_vec,
        R_cam_base=R,
        euler_xyz_deg=euler_deg,
        meta=meta,
        verification=verification,
        quality=quality,
        quality_hint=hint,
        collect_meta=collect_meta,
    )


def format_calibration_report(report: CalibReport) -> str:
    lines: list[str] = []
    sep = "=" * 62
    lines.append(sep)
    lines.append("  眼在手外 标定结果报告 (T_cam_base)")
    lines.append(sep)
    lines.append(f"  结果文件: {report.result_path}")
    lines.append(f"  标定类型: {report.calib_type}")
    lines.append(f"  算法:     {report.meta.get('method', 'unknown')}")
    lines.append(f"  样本数:   {report.meta.get('num_samples', '?')}")

    if report.collect_meta:
        cm = report.collect_meta
        lines.append(f"  采集模式: {cm.get('pose_mode', '-')}")
        lines.append(
            f"  采集统计: {cm.get('num_collected', '?')}/{cm.get('num_requested', '?')} "
            f"(跳过视野外 {cm.get('skipped_out_of_view', 0)} 次)"
        )
        if cm.get("gripper_j6_locked") is not None:
            lines.append(f"  锁定 j6:  {cm['gripper_j6_locked']:.1f}°")

    fusion = report.meta.get("fusion")
    if fusion:
        lines.append("")
        lines.append("  [融合标定]")
        lines.append(f"    融合残差 mean: {fusion.get('residual_mean_m', 0)*1000:.2f} mm")
        lines.append(f"    融合残差 max:  {fusion.get('residual_max_m', 0)*1000:.2f} mm")
        for m in fusion.get("methods", []):
            lines.append(
                f"    - {m['name']:10s} mean={m['residual_mean_m']*1000:.2f} mm "
                f"weight={m.get('weight', 0):.3f}"
            )

    lines.append("")
    lines.append("  [外参 t_cam_base] (相机原点在基座系, 米)")
    lines.append(f"    X {report.t_cam_base[0]:+.6f}")
    lines.append(f"    Y {report.t_cam_base[1]:+.6f}")
    lines.append(f"    Z {report.t_cam_base[2]:+.6f}")

    rx, ry, rz = report.euler_xyz_deg
    euler_deg, gimbal_lock = _R_to_euler_xyz_deg(report.R_cam_base)
    lines.append("")
    lines.append("  [外参旋转] 欧拉角 XYZ (度, 固定轴)")
    lines.append(f"    roll={rx:+.2f}  pitch={ry:+.2f}  yaw={rz:+.2f}")
    if gimbal_lock:
        lines.append("    ⚠ 接近万向锁奇异点，roll/yaw 不唯一，请参考旋转矩阵")

    lines.append("")
    lines.append("  [齐次矩阵 T_cam_base]")
    for row in report.T_cam_base:
        lines.append("    " + "  ".join(f"{v:+.6f}" for v in row))

    if report.verification:
        v = report.verification
        lines.append("")
        lines.append("  [验证] 标定板原点在基座系一致性")
        lines.append(f"    验证样本: {v.num_verified}/{v.num_poses}")
        lines.append(
            f"    板原点均值 (base): "
            f"[{v.board_mean_base[0]:+.4f}, {v.board_mean_base[1]:+.4f}, {v.board_mean_base[2]:+.4f}]"
        )
        lines.append(f"    位置残差 mean: {v.residual_mean_m*1000:.2f} mm")
        lines.append(f"    位置残差 max:  {v.residual_max_m*1000:.2f} mm")
        lines.append(f"    位置残差 std:  {v.residual_std_m*1000:.2f} mm")
        if v.per_sample_mm:
            preview = ", ".join(f"{x:.1f}" for x in v.per_sample_mm[:8])
            suffix = " ..." if len(v.per_sample_mm) > 8 else ""
            lines.append(f"    各样本 (mm): {preview}{suffix}")
        if v.outlier_indices:
            lines.append(
                f"    ⚠ 离群样本 (>3σ): "
                f"{[idx + 1 for idx in v.outlier_indices]}"
            )
        if v.per_sample_reproj_px:
            reproj_mean = float(np.mean(v.per_sample_reproj_px))
            reproj_max = float(np.max(v.per_sample_reproj_px))
            lines.append(
                f"    棋盘重投影误差: mean={reproj_mean:.2f} px, max={reproj_max:.2f} px"
            )

    lines.append("")
    lines.append(f"  [质量评估] {report.quality}")
    lines.append(f"    {report.quality_hint}")
    lines.append(sep)
    lines.append("  用法: p_base = R @ p_cam + t")
    lines.append(sep)
    return "\n".join(lines)


def save_calibration_report(
    report: CalibReport,
    path: Path | None = None,
    data_dir: Path | None = None,
) -> Path:
    path = path or (report.result_path.parent / "calibration_report.txt")
    path.parent.mkdir(parents=True, exist_ok=True)
    text = format_calibration_report(report)
    path.write_text(text + "\n", encoding="utf-8")

    # 生成可视化 PNG
    try:
        from .calib_visualizer import generate_report_figure
        vis_data_dir = data_dir or report.result_path.parent.parent / "data"
        png_path = path.with_suffix(".png")
        generate_report_figure(report, data_dir=vis_data_dir, save_path=png_path)
    except Exception as exc:
        print(f"[calib_report] 可视化报告生成失败: {exc}")

    return path


def print_calibration_report(
    result_path: Path,
    data_dir: Path | None = None,
    board: BoardConfig | None = None,
    collect_meta: dict[str, Any] | None = None,
    save: bool = True,
    auto_camera: bool = False,
    use_file_only: bool = False,
) -> CalibReport:
    report = build_calibration_report(
        result_path=result_path,
        data_dir=data_dir,
        board=board,
        collect_meta=collect_meta,
        auto_camera=auto_camera,
        use_file_only=use_file_only,
    )
    print("\n" + format_calibration_report(report))
    if save:
        out = save_calibration_report(report)
        print(f"\n报告已保存 -> {out}")
    return report


def show_calibration_result_window(
    report: CalibReport,
    title: str = "标定结果",
    wait_ms: int = 0,
) -> None:
    """在 OpenCV 窗口中展示标定摘要（按任意键关闭）。"""
    try:
        import cv2
    except ImportError:
        return

    lines = format_calibration_report(report).split("\n")
    line_h = 22
    pad = 16
    w = 720
    h = max(480, len(lines) * line_h + pad * 2)
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (28, 28, 28)

    y = pad + 18
    for line in lines:
        color = (200, 200, 200)
        if line.strip().startswith("[质量评估]"):
            q = report.quality
            color = (
                (80, 220, 80)
                if q == "优秀"
                else (80, 200, 255)
                if q in ("良好", "可用")
                else (80, 80, 255)
            )
        elif line.strip().startswith("="):
            color = (120, 120, 120)
        cv2.putText(
            canvas,
            line,
            (pad, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )
        y += line_h

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, w, min(h, 900))
    cv2.imshow(title, canvas)
    if wait_ms > 0:
        cv2.waitKey(wait_ms)
    else:
        print("[标定结果] 按任意键关闭结果窗口...")
        cv2.waitKey(0)
    cv2.destroyWindow(title)


# ---------------------------------------------------------------------------
# 逐样本重投影误差 & 留一交叉验证
# ---------------------------------------------------------------------------


def compute_reprojection_errors(
    data_dir: Path,
    R_cam2base: np.ndarray,
    t_cam2base: np.ndarray,
    board: BoardConfig | None = None,
    use_file_only: bool = False,
) -> list[float]:
    """逐样本重投影误差（像素 RMS）。

    将标定板角点通过完整标定链 (base->camera->image) 投影，
    与实际检测到的角点比较，返回每个有效样本的 RMS 误差列表。
    """
    data_dir = Path(data_dir).resolve()
    board = board or BoardConfig()

    # 加载内参
    camera_file = data_dir / "camera_intrinsics.json"
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=not camera_file.exists(),
        images_dir=data_dir / "images",
        save_to=camera_file,
        prefer_live=not use_file_only,
        use_file_only=use_file_only,
    )

    samples = load_pose_list(data_dir / "poses.json")
    objp = make_object_points(board)
    errors: list[float] = []

    for i, sample in enumerate(samples):
        img_path = data_dir / "images" / sample.get("image", f"{i:04d}.png")
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        det = detect_board_pose_from_image(img, K, dist, board)
        if det is None:
            continue

        R_t2c, t_t2c, corners = det
        rvec, _ = cv2.Rodrigues(R_t2c)
        projected, _ = cv2.projectPoints(objp, rvec, t_t2c, K, dist)

        # 角点数可能因 CharUco 而少于 objp 行数，取实际检测数量
        n = min(len(corners), len(projected))
        diff = corners[:n].reshape(-1, 2) - projected[:n].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))
        errors.append(rms)

    return errors


def leave_one_out_cross_validation(
    data_dir: Path,
    board: BoardConfig | None = None,
    method: str = "TSAI",
    use_file_only: bool = False,
) -> dict[str, Any]:
    """留一交叉验证：逐一剔除样本后重新求解手眼，评估一致性。

    对 N 个有效样本，每次留出 1 个，用剩余 N-1 个求解 hand-eye，
    再计算被留出样本的标定板位置残差（预测位置 vs 实际位置），单位 mm。

    返回字典包含:
        - per_sample_residual_mm: 每个有效样本的残差 (mm)
        - mean_residual_mm: 平均残差
        - std_residual_mm: 残差标准差
        - num_valid: 有效样本数
    """
    _METHOD_MAP = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    cv_method = _METHOD_MAP.get(method.upper(), cv2.CALIB_HAND_EYE_TSAI)

    data_dir = Path(data_dir).resolve()
    board = board or BoardConfig()

    # 加载内参
    camera_file = data_dir / "camera_intrinsics.json"
    K, dist, _ = load_or_detect_intrinsics(
        camera_file=camera_file,
        auto_realsense=not camera_file.exists(),
        images_dir=data_dir / "images",
        save_to=camera_file,
        prefer_live=not use_file_only,
        use_file_only=use_file_only,
    )

    samples = load_pose_list(data_dir / "poses.json")

    # 尝试加载 FK（用于从 joints 推算 gripper 位姿）
    fk = None
    try:
        from .d1_fk import load_d1_fk, joint_angles_to_robot_pose
        fk = load_d1_fk()
    except Exception:
        pass

    # ------------------------------------------------------------------
    # 收集所有有效样本的 (R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
    # ------------------------------------------------------------------
    valid_data: list[
        tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ] = []  # (R_g2b, t_g2b, R_t2c, t_t2c, board_origin_in_cam)

    for i, sample in enumerate(samples):
        # --- 检测标定板位姿 ---
        img_path = data_dir / "images" / sample.get("image", f"{i:04d}.png")
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        det = detect_board_pose_from_image(img, K, dist, board)
        if det is None:
            continue
        R_t2c, t_t2c, _ = det

        # --- 提取 gripper->base 位姿 ---
        R_g2b: np.ndarray | None = None
        t_g2b: np.ndarray | None = None

        # 方式 1: 直接存储的旋转矩阵 / 平移向量
        if "R_gripper2base" in sample and "t_gripper2base" in sample:
            R_g2b = np.asarray(sample["R_gripper2base"], dtype=np.float64).reshape(3, 3)
            t_g2b = np.asarray(sample["t_gripper2base"], dtype=np.float64).reshape(3, 1)
        # 方式 2: pose 字段（position + quaternion / euler）
        elif "pose" in sample and isinstance(sample["pose"], dict):
            pose = sample["pose"]
            pos = pose.get("position")
            quat = pose.get("quaternion")  # [x, y, z, w] 或 [w, x, y, z]
            if pos is not None and quat is not None:
                pos = np.asarray(pos, dtype=np.float64).reshape(3, 1)
                q = np.asarray(quat, dtype=np.float64).reshape(4)
                # 假设 [x, y, z, w] 格式
                x, y, z, w = q
                R_g2b = np.array([
                    [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                    [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                    [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
                ], dtype=np.float64)
                t_g2b = pos
        # 方式 3: joints -> FK
        elif "joints" in sample and fk is not None:
            try:
                joints = sample["joints"]
                q_rad = np.deg2rad(np.asarray(joints[:6], dtype=np.float64))
                R_fk, t_fk = fk.fk(q_rad)
                R_g2b = R_fk.astype(np.float64)
                t_g2b = t_fk.reshape(3, 1).astype(np.float64)
            except Exception:
                pass

        if R_g2b is None or t_g2b is None:
            continue

        valid_data.append((R_g2b, t_g2b, R_t2c, t_t2c, t_t2c.copy()))

    num_valid = len(valid_data)
    if num_valid < 4:
        return {
            "per_sample_residual_mm": [],
            "mean_residual_mm": float("nan"),
            "std_residual_mm": float("nan"),
            "num_valid": num_valid,
        }

    # ------------------------------------------------------------------
    # 留一交叉验证
    # ------------------------------------------------------------------
    residuals_mm: list[float] = []

    for i in range(num_valid):
        # 构建 leave-one-out 数据集
        R_gripper_list = [valid_data[j][0] for j in range(num_valid) if j != i]
        t_gripper_list = [valid_data[j][1] for j in range(num_valid) if j != i]
        R_target_list = [valid_data[j][2] for j in range(num_valid) if j != i]
        t_target_list = [valid_data[j][3] for j in range(num_valid) if j != i]

        # 求解 hand-eye
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper_list, t_gripper_list,
            R_target_list, t_target_list,
            method=cv_method,
        )

        # 用求解结果计算被留出样本的标定板在 base 系位置
        R_g2b_i, t_g2b_i, R_t2c_i, t_t2c_i, _ = valid_data[i]
        T_g2b = rt_to_homogeneous(R_g2b_i, t_g2b_i)
        T_c2g = rt_to_homogeneous(R_cam2gripper, t_cam2gripper)
        T_t2c = rt_to_homogeneous(R_t2c_i, t_t2c_i)

        # board_in_base = T_g2b @ T_c2g @ T_t2c 的平移部分
        T_board_base_i = T_g2b @ T_c2g @ T_t2c
        pos_i = T_board_base_i[:3, 3]

        # 用全部其他样本计算"参考"标定板在 base 系的均值位置
        other_positions: list[np.ndarray] = []
        for j in range(num_valid):
            if j == i:
                continue
            R_g2b_j, t_g2b_j, R_t2c_j, t_t2c_j, _ = valid_data[j]
            T_g2b_j = rt_to_homogeneous(R_g2b_j, t_g2b_j)
            T_board_j = T_g2b_j @ T_c2g @ T_t2c_j
            # 注意: 这里用的是从 i 的 leave-out 得到的 T_c2g
            # 对于 eye-to-hand，标定板固定，所有帧的 board_in_base 应一致
            T_t2c_j_full = rt_to_homogeneous(R_t2c_j, t_t2c_j)
            T_board_base_j = T_g2b_j @ T_c2g @ T_t2c_j_full
            other_positions.append(T_board_base_j[:3, 3])

        mean_pos = np.mean(other_positions, axis=0)
        residual_m = float(np.linalg.norm(pos_i - mean_pos))
        residuals_mm.append(residual_m * 1000.0)

    return {
        "per_sample_residual_mm": residuals_mm,
        "mean_residual_mm": float(np.mean(residuals_mm)),
        "std_residual_mm": float(np.std(residuals_mm)),
        "num_valid": num_valid,
    }
