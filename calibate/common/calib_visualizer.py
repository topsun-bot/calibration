"""标定可视化报告：matplotlib 多面板 PNG 图表。

生成包含以下面板的综合图：
  1. 各样本残差柱状图（含离群阈值线）
  2. 各算法对比（融合标定时）
  3. 验证点 3D 散点图
  4. 质量仪表盘
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

# matplotlib 为可选依赖；无 GUI 环境用 Agg 后端
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def _setup_chinese_fonts():
    """尝试配置中文字体，失败则回退."""
    try:
        plt.rcParams["font.sans-serif"] = [
            "WenQuanYi Micro Hei",
            "WenQuanYi Zen Hei",
            "Noto Sans CJK SC",
            "SimHei",
            "DejaVu Sans",
        ]
        plt.rcParams["axes.unicode_minus"] = False
    except Exception:
        pass


def _quality_color(mean_mm: float, max_mm: float) -> str:
    """残差 → 颜色."""
    if mean_mm <= 5.0 and max_mm <= 12.0:
        return "#2ecc71"  # 绿
    if mean_mm <= 15.0 and max_mm <= 30.0:
        return "#f1c40f"  # 黄
    if mean_mm <= 30.0 and max_mm <= 60.0:
        return "#e67e22"  # 橙
    return "#e74c3c"  # 红


def _quality_label(mean_mm: float, max_mm: float) -> str:
    if mean_mm <= 5.0 and max_mm <= 12.0:
        return "Excellent"
    if mean_mm <= 15.0 and max_mm <= 30.0:
        return "Good"
    if mean_mm <= 30.0 and max_mm <= 60.0:
        return "Usable"
    return "Poor"


# ===================================================================
# 单面板绘图函数
# ===================================================================


def _plot_residual_bars(ax, per_sample_mm: list[float], mean_mm: float,
                        std_mm: float, max_mm: float):
    """各样本残差水平柱状图，含 2σ/3σ 阈值线."""
    n = len(per_sample_mm)
    if n == 0:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
        return

    y = list(range(n))
    colors = []
    threshold_2s = mean_mm + 2 * std_mm
    threshold_3s = mean_mm + 3 * std_mm
    for v in per_sample_mm:
        if v > threshold_3s:
            colors.append("#e74c3c")
        elif v > threshold_2s:
            colors.append("#e67e22")
        else:
            colors.append("#3498db")

    bars = ax.barh(y, per_sample_mm, color=colors, edgecolor="white", linewidth=0.3, height=0.7)
    ax.axvline(mean_mm, color="#2c3e50", linestyle="-", linewidth=1.5, label=f"mean={mean_mm:.1f}")
    if threshold_2s < max_mm:
        ax.axvline(threshold_2s, color="#f39c12", linestyle="--", linewidth=1.0,
                   label=f"mean+2σ={threshold_2s:.1f}")
    if threshold_3s < max_mm:
        ax.axvline(threshold_3s, color="#e74c3c", linestyle=":", linewidth=1.0,
                   label=f"mean+3σ={threshold_3s:.1f}")

    ax.set_yticks(y)
    ax.set_yticklabels([f"#{i+1}" for i in y], fontsize=7)
    ax.set_xlabel("Residual (mm)")
    ax.set_title("Per-Sample Board Origin Residual", fontweight="bold")
    ax.legend(fontsize=7, loc="lower right")
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=0.3)


def _plot_method_comparison(ax, methods: list[dict]):
    """各算法残差对比（融合标定）."""
    if not methods:
        ax.text(0.5, 0.5, "Single method", ha="center", va="center",
                transform=ax.transAxes, fontsize=12)
        return

    names = [m.get("name", "?") for m in methods]
    means = [m.get("residual_mean_m", 0) * 1000 for m in methods]  # → mm
    weights = [m.get("weight", 0) for m in methods]

    x = np.arange(len(names))
    colors = ["#3498db" if w > 0 else "#bdc3c7" for w in weights]
    bars = ax.bar(x, means, color=colors, edgecolor="white", linewidth=0.5)

    # 标注权重
    for i, (bar, w) in enumerate(zip(bars, weights)):
        if w > 0.01:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                    f"w={w:.2f}", ha="center", fontsize=7, color="#2c3e50")

    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("Mean residual (mm)")
    ax.set_title("Algorithm Comparison", fontweight="bold")
    ax.grid(axis="y", alpha=0.3)


def _plot_3d_spread(ax, positions: np.ndarray, mean_pos: np.ndarray):
    """标定板原点在基座系下的 3D 散点."""
    if len(positions) < 2:
        ax.text2D(0.5, 0.5, "Insufficient data for 3D plot", ha="center",
                  va="center", transform=ax.transAxes)
        return

    # 中心化以便观察离散度
    centered = positions - mean_pos
    ax.scatter(centered[:, 0] * 1000, centered[:, 1] * 1000,
               centered[:, 2] * 1000, c="#e74c3c", s=30, alpha=0.7, edgecolors="white",
               linewidth=0.5)
    ax.scatter(0, 0, 0, c="#2ecc71", s=80, marker="*", edgecolors="black",
               linewidth=0.5, label="mean")

    ax.set_xlabel("dX (mm)")
    ax.set_ylabel("dY (mm)")
    ax.set_zlabel("dZ (mm)")
    ax.set_title("Board Origin Spread (centered)", fontweight="bold")
    ax.legend(fontsize=7)


def _plot_quality_gauge(ax, mean_mm: float, max_mm: float):
    """质量仪表盘."""
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_aspect("equal")
    ax.axis("off")

    color = _quality_color(mean_mm, max_mm)
    label = _quality_label(mean_mm, max_mm)

    # 背景圆环
    theta = np.linspace(0.5 * np.pi, 2.5 * np.pi, 200)
    r = 0.7
    ax.fill_between(r * np.cos(theta), r * np.sin(theta), alpha=0.08, color="#bdc3c7")

    # 填充弧：根据质量等级
    levels = [
        (0.00, 0.25, "#2ecc71"),
        (0.25, 0.50, "#f1c40f"),
        (0.50, 0.75, "#e67e22"),
        (0.75, 1.00, "#e74c3c"),
    ]
    for start_f, end_f, c in levels:
        t0 = 0.5 * np.pi + start_f * 2 * np.pi
        t1 = 0.5 * np.pi + end_f * 2 * np.pi
        tt = np.linspace(t0, t1, 60)
        ax.fill_between(r * np.cos(tt), r * np.sin(tt), alpha=0.2, color=c)

    # 指针：mean 映射到 0-60mm → 0-100%
    pct = np.clip(mean_mm / 60.0, 0.0, 1.0)
    angle = 0.5 * np.pi + pct * 2 * np.pi
    ax.arrow(0, 0, 0.55 * np.cos(angle), 0.55 * np.sin(angle),
             head_width=0.06, head_length=0.08, fc=color, ec=color, linewidth=2)

    ax.text(0, -0.2, f"{mean_mm:.1f} mm", ha="center", va="center",
            fontsize=16, fontweight="bold", color=color)
    ax.text(0, -0.45, label, ha="center", va="center", fontsize=11, color=color)


# ===================================================================
# 主入口
# ===================================================================


def generate_report_figure(
    report: Any,  # CalibReport (避免循环导入)
    data_dir: Path | None = None,
    save_path: Path | None = None,
) -> Path | None:
    """生成多面板标定可视化报告 (PNG)。

    返回保存路径，若 matplotlib 不可用则返回 None。
    """
    if not HAS_MPL:
        print("[calib_visualizer] matplotlib 不可用，跳过图表生成")
        return None

    _setup_chinese_fonts()

    # 确定布局
    has_fusion = bool(report.meta.get("fusion"))
    has_verification = report.verification is not None

    if has_fusion and has_verification:
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.30)
        ax_residual = fig.add_subplot(gs[0, 0])
        ax_method = fig.add_subplot(gs[0, 1])
        ax_3d = fig.add_subplot(gs[0, 2], projection="3d")
        ax_gauge = fig.add_subplot(gs[1, 0])
        ax_info = fig.add_subplot(gs[1, 1:])
    elif has_verification:
        fig = plt.figure(figsize=(14, 8))
        gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.30)
        ax_residual = fig.add_subplot(gs[0, 0])
        ax_3d = fig.add_subplot(gs[0, 1], projection="3d")
        ax_gauge = fig.add_subplot(gs[1, 0])
        ax_info = fig.add_subplot(gs[1, 1])
    else:
        fig = plt.figure(figsize=(10, 6))
        gs = fig.add_gridspec(1, 2, wspace=0.30)
        ax_info = fig.add_subplot(gs[0, 0])
        ax_gauge = fig.add_subplot(gs[0, 1])

    fig.suptitle("Hand-Eye Calibration Report", fontsize=16, fontweight="bold", y=0.98)

    # --- 面板 1: 残差柱状图 ---
    if has_verification:
        v = report.verification
        _plot_residual_bars(ax_residual, v.per_sample_mm,
                            v.residual_mean_m * 1000, v.residual_std_m * 1000,
                            v.residual_max_m * 1000)
        # 标记离群样本
        threshold = v.residual_mean_m * 1000 + 3 * v.residual_std_m * 1000
        outlier_indices = [i for i, r in enumerate(v.per_sample_mm) if r > threshold]
        if outlier_indices:
            ax_residual.text(0.98, 0.02,
                             f"Outliers: {[i+1 for i in outlier_indices]}",
                             transform=ax_residual.transAxes, fontsize=8,
                             color="#e74c3c", ha="right", fontweight="bold")

    # --- 面板 2: 算法对比 ---
    if has_fusion and has_verification:
        _plot_method_comparison(ax_method, report.meta["fusion"].get("methods", []))

    # --- 面板 3: 3D 散点 ---
    if has_verification:
        v = report.verification
        # 重建各样本位置
        positions = _reconstruct_verification_positions(report, data_dir)
        if positions is not None:
            _plot_3d_spread(ax_3d, positions, v.board_mean_base)
        else:
            ax_3d.text2D(0.5, 0.5, "Insufficient data", ha="center",
                         va="center", transform=ax_3d.transAxes)

    # --- 面板 4: 质量仪表 ---
    if has_verification:
        _plot_quality_gauge(ax_gauge,
                            report.verification.residual_mean_m * 1000,
                            report.verification.residual_max_m * 1000)
    else:
        ax_gauge.text(0.5, 0.5, "Not verified", ha="center", va="center",
                      transform=ax_gauge.transAxes, fontsize=12)
        ax_gauge.axis("off")

    # --- 面板 5: 信息卡 ---
    _draw_info_card(ax_info, report)
    ax_info.axis("off")

    # 保存
    out = save_path or (report.result_path.parent / "calibration_report.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"[calib_visualizer] 可视化报告已保存 -> {out}")
    return out


def _draw_info_card(ax, report: Any):
    """绘制信息摘要卡."""
    lines = []
    lines.append(("Result", str(report.result_path.name)))
    lines.append(("Type", report.calib_type))
    lines.append(("Algorithm", str(report.meta.get("method", "unknown"))))
    lines.append(("Samples", str(report.meta.get("num_samples", "?"))))
    lines.append(("Quality", report.quality))
    lines.append(("", ""))
    lines.append(("Translation (m)",
                  f"X={report.t_cam_base[0]:+.4f}  Y={report.t_cam_base[1]:+.4f}  "
                  f"Z={report.t_cam_base[2]:+.4f}"))

    rx, ry, rz = report.euler_xyz_deg
    lines.append(("Rotation XYZ (deg)",
                  f"R={rx:+.1f}  P={ry:+.1f}  Y={rz:+.1f}"))

    if report.verification:
        v = report.verification
        lines.append(("", ""))
        lines.append(("Residual mean", f"{v.residual_mean_m*1000:.2f} mm"))
        lines.append(("Residual std", f"{v.residual_std_m*1000:.2f} mm"))
        lines.append(("Residual max", f"{v.residual_max_m*1000:.2f} mm"))
        lines.append(("Verified frames", f"{v.num_verified}/{v.num_poses}"))

    if report.collect_meta:
        cm = report.collect_meta
        lines.append(("", ""))
        lines.append(("Pose mode", str(cm.get("pose_mode", "-"))))
        lines.append(("Collected", f"{cm.get('num_collected', '?')}/{cm.get('num_requested', '?')}"))

    y = 0.92
    for label, value in lines:
        if label:
            ax.text(0.05, y, f"{label}:", fontsize=9, fontweight="bold",
                    color="#2c3e50", va="top", fontfamily="monospace")
            ax.text(0.35, y, value, fontsize=9, color="#34495e", va="top",
                    fontfamily="monospace")
        y -= 0.055 if label else 0.02
        if y < 0.02:
            break


def plot_coordinate_frames(
    T_cam2base: np.ndarray,
    T_g2b_list: list[np.ndarray] | None = None,
    T_t2c_list: list[np.ndarray] | None = None,
    save_path: Path | None = None,
    title: str = "Hand-Eye Calibration Coordinate Frames",
) -> Path | None:
    """3D 坐标系可视化（MoveIt2 Planning Scene 风格）。

    绘制 base、camera、gripper（多帧）、target 四种坐标系的空间关系，
    帮助直观理解标定结果的几何含义。

    参数:
        T_cam2base: 4x4 标定结果 (cam → base)
        T_g2b_list: 各帧 gripper-to-base 变换列表（可选）
        T_t2c_list: 各帧 target-to-camera 变换列表（可选）
        save_path: 保存路径（默认不保存，仅返回 fig）
        title: 图标题
    """
    if not HAS_MPL:
        return None

    _setup_chinese_fonts()
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(title, fontsize=14, fontweight="bold")

    def _draw_frame(ax, T: np.ndarray, label: str, scale: float = 0.08, lw: float = 2.0):
        """绘制坐标系三轴 (X红 Y绿 Z蓝)。"""
        origin = T[:3, 3]
        R = T[:3, :3]
        colors = ["#e74c3c", "#2ecc71", "#3498db"]  # R G B
        axis_labels = ["X", "Y", "Z"]
        for i, (c, al) in enumerate(zip(colors, axis_labels)):
            end = origin + R[:, i] * scale
            ax.plot3D(*zip(origin, end), color=c, linewidth=lw)
        ax.text(*origin, f" {label}", fontsize=8, color="#2c3e50")

    # Base frame (origin)
    T_base = np.eye(4)
    _draw_frame(ax, T_base, "Base", scale=0.12, lw=3.0)

    # Camera frame
    _draw_frame(ax, T_cam2base, "Camera", scale=0.10, lw=2.5)

    # 画 camera → base 的连线
    ax.plot3D(*zip(T_base[:3, 3], T_cam2base[:3, 3]),
              color="#95a5a6", linewidth=1, linestyle="--", alpha=0.6)

    # Gripper frames (if provided)
    if T_g2b_list:
        for i, T_g2b in enumerate(T_g2b_list[:8]):  # 最多显示 8 个
            alpha = 0.4 + 0.6 * (i / max(len(T_g2b_list) - 1, 1))
            _draw_frame(ax, T_g2b, f"G{i}" if i < 3 else "", scale=0.05, lw=1.0)

    # Target frames in base (if provided)
    if T_t2c_list and T_cam2base is not None:
        for i, T_t2c in enumerate(T_t2c_list[:8]):
            T_target_base = T_cam2base @ T_t2c
            _draw_frame(ax, T_target_base, f"T{i}" if i < 3 else "", scale=0.04, lw=0.8)

    # 美化
    ax.set_xlabel("X (m)", fontsize=10)
    ax.set_ylabel("Y (m)", fontsize=10)
    ax.set_zlabel("Z (m)", fontsize=10)
    ax.set_box_aspect([1, 1, 1])

    # 添加图例说明
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color="#e74c3c", lw=2, label="X-axis"),
        Line2D([0], [0], color="#2ecc71", lw=2, label="Y-axis"),
        Line2D([0], [0], color="#3498db", lw=2, label="Z-axis"),
    ]
    ax.legend(handles=legend_elements, loc="upper left", fontsize=8)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        print(f"[calib_visualizer] 坐标系图已保存 -> {save_path}")
        return save_path
    plt.close(fig)
    return None


def plot_reprojection_errors(
    per_sample_errors_px: list[float],
    save_path: Path | None = None,
    title: str = "Per-Sample Reprojection Error",
) -> Path | None:
    """重投影误差分布图（easy_handeye2 evaluator 风格）。

    参数:
        per_sample_errors_px: 每样本 RMS 重投影误差 (像素)
        save_path: 保存路径
    """
    if not HAS_MPL or not per_sample_errors_px:
        return None

    _setup_chinese_fonts()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(title, fontsize=14, fontweight="bold")

    n = len(per_sample_errors_px)
    errors = np.array(per_sample_errors_px)
    mean_err = float(errors.mean())
    std_err = float(errors.std())

    # 左: 柱状图
    colors = ["#2ecc71" if e < mean_err + std_err else "#e67e22" if e < mean_err + 2 * std_err else "#e74c3c"
              for e in errors]
    ax1.bar(range(1, n + 1), errors, color=colors, alpha=0.8, edgecolor="#2c3e50", linewidth=0.5)
    ax1.axhline(mean_err, color="#3498db", linestyle="--", linewidth=1.5, label=f"Mean: {mean_err:.2f} px")
    ax1.axhline(mean_err + 2 * std_err, color="#e74c3c", linestyle=":", linewidth=1.0, label=f"Mean+2σ: {mean_err + 2 * std_err:.2f} px")
    ax1.set_xlabel("Sample Index", fontsize=10)
    ax1.set_ylabel("RMS Reproj. Error (px)", fontsize=10)
    ax1.set_title("Per-Sample Error", fontsize=11)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # 右: 直方图
    ax2.hist(errors, bins=min(n, 20), color="#3498db", alpha=0.7, edgecolor="white")
    ax2.axvline(mean_err, color="#e74c3c", linestyle="--", linewidth=2, label=f"Mean: {mean_err:.2f} px")
    ax2.set_xlabel("Reproj. Error (px)", fontsize=10)
    ax2.set_ylabel("Count", fontsize=10)
    ax2.set_title("Error Distribution", fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        print(f"[calib_visualizer] 重投影误差图已保存 -> {save_path}")
        return save_path
    plt.close(fig)
    return None


def plot_sim_gt_comparison(
    result,  # SimCalibrationResult
    save_path: Path | None = None,
) -> Path | None:
    """仿真标定 GT 对比可视化。

    显示各算法的 GT 平移/旋转误差对比柱状图。
    """
    if not HAS_MPL:
        return None

    _setup_chinese_fonts()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(f"Sim Calibration GT Comparison — Quality: {result.quality_label}",
                 fontsize=14, fontweight="bold")

    names = [a.name for a in result.per_algorithm]
    t_errors = [a.translation_error_mm for a in result.per_algorithm]
    r_errors = [a.rotation_error_deg for a in result.per_algorithm]
    weights = [a.weight for a in result.per_algorithm]

    # 颜色按权重
    max_w = max(weights) if weights else 1.0
    colors = [plt.cm.viridis(w / max_w) for w in weights]

    # 左: 平移误差
    bars1 = ax1.bar(names, t_errors, color=colors, alpha=0.85, edgecolor="#2c3e50")
    ax1.axhline(result.translation_error_mm, color="#e74c3c", linestyle="--",
                linewidth=2, label=f"Fused: {result.translation_error_mm:.2f} mm")
    ax1.set_ylabel("Translation Error (mm)", fontsize=10)
    ax1.set_title("Translation GT Error per Algorithm", fontsize=11)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3, axis="y")

    # 右: 旋转误差
    bars2 = ax2.bar(names, r_errors, color=colors, alpha=0.85, edgecolor="#2c3e50")
    ax2.axhline(result.rotation_error_deg, color="#e74c3c", linestyle="--",
                linewidth=2, label=f"Fused: {result.rotation_error_deg:.3f}°")
    ax2.set_ylabel("Rotation Error (deg)", fontsize=10)
    ax2.set_title("Rotation GT Error per Algorithm", fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        print(f"[calib_visualizer] GT对比图已保存 -> {save_path}")
        return save_path
    plt.close(fig)
    return None


def _reconstruct_verification_positions(
    report: Any, data_dir: Path | None
) -> np.ndarray | None:
    """从 data_dir 重建验证用的各帧标定板位置（用于 3D 散点）."""
    if data_dir is None:
        data_dir = report.result_path.parent.parent / "data"
    if not (data_dir / "poses.json").exists():
        return None

    try:
        from .board import BoardConfig, detect_board_pose
        from .io_utils import load_calibration_result, load_pose_list
        from .realsense import load_or_detect_intrinsics
        from .transforms import rt_to_homogeneous

        board = BoardConfig()
        camera_file = data_dir / "camera_intrinsics.json"
        K, dist, _ = load_or_detect_intrinsics(
            camera_file=camera_file,
            auto_realsense=False,
            images_dir=data_dir / "images",
            use_file_only=True,
        )
        R_c2b, t_c2b, _ = load_calibration_result(report.result_path)
        T_c2b = rt_to_homogeneous(R_c2b, t_c2b)

        samples = load_pose_list(data_dir / "poses.json")
        positions = []
        for i, sample in enumerate(samples):
            img = data_dir / "images" / sample.get("image", f"{i:04d}.png")
            det = detect_board_pose(img, K, dist, board)
            if det is None:
                continue
            R_t2c, t_t2c = det
            T_t2c = rt_to_homogeneous(R_t2c, t_t2c)
            T_b2t = T_c2b @ np.linalg.inv(T_t2c)
            positions.append(T_b2t[:3, 3])

        if positions:
            return np.array(positions)
    except Exception:
        pass
    return None
