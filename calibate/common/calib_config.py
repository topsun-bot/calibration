"""标定系统全局配置：质量阈值、可视化参数、报告选项。

所有阈值和颜色的唯一定义来源，避免模块间重复。
"""

from __future__ import annotations

from dataclasses import dataclass, field


# ===================================================================
# 质量评估阈值
# ===================================================================


@dataclass
class QualityThresholds:
    """标定残差质量评估阈值 (单位: mm)。

    四级评价体系:
      - 优秀: mean ≤ excellent_mean 且 max ≤ excellent_max
      - 良好: mean ≤ good_mean 且 max ≤ good_max
      - 可用: mean ≤ usable_mean 且 max ≤ usable_max
      - 较差: 超出 usable 阈值
    """

    excellent_mean_mm: float = 5.0
    excellent_max_mm: float = 12.0
    good_mean_mm: float = 15.0
    good_max_mm: float = 30.0
    usable_mean_mm: float = 30.0
    usable_max_mm: float = 60.0

    # 颜色定义（Hex）
    color_excellent: str = "#2ecc71"
    color_good: str = "#f1c40f"
    color_usable: str = "#e67e22"
    color_poor: str = "#e74c3c"

    def classify(self, mean_mm: float, max_mm: float) -> tuple[str, str, str]:
        """残差 → (质量标签, 英文标签, 颜色 hex)。

        Returns
        -------
        tuple of (label_zh, label_en, color_hex)
        """
        if mean_mm <= self.excellent_mean_mm and max_mm <= self.excellent_max_mm:
            return "优秀", "Excellent", self.color_excellent
        if mean_mm <= self.good_mean_mm and max_mm <= self.good_max_mm:
            return "良好", "Good", self.color_good
        if mean_mm <= self.usable_mean_mm and max_mm <= self.usable_max_mm:
            return "可用", "Usable", self.color_usable
        return "较差", "Poor", self.color_poor


# ===================================================================
# 可视化配置
# ===================================================================


@dataclass
class VisualizationConfig:
    """可视化报告面板配置。"""

    dpi: int = 150
    # 面板开关
    show_residual_bars: bool = True
    show_method_comparison: bool = True
    show_3d_spread: bool = True
    show_quality_gauge: bool = True
    show_coverage_heatmap: bool = True
    show_convergence: bool = True
    show_reproj_distribution: bool = True
    show_info_card: bool = True
    # 仪表盘
    gauge_max_mm: float = 60.0
    # 柱状图颜色
    color_normal_bar: str = "#3498db"
    color_warn_bar: str = "#e67e22"
    color_outlier_bar: str = "#e74c3c"
    # 覆盖热力图
    heatmap_grid_size: int = 5
    # 收敛曲线
    convergence_min_samples: int = 3


# ===================================================================
# 报告配置
# ===================================================================


@dataclass
class ReportConfig:
    """报告生成总配置。"""

    thresholds: QualityThresholds = field(default_factory=QualityThresholds)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    # 输出格式
    output_formats: list[str] = field(default_factory=lambda: ["txt", "png", "html"])
    # 离群检测统一方法
    outlier_method: str = "iqr"  # "iqr" | "sigma"
    outlier_iqr_factor: float = 1.5
    outlier_sigma_factor: float = 2.5
    # 文本报告
    separator_width: int = 62


# ===================================================================
# 验证配置
# ===================================================================


@dataclass
class ValidationConfig:
    """标定结果验证配置（语义检查）。"""

    # 数学检查
    orthogonality_tol: float = 0.01
    det_tol: float = 0.01
    quaternion_norm_tol: float = 0.001
    # 物理合理性检查
    max_translation_m: float = 3.0
    min_translation_m: float = 0.005
    expected_z_positive: bool = True  # 相机通常在 base 上方
    # 重投影误差阈值
    max_reprojection_rms_px: float = 2.0


# ===================================================================
# 全局默认实例
# ===================================================================

DEFAULT_THRESHOLDS = QualityThresholds()
DEFAULT_VIS_CONFIG = VisualizationConfig()
DEFAULT_REPORT_CONFIG = ReportConfig()
DEFAULT_VALIDATION_CONFIG = ValidationConfig()
