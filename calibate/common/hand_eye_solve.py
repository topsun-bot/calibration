"""手眼标定：多算法求解与残差加权融合。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

from .transforms import invert_rt, rt_to_homogeneous


METHOD_MAP: dict[str, int] = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

CalibrationMode = Literal["eye_in_hand", "eye_to_hand"]


@dataclass
class MethodResult:
    name: str
    R: np.ndarray
    t: np.ndarray
    residual_mean: float
    residual_max: float
    weight: float


@dataclass
class FusedCalibrationResult:
    R: np.ndarray
    t: np.ndarray
    method: str
    methods: list[MethodResult]
    residual_mean: float
    residual_max: float


def _R_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数 [w, x, y, z]。
    注意：外部接口统一使用 ROS 标准 [x,y,z,w]，本函数仅用于内部 Markley 平均。
    """
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(max(1e-12, tr + 1.0))
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
        w = (R[2, 1] - R[1, 2]) / max(s, 1e-12)
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / max(s, 1e-12)
        z = (R[0, 2] + R[2, 0]) / max(s, 1e-12)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
        w = (R[0, 2] - R[2, 0]) / max(s, 1e-12)
        x = (R[0, 1] + R[1, 0]) / max(s, 1e-12)
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / max(s, 1e-12)
    else:
        s = 2.0 * np.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
        w = (R[1, 0] - R[0, 1]) / max(s, 1e-12)
        x = (R[0, 2] + R[2, 0]) / max(s, 1e-12)
        y = (R[1, 2] + R[2, 1]) / max(s, 1e-12)
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def _quat_wxyz_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _is_valid_rotation(R: np.ndarray, det_tol: float = 0.005, ortho_tol: float = 0.005) -> bool:
    if not np.all(np.isfinite(R)):
        return False
    det = float(np.linalg.det(R))
    ortho = float(np.linalg.norm(R @ R.T - np.eye(3)))
    return abs(det - 1.0) < det_tol and ortho < ortho_tol


def average_rotations(Rs: list[np.ndarray], weights: np.ndarray) -> np.ndarray:
    """Markley 四元数加权平均。"""
    weights = np.asarray(weights, dtype=np.float64)
    w_sum = weights.sum()
    if w_sum < 1e-12:
        return Rs[0].copy()
    weights = weights / w_sum
    Q = np.zeros((4, 4))
    ref = _R_to_quat_wxyz(Rs[0])
    for R, w in zip(Rs, weights):
        q = _R_to_quat_wxyz(R)
        if np.dot(ref, q) < 0:
            q = -q
        Q += w * np.outer(q, q)
    try:
        _, vecs = np.linalg.eigh(Q)
        q_avg = vecs[:, -1]
    except np.linalg.LinAlgError:
        idx = int(np.argmax(weights))
        return Rs[idx].copy()
    norm = np.linalg.norm(q_avg)
    if norm < 1e-10:
        idx = int(np.argmax(weights))
        return Rs[idx].copy()
    q_avg /= norm
    return _quat_wxyz_to_R(q_avg)


def fuse_transforms(
    Rs: list[np.ndarray],
    ts: list[np.ndarray],
    weights: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    weights = np.asarray(weights, dtype=np.float64)
    weights = weights / (weights.sum() + 1e-12)
    R = average_rotations(Rs, weights)
    t = sum(w * ti.reshape(3, 1) for w, ti in zip(weights, ts))
    return R, t


def _board_positions_in_base_eye_in_hand(
    R_c2g: np.ndarray,
    t_c2g: np.ndarray,
    R_g2b_list: list[np.ndarray],
    t_g2b_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> np.ndarray:
    """计算各帧标定板原点在机器人基座系下的位置 (eye-in-hand)。

    坐标链: target -> camera -> gripper -> base
    T_target2base = T_gripper2base @ T_camera2gripper @ T_target2camera
    """
    T_c2g = rt_to_homogeneous(R_c2g, t_c2g)
    positions = []
    for R_g2b, t_g2b, R_t2c, t_t2c in zip(R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list):
        T_g2b = rt_to_homogeneous(R_g2b, t_g2b)
        T_t2c = rt_to_homogeneous(R_t2c, t_t2c)
        T_target2base = T_g2b @ T_c2g @ T_t2c
        positions.append(T_target2base[:3, 3])
    return np.asarray(positions)


def _board_positions_in_gripper_eye_to_hand(
    R_c2b: np.ndarray,
    t_c2b: np.ndarray,
    R_g2b_list: list[np.ndarray],
    t_g2b_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> np.ndarray:
    """计算各帧标定板原点在夹爪系下的位置 (eye-to-hand)。

    坐标链: target -> camera -> base -> gripper
    T_target2gripper = inv(T_gripper2base) @ T_camera2base @ T_target2camera

    标定板固定在夹爪上，各帧的 target-in-gripper 应一致，其离散度即残差。
    """
    T_c2b = rt_to_homogeneous(R_c2b, t_c2b)
    positions = []
    for R_g2b, t_g2b, R_t2c, t_t2c in zip(R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list):
        T_g2b = rt_to_homogeneous(R_g2b, t_g2b)
        T_t2c = rt_to_homogeneous(R_t2c, t_t2c)
        T_target2gripper = np.linalg.inv(T_g2b) @ T_c2b @ T_t2c
        positions.append(T_target2gripper[:3, 3])
    return np.asarray(positions)


def evaluate_residual(
    R: np.ndarray,
    t: np.ndarray,
    mode: CalibrationMode,
    R_a_list: list[np.ndarray],
    t_a_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> tuple[float, float]:
    """
    一致性残差：标定板原点在基座系下的位置离散程度 (m)。
    返回 (mean, max)。
    """
    if mode == "eye_in_hand":
        pts = _board_positions_in_base_eye_in_hand(
            R, t, R_a_list, t_a_list, R_t2c_list, t_t2c_list
        )
    else:
        pts = _board_positions_in_gripper_eye_to_hand(
            R, t, R_a_list, t_a_list, R_t2c_list, t_t2c_list
        )
    if len(pts) < 2:
        return float("inf"), float("inf")
    spread = np.linalg.norm(pts - pts.mean(axis=0), axis=1)
    return float(spread.mean()), float(spread.max())


def check_motion_diversity(
    R_list: list[np.ndarray],
    t_list: list[np.ndarray],
    min_rotation_span_deg: float = 10.0,
    min_translation_span_m: float = 0.02,
) -> dict[str, bool | float]:
    """检测标定输入的退化情况。

    返回字典:
      - is_degenerate: 是否退化（旋转多样性不足或纯平移）
      - rotation_span_deg: 旋转轴之间的最大夹角
      - translation_span_m: 平移的最大距离
    """
    if len(R_list) < 3:
        return {"is_degenerate": True, "rotation_span_deg": 0.0, "translation_span_m": 0.0}

    # 计算两两相对旋转的轴方向
    axes = []
    for i in range(len(R_list)):
        for j in range(i + 1, len(R_list)):
            R_rel = R_list[j] @ R_list[i].T
            angle = np.arccos(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
            if angle > np.deg2rad(2.0):
                rvec, _ = cv2.Rodrigues(R_rel)
                axis = rvec.ravel()
                norm = np.linalg.norm(axis)
                if norm > 1e-9:
                    axes.append(axis / norm)

    if len(axes) < 2:
        rotation_span = 0.0
    else:
        # 轴之间的最大夹角
        axes_arr = np.array(axes)
        dots = np.abs(axes_arr @ axes_arr.T)
        np.fill_diagonal(dots, 1.0)
        max_angle = np.arccos(np.clip(np.min(dots), -1.0, 1.0))
        rotation_span = float(np.rad2deg(max_angle))

    # 平移范围
    ts = np.array([ti.ravel() for ti in t_list])
    translation_span = float(np.max(np.linalg.norm(ts - ts.mean(axis=0), axis=1)))

    is_degenerate = rotation_span < min_rotation_span_deg or translation_span < min_translation_span_m
    return {
        "is_degenerate": is_degenerate,
        "rotation_span_deg": rotation_span,
        "translation_span_m": translation_span,
    }


def calibrate_single(
    R_a_list: list[np.ndarray],
    t_a_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
    method: str,
    mode: CalibrationMode = "eye_in_hand",
) -> tuple[np.ndarray, np.ndarray]:
    """单算法手眼标定。

    参数:
        R_a_list, t_a_list: 机器人位姿 (gripper-to-base)。
        R_t2c_list, t_t2c_list: 棋盘→相机检测结果。
        method: 算法名。
        mode: "eye_in_hand" 或 "eye_to_hand"。
              eye_to_hand 模式下自动取逆（base-to-gripper）后传入 OpenCV。

    返回:
        eye_in_hand: R_cam2gripper, t_cam2gripper
        eye_to_hand: R_cam2base, t_cam2base
    """
    if method not in METHOD_MAP:
        raise ValueError(f"未知算法: {method}")
    if len(R_a_list) < 3:
        raise ValueError(f"至少需要 3 组数据，当前仅 {len(R_a_list)} 组")
    if len(R_a_list) != len(R_t2c_list):
        raise ValueError(
            f"机器人位姿数量 ({len(R_a_list)}) 与棋盘检测数量 ({len(R_t2c_list)}) 不匹配"
        )

    # 眼在手外：OpenCV calibrateHandEye 适用于 eye-in-hand，
    # 对于 eye-to-hand 需传入 base-to-gripper (= inv(gripper-to-base))，
    # 输出为 T_cam2gripper，在 swapped 问题中 = T_cam2base。
    if mode == "eye_to_hand":
        R_input = []
        t_input = []
        for Ri, ti in zip(R_a_list, t_a_list):
            R_inv, t_inv = invert_rt(Ri, ti.reshape(3, 1))
            R_input.append(R_inv)
            t_input.append(t_inv)
    else:
        R_input = R_a_list
        t_input = t_a_list

    R, t = cv2.calibrateHandEye(
        R_input,
        t_input,
        R_t2c_list,
        t_t2c_list,
        method=METHOD_MAP[method],
    )
    return R, t


def _refine_nonlinear(
    R_init: np.ndarray,
    t_init: np.ndarray,
    mode: CalibrationMode,
    R_a_list: list[np.ndarray],
    t_a_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Levenberg-Marquardt 非线性优化精修手眼标定结果。

    最小化标定板位置一致性残差（position spread）。
    """
    try:
        from scipy.optimize import least_squares
    except ImportError:
        return R_init, t_init

    rvec_init, _ = cv2.Rodrigues(R_init)
    x0 = np.concatenate([rvec_init.ravel(), t_init.reshape(3)])

    def _residual_fn(x):
        rvec = x[:3].reshape(3, 1)
        tvec = x[3:6].reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        T = rt_to_homogeneous(R, tvec)

        if mode == "eye_in_hand":
            pts = _board_positions_in_base_eye_in_hand(
                R, tvec, R_a_list, t_a_list, R_t2c_list, t_t2c_list
            )
        else:
            pts = _board_positions_in_gripper_eye_to_hand(
                R, tvec, R_a_list, t_a_list, R_t2c_list, t_t2c_list
            )
        if len(pts) < 2:
            return np.zeros(1)
        centroid = pts.mean(axis=0)
        return (pts - centroid).ravel()

    result = least_squares(_residual_fn, x0, method='lm', max_nfev=200)
    if result.success or result.cost < np.sum(_residual_fn(x0) ** 2):
        R_opt, _ = cv2.Rodrigues(result.x[:3].reshape(3, 1))
        t_opt = result.x[3:6].reshape(3, 1)
        if _is_valid_rotation(R_opt):
            return R_opt, t_opt
    return R_init, t_init


def _filter_outlier_methods(results: list[MethodResult]) -> list[MethodResult]:
    """移除残差远超中位值的异常算法（MAD-based）。"""
    if len(results) <= 2:
        return results
    residuals = np.array([r.residual_mean for r in results])
    median_r = np.median(residuals)
    mad = np.median(np.abs(residuals - median_r))
    threshold = median_r + 3.0 * max(mad, 1e-6)
    filtered = [r for r in results if r.residual_mean <= threshold]
    if len(filtered) < 1:
        return results
    if len(filtered) < len(results):
        removed = [r.name for r in results if r.residual_mean > threshold]
        print(f"[融合] 移除异常算法: {removed} (残差 > {threshold*1000:.2f}mm)")
    return filtered


def calibrate_fused(
    R_a_list: list[np.ndarray],
    t_a_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
    mode: CalibrationMode,
    methods: list[str] | None = None,
    min_weight: float = 1e-6,
    refine: bool = True,
) -> FusedCalibrationResult:
    """
    运行多种 hand-eye 算法，按一致性残差加权融合 + 非线性精修。

    权重: softmax(-residual / tau)，其中 tau = median(residuals)
    旋转: 四元数 Markley 加权平均
    平移: 加权平均
    精修: Levenberg-Marquardt 最小化位置一致性残差
    """
    # 退化检测
    diversity = check_motion_diversity(R_a_list, t_a_list)
    if diversity["is_degenerate"]:
        print(
            f"[警告] 标定输入可能退化: "
            f"旋转跨度 {diversity['rotation_span_deg']:.1f}°, "
            f"平移跨度 {diversity['translation_span_m']*1000:.1f} mm. "
            f"建议增加位姿多样性（多方向旋转 + 平移）。"
        )

    names = methods or list(METHOD_MAP.keys())
    results: list[MethodResult] = []

    for name in names:
        try:
            R, t = calibrate_single(R_a_list, t_a_list, R_t2c_list, t_t2c_list, name, mode=mode)
        except cv2.error as exc:
            print(f"[跳过] {name}: OpenCV 求解失败 ({exc})")
            continue
        if not _is_valid_rotation(R) or not np.all(np.isfinite(t)):
            print(f"[跳过] {name}: 旋转矩阵无效")
            continue
        mean_r, max_r = evaluate_residual(
            R, t, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
        )
        if not np.isfinite(mean_r):
            print(f"[跳过] {name}: 残差无效")
            continue
        results.append(MethodResult(name, R, t, mean_r, max_r, 0.0))

    if not results:
        raise RuntimeError("所有手眼算法均求解失败")

    if len(results) == 1:
        only = results[0]
        R_out, t_out = only.R, only.t
        if refine:
            R_out, t_out = _refine_nonlinear(
                R_out, t_out, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
            )
            mean_r, max_r = evaluate_residual(
                R_out, t_out, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
            )
            if mean_r < only.residual_mean:
                only.R, only.t = R_out, t_out
                only.residual_mean, only.residual_max = mean_r, max_r
        return FusedCalibrationResult(
            R=only.R,
            t=only.t,
            method="fused",
            methods=results,
            residual_mean=only.residual_mean,
            residual_max=only.residual_max,
        )

    # 移除异常算法
    results = _filter_outlier_methods(results)

    # Softmax 权重（比 1/r² 更温和，避免 winner-take-all）
    residuals = np.array([r.residual_mean for r in results])
    tau = max(np.median(residuals), 1e-9)
    weights = np.exp(-residuals / tau)
    weights = np.maximum(weights, min_weight)
    weights = weights / weights.sum()
    for r, w in zip(results, weights):
        r.weight = float(w)

    R_f, t_f = fuse_transforms(
        [r.R for r in results],
        [r.t for r in results],
        weights,
    )

    # 非线性精修
    if refine:
        R_f, t_f = _refine_nonlinear(
            R_f, t_f, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
        )

    mean_r, max_r = evaluate_residual(
        R_f, t_f, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
    )

    best = min(results, key=lambda x: x.residual_mean)
    if mean_r > best.residual_mean:
        # 融合劣于最优：用最优单算法精修
        R_best, t_best = best.R, best.t
        if refine:
            R_best, t_best = _refine_nonlinear(
                R_best, t_best, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
            )
        best_r, best_max = evaluate_residual(
            R_best, t_best, mode, R_a_list, t_a_list, R_t2c_list, t_t2c_list
        )
        print(
            f"[融合] 加权融合残差 ({mean_r*1000:.3f}mm) 劣于最优单算法 {best.name} "
            f"({best_r*1000:.3f}mm)，采用 {best.name} + 精修"
        )
        R_f, t_f = R_best, t_best
        mean_r, max_r = best_r, best_max

    return FusedCalibrationResult(
        R=R_f,
        t=t_f,
        method="fused",
        methods=results,
        residual_mean=mean_r,
        residual_max=max_r,
    )


def print_method_comparison(result: FusedCalibrationResult) -> None:
    print("\n--- 各算法结果对比 ---")
    print(f"{'算法':<12} {'残差mean(m)':<14} {'残差max(m)':<14} {'权重':<8}")
    for m in result.methods:
        print(f"{m.name:<12} {m.residual_mean:<14.6f} {m.residual_max:<14.6f} {m.weight:<8.3f}")
    best = min(result.methods, key=lambda x: x.residual_mean)
    print(f"\n单算法最优: {best.name} (mean={best.residual_mean:.6f} m)")
    print(f"融合结果:   fused (mean={result.residual_mean:.6f} m, max={result.residual_max:.6f} m)")
