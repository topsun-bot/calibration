"""D1 标定位姿规划：保证棋盘留在相机视野内。"""

from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Any

import numpy as np

# D1 六轴关节限位 (度)，第 7 轴为夹爪
JOINT_LIMITS_DEG = np.array(
    [
        [-135, 135],
        [-90, 90],
        [-90, 90],
        [-135, 135],
        [-90, 90],
        [-135, 135],
        [-40, 20],  # 夹爪 j6：约 -40° 全开 ~ 20° 全闭
    ],
    dtype=np.float64,
)


def clamp_joints(angles_deg: list[float] | np.ndarray) -> np.ndarray:
    q = np.asarray(angles_deg, dtype=np.float64).reshape(-1)
    if q.size < 7:
        q = np.pad(q, (0, 7 - q.size))
    out = q.copy()
    for i in range(min(7, len(JOINT_LIMITS_DEG))):
        lo, hi = JOINT_LIMITS_DEG[i]
        out[i] = np.clip(out[i], lo, hi)
    return out


def load_pose_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_seed_poses(path: Path) -> list[np.ndarray]:
    data = load_pose_config(path)
    return [clamp_joints(p) for p in data.get("poses", [])]


def interpolate_joints(
    start: np.ndarray,
    target: np.ndarray,
    steps: int,
) -> list[np.ndarray]:
    """从 start 到 target 的分段插值（含终点）。"""
    if steps < 1:
        return [clamp_joints(target)]
    start = clamp_joints(start)
    target = clamp_joints(target)
    out = []
    for s in range(1, steps + 1):
        alpha = s / steps
        q = start + alpha * (target - start)
        out.append(clamp_joints(q))
    return out


def random_pose_near(
    base: np.ndarray,
    max_delta_deg: float,
    rng: random.Random | None = None,
) -> np.ndarray:
    """在 base 附近随机采样，单关节最大变化 max_delta_deg。"""
    rng = rng or random.Random()
    base = clamp_joints(base)
    delta = np.array(
        [rng.uniform(-max_delta_deg, max_delta_deg) for _ in range(6)],
        dtype=np.float64,
    )
    q = base.copy()
    q[:6] += delta
    q[6] = base[6]  # 夹爪保持不变
    return clamp_joints(q)


def generate_grid_poses(
    center: np.ndarray,
    max_delta_deg: float,
    num_poses: int,
    rng: random.Random | None = None,
) -> list[np.ndarray]:
    """围绕 center 生成 num_poses 个候选位姿（优先小扰动）。"""
    rng = rng or random.Random()
    center = clamp_joints(center)
    candidates = [center.copy()]
    while len(candidates) < num_poses * 3:
        scale = rng.uniform(0.3, 1.0)
        candidates.append(random_pose_near(center, max_delta_deg * scale, rng))
    rng.shuffle(candidates)
    return candidates



def save_poses_config(
    path: Path,
    poses: list[np.ndarray],
    description: str = "自动规划的有效标定位姿",
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "description": description,
        "poses": [q.reshape(-1).tolist() for q in poses],
    }
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def align_seed_poses_base_yaw(
    seed_poses: list[np.ndarray],
    current_joints_deg: list[float] | np.ndarray,
) -> tuple[list[np.ndarray], float]:
    """
    将种子位姿的 j0（基座旋转）对齐到当前机械臂姿态。

    眼在手外常见场景：config 里 j0≈0，但现场机械臂相对标定板已旋转约 ±90°。
    用当前 j0 与首个种子 j0 的差值平移所有种子，避免锚点搜索把臂拧回错误朝向。
    """
    current = clamp_joints(current_joints_deg)
    if not seed_poses:
        return [current.copy()], 0.0

    ref = clamp_joints(seed_poses[0])
    delta_j0 = float(current[0] - ref[0])

    aligned: list[np.ndarray] = []
    for pose in seed_poses:
        q = clamp_joints(pose)
        q[0] = q[0] + delta_j0
        aligned.append(q)
    return aligned, delta_j0


def load_manual_prep_fold_pose(path: Path) -> np.ndarray:
    """手动摆位前的折叠安全位姿（关节收拢，避免全零伸展时突然卸力下坠）。"""
    data = load_pose_config(path)
    fold = data.get("manual_prep_fold_pose")
    if fold is None and data.get("poses"):
        fold = data["poses"][0]
    if fold is None:
        fold = [0, -55, 85, 0, 40, 0, 0]
    return clamp_joints(fold)


def resolve_manual_prep_fold(
    fold_pose: list[float] | np.ndarray,
    current_joints_deg: list[float] | np.ndarray,
) -> np.ndarray:
    """折叠位姿保留当前 j0，避免基座旋转突变。"""
    q = clamp_joints(fold_pose)
    q[0] = float(clamp_joints(current_joints_deg)[0])
    return q


def build_manual_prep_seeds(
    manual_joints_deg: list[float] | np.ndarray,
    seed_poses: list[np.ndarray],
) -> list[np.ndarray]:
    """
    以手动摆位为锚点，将 config 种子位姿转为相对偏移（不再跳回 config 绝对角）。
    """
    manual = clamp_joints(manual_joints_deg)
    if not seed_poses:
        return [manual.copy()]
    ref = clamp_joints(seed_poses[0])
    aligned: list[np.ndarray] = []
    for pose in seed_poses:
        delta = clamp_joints(pose) - ref
        aligned.append(clamp_joints(manual + delta))
    return aligned


def score_pose_diversity(
    candidate_joints: np.ndarray,
    existing_samples_R: list[np.ndarray],
    existing_samples_t: list[np.ndarray],
    existing_joints: list[np.ndarray] | None = None,
) -> float:
    """
    评估候选位姿相对于已采集样本的多样性得分（综合关节空间与笛卡尔空间）。

    得分越高表示候选位姿与已有样本差异越大，鼓励探索欠表征的方向。

    评分策略：
    - 关节空间分量：候选关节角与已有关节角的最小 L2 距离，以 180° 归一化
    - 笛卡尔旋转分量：已有 R 两两最近邻测地距离均值，以 π 归一化（覆盖密度）
    - 笛卡尔平移分量：已有 t 两两最近邻欧氏距离均值，以 0.8m（臂展）归一化
    - 笛卡尔分量加权 0.6*旋转 + 0.4*平移；与关节分量等权混合，钳位到 [0, 1]

    Parameters
    ----------
    candidate_joints : np.ndarray
        候选关节角度（度），7 元素数组。
    existing_samples_R : list[np.ndarray]
        已采集样本的旋转矩阵列表（3x3），来自标定板检测。
    existing_samples_t : list[np.ndarray]
        已采集样本的平移向量列表（3 元素），来自标定板检测。
    existing_joints : list[np.ndarray] | None
        已采集样本对应的关节角度列表（度），可选。

    Returns
    -------
    float
        多样性得分，范围 [0, 1]。1.0 表示最大多样性。
    """
    has_joints = existing_joints is not None and len(existing_joints) > 0
    has_cartesian = len(existing_samples_R) > 0 and len(existing_samples_t) > 0

    # 无已有样本时返回最大多样性
    if not has_joints and not has_cartesian:
        return 1.0

    components: list[float] = []
    candidate = np.asarray(candidate_joints, dtype=np.float64).reshape(-1)

    # --- 关节空间分量 ---
    if has_joints:
        assert existing_joints is not None
        min_joint_dist = float("inf")
        for ej in existing_joints:
            ej_arr = np.asarray(ej, dtype=np.float64).reshape(-1)
            dist = float(np.linalg.norm(candidate - ej_arr))
            if dist < min_joint_dist:
                min_joint_dist = dist
        # 归一化：以 180° 为合理最大单轴范围的 L2 尺度
        joint_score = min(min_joint_dist / 180.0, 1.0)
        components.append(joint_score)

    # --- 笛卡尔分量（基于已有样本覆盖密度） ---
    # 注意：候选仅有关节角，无 FK 无法直接计算候选 R/t。
    # 此处用已有样本的覆盖稀疏度作为代理指标——样本越稀疏，新位姿价值越高。
    if has_cartesian:
        max_reach = 0.8  # 臂展近似值 (m)

        # 旋转覆盖度：已有 R 两两之间的平均最近邻测地距离
        n_r = len(existing_samples_R)
        if n_r >= 2:
            min_angles: list[float] = []
            for i in range(n_r):
                Ri = np.asarray(existing_samples_R[i], dtype=np.float64).reshape(3, 3)
                nearest = float("inf")
                for j in range(n_r):
                    if i == j:
                        continue
                    Rj = np.asarray(
                        existing_samples_R[j], dtype=np.float64
                    ).reshape(3, 3)
                    trace_val = np.clip(
                        (np.trace(Ri.T @ Rj) - 1.0) / 2.0, -1.0, 1.0
                    )
                    angle = float(np.arccos(trace_val))
                    if angle < nearest:
                        nearest = angle
                min_angles.append(nearest)
            # 平均最近邻角度越大 → 样本越稀疏 → 新样本越有价值
            avg_min_angle = float(np.mean(min_angles))
            rot_score = min(avg_min_angle / np.pi, 1.0)
        else:
            rot_score = 1.0  # 仅 1 个已有样本，任何新位姿都有旋转多样性

        # 平移覆盖度：已有 t 两两之间的平均最近邻欧氏距离
        n_t = len(existing_samples_t)
        if n_t >= 2:
            min_dists: list[float] = []
            for i in range(n_t):
                ti = np.asarray(existing_samples_t[i], dtype=np.float64).reshape(-1)
                nearest_d = float("inf")
                for j in range(n_t):
                    if i == j:
                        continue
                    tj = np.asarray(
                        existing_samples_t[j], dtype=np.float64
                    ).reshape(-1)
                    d = float(np.linalg.norm(ti - tj))
                    if d < nearest_d:
                        nearest_d = d
                min_dists.append(nearest_d)
            avg_min_dist = float(np.mean(min_dists))
            trans_score = min(avg_min_dist / max_reach, 1.0)
        else:
            trans_score = 1.0  # 仅 1 个已有样本，任何新位姿都有平移多样性

        # 旋转与平移加权组合为笛卡尔分量
        cartesian_score = 0.6 * rot_score + 0.4 * trans_score
        components.append(cartesian_score)

    if not components:
        return 0.5

    score = float(np.mean(components))
    return float(np.clip(score, 0.0, 1.0))
