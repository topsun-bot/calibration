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
