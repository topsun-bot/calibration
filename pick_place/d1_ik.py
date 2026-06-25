"""D1 数值逆运动学（基于 FK + 阻尼最小二乘）。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "calibate"))

from common.d1_fk import JsonChainForwardKinematics, UrdfForwardKinematics, load_d1_fk  # noqa: E402
from common.pose_planner import JOINT_LIMITS_DEG, clamp_joints  # noqa: E402


class D1InverseKinematics:
    def __init__(
        self,
        fk: UrdfForwardKinematics | JsonChainForwardKinematics | None = None,
        urdf_path: Path | None = None,
    ) -> None:
        self.fk = fk or load_d1_fk(urdf_path=urdf_path)
        self.num_arm_joints = len(self.fk.joint_names) if hasattr(self.fk, "joint_names") else 6

    def fk_position(self, joints_deg: np.ndarray) -> np.ndarray:
        q = np.deg2rad(np.asarray(joints_deg, dtype=np.float64).reshape(-1)[: self.num_arm_joints])
        _, t = self.fk.fk(q)
        return t.reshape(3)

    def solve_position(
        self,
        target_base: np.ndarray,
        seed_joints_deg: np.ndarray,
        max_iter: int = 80,
        tol_m: float = 0.003,
        damping: float = 0.05,
        step_deg: float = 8.0,
    ) -> tuple[np.ndarray, float]:
        """
        仅位置 IK，返回 (7,) 关节角(度) 与最终位置误差(米)。
        第 7 轴夹爪保持 seed 值。
        """
        target = np.asarray(target_base, dtype=np.float64).reshape(3)
        q = clamp_joints(seed_joints_deg).copy()
        best_q = q.copy()
        best_err = float("inf")

        for _ in range(max_iter):
            pos = self.fk_position(q)
            err = target - pos
            err_norm = float(np.linalg.norm(err))
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()
            if err_norm <= tol_m:
                return best_q, err_norm

            J = self._numerical_jacobian(q)
            JJt = J @ J.T + (damping ** 2) * np.eye(3)
            dq_arm = J.T @ np.linalg.solve(JJt, err)

            delta = np.zeros(7)
            delta[:6] = np.rad2deg(dq_arm[:6])
            norm = np.linalg.norm(delta[:6])
            if norm > step_deg:
                delta[:6] *= step_deg / norm

            q[:6] = clamp_joints(q + delta)[:6]
            q[6] = seed_joints_deg.reshape(-1)[6] if seed_joints_deg.size >= 7 else q[6]

        return best_q, best_err

    def _numerical_jacobian(self, joints_deg: np.ndarray, eps_deg: float = 0.5) -> np.ndarray:
        base = self.fk_position(joints_deg)
        J = np.zeros((3, 6))
        for i in range(6):
            perturbed = joints_deg.copy()
            perturbed[i] += eps_deg
            perturbed = clamp_joints(perturbed)
            J[:, i] = (self.fk_position(perturbed) - base) / eps_deg
        return J

    @staticmethod
    def within_limits(joints_deg: np.ndarray) -> bool:
        q = clamp_joints(joints_deg)
        return np.allclose(q[:6], joints_deg[:6], atol=0.01)
