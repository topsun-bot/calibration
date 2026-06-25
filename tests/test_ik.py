"""IK 单元测试。"""

from __future__ import annotations

import numpy as np

from pick_place.d1_ik import D1InverseKinematics


def test_ik_position_reach():
    ik = D1InverseKinematics()
    seed = np.array([0, -30, 60, 0, 30, 0, -40], dtype=float)
    target = ik.fk_position(seed) + np.array([0.02, 0.015, -0.01])
    q, err = ik.solve_position(target, seed, tol_m=0.005)
    reached = ik.fk_position(q)
    assert err < 0.008
    assert np.linalg.norm(reached - target) < 0.01
