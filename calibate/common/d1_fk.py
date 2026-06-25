"""Unitree D1 正运动学：URDF 或 JSON 链。"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from .transforms import rt_to_homogeneous
from .d1_urdf import DEFAULT_TIP_LINK, DEFAULT_URDF, ensure_d1_urdf


def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle_R(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )


def _parse_floats(text: str) -> np.ndarray:
    return np.asarray([float(x) for x in text.split()], dtype=np.float64)


class UrdfForwardKinematics:
    """解析 D1 URDF（revolute 关节链）。"""

    def __init__(self, urdf_path: str | Path, tip_link: str | None = None) -> None:
        self.urdf_path = Path(urdf_path)
        self.tip_link = tip_link or self._detect_tip_link()
        self.joint_names, self.origins, self.axes = self._parse_urdf()

    def _detect_tip_link(self) -> str:
        root = ET.parse(self.urdf_path).getroot()
        revolute_children = {
            joint.find("child").get("link")
            for joint in root.findall("joint")
            if joint.get("type") == "revolute"
        }
        for candidate in (DEFAULT_TIP_LINK, "Link6", "Empty_Link6", "link6"):
            if candidate in revolute_children:
                return candidate
        if revolute_children:
            return sorted(revolute_children)[-1]
        raise ValueError(f"URDF 中未找到 revolute 末端连杆: {self.urdf_path}")

    def _parse_urdf(self) -> tuple[list[str], list[tuple[np.ndarray, np.ndarray]], list[np.ndarray]]:
        root = ET.parse(self.urdf_path).getroot()
        joints_by_child: dict[str, ET.Element] = {}
        for joint in root.findall("joint"):
            if joint.get("type") != "revolute":
                continue
            child = joint.find("child").get("link")
            joints_by_child[child] = joint

        link = self.tip_link
        chain: list[ET.Element] = []
        visited = set()
        while link in joints_by_child and link not in visited:
            visited.add(link)
            joint = joints_by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        chain.reverse()

        names, origins, axes = [], [], []
        for joint in chain:
            names.append(joint.get("name"))
            origin = joint.find("origin")
            xyz = _parse_floats(origin.get("xyz", "0 0 0")) if origin is not None else np.zeros(3)
            rpy = _parse_floats(origin.get("rpy", "0 0 0")) if origin is not None else np.zeros(3)
            axis_el = joint.find("axis")
            axis = _parse_floats(axis_el.get("xyz", "0 0 1")) if axis_el is not None else np.array([0, 0, 1.0])
            origins.append((xyz, rpy))
            axes.append(axis)
        return names, origins, axes

    def fk(self, joint_angles_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if len(joint_angles_rad) < len(self.joint_names):
            raise ValueError(
                f"需要至少 {len(self.joint_names)} 个关节角，收到 {len(joint_angles_rad)}"
            )
        T = np.eye(4)
        for i, (xyz, rpy) in enumerate(self.origins):
            T_origin = np.eye(4)
            T_origin[:3, :3] = _rpy_to_R(rpy)
            T_origin[:3, 3] = xyz
            T_joint = np.eye(4)
            T_joint[:3, :3] = _axis_angle_R(self.axes[i], float(joint_angles_rad[i]))
            T = T @ T_origin @ T_joint
        return T[:3, :3].copy(), T[:3, 3].reshape(3, 1)


class JsonChainForwardKinematics:
    """JSON 定义的 fallback 运动学链。"""

    def __init__(self, chain_path: str | Path) -> None:
        with Path(chain_path).open("r", encoding="utf-8") as f:
            cfg = json.load(f)
        self.joints = cfg["joints"]
        tcp = cfg.get("tcp_offset", {})
        self.tcp_xyz = np.asarray(tcp.get("xyz", [0, 0, 0]), dtype=np.float64)
        self.tcp_rpy = np.deg2rad(np.asarray(tcp.get("rpy_deg", [0, 0, 0]), dtype=np.float64))

    def fk(self, joint_angles_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        T = np.eye(4)
        for i, joint in enumerate(self.joints):
            xyz = np.asarray(joint.get("xyz", [0, 0, 0]), dtype=np.float64)
            rpy = np.deg2rad(np.asarray(joint.get("rpy_deg", [0, 0, 0]), dtype=np.float64))
            axis = np.asarray(joint.get("axis", [0, 0, 1]), dtype=np.float64)
            T_i = np.eye(4)
            T_i[:3, :3] = _rpy_to_R(rpy)
            T_i[:3, 3] = xyz
            T_j = np.eye(4)
            T_j[:3, :3] = _axis_angle_R(axis, float(joint_angles_rad[i]))
            T = T @ T_i @ T_j
        T_tcp = np.eye(4)
        T_tcp[:3, :3] = _rpy_to_R(self.tcp_rpy)
        T_tcp[:3, 3] = self.tcp_xyz
        T = T @ T_tcp
        return T[:3, :3].copy(), T[:3, 3].reshape(3, 1)


def load_d1_fk(
    urdf_path: str | Path | None = None,
    chain_path: str | Path | None = None,
    auto_download_urdf: bool = True,
) -> UrdfForwardKinematics | JsonChainForwardKinematics:
    candidates = []
    if urdf_path:
        candidates.append(Path(urdf_path))
    default_urdf = DEFAULT_URDF
    if auto_download_urdf and not default_urdf.exists():
        try:
            ensure_d1_urdf(default_urdf)
        except RuntimeError as exc:
            print(f"[URDF] 自动下载失败: {exc}")
    candidates.append(default_urdf)
    for p in candidates:
        if p.exists():
            fk = UrdfForwardKinematics(p)
            print(f"[FK] 使用 URDF: {p} (tip={fk.tip_link}, joints={len(fk.joint_names)})")
            return fk

    chain = chain_path or Path(__file__).resolve().parents[1] / "config" / "d1_fk_chain.json"
    if not Path(chain).exists():
        raise FileNotFoundError(
            "未找到 D1 URDF。请运行: python tools/get_d1_urdf.py"
        )
    print(f"[FK] 使用 JSON 近似链: {chain}")
    return JsonChainForwardKinematics(chain)


def joint_angles_to_robot_pose(
    joint_angles_deg: list[float] | np.ndarray,
    fk: UrdfForwardKinematics | JsonChainForwardKinematics,
    num_arm_joints: int = 6,
) -> dict[str, Any]:
    """6 个臂关节角(度) -> base->gripper 位姿 dict。"""
    angles = np.asarray(joint_angles_deg, dtype=np.float64).reshape(-1)
    q = np.deg2rad(angles[:num_arm_joints])
    R, t = fk.fk(q)
    # 旋转矩阵 -> 欧拉角 (度) 供 calibrate.py 使用
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return {
        "position": t.reshape(3).tolist(),
        "euler_xyz": np.rad2deg([roll, pitch, yaw]).tolist(),
        "joint_angles_deg": angles.tolist(),
    }
