"""MuJoCo 眼在手外仿真环境。"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

SIM_ROOT = Path(__file__).resolve().parent
MJCF_PATH = SIM_ROOT / "mjcf" / "d1_sim.xml"

CALIB_ROOT = SIM_ROOT.parent / "calibate"
sys.path.insert(0, str(CALIB_ROOT))

from common.board import BoardConfig  # noqa: E402
from common.d1_fk import joint_angles_to_robot_pose, load_d1_fk  # noqa: E402
from common.synthetic_board import render_board_for_robot_pose  # noqa: E402
from common.transforms import rt_to_homogeneous  # noqa: E402

# T_cam2base: 相机在基座系 [0.45, 0.0, 0.80] 位置，俯视工作台。
# 语义：p_base = T_cam_base @ p_cam（相机坐标 → 基座坐标）
# 渲染：T_t2c = inv(T_cam_base) @ T_g2b = T_base2cam @ T_g2b （正 Z，几何一致）
DEFAULT_SIM_T_CAM_BASE = np.array(
    [
        [0.0, 0.8637789, -0.50387103, 0.45],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -0.50387103, -0.8637789, 0.80],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# 仿真相机内参（与 mock 一致）
SIM_CAMERA_MATRIX = np.array(
    [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64
)
SIM_DIST_COEFFS = np.zeros(5)
SIM_DEPTH_SCALE = 0.001

JOINT_NAMES = ["j0", "j1", "j2", "j3", "j4", "j5", "j6"]
# j0-j5 度，j6 滑移(m) 映射为等效“度”供接口统一
J6_OPEN_M = -0.028
J6_CLOSE_M = 0.012


def j6_deg_to_slide(j6_deg: float) -> float:
    """D1 j6 度 -> MuJoCo slide 位移 (m)。"""
    j6_open, j6_close = -40.0, 20.0
    ratio = (j6_close - j6_deg) / (j6_close - j6_open)
    ratio = float(np.clip(ratio, 0.0, 1.0))
    return J6_OPEN_M + ratio * (J6_CLOSE_M - J6_OPEN_M)


def j6_slide_to_deg(slide_m: float) -> float:
    j6_open, j6_close = -40.0, 20.0
    ratio = (slide_m - J6_OPEN_M) / (J6_CLOSE_M - J6_OPEN_M)
    ratio = float(np.clip(ratio, 0.0, 1.0))
    return j6_close - ratio * (j6_close - j6_open)


@dataclass
class SimObject:
    class_name: str
    position_base: np.ndarray
    size_m: tuple[float, float, float] = (0.04, 0.04, 0.14)


@dataclass
class MuJoCoSimEnv:
    """MuJoCo 仿真：D1 臂 + 固定相机 + 棋盘 + 物体。"""

    width: int = 640
    height: int = 480
    board: BoardConfig = field(default_factory=BoardConfig)
    headless: bool = True

    model: Any = field(init=False, repr=False)
    data: Any = field(init=False, repr=False)
    renderer: Any = field(init=False, repr=False)
    fk: Any = field(init=False, repr=False)
    T_cam_base_gt: np.ndarray = field(init=False)
    T_target_gripper: np.ndarray = field(init=False)
    _joint_qpos_idx: list[int] = field(init=False, default_factory=list)
    _objects: dict[str, SimObject] = field(init=False, default_factory=dict)
    _bottle_mocap_id: int = field(init=False, default=-1)
    _bottle_home: np.ndarray = field(init=False, default_factory=lambda: np.array([0.32, 0.08, 0.075]))
    _ee_body_id: int = field(init=False, default=0)
    _bottle_body_id: int = field(init=False, default=0)
    _grasp_attached: bool = field(init=False, default=False)
    _grasp_local_offset: np.ndarray = field(init=False, default_factory=lambda: np.zeros(3))
    demo_grasp_snap: bool = True

    def __post_init__(self) -> None:
        import mujoco

        if self.headless and "MUJOCO_GL" not in os.environ:
            os.environ.setdefault("MUJOCO_GL", "egl")

        self.model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, self.height, self.width)
        self.fk = load_d1_fk()
        self.T_target_gripper = np.eye(4)
        self._joint_qpos_idx = [
            self.model.joint(name).qposadr[0] for name in JOINT_NAMES
        ]
        self._ee_body_id = int(self.model.body("ee").id)
        self._bottle_body_id = int(self.model.body("bottle").id)
        self._bottle_mocap_id = int(self.model.body_mocapid[self._bottle_body_id])
        self._bottle_home = np.array([0.32, 0.08, 0.075], dtype=np.float64)
        self._grasp_attached = False
        self._grasp_local_offset = np.zeros(3)
        self.reset()
        self.T_cam_base_gt = DEFAULT_SIM_T_CAM_BASE.copy()
        self.T_cam_base_render = self._compute_T_cam_base_gt()
        self._objects = {
            "bottle": SimObject("bottle", np.array([0.32, 0.08, 0.075])),
            "cup": SimObject("cup", np.array([0.32, 0.08, 0.075])),
        }

    def _set_bottle_pose(self, pos: np.ndarray, quat_wxyz: np.ndarray | None = None) -> None:
        import mujoco

        if quat_wxyz is None:
            quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        self.data.mocap_pos[self._bottle_mocap_id] = np.asarray(pos, dtype=np.float64).reshape(3)
        self.data.mocap_quat[self._bottle_mocap_id] = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
        mujoco.mj_forward(self.model, self.data)

    def reset_bottle_home(self) -> None:
        """重置瓶子到初始抓取位。"""
        self._grasp_attached = False
        self._set_bottle_pose(self._bottle_home.copy())

    def attach_bottle(self, force: bool = False) -> bool:
        """将瓶子附着到末端（演示用运动学抓取）。"""
        import mujoco

        ee_pos = self.data.xpos[self._ee_body_id].copy()
        ee_mat = self.data.xmat[self._ee_body_id].reshape(3, 3).copy()
        bottle_pos = self.data.xpos[self._bottle_body_id].copy()
        dist = float(np.linalg.norm(bottle_pos - ee_pos))
        if dist > 0.18 and not force:
            return False
        if force or dist > 0.18:
            # 演示模式：将瓶子吸附到夹爪下方
            target = ee_pos + ee_mat @ np.array([0.0, 0.0, -0.06])
        else:
            target = bottle_pos.copy()
            target[2] = max(target[2], ee_pos[2] - 0.02)
        self._grasp_local_offset = ee_mat.T @ (target - ee_pos)
        self._grasp_attached = True
        self._sync_grasped_bottle()
        mujoco.mj_forward(self.model, self.data)
        return True

    def detach_bottle(self) -> None:
        self._grasp_attached = False

    def _sync_grasped_bottle(self) -> None:
        if not self._grasp_attached:
            return
        ee_pos = self.data.xpos[self._ee_body_id].copy()
        ee_mat = self.data.xmat[self._ee_body_id].reshape(3, 3).copy()
        pos = ee_pos + ee_mat @ self._grasp_local_offset
        self._set_bottle_pose(pos)

    def on_gripper_j6(self, j6_deg: float) -> None:
        """夹爪 j6 变化时更新抓取状态（由控制器调用）。"""
        j6_deg = float(j6_deg)
        if j6_deg > -20.0:
            if not self.attach_bottle():
                self.attach_bottle(force=True)
        elif j6_deg < -35.0:
            self.detach_bottle()

    def _update_grasp_physics(self) -> None:
        """已附着时同步瓶子；未附着时固定在初始位。"""
        if not self.demo_grasp_snap:
            return
        if self._grasp_attached:
            self._sync_grasped_bottle()
            return
        self._set_bottle_pose(self._bottle_home.copy())

    def reset(self) -> None:
        import mujoco

        mujoco.mj_resetData(self.model, self.data)
        self._grasp_attached = False
        self.set_joints_deg(np.array([0.0, -30.0, 60.0, 0.0, 30.0, 0.0, -40.0]))
        self.reset_bottle_home()
        for _ in range(50):
            mujoco.mj_step(self.model, self.data)

    def _compute_T_cam_base_gt(self) -> np.ndarray:
        """Ground truth: 相机系 → 基座系 (world)。"""
        import mujoco

        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        body_id = self.model.cam_bodyid[cam_id]
        pos = self.data.xpos[body_id].copy()
        mat = self.data.xmat[body_id].reshape(3, 3).copy()
        # MuJoCo 相机: +X右 +Y上 -Z前 → RealSense: +X右 +Y下 +Z前
        rs_from_mj = np.diag([1.0, -1.0, -1.0])
        R_c2b = mat.T @ rs_from_mj
        t_c2b = pos
        return rt_to_homogeneous(R_c2b, t_c2b.reshape(3, 1))

    def set_joints_deg(self, joints_deg: np.ndarray) -> None:
        import mujoco

        q = np.asarray(joints_deg, dtype=np.float64).reshape(-1)
        if q.size < 7:
            q = np.pad(q, (0, 7 - q.size))
        for i in range(6):
            self.data.qpos[self._joint_qpos_idx[i]] = np.deg2rad(q[i])
        self.data.qpos[self._joint_qpos_idx[6]] = j6_deg_to_slide(q[6])
        self.data.ctrl[:] = self.data.qpos[self._joint_qpos_idx]
        mujoco.mj_forward(self.model, self.data)
        self._update_grasp_physics()

    def read_joints_deg(self) -> np.ndarray:
        out = np.zeros(7)
        for i in range(6):
            out[i] = np.rad2deg(self.data.qpos[self._joint_qpos_idx[i]])
        out[6] = j6_slide_to_deg(self.data.qpos[self._joint_qpos_idx[6]])
        return out

    def move_joints_deg(self, target_deg: np.ndarray, steps: int = 15, settle: bool = True) -> None:
        import mujoco

        target = np.asarray(target_deg, dtype=np.float64).reshape(-1)
        start = self.read_joints_deg()
        for s in range(1, steps + 1):
            alpha = s / steps
            q = start + alpha * (target - start)
            self.set_joints_deg(q)
            for _ in range(5):
                mujoco.mj_step(self.model, self.data)
                self._update_grasp_physics()
            time.sleep(0.01 if settle else 0.0)

    def robot_pose_dict(self) -> dict:
        return joint_angles_to_robot_pose(self.read_joints_deg(), self.fk)

    def render_calib_rgb(self) -> np.ndarray:
        """OpenCV 可检测的棋盘合成图（用于标定求解）。"""
        return render_board_for_robot_pose(
            self.robot_pose_dict(),
            self.T_cam_base_gt,
            self.T_target_gripper,
            SIM_CAMERA_MATRIX,
            SIM_DIST_COEFFS,
            self.board,
            (self.width, self.height),
        )

    def render_rgb(self) -> np.ndarray:
        """MuJoCo 渲染 RGB（固定相机视角）。"""
        return self._render_camera("fixed_cam")

    def render_overview(self) -> np.ndarray:
        """侧视全景（演示录制用）。"""
        return self._render_camera("overview_cam")

    def _render_camera(self, camera_name: str) -> np.ndarray:
        import mujoco

        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        self.renderer.update_scene(self.data, camera=cam_id)
        rgb = self.renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def render_depth(self) -> np.ndarray:
        import mujoco

        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        self.renderer.update_scene(self.data, camera=cam_id)
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()
        # MuJoCo depth: 0=远, 1=近 -> 毫米 z16
        depth_m = self.model.stat.extent / (depth + 1e-6)
        depth_u16 = np.clip(depth_m / SIM_DEPTH_SCALE, 0, 65535).astype(np.uint16)
        return depth_u16

    def object_cam_position(self, class_name: str) -> np.ndarray:
        """物体在 RealSense 相机坐标系下的位置 (m)。"""
        import mujoco

        name_map = {"bottle": "bottle", "cup": "bottle"}
        body = name_map.get(class_name.lower(), "bottle")
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        pos_base = self.data.xpos[bid].copy()

        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        cam_body = self.model.cam_bodyid[cam_id]
        cam_pos = self.data.xpos[cam_body]
        cam_mat = self.data.xmat[cam_body].reshape(3, 3)
        p_cam = cam_mat @ (pos_base - cam_pos)
        p_cam[1] *= -1
        p_cam[2] *= -1
        return p_cam

    def project_cam_to_pixel(self, p_cam: np.ndarray) -> tuple[int, int] | None:
        """相机坐标 -> 像素 (u, v)。"""
        p = np.asarray(p_cam, dtype=np.float64).reshape(3)
        z = float(p[2])
        if z <= 0.02:
            return None
        u = int(SIM_CAMERA_MATRIX[0, 0] * p[0] / z + SIM_CAMERA_MATRIX[0, 2])
        v = int(SIM_CAMERA_MATRIX[1, 1] * p[1] / z + SIM_CAMERA_MATRIX[1, 2])
        u = int(np.clip(u, 0, self.width - 1))
        v = int(np.clip(v, 0, self.height - 1))
        return u, v

    def get_object_in_cam(self, class_name: str) -> SimObject:
        import mujoco

        name_map = {"bottle": "bottle", "cup": "bottle"}
        body = name_map.get(class_name.lower(), "bottle")
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        pos_base = self.data.xpos[bid].copy()
        obj = self._objects.get(class_name.lower(), SimObject(class_name, pos_base))
        return SimObject(class_name, pos_base, obj.size_m)

    def close(self) -> None:
        if hasattr(self, "renderer") and self.renderer is not None:
            self.renderer.close()
            self.renderer = None


def create_sim_env(headless: bool = True) -> MuJoCoSimEnv:
    return MuJoCoSimEnv(headless=headless)
