"""数据读写：标准化 JSON 格式，版本管理，Schema 验证。"""

from __future__ import annotations

import hashlib
import json
import subprocess
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# 格式版本号（SemVer，随格式变更递增 MAJOR）
# ---------------------------------------------------------------------------
FORMAT_VERSION = "1.0.0"

# ---------------------------------------------------------------------------
# 坐标帧命名规范（与 D1 URDF link 名称对齐）
# ---------------------------------------------------------------------------
FRAME_BASE = "base_link"
FRAME_GRIPPER = "gripper_link"
FRAME_CAMERA = "camera_link"
FRAME_CAMERA_OPTICAL = "camera_color_optical_frame"
FRAME_TARGET = "target_frame"  # 标定板


def _make_ros_quaternion(R: np.ndarray) -> list[float]:
    """旋转矩阵 -> ROS 标准四元数 [x, y, z, w]."""
    from .hand_eye_solve import _R_to_quat_wxyz

    wxyz = _R_to_quat_wxyz(R)
    w, x, y, z = wxyz.tolist()
    return [x, y, z, w]


def _make_units_block() -> dict[str, str]:
    return {
        "length": "meters",
        "angle": "radians",
        "rotation_format": "rotation_matrix_3x3",
        "quaternion_convention": "xyzw",  # ROS / tf2 标准
        "euler_convention": "extrinsic_xyz",
    }


def _try_git_sha() -> str | None:
    try:
        return (
            subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
            .decode()
            .strip()
        )
    except Exception:
        return None


def _try_opencv_version() -> str | None:
    try:
        import cv2

        return cv2.__version__
    except Exception:
        return None


def generate_run_id() -> str:
    """生成 UUID 用于关联 poses.json、intrinsics、标定结果."""
    return uuid.uuid4().hex[:12]


# ===================================================================
# 位姿列表
# ===================================================================


def load_pose_list(path: str | Path) -> list[dict]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict) and "samples" in data:
        return data["samples"]
    if isinstance(data, list):
        return data
    raise ValueError(f"无法解析位姿文件: {path}")


# ===================================================================
# 标定结果 保存 / 加载
# ===================================================================


def _build_calibration_payload(
    R: np.ndarray,
    t: np.ndarray,
    parent_frame: str,
    child_frame: str,
    meta: dict[str, Any] | None = None,
    run_id: str | None = None,
    intrinsics_sha256: str | None = None,
    board_config: dict[str, Any] | None = None,
    hardware_info: dict[str, Any] | None = None,
) -> dict:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)

    payload: dict[str, Any] = {
        "format_version": FORMAT_VERSION,
        "run_id": run_id or generate_run_id(),
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "transform": {
            "translation": {
                "x": float(t.flat[0]),
                "y": float(t.flat[1]),
                "z": float(t.flat[2]),
            },
            "rotation": {
                "matrix_3x3": R.tolist(),
                "quaternion_xyzw": _make_ros_quaternion(R),
            },
        },
        "T": T.tolist(),
        # 向后兼容旧格式
        "R": R.tolist(),
        "t": t.reshape(3).tolist(),
        "_units": _make_units_block(),
        "timestamp_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "software": {
            "pipeline": "calibration",
            "git_sha": _try_git_sha(),
            "opencv_version": _try_opencv_version(),
        },
    }
    if intrinsics_sha256:
        payload["intrinsics_sha256"] = intrinsics_sha256
    if board_config:
        payload["board_config"] = board_config
    if hardware_info:
        payload["hardware"] = hardware_info
    if meta:
        payload["meta"] = meta
    return payload


def save_calibration_result(
    path: str | Path,
    R: np.ndarray,
    t: np.ndarray,
    meta: dict[str, Any] | None = None,
    *,
    parent_frame: str = "base_link",
    child_frame: str = "camera_link",
    run_id: str | None = None,
    intrinsics_sha256: str | None = None,
    board_config: dict[str, Any] | None = None,
    hardware_info: dict[str, Any] | None = None,
) -> None:
    """保存标定结果（标准化 JSON v1.0）。

    参数
    ----
    R, t : 旋转矩阵与平移向量 (相机在 base 系)
    parent_frame : 父坐标系名 (如 base_link)
    child_frame : 子坐标系名 (如 camera_link)
    intrinsics_sha256 : camera_intrinsics.json 的 SHA256，保证可追溯
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = _build_calibration_payload(
        R=R,
        t=t,
        parent_frame=parent_frame,
        child_frame=child_frame,
        meta=meta,
        run_id=run_id,
        intrinsics_sha256=intrinsics_sha256,
        board_config=board_config,
        hardware_info=hardware_info,
    )
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def load_calibration_result(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict]:
    """加载标定结果（兼容新旧格式）。

    返回 (R, t, meta_dict)。
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    version = data.get("format_version", "0.0.0")
    if version != FORMAT_VERSION and version != "0.0.0":
        print(
            f"[io_utils] 标定结果格式版本 {version} != 当前 {FORMAT_VERSION}，"
            f"尝试兼容加载"
        )

    # 优先从新格式读取
    if "transform" in data:
        tx = data["transform"]
        t = np.array(
            [tx["translation"]["x"], tx["translation"]["y"], tx["translation"]["z"]],
            dtype=np.float64,
        ).reshape(3, 1)
        R = np.asarray(tx["rotation"]["matrix_3x3"], dtype=np.float64)
    else:
        # 旧格式兼容
        R = np.asarray(data["R"], dtype=np.float64)
        t = np.asarray(data["t"], dtype=np.float64).reshape(3, 1)

    meta = data.get("meta", {})
    # 将标准化字段也注入 meta 供报告使用
    for key in ("format_version", "run_id", "parent_frame", "child_frame",
                 "timestamp_utc", "software", "hardware", "board_config",
                 "intrinsics_sha256", "_units"):
        if key in data and key not in meta:
            meta[key] = data[key]

    return R, t, meta


def compute_intrinsics_hash(path: str | Path) -> str | None:
    """计算 camera_intrinsics.json 的 SHA256，用于标定结果溯源."""
    p = Path(path)
    if not p.exists():
        return None
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


# ===================================================================
# ROS / tf2 互操作
# ===================================================================


def export_tf2_transform(
    calib_path: str | Path,
    camera_link: str = "camera_link",
    base_link: str = "base_link",
) -> dict:
    """导出 ROS tf2 TransformStamped 兼容字典。

    可直接序列化为 JSON 供 rosbridge 或自定义 tf2 publisher 使用。
    """
    R, t, meta = load_calibration_result(calib_path)
    quat = _make_ros_quaternion(R)
    return {
        "header": {
            "frame_id": base_link,
            "stamp": {
                "sec": 0,
                "nanosec": 0,
            },
        },
        "child_frame_id": camera_link,
        "transform": {
            "translation": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
            "rotation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]},
        },
        "meta": meta,
    }


def export_urdf_fragment(
    calib_path: str | Path,
    camera_link: str = "camera_link",
    base_link: str = "base_link",
) -> str:
    """生成 URDF / xacro 固定关节片段，可直接嵌入机器人 URDF。"""
    R, t, _ = load_calibration_result(calib_path)
    quat = _make_ros_quaternion(R)
    return (
        f'<joint name="{camera_link}_joint" type="fixed">\n'
        f'  <parent link="{base_link}"/>\n'
        f'  <child link="{camera_link}"/>\n'
        f'  <origin xyz="{float(t.flat[0]):.6f} {float(t.flat[1]):.6f} {float(t.flat[2]):.6f}"\n'
        f'          rpy="0 0 0"/>\n'
        f'  <!-- quaternion: {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f} -->\n'
        f"</joint>\n"
    )


# ===================================================================
# 验证
# ===================================================================


def validate_calibration_file(path: str | Path) -> dict[str, Any]:
    """基本完整性校验：R 正交性、T 合法性、必需字段."""
    path = Path(path)
    if not path.exists():
        return {"valid": False, "errors": [f"文件不存在: {path}"]}

    try:
        R, t, meta = load_calibration_result(path)
    except Exception as exc:
        return {"valid": False, "errors": [f"加载失败: {exc}"]}

    errors: list[str] = []
    warnings: list[str] = []

    # R 正交性
    det = float(np.linalg.det(R))
    ortho = float(np.linalg.norm(R @ R.T - np.eye(3)))
    if abs(det - 1.0) > 0.01:
        errors.append(f"det(R)={det:.4f} (期望 1.0)")
    if ortho > 0.01:
        errors.append(f"|R@R^T - I|={ortho:.4f} (期望 0.0)")

    # 平移有限
    if not np.all(np.isfinite(t)):
        errors.append("平移向量含 NaN/Inf")

    # 四元数归一化（如果存在新格式）
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if "transform" in data:
            quat = data["transform"]["rotation"].get("quaternion_xyzw")
            if quat:
                qn = np.linalg.norm(quat)
                if abs(qn - 1.0) > 0.001:
                    warnings.append(f"四元数范数={qn:.6f} (期望 1.0)")
    except Exception:
        pass

    return {
        "valid": len(errors) == 0,
        "errors": errors,
        "warnings": warnings,
        "R": R.tolist(),
        "t": t.reshape(3).tolist(),
    }


def export_static_transform_yaml(
    result_path: str | Path,
    output_path: str | Path,
) -> Path:
    """导出 ROS2 static_transform_publisher 兼容的 YAML 参数文件。

    根据标定结果生成标准 ROS2 launch 参数格式的 YAML 文件，
    可直接用于 static_transform_publisher 节点。

    参数
    ----
    result_path : 标定结果 JSON 文件路径
    output_path : 输出 YAML 文件路径

    返回
    ----
    Path : 写入的 YAML 文件路径
    """
    result_path = Path(result_path)
    output_path = Path(output_path)

    R, t, meta = load_calibration_result(result_path)
    quat = _make_ros_quaternion(R)  # [x, y, z, w]

    parent_frame = meta.get("parent_frame", "base_link")
    child_frame = meta.get("child_frame", "camera_link")

    timestamp = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    yaml_content = (
        f"# Auto-generated static transform from hand-eye calibration\n"
        f"# Source: {result_path}\n"
        f"# Generated: {timestamp}\n"
        f"static_transform_publisher:\n"
        f"  ros__parameters:\n"
        f'    frame_id: "{parent_frame}"\n'
        f'    child_frame_id: "{child_frame}"\n'
        f"    translation:\n"
        f"      x: {float(t.flat[0]):.6f}\n"
        f"      y: {float(t.flat[1]):.6f}\n"
        f"      z: {float(t.flat[2]):.6f}\n"
        f"    rotation:\n"
        f"      x: {quat[0]:.6f}\n"
        f"      y: {quat[1]:.6f}\n"
        f"      z: {quat[2]:.6f}\n"
        f"      w: {quat[3]:.6f}\n"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        f.write(yaml_content)

    return output_path
