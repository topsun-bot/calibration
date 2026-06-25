"""Unitree D1 机械臂控制与状态读取。

基于 [D1Arm 服务接口](https://support.unitree.com/home/zh/developer/D1Arm_services)：
- 命令话题: rt/arm_Command (JSON ArmString)
- 反馈话题: current_servo_angle (PubServoInfo, 7 个关节角，度)
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BRIDGE_DIR = ROOT / "tools" / "d1_bridge" / "build"
D1_SUBNET_PREFIX = "192.168.123."


@dataclass
class D1Config:
    network_interface: str | None = None
    bridge_dir: Path = DEFAULT_BRIDGE_DIR
    settle_time: float = 3.0
    command_interval: float = 0.5
    joint_tolerance_deg: float = 2.0
    max_wait: float = 20.0

    def __post_init__(self) -> None:
        self.network_interface = resolve_d1_network_interface(self.network_interface)


def list_ipv4_interfaces() -> list[dict[str, str]]:
    """列出 UP 状态网卡的 IPv4 地址。"""
    try:
        out = subprocess.run(
            ["ip", "-4", "-br", "addr", "show", "up"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError):
        return []

    interfaces: list[dict[str, str]] = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        name, _, addr = parts[0], parts[1], parts[2]
        if addr.startswith("127."):
            continue
        interfaces.append({"name": name, "addr": addr.split("/")[0]})
    return interfaces


def find_d1_subnet_interface() -> str | None:
    """查找位于 D1 默认网段 192.168.123.0/24 的网卡。"""
    for iface in list_ipv4_interfaces():
        if iface["addr"].startswith(D1_SUBNET_PREFIX):
            return iface["name"]
    return None


def resolve_d1_network_interface(requested: str | None = None) -> str | None:
    """
    解析 D1 DDS 网卡名。

    优先级：有效 requested → $D1_NETWORK_INTERFACE → 192.168.123.x 网段自动检测 → None（SDK 默认）
    """
    up_names = {i["name"] for i in list_ipv4_interfaces()}

    def valid(name: str | None) -> bool:
        return bool(name and name in up_names)

    if valid(requested):
        return requested

    env_iface = os.environ.get("D1_NETWORK_INTERFACE")
    if requested and not valid(requested):
        auto = find_d1_subnet_interface()
        if auto:
            print(
                f"[D1] 警告: 网卡 '{requested}' 不存在或未启用，"
                f"自动改用 {auto} ({D1_SUBNET_PREFIX}*)"
            )
            return auto
        hint = ", ".join(sorted(up_names)) or "(无)"
        raise RuntimeError(
            f"D1 网卡 '{requested}' 不可用。当前 UP 网卡: {hint}\n"
            f"请指定正确网卡，例如: --network enx6c1ff765a912\n"
            f"或设置: export D1_NETWORK_INTERFACE=你的网卡名"
        )

    if valid(env_iface):
        return env_iface

    auto = find_d1_subnet_interface()
    if auto:
        print(f"[D1] 自动选择网卡 {auto} ({D1_SUBNET_PREFIX}*)")
        return auto

    return None


def format_d1_network_hint() -> str:
    ifaces = list_ipv4_interfaces()
    if not ifaces:
        return "未检测到 UP 网卡"
    lines = [f"  {i['name']:16s}  {i['addr']}" for i in ifaces]
    d1 = find_d1_subnet_interface()
    if d1:
        lines.append(f"  → 推荐 D1 网卡: {d1}")
    return "\n".join(lines)


class MockD1Arm:
    """无真机时的仿真后端。"""

    def __init__(self) -> None:
        self._angles = np.array([0.0, -30.0, 60.0, 0.0, 30.0, 0.0, 0.0])

    def read_joints(self) -> np.ndarray:
        return self._angles.copy()

    def move_joints(
        self,
        angles_deg: list[float] | np.ndarray,
        wait: bool = True,
        *,
        quick: bool = False,
    ) -> None:
        target = np.asarray(angles_deg, dtype=np.float64).reshape(-1)
        if target.size != 7:
            raise ValueError("D1 需要 7 个关节角 (6 臂 + 1 夹爪)")
        if wait:
            steps = 3 if quick else 10
            delay = 0.02 if quick else 0.05
            start = self._angles.copy()
            for s in range(1, steps + 1):
                self._angles = start + (target - start) * (s / steps)
                time.sleep(delay)
        else:
            self._angles = target.copy()

    def go_zero(self) -> None:
        self.move_joints([0, 0, 0, 0, 0, 0, 0])

    def set_manual_mode(self, enabled: bool, *, countdown_s: float = 3.0) -> None:
        if enabled and countdown_s > 0:
            time.sleep(min(countdown_s, 0.01))

    def close(self) -> None:
        pass


class D1BridgeArm:
    """通过 tools/d1_bridge 编译产物与 D1 通信。"""

    def __init__(self, cfg: D1Config) -> None:
        self.cfg = cfg
        self.send_bin = self._find("d1_send_joints")
        self.read_bin = self._find("d1_read_joints")
        self.zero_bin = self._find("d1_arm_zero")
        self.enable_bin = self._find("d1_joint_enable", required=False)
        self.release_bin = self._find("d1_arm_release", required=False)
        self._drag_active = False
        self._shutdown_done = False
        env = os.environ.copy()
        if cfg.network_interface:
            env["D1_NETWORK_INTERFACE"] = cfg.network_interface
        self.env = env

    def _find(self, name: str, required: bool = True) -> Path | None:
        local = self.cfg.bridge_dir / name
        if local.exists():
            return local
        which = shutil.which(name)
        if which:
            return Path(which)
        if not required:
            return None
        raise FileNotFoundError(
            f"未找到 {name}。\n"
            f"  期望路径: {local}\n"
            f"  请先编译 D1 bridge:\n"
            f"    export D1_SDK=/home/unitree/d1_sdk   # 按实际 SDK 路径修改\n"
            f"    ./scripts/build_d1_bridge.sh\n"
            f"  无真机调试可用: --mock-arm\n"
            f"  详见: calibate/tools/d1_bridge/README.md"
        )

    def _run_bridge(self, bin_path: Path, args: list[str] | None = None, timeout: float = 10) -> None:
        cmd = [str(bin_path), *(args or [])]
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, env=self.env, check=False
        )
        if proc.returncode != 0:
            raise RuntimeError(f"{bin_path.name} 失败: {proc.stderr or proc.stdout}")

    def read_joints(self) -> np.ndarray:
        proc = subprocess.run(
            [str(self.read_bin)],
            capture_output=True,
            text=True,
            timeout=5,
            env=self.env,
            check=False,
        )
        if proc.returncode != 0:
            err = proc.stderr or proc.stdout or ""
            if "does not match an available interface" in err:
                raise RuntimeError(
                    f"读取关节角失败: D1 网卡配置错误。\n{err.strip()}\n"
                    f"可用网卡:\n{format_d1_network_hint()}\n"
                    f"请使用: --network <网卡名>  或  export D1_NETWORK_INTERFACE=<网卡名>"
                ) from None
            raise RuntimeError(f"读取关节角失败: {err}")
        data = json.loads(proc.stdout.strip())
        return np.asarray(data["joints_deg"], dtype=np.float64)

    def move_joints(
        self,
        angles_deg: list[float] | np.ndarray,
        wait: bool = True,
        *,
        quick: bool = False,
    ) -> None:
        angles = np.asarray(angles_deg, dtype=np.float64).reshape(-1)
        if angles.size != 7:
            raise ValueError("D1 需要 7 个关节角")
        args = [str(self.send_bin), *[str(float(a)) for a in angles]]
        proc = subprocess.run(
            args, capture_output=True, text=True, timeout=10, env=self.env, check=False
        )
        if proc.returncode != 0:
            raise RuntimeError(f"发送关节命令失败: {proc.stderr or proc.stdout}")
        if wait:
            self._wait_until_settled(angles, quick=quick)

    def _wait_until_settled(self, target: np.ndarray, *, quick: bool = False) -> None:
        interval = 0.15 if quick else self.cfg.command_interval
        settle = 0.5 if quick else self.cfg.settle_time
        tolerance = 3.0 if quick else self.cfg.joint_tolerance_deg
        max_wait = 6.0 if quick else self.cfg.max_wait
        time.sleep(interval)
        t0 = time.time()
        while time.time() - t0 < max_wait:
            current = self.read_joints()
            if np.max(np.abs(current - target)) <= tolerance:
                time.sleep(settle)
                return
            time.sleep(0.15 if quick else 0.2)
        if not quick:
            print("[警告] 关节未在预期时间内到位，继续采集")

    def go_zero(self) -> None:
        proc = subprocess.run(
            [str(self.zero_bin)],
            capture_output=True,
            text=True,
            timeout=10,
            env=self.env,
            check=False,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"回零失败: {proc.stderr or proc.stdout}")
        time.sleep(self.cfg.settle_time + 1.0)

    def set_joint_enable(self, enabled: bool) -> None:
        """funcode=5：mode=0 卸力可手拖，mode=1 位置保持。"""
        if self.enable_bin is None:
            raise FileNotFoundError(
                "未找到 d1_joint_enable。请重新编译 bridge:\n"
                "  ./scripts/build_d1_bridge.sh"
            )
        mode = 1 if enabled else 0
        self._run_bridge(self.enable_bin, [str(mode)])
        time.sleep(0.3)

    def set_manual_mode(self, enabled: bool, *, countdown_s: float = 3.0) -> None:
        """开启/关闭手动拖拽；关闭时用快速锁定，避免长时间等待。"""
        if enabled:
            if countdown_s > 0:
                print(f"[D1] {countdown_s:.0f} 秒后卸力，请扶住机械臂...")
                for remain in range(int(countdown_s), 0, -1):
                    print(f"  {remain}...")
                    time.sleep(1.0)
            self.set_joint_enable(False)
            self._drag_active = True
            print("[D1] 已开启手动拖拽模式（关节卸力，可手拖摆位）")
            return
        joints = self.read_joints()
        self.set_joint_enable(True)
        self._drag_active = False
        self.move_joints(joints, wait=True, quick=True)
        print("[D1] 已关闭手动拖拽，锁定当前位姿")

    def release_control(self) -> None:
        """funcode=7：退出 SDK 控制会话。"""
        if self.release_bin is None:
            print("[D1] 警告: 未找到 d1_arm_release，跳过退出控制")
            return
        self._run_bridge(self.release_bin, [])
        time.sleep(0.2)

    def shutdown_safely(
        self,
        fold_pose: list[float] | np.ndarray | None = None,
        *,
        fold: bool = True,
    ) -> None:
        """程序退出：折叠 → 退出控制 → 卸力。"""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        try:
            print("[D1] 程序退出：折叠并释放机械臂...")
            if fold and fold_pose is not None:
                target = np.asarray(fold_pose, dtype=np.float64).reshape(-1)
                if self._drag_active:
                    self.set_joint_enable(True)
                    self._drag_active = False
                    time.sleep(0.2)
                self.move_joints(target, wait=True, quick=True)
            self.release_control()
            if self.enable_bin is not None:
                self.set_joint_enable(False)
                self._drag_active = False
            print("[D1] 已折叠、退出控制并卸力")
        except Exception as exc:
            print(f"[D1] 退出清理失败: {exc}")

    def close(self) -> None:
        pass


def create_d1_arm(mock: bool = False, cfg: D1Config | None = None) -> MockD1Arm | D1BridgeArm:
    if mock:
        return MockD1Arm()
    return D1BridgeArm(cfg or D1Config())


def build_arm_command_json(angles_deg: list[float] | np.ndarray, seq: int = 4) -> str:
    """构造 rt/arm_Command 的 JSON 载荷（供 bridge C++ 参考）。"""
    angles = np.asarray(angles_deg, dtype=np.float64).reshape(-1)
    payload = {
        "seq": seq,
        "address": 1,
        "funcode": 2,
        "data": {
            "mode": 1,
            **{f"angle{i}": float(angles[i]) for i in range(7)},
        },
    }
    return json.dumps(payload, separators=(",", ":"))
