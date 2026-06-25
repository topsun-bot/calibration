#!/usr/bin/env python3
"""自动获取 D1 bridge 所需依赖：DDS 消息文件 + unitree_sdk2。"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOLS = Path(__file__).resolve().parent
BRIDGE_MSG = TOOLS / "d1_bridge" / "msg"
THIRD_PARTY = ROOT / "third_party"
UNITREE_SRC = THIRD_PARTY / "unitree_sdk2"

MSG_BASE = (
    "https://raw.githubusercontent.com/chen37058/Grasp-with-the-Unitree-D1/main/src/msg"
)
MSG_FILES = [
    "ArmString_.cpp",
    "ArmString_.hpp",
    "PubServoInfo_.cpp",
    "PubServoInfo_.hpp",
]

UNITREE_REPO = "https://github.com/unitreerobotics/unitree_sdk2.git"


def log(msg: str) -> None:
    print(f"==> {msg}")


def download_file(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        log(f"已存在，跳过: {dest.name}")
        return
    log(f"下载: {url}")
    urllib.request.urlretrieve(url, dest)


def download_msg_files(force: bool = False) -> Path:
    BRIDGE_MSG.mkdir(parents=True, exist_ok=True)
    for name in MSG_FILES:
        dest = BRIDGE_MSG / name
        if force and dest.exists():
            dest.unlink()
        download_file(f"{MSG_BASE}/{name}", dest)
    log(f"DDS 消息文件: {BRIDGE_MSG}")
    return BRIDGE_MSG


def run(cmd: list[str], *, cwd: Path | None = None) -> None:
    log(" ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def clone_unitree_sdk2(force: bool = False) -> Path:
    THIRD_PARTY.mkdir(parents=True, exist_ok=True)
    if UNITREE_SRC.exists():
        if force:
            shutil.rmtree(UNITREE_SRC)
        else:
            log(f"unitree_sdk2 源码已存在: {UNITREE_SRC}")
            return UNITREE_SRC
    run(["git", "clone", "--depth", "1", UNITREE_REPO, str(UNITREE_SRC)])
    return UNITREE_SRC


def sdk_ready(install_prefix: Path) -> bool:
    lib_dir = install_prefix / "lib"
    inc_dir = install_prefix / "include"
    if not lib_dir.is_dir() or not inc_dir.is_dir():
        return False
    has_sdk = any((lib_dir / n).exists() for n in ("libunitree_sdk2.a", "libunitree_sdk2.so"))
    has_dds = (lib_dir / "libddsc.so").exists() and (lib_dir / "libddscxx.so").exists()
    return has_sdk and has_dds


def build_unitree_sdk2(install_prefix: Path, force: bool = False) -> None:
    if sdk_ready(install_prefix) and not force:
        log(f"unitree_sdk2 已安装: {install_prefix}")
        return

    src = clone_unitree_sdk2(force=force)
    build_dir = src / "build"
    if force and build_dir.exists():
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)

    install_prefix.mkdir(parents=True, exist_ok=True)
    run(
        [
            "cmake",
            "..",
            f"-DCMAKE_INSTALL_PREFIX={install_prefix}",
            "-DBUILD_EXAMPLES=OFF",
        ],
        cwd=build_dir,
    )
    run(["make", "-j", str(_cpu_count())], cwd=build_dir)
    run(["make", "install"], cwd=build_dir)

    if not sdk_ready(install_prefix):
        raise RuntimeError(f"安装后仍缺少库文件，请检查: {install_prefix}/lib")


def sync_msg_to_sdk(install_prefix: Path) -> None:
    """兼容 CMakeLists 中 D1_SDK/example/src/msg/ 路径。"""
    sdk_msg = install_prefix / "example" / "src" / "msg"
    sdk_msg.mkdir(parents=True, exist_ok=True)
    for name in MSG_FILES:
        src = BRIDGE_MSG / name
        dst = sdk_msg / name
        if not src.exists():
            continue
        if not dst.exists() or src.read_bytes() != dst.read_bytes():
            shutil.copy2(src, dst)


def _cpu_count() -> int:
    try:
        return max(1, len(os.sched_getaffinity(0)))
    except Exception:
        return max(1, os.cpu_count() or 4)


def _path_usable(prefix: Path) -> bool:
    """目录存在且可写，或父目录可创建。"""
    try:
        if prefix.exists():
            test = prefix / ".write_test"
            test.touch()
            test.unlink()
            return True
        prefix.parent.mkdir(parents=True, exist_ok=True)
        prefix.mkdir(parents=True, exist_ok=True)
        return True
    except OSError:
        return False


def default_install_prefix() -> Path:
    project_sdk = ROOT / "third_party" / "d1_sdk"
    env = os.environ.get("D1_SDK")
    if env:
        p = Path(env)
        if sdk_ready(p):
            return p
        if _path_usable(p):
            return p
        log(f"警告: D1_SDK={env} 不可用或无写权限，改用 {project_sdk}")
    home_sdk = Path("/home/unitree/d1_sdk")
    if home_sdk.is_dir() and sdk_ready(home_sdk):
        return home_sdk
    return project_sdk


def main() -> None:
    parser = argparse.ArgumentParser(description="下载 D1 bridge 依赖并安装 unitree_sdk2")
    parser.add_argument(
        "--install-prefix",
        type=Path,
        default=None,
        help="安装目录（默认: $D1_SDK 或 third_party/d1_sdk）",
    )
    parser.add_argument("--force", action="store_true", help="强制重新下载/编译")
    args = parser.parse_args()

    prefix = args.install_prefix or default_install_prefix()
    log(f"安装目录: {prefix.resolve()}")

    if not shutil.which("cmake"):
        print("错误: 未找到 cmake，请执行: sudo apt install -y cmake build-essential", file=sys.stderr)
        sys.exit(1)
    if not shutil.which("git"):
        print("错误: 未找到 git，请执行: sudo apt install -y git", file=sys.stderr)
        sys.exit(1)
    if not shutil.which("g++"):
        print("错误: 未找到 g++，请执行: sudo apt install -y build-essential", file=sys.stderr)
        sys.exit(1)

    download_msg_files(force=args.force)
    build_unitree_sdk2(prefix, force=args.force)
    sync_msg_to_sdk(prefix)

    print("")
    print("完成。编译 d1_bridge 请执行:")
    print(f"  export D1_SDK={prefix.resolve()}")
    print("  ./scripts/build_d1_bridge.sh")


if __name__ == "__main__":
    main()
