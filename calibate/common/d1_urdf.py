"""Unitree D1 URDF 自动下载与安装到 config/。"""

from __future__ import annotations

import os
import shutil
import tempfile
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
DEFAULT_URDF = CONFIG_DIR / "d1.urdf"
URDF_ZIP_URL = "https://oss-global-cdn.unitree.com/static/9b20252a26374d50aa369532657d0143.zip"
URDF_ZIP_MEMBER = "d1_550_description/urdf/d1_550_description.urdf"
DEFAULT_TIP_LINK = "Empty_Link6"


def _find_urdf_in_sdk(sdk_root: Path) -> Path | None:
    patterns = [
        "d1_description/urdf/d1.urdf",
        "d1_550_description/urdf/d1_550_description.urdf",
        "**/d1.urdf",
        "**/d1_550_description.urdf",
    ]
    for pattern in patterns:
        matches = sorted(sdk_root.glob(pattern))
        if matches:
            return matches[0]
    return None


def _copy_urdf(src: Path, dst: Path) -> Path:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return dst


def download_d1_urdf(
    output: Path | None = None,
    force: bool = False,
    timeout: float = 120.0,
) -> Path:
    """
    下载官方 D1 URDF 到 config/d1.urdf。

    来源: Unitree Go2/D1 开发包中的 d1_550_description
    文档: https://support.unitree.com/home/zh/developer/D1Arm_services
    """
    output = Path(output or DEFAULT_URDF)
    if output.exists() and not force:
        print(f"[URDF] 已存在，跳过下载: {output}")
        return output

    sdk_env = os.environ.get("D1_SDK")
    if sdk_env:
        sdk_urdf = _find_urdf_in_sdk(Path(sdk_env))
        if sdk_urdf is not None and sdk_urdf.exists():
            _copy_urdf(sdk_urdf, output)
            print(f"[URDF] 从 D1_SDK 复制: {sdk_urdf} -> {output}")
            return output

    print(f"[URDF] 正在下载: {URDF_ZIP_URL}")
    with tempfile.TemporaryDirectory() as tmp:
        zip_path = Path(tmp) / "d1_urdf.zip"
        try:
            with urllib.request.urlopen(URDF_ZIP_URL, timeout=timeout) as resp:
                zip_path.write_bytes(resp.read())
        except urllib.error.URLError as exc:
            raise RuntimeError(
                f"下载 D1 URDF 失败: {exc}\n"
                f"可手动下载 {URDF_ZIP_URL} 并将 urdf 文件放到 {output}"
            ) from exc

        with zipfile.ZipFile(zip_path, "r") as zf:
            names = zf.namelist()
            member = URDF_ZIP_MEMBER
            if member not in names:
                candidates = [n for n in names if n.lower().endswith(".urdf")]
                if not candidates:
                    raise RuntimeError(f"ZIP 中未找到 URDF: {names[:10]}")
                member = candidates[0]
            output.parent.mkdir(parents=True, exist_ok=True)
            with zf.open(member) as src, output.open("wb") as dst:
                shutil.copyfileobj(src, dst)

    print(f"[URDF] 已保存 -> {output.resolve()}")
    return output


def ensure_d1_urdf(
    output: Path | None = None,
    force: bool = False,
) -> Path:
    """若 config/d1.urdf 不存在则自动下载。"""
    output = Path(output or DEFAULT_URDF)
    if output.exists() and not force:
        return output
    return download_d1_urdf(output=output, force=force)
