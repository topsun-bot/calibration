#!/usr/bin/env bash
# 编译 D1 通信桥接（d1_send_joints / d1_read_joints / d1_arm_zero）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BRIDGE="$ROOT/calibate/tools/d1_bridge"
BUILD="$BRIDGE/build"

# D1 SDK 路径（可通过环境变量覆盖；无效时自动用项目内 third_party/d1_sdk）
PROJECT_D1_SDK="$ROOT/third_party/d1_sdk"

_sdk_ok() {
  [[ -d "$1/include" && -d "$1/lib" ]] || return 1
  [[ -f "$1/lib/libddsc.so" && -f "$1/lib/libddscxx.so" ]] || return 1
  [[ -f "$1/lib/libunitree_sdk2.a" || -f "$1/lib/libunitree_sdk2.so" ]] || return 1
}

_resolve_d1_sdk() {
  local requested="${D1_SDK:-}"
  if [[ -n "$requested" ]] && _sdk_ok "$requested"; then
    echo "$requested"
    return 0
  fi
  if [[ -n "$requested" ]]; then
    echo "警告: D1_SDK='$requested' 不可用或未编译完整，改用 $PROJECT_D1_SDK" >&2
    echo "      若曾设置 export D1_SDK=/home/unitree/d1_sdk，可 unset D1_SDK 或改为项目内路径" >&2
  fi
  echo "$PROJECT_D1_SDK"
}

D1_SDK="$(_resolve_d1_sdk)"

echo "==> 项目根目录: $ROOT"
echo "==> D1 bridge:  $BRIDGE"
echo "==> D1_SDK:     $D1_SDK"

if ! _sdk_ok "$D1_SDK"; then
  echo ""
  echo "未找到完整 D1 SDK，正在自动下载并编译 unitree_sdk2 ..."
  python3 "$ROOT/calibate/tools/get_d1_sdk.py" --install-prefix "$D1_SDK"
  echo ""
fi

if ! _sdk_ok "$D1_SDK"; then
  echo ""
  echo "错误: D1 SDK 仍不完整: $D1_SDK"
  echo "可执行: unset D1_SDK && ./scripts/build_d1_bridge.sh"
  echo "或查看: calibate/tools/d1_bridge/README.md"
  exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
  echo ""
  echo "错误: 未安装 cmake"
  echo "请执行: sudo apt install -y cmake build-essential"
  echo "  或:   sudo snap install cmake"
  exit 1
fi

if ! command -v g++ >/dev/null 2>&1; then
  echo "错误: 未安装 g++，请执行: sudo apt install -y build-essential"
  exit 1
fi

mkdir -p "$BUILD"
cd "$BUILD"
echo "==> cmake .. -DD1_SDK=$D1_SDK"
cmake .. -DD1_SDK="$D1_SDK"
echo "==> make -j$(nproc 2>/dev/null || echo 4)"
make -j"$(nproc 2>/dev/null || echo 4)"

echo ""
echo "==> 编译成功，可执行文件:"
ls -la "$BUILD"/d1_send_joints "$BUILD"/d1_read_joints "$BUILD"/d1_arm_zero "$BUILD"/d1_joint_enable "$BUILD"/d1_arm_release
echo ""
echo "测试（需 D1 已连接）:"
echo "  export D1_NETWORK_INTERFACE=eth0"
echo "  $BUILD/d1_read_joints"
