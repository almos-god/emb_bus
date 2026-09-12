#!/usr/bin/env bash
# ============================================================================
# 文件: emb_bus/scripts/run_tests.sh
# 描述: emb_bus 一键构建+测试脚本（本地 CI）
# 用法: ./scripts/run_tests.sh [--bench] [--sanitizer]
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build}"
ENABLE_BENCH=0
SANITIZER=OFF

for arg in "$@"; do
  case "$arg" in
    --bench) ENABLE_BENCH=1 ;;
    --sanitizer) SANITIZER=ON ;;
    *) echo "未知参数: $arg"; exit 1 ;;
  esac
done

# ROS2 环境（存在则启用 ROS2 模式构建）；ament 脚本与 set -u 不兼容，临时关闭
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  echo "[1/3] 配置（ROS2 模式, sanitizer=$SANITIZER）"
else
  echo "[1/3] 配置（独立模式, sanitizer=$SANITIZER）"
fi

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DEMB_BUS_SANITIZER=$SANITIZER \
  -DBUILD_TESTING=ON

echo "[2/3] 构建"
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "[3/3] 测试"
cd "$BUILD_DIR"
ctest --output-on-failure

if [ "$ENABLE_BENCH" -eq 1 ]; then
  echo "运行基准（快速模式）"
  if [ -x "$BUILD_DIR/benchmark_latency" ]; then
    "$BUILD_DIR/benchmark_latency" --quick 2>/dev/null || "$BUILD_DIR/benchmark_latency"
  else
    echo "未找到 benchmark_latency 可执行文件，跳过"
  fi
fi

echo "全部通过 ✓"
