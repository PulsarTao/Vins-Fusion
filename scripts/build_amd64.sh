#!/bin/bash
# =============================================================================
#  在 x86 机器上构建本机用的 amd64 镜像（仿真 / 本地 D435i 实测都用它）
#
#  用法:
#    ./scripts/build_amd64.sh
#
#  说明：与 build_arm64.sh 共用同一份 docker/Dockerfile —— 它是架构无关的，
#  靠 --platform 决定目标架构。x86 上是原生构建，不走 QEMU，速度快很多。
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_CTX="$ROOT/build_ctx"
IMAGE=${IMAGE:-vins-ros2:humble-amd64}

echo "════════════════════════════════════════════════════════"
echo "  构建 $IMAGE  (linux/amd64, 原生)"
echo "════════════════════════════════════════════════════════"

[ -d "$ROOT/src_upstream/vins" ] || { echo "✗ 缺少源码，先运行: ./scripts/fetch_upstream.sh"; exit 1; }

# ---- 准备构建上下文（源码 + 配置 + launch + 脚本一起打进镜像）----
echo "▸ 准备构建上下文..."
rm -rf "$BUILD_CTX"
mkdir -p "$BUILD_CTX"
cp -r "$ROOT/src_upstream" "$BUILD_CTX/src"
rm -rf "$BUILD_CTX/src/.git" "$BUILD_CTX/src/docker"
cp -r "$ROOT/config" "$ROOT/launch" "$ROOT/scripts" "$BUILD_CTX/"
cp "$ROOT/docker/egm96-5.tar.bz2" "$BUILD_CTX/"
cp "$ROOT/docker/Dockerfile" "$BUILD_CTX/"

# ---- 构建 ----
# 用 buildx + 宿主代理：docker-container 驱动不继承 daemon 的 registry-mirrors
# 和 proxies，国内网络直连 Docker Hub 会超时（build_arm64.sh 里有同样的处理）。
if docker buildx inspect vins-builder >/dev/null 2>&1; then
    echo "▸ 使用已有 buildx 构建器 (含代理配置)"
    docker buildx use vins-builder
    docker buildx build --platform linux/amd64 \
        --file "$BUILD_CTX/Dockerfile" --tag "$IMAGE" --load "$BUILD_CTX"
else
    echo "▸ 用默认 builder（若卡在拉取基础镜像，先跑一次 build_arm64.sh 建带代理的 builder）"
    docker build --file "$BUILD_CTX/Dockerfile" --tag "$IMAGE" "$BUILD_CTX"
fi


# ---- 清理构建产生的悬空镜像(<none>) ----
# 每次重建都会把旧镜像变成 <none>，不清会越积越多占满磁盘
DANGLING=$(docker images -f "dangling=true" -q 2>/dev/null | wc -l)
if [ "${DANGLING:-0}" -gt 0 ]; then
    echo "▸ 清理 $DANGLING 个悬空镜像(<none>)..."
    docker image prune -f >/dev/null 2>&1 || true
fi
echo "✓ 构建完成"
docker image inspect "$IMAGE" --format '  大小: {{.Size}} bytes  架构: {{.Architecture}}'
echo
echo "下一步: ./run.sh doctor   # 环境自检"
echo "        ./run.sh          # 启动仿真"
