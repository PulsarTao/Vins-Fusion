#!/bin/bash
# =============================================================================
#  在 x86 开发机上交叉构建 aarch64 镜像，并导出可直接拷贝到板子的 tar 包
#
#  用法:
#    ./build_arm64.sh              构建通用镜像(Pi5 / RDK X5 / Jetson 通用, CPU)
#    ./build_arm64.sh --jetson     构建 Jetson CUDA 加速变体
#    ./build_arm64.sh --no-export  只构建不导出 tar
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_CTX="$ROOT/build_ctx"
OUT_DIR="$ROOT/dist"

VARIANT=universal
EXPORT=true
for a in "$@"; do
    case "$a" in
        --jetson)    VARIANT=jetson ;;
        --no-export) EXPORT=false ;;
        *) echo "未知参数: $a"; exit 1 ;;
    esac
done

if [ "$VARIANT" = jetson ]; then
    DOCKERFILE=Dockerfile.jetson
    IMAGE=vins-ros2:humble-jetson
    TARBALL=vins-ros2-humble-jetson.tar
else
    DOCKERFILE=Dockerfile          # 架构无关：靠 --platform 决定 amd64/arm64
    IMAGE=vins-ros2:humble-arm64
    TARBALL=vins-ros2-humble-arm64.tar
fi

echo "════════════════════════════════════════════════════════"
echo "  构建 $IMAGE  (linux/arm64)"
echo "  变体: $VARIANT"
echo "════════════════════════════════════════════════════════"

# ---- 1. 确认 QEMU aarch64 模拟可用 ----
if [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
    echo "▸ 注册 QEMU 多架构模拟..."
    docker run --privileged --rm tonistiigi/binfmt --install arm64
fi

# ---- 2. 准备构建上下文（源码 + 配置 + launch + 脚本打进镜像）----
[ -d "$ROOT/src_upstream/vins" ] || { echo "✗ 缺少源码，先运行: ./scripts/fetch_upstream.sh"; exit 1; }
echo "▸ 准备构建上下文..."
rm -rf "$BUILD_CTX"
mkdir -p "$BUILD_CTX"
cp -r "$ROOT/src_upstream" "$BUILD_CTX/src"
rm -rf "$BUILD_CTX/src/.git" "$BUILD_CTX/src/docker"
cp -r "$ROOT/config" "$ROOT/launch" "$ROOT/scripts" "$BUILD_CTX/"
cp "$ROOT/docker/egm96-5.tar.bz2" "$BUILD_CTX/"
cp "$ROOT/docker/$DOCKERFILE" "$BUILD_CTX/"

# ---- 3. buildx 构建器 ----
# 关键：docker-container 驱动跑在独立容器里，既不继承宿主 daemon 的 registry-mirrors，
# 也不继承 /etc/docker/daemon.json 里的 proxies。国内网络下直连 Docker Hub 必然超时，
# 所以这里显式把宿主代理透传进去，并用 network=host 让容器能访问 127.0.0.1 上的代理。
detect_proxy() {
    # 优先用环境变量，其次读 daemon.json，最后探测常见本地代理端口
    local p="${HTTPS_PROXY:-${https_proxy:-}}"
    if [ -z "$p" ] && [ -f /etc/docker/daemon.json ]; then
        p=$(python3 -c "
import json,sys
try:
    d=json.load(open('/etc/docker/daemon.json'))
    print(d.get('proxies',{}).get('https-proxy',''))
except Exception:
    print('')" 2>/dev/null)
    fi
    if [ -z "$p" ]; then
        for port in 10808 7890 1080 8118; do
            if ss -tln 2>/dev/null | grep -q ":$port "; then p="http://127.0.0.1:$port"; break; fi
        done
    fi
    echo "$p"
}

PROXY=$(detect_proxy)
if ! docker buildx inspect vins-builder >/dev/null 2>&1; then
    echo "▸ 创建 buildx 构建器..."
    OPTS=(--name vins-builder --driver docker-container --driver-opt network=host)
    if [ -n "$PROXY" ]; then
        PROXY="${PROXY%/}"        # 去掉尾斜杠，否则 buildkit 解析代理地址会失败
        echo "  使用代理: $PROXY"
        # 注意：--driver-opt 的值内不能含逗号（buildx 用逗号分隔 k=v），
        # 所以这里不设 NO_PROXY（多主机必然要用逗号）。
        OPTS+=(--driver-opt "env.HTTP_PROXY=$PROXY"
               --driver-opt "env.HTTPS_PROXY=$PROXY")
    else
        echo "  未检测到代理，直连 Docker Hub（国内网络可能超时）"
    fi
    docker buildx create "${OPTS[@]}" --use >/dev/null
fi
docker buildx use vins-builder

# ---- 4. 构建（QEMU 模拟编译较慢，首次约 40-90 分钟）----
echo "▸ 开始构建（QEMU 模拟 aarch64，请耐心等待）..."
docker buildx build \
    --platform linux/arm64 \
    --file "$BUILD_CTX/$DOCKERFILE" \
    --tag "$IMAGE" \
    --load \
    "$BUILD_CTX"


# ---- 清理构建产生的悬空镜像(<none>) ----
# 每次重建都会把旧镜像变成 <none>，不清会越积越多占满磁盘
DANGLING=$(docker images -f "dangling=true" -q 2>/dev/null | wc -l)
if [ "${DANGLING:-0}" -gt 0 ]; then
    echo "▸ 清理 $DANGLING 个悬空镜像(<none>)..."
    docker image prune -f >/dev/null 2>&1 || true
fi
echo "✓ 镜像构建完成: $IMAGE"
docker image inspect "$IMAGE" --format '  大小: {{.Size}} bytes  架构: {{.Architecture}}'

# ---- 5. 导出为 tar，供拷贝到板子 ----
if $EXPORT; then
    mkdir -p "$OUT_DIR"
    echo "▸ 导出镜像到 $OUT_DIR/$TARBALL ..."
    docker save "$IMAGE" -o "$OUT_DIR/$TARBALL"
    gzip -f "$OUT_DIR/$TARBALL"
    cp "$ROOT/scripts/deploy.sh" "$OUT_DIR/"
    echo "✓ 导出完成:"
    ls -lh "$OUT_DIR/"
    echo
    echo "下一步 —— 拷贝到板子并部署:"
    echo "  scp $OUT_DIR/${TARBALL}.gz $OUT_DIR/deploy.sh <用户>@<板子IP>:~/"
    echo "  ssh <用户>@<板子IP> 'bash ~/deploy.sh ~/${TARBALL}.gz'"
fi
