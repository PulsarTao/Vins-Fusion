#!/bin/bash
# =============================================================================
#  获取第三方 VINS-Fusion ROS2 源码（首次使用必须先跑这个）
#
#  为什么源码不在仓库里：
#    它是上游 GPL-3.0 项目，有明确的仓库和提交号；且含一个 58MB 的
#    词袋二进制文件(support_files/brief_k10L6.bin)，纳入版本控制会让
#    每次 clone 都变慢。这里锁定提交号获取，保证可复现。
#
#  用法:
#    ./scripts/fetch_upstream.sh          获取锁定版本
#    ./scripts/fetch_upstream.sh --latest 获取上游最新版（可能不兼容，谨慎）
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DST="$ROOT/src_upstream"
REPO=https://github.com/zinuok/VINS-Fusion-ROS2.git
# 锁定到实测可用的提交。换版本前请先读 docs/UPSTREAM.md 里的依赖约束。
PINNED=72023bc

USE_LATEST=false
[ "${1:-}" = "--latest" ] && USE_LATEST=true

PATCH_DIR="$ROOT/patches"

# 本项目对上游的必要改动放在 patches/，clone 之后统一打上。
# 不直接把改过的源码提交进仓库，是为了保持「上游代码 + 可审查的差异」这种形式：
# 换上游版本时能立刻看出哪些改动需要重新适配。
apply_patches() {
    [ -d "$PATCH_DIR" ] || return 0
    local p
    for p in "$PATCH_DIR"/*.patch; do
        [ -e "$p" ] || continue
        if git -C "$DST" apply --check "$p" 2>/dev/null; then
            git -C "$DST" apply "$p"
            echo "  ✓ 已打补丁 $(basename "$p")"
        elif git -C "$DST" apply --reverse --check "$p" 2>/dev/null; then
            echo "  · 已是打过的状态，跳过 $(basename "$p")"
        else
            echo "  ✗ 补丁打不上: $(basename "$p")"
            echo "    多半是换了上游版本导致上下文对不上，需要手工适配后重新生成。"
            exit 1
        fi
    done
}

if [ -d "$DST/vins" ]; then
    echo "✓ 源码已存在: $DST"
    echo "▸ 检查补丁..."
    apply_patches
    echo "  要重新获取请先删除: rm -rf $DST"
    exit 0
fi

echo "▸ 获取 VINS-Fusion ROS2 源码..."
if $USE_LATEST; then
    echo "  ⚠ 使用上游最新版，可能与本项目的 Dockerfile 依赖约束不兼容"
    git clone --depth 1 "$REPO" "$DST"
else
    echo "  锁定版本: $PINNED"
    git clone "$REPO" "$DST"
    (cd "$DST" && git checkout -q "$PINNED")
fi

echo "▸ 应用本项目补丁..."
apply_patches

echo "✓ 完成: $DST"
echo
echo "下一步:"
echo "  ./scripts/build_amd64.sh    # x86 本机构建"
echo "  ./scripts/build_arm64.sh    # 交叉构建给边缘板子"
