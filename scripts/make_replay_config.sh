#!/usr/bin/env bash
# 生成「回放专用」配置：把 VINS 订阅的话题改到 /replay/*，
# 这样回放 bag 时不会和实时驱动发布的同名话题串在一起。
#
# 为什么要单独一份而不是直接回放到原话题:
#   实时驱动如果还开着，两路数据会同时进 VINS，时间戳互相穿插，
#   表现为大量重复/回退时间戳 —— 排查时极难看出是数据源的问题。
#   用独立话题名从物理上杜绝这种串扰。
#
# 用法:
#   ./scripts/make_replay_config.sh              # 写进容器 /ros2_ws/vins_config/seeker/_rp.yaml
#   ros2 bag play <bag> --remap /imu_data_raw:=/replay/imu \
#       /cam0/image_raw:=/replay/cam0 /cam1/image_raw:=/replay/cam1
#   docker exec ... ros2 run vins vins_node /ros2_ws/vins_config/seeker/_rp.yaml
#
# 教训: 这个文件早前是手工在容器里改出来的，容器一重建就丢，
# 导致「同一条命令昨天能跑今天报 config_file dosen't exist」。凡是跑测试要用的
# 东西都必须能从版本控制里重建。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=${CONTAINER:-vins-ros2-local}
SRC="$ROOT/config/seeker/seeker_stereo_imu.yaml"
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT

sed -e 's|^imu_topic:.*|imu_topic: "/replay/imu"|' \
    -e 's|^image0_topic:.*|image0_topic: "/replay/cam0"|' \
    -e 's|^image1_topic:.*|image1_topic: "/replay/cam1"|' \
    "$SRC" > "$OUT"

for k in imu_topic image0_topic image1_topic; do
    grep -q "^$k: \"/replay/" "$OUT" || { echo "✗ $k 未被替换，检查 $SRC 的字段名" >&2; exit 1; }
done

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker cp "$OUT" "$CONTAINER:/ros2_ws/vins_config/seeker/_rp.yaml"
    echo "✓ 已写入 $CONTAINER:/ros2_ws/vins_config/seeker/_rp.yaml"
else
    cp "$OUT" "$ROOT/config/seeker/_rp.yaml"
    echo "✓ 容器未运行，已写入 $ROOT/config/seeker/_rp.yaml"
fi
