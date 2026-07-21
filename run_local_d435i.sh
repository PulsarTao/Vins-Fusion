#!/bin/bash
# =============================================================================
#  本机 D435i 实测 (ROS2 Humble):相机 + VINS-Fusion + rviz2 一键启动
#
#  用法:
#    ./run_local_d435i.sh              启动(官方 realsense2_camera 驱动 + rviz2)
#    ./run_local_d435i.sh --fallback   驱动不兼容固件时,改用 pyrealsense2 发布器
#    ./run_local_d435i.sh --stop       停止
#    ./run_local_d435i.sh --check      仅相机自检
#    ./run_local_d435i.sh --status     查看话题频率与 VINS 状态
# =============================================================================
set -uo pipefail

IMAGE=vins-ros2:humble-amd64
CONTAINER=vins-ros2-local
WS=/ros2_ws
CFG=$WS/vins_config/d435i/d435i_stereo_imu.yaml

C_G='\033[0;32m'; C_Y='\033[1;33m'; C_R='\033[0;31m'; C_B='\033[0;36m'; C_N='\033[0m'
ok(){ echo -e "${C_G}✓${C_N} $*"; }; info(){ echo -e "${C_B}▸${C_N} $*"; }
warn(){ echo -e "${C_Y}!${C_N} $*"; }; err(){ echo -e "${C_R}✗${C_N} $*" >&2; }

inc(){ docker exec "$CONTAINER" /ros_entrypoint.sh bash -c "$1"; }

# --------------------------------------------------------------- 停止
stop_all(){
    # 关键：模式要用 [x] 括号写法。否则 pkill -f 会匹配到承载它自己的
    # `docker exec bash -c "...pkill -f d435i_publisher..."` 命令行，
    # 把自己所在的 shell 先杀掉，后面的清理一条都执行不到 ——
    # 表现就是“明明 stop 了，进程还占着相机”，然后一切看起来都像硬件坏了。
    docker exec "$CONTAINER" bash -c '
        pkill -9 -f "[r]ealsense2_camera" 2>/dev/null
        pkill -9 -f "[d]435i_publisher"   2>/dev/null
        pkill -9 -f "[v]ins_node"         2>/dev/null
        pkill -9 -f "[r]viz2"             2>/dev/null
        pkill -9 -f "[l]oop_fusion"       2>/dev/null
        exit 0' >/dev/null 2>&1 || true
    sleep 3
    # 复核：确认真的清干净了，没清掉就再来一次
    local left
    left=$(docker exec "$CONTAINER" bash -c \
        'ps -eo args --no-headers 2>/dev/null | grep -cE "[d]435i_publisher|[v]ins_node|[r]ealsense2_camera" || true' 2>/dev/null || echo 0)
    if [ "${left:-0}" -gt 0 ] 2>/dev/null; then
        docker exec "$CONTAINER" bash -c \
            'for p in $(ps -eo pid,args --no-headers | grep -E "[d]435i_publisher|[v]ins_node|[r]ealsense2_camera" | awk "{print \$1}"); do kill -9 $p 2>/dev/null; done; exit 0' >/dev/null 2>&1 || true
        sleep 2
    fi
    ok "已停止"
}

# --------------------------------------------------------------- 容器
ensure_container(){
    docker image inspect "$IMAGE" >/dev/null 2>&1 || {
        err "镜像 $IMAGE 不存在。先构建："
        err "  cd $(dirname "$0") && docker buildx build --platform linux/amd64 -t $IMAGE --load build_ctx"
        exit 1; }
    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
        info "创建容器 $CONTAINER ..."
        xhost +local:docker >/dev/null 2>&1
        docker run -d --name "$CONTAINER" --init \
            --privileged --net=host --ipc=host \
            -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
            -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
            -v /dev:/dev -v /run/udev:/run/udev:ro \
            -v "$HOME/vins_output:/root/output" \
            "$IMAGE" sleep infinity >/dev/null
        mkdir -p "$HOME/vins_output"
        sleep 3
    fi
    ok "容器就绪"
}

# --------------------------------------------------------------- 相机自检
# 踩过的坑：残留进程占着相机时，一切表现都像“硬件坏了”（控制传输失败、
# IR stream start failure）。所以自检第一步永远是先清理残留进程。
usb_reset(){
    local L BUS DEV
    L=$(lsusb | grep -i "8086:0b3a") || return 1
    BUS=$(echo "$L" | sed 's/Bus \([0-9]*\).*/\1/')
    DEV=$(echo "$L" | sed 's/.*Device \([0-9]*\).*/\1/')
    python3 -c "
import fcntl
with open('/dev/bus/usb/$BUS/$DEV','wb') as f:
    fcntl.ioctl(f, ord('U')<<8|20, 0)" 2>/dev/null && sleep 4
}

check_camera(){
    info "清理可能占用相机的残留进程..."
    stop_all >/dev/null 2>&1
    lsusb | grep -qi "8086:0b3a" || { err "USB 上找不到 D435i"; return 1; }
    ok "USB 枚举正常"
    usb_reset && ok "USB 已复位"
    inc "python3 -c \"
import pyrealsense2 as rs, sys
d = rs.context().query_devices()
if len(d)==0:
    print('✗ SDK 未检测到设备'); sys.exit(1)
print('✓ 设备:', d[0].get_info(rs.camera_info.name), '固件', d[0].get_info(rs.camera_info.firmware_version))
p=rs.pipeline(); c=rs.config()
c.enable_stream(rs.stream.infrared,1,640,480,rs.format.y8,30)
try:
    p.start(c); p.wait_for_frames(4000); p.stop(); print('✓ 取帧正常')
except Exception as e:
    print('✗ 取帧失败:', str(e)[:70]); sys.exit(1)
\"" 2>&1 | grep -vE '^$'
}

# --------------------------------------------------------------- 状态
show_status(){
    inc '
    echo "=== 话题频率 ==="
    for t in /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw /camera/camera/imu /odometry; do
        r=$(timeout 6 ros2 topic hz $t 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1)
        echo "  $t : ${r:-无数据}"
    done'
}

case "${1:-start}" in
    --stop|stop)     ensure_container >/dev/null; stop_all; exit 0 ;;
    --check|check)   ensure_container; check_camera; exit $? ;;
    --status|status) ensure_container >/dev/null; show_status; exit 0 ;;
esac

FALLBACK=false
[ "${1:-}" = "--fallback" ] && FALLBACK=true

ensure_container
check_camera || { err "相机自检未通过"; exit 1; }
xhost +local:docker >/dev/null 2>&1

# --------------------------------------------------------------- 相机节点
if $FALLBACK; then
    info "启动 D435i 发布器 (pyrealsense2 备用方案)..."
    docker exec -d "$CONTAINER" /ros_entrypoint.sh bash -c \
        "python3 $WS/scripts/d435i_publisher_ros2.py > /root/output/d435i.log 2>&1"
else
    info "启动 realsense2_camera (ROS2 官方驱动)..."
    # 注意：不要传 camera_namespace:=''（空值会被判为 malformed 参数直接启动失败）。
    # 用默认命名空间，话题即 /camera/camera/...，VINS 配置里已按此填写。
    # enable_depth 必须为 true：红外流属于立体模组，关掉 depth 会连带关掉它。
    docker exec -d "$CONTAINER" /ros_entrypoint.sh bash -c \
        "ros2 launch realsense2_camera rs_launch.py \
            enable_infra1:=true enable_infra2:=true \
            enable_color:=false enable_depth:=true \
            depth_module.emitter_enabled:=0 \
            enable_gyro:=true enable_accel:=true \
            unite_imu_method:=2 gyro_fps:=200 accel_fps:=200 \
            > /root/output/rs.log 2>&1"
fi

info "等待图像数据..."
GOT=false
for i in $(seq 1 12); do
    sleep 3
    if inc "timeout 4 ros2 topic hz /camera/camera/infra1/image_rect_raw 2>/dev/null | grep -q 'average rate'"; then
        GOT=true; break
    fi
done
if ! $GOT; then
    err "没有图像数据"
    if ! $FALLBACK; then
        warn "官方驱动可能与相机固件不兼容，试试备用发布器："
        warn "  ./run_local_d435i.sh --fallback"
    fi
    docker exec "$CONTAINER" tail -15 /root/output/rs.log 2>/dev/null
    exit 1
fi
ok "图像数据正常"

# --------------------------------------------------------------- VINS + rviz2
info "启动 VINS-Fusion + rviz2 ..."
docker exec -d "$CONTAINER" /ros_entrypoint.sh bash -c \
    "ros2 run vins vins_node $CFG > /root/output/vins.log 2>&1"
docker exec -d "$CONTAINER" /ros_entrypoint.sh bash -c \
    "rviz2 -d $WS/vins_config/vins_rviz2.rviz > /root/output/rviz.log 2>&1"
sleep 10

echo
echo "────────────────────────────────────────────────────────"
ok "已启动，rviz2 窗口应已出现在屏幕上"
echo
echo -e "  ${C_Y}初始化提示${C_N}：VINS 需要运动才能收敛。"
echo "  请拿起相机缓慢平移 + 轻微旋转几秒。"
echo "  注意不要原地纯旋转 —— 纯旋转没有视差，三角化不出深度。"
echo
echo "  查看状态: ./run_local_d435i.sh --status"
echo "  查看日志: docker exec $CONTAINER tail -f /root/output/vins.log"
echo "  停止:     ./run_local_d435i.sh --stop"
echo "────────────────────────────────────────────────────────"
