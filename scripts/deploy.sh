#!/bin/bash
# =============================================================================
#  在【目标板子】上一键部署（树莓派 5 / RDK X5 / Jetson AGX Orin）
#
#  用法:
#    bash deploy.sh vins-ros2-humble-arm64.tar.gz
#    bash deploy.sh vins-ros2-humble-jetson.tar.gz     # Jetson CUDA 变体
#
#  自动完成: 检查 docker → 导入镜像 → 识别板型 → 创建容器 → 生成 vins 快捷命令
# =============================================================================
set -euo pipefail

TARBALL="${1:-}"
CONTAINER=vins-ros2

C_G='\033[0;32m'; C_Y='\033[1;33m'; C_R='\033[0;31m'; C_B='\033[0;36m'; C_N='\033[0m'
ok()   { echo -e "${C_G}✓${C_N} $*"; }
info() { echo -e "${C_B}▸${C_N} $*"; }
warn() { echo -e "${C_Y}!${C_N} $*"; }
err()  { echo -e "${C_R}✗${C_N} $*" >&2; }

[ -n "$TARBALL" ] || { err "用法: bash deploy.sh <镜像tar包>"; exit 1; }
[ -f "$TARBALL" ] || { err "找不到文件: $TARBALL"; exit 1; }

# ---------------------------------------------------------------- 环境检查
command -v docker >/dev/null || {
    err "未安装 docker。安装命令:"
    echo "  curl -fsSL https://get.docker.com | sh"
    echo "  sudo usermod -aG docker \$USER   # 然后重新登录"
    exit 1; }
docker info >/dev/null 2>&1 || { err "当前用户无 docker 权限。执行: sudo usermod -aG docker \$USER 后重新登录"; exit 1; }

ARCH=$(uname -m)
[ "$ARCH" = "aarch64" ] || warn "当前架构是 $ARCH，本镜像为 aarch64，可能无法运行"

# ---------------------------------------------------------------- 识别板型
BOARD="未知 aarch64 设备"
GPU_ARGS=""
if [ -f /etc/nv_tegra_release ]; then
    BOARD="NVIDIA Jetson ($(head -1 /etc/nv_tegra_release | grep -oE 'R[0-9]+' || echo '?'))"
    if docker info 2>/dev/null | grep -qi "nvidia"; then
        GPU_ARGS="--runtime nvidia"
        ok "检测到 nvidia 容器运行时，将启用 GPU"
    else
        warn "未检测到 nvidia 容器运行时，将以 CPU 模式运行"
        warn "安装: sudo apt install nvidia-container-toolkit && sudo systemctl restart docker"
    fi
elif grep -qi "raspberry" /proc/device-tree/model 2>/dev/null; then
    BOARD="树莓派 ($(tr -d '\0' < /proc/device-tree/model))"
elif grep -qiE "horizon|rdk|x5" /proc/device-tree/model 2>/dev/null; then
    BOARD="地平线 RDK ($(tr -d '\0' < /proc/device-tree/model))"
fi
info "目标板: $BOARD"

# ---------------------------------------------------------------- 导入镜像
info "导入镜像（几分钟，取决于存储速度）..."
if [[ "$TARBALL" == *.gz ]]; then
    gunzip -c "$TARBALL" | docker load
else
    docker load -i "$TARBALL"
fi
IMAGE=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep '^vins-ros2:' | head -1)
[ -n "$IMAGE" ] || { err "镜像导入失败"; exit 1; }
ok "镜像已导入: $IMAGE"

# ---------------------------------------------------------------- 创建容器
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
info "创建容器 $CONTAINER ..."

# --privileged + /dev 挂载: RealSense 走 USB，需要直接访问设备节点
# --net=host: ROS2 DDS 通信 & mavros UDP
# /dev/bus/usb: 相机热插拔后容器内仍能识别
docker run -d \
    --name "$CONTAINER" \
    --restart unless-stopped \
    --privileged \
    --net=host \
    --ipc=host \
    $GPU_ARGS \
    -v /dev:/dev \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$HOME/vins_output:/root/output" \
    -e DISPLAY="${DISPLAY:-}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "$IMAGE" \
    sleep infinity >/dev/null

mkdir -p "$HOME/vins_output"
ok "容器已启动"

# ---------------------------------------------------------------- 快捷命令
BIN="$HOME/.local/bin"; mkdir -p "$BIN"
cat > "$BIN/vins" <<'SHIM'
#!/bin/bash
# VINS-Fusion 快捷命令
C=vins-ros2
case "${1:-help}" in
    start)   shift
             docker exec -d $C /ros_entrypoint.sh bash -c \
               "ros2 launch /ros2_ws/vins_launch/vins_d435i.launch.py $* > /root/output/vins.log 2>&1"
             echo "已启动。查看日志: vins logs" ;;
    stop)    docker exec $C bash -c 'pkill -f "ros2 launch|vins_node|realsense2_camera|mavros_node" 2>/dev/null; exit 0'
             echo "已停止" ;;
    logs)    docker exec $C tail -f /root/output/vins.log ;;
    status)  docker exec $C /ros_entrypoint.sh bash -c 'ros2 node list; echo "--- 话题频率 ---"; timeout 5 ros2 topic hz /vins_estimator/odometry' ;;
    calib)   docker exec -it $C /ros_entrypoint.sh python3 /ros2_ws/scripts/gen_d435i_config.py --out /ros2_ws/vins_config/d435i ;;
    shell)   docker exec -it $C /ros_entrypoint.sh bash ;;
    restart) docker restart $C >/dev/null; echo "容器已重启" ;;
    *) cat <<'EOF'
VINS-Fusion 快捷命令:
  vins start [参数]   启动（参数如 mavros:=true rviz:=true loop_fusion:=true）
  vins stop           停止
  vins logs           查看实时日志
  vins status         查看节点与里程计频率
  vins calib          从相机读出厂标定生成配置（首次务必执行）
  vins shell          进入容器
  vins restart        重启容器
EOF
    ;;
esac
SHIM
chmod +x "$BIN/vins"

echo
echo "════════════════════════════════════════════════════════"
ok "部署完成"
echo
echo "  1. 插好 D435i，生成本机相机标定（首次必做）:"
echo "       vins calib"
echo "  2. 启动:"
echo "       vins start"
echo "     带飞控回灌:"
echo "       vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600"
echo "  3. 看状态 / 日志:"
echo "       vins status ; vins logs"
echo
case ":$PATH:" in
    *":$BIN:"*) ;;
    *) warn "请把 $BIN 加入 PATH（或重新登录）:"
       echo "       echo 'export PATH=\$PATH:$BIN' >> ~/.bashrc && source ~/.bashrc" ;;
esac
echo "════════════════════════════════════════════════════════"
