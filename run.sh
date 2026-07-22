#!/bin/bash
# =============================================================================
#  VINS-Fusion 仿真一键启动 (ROS2 Humble + PX4 SITL + Gazebo Harmonic)
#
#  用法:
#    ./run.sh                启动完整仿真
#    ./run.sh --headless     无 GUI（省资源，适合跑批量测试）
#    ./run.sh takeoff        让无人机起飞（VINS 需要运动才能初始化）
#
#  本地真实相机（不用仿真）:
#    ./run.sh local seeker   seeker1 四目鱼眼环视相机（双目+IMU）
#    ./run.sh local d435i    RealSense D435i（双目+IMU）
#    ./run.sh status         查看各环节状态
#    ./run.sh logs [名称]    查看日志: gz|px4|bridge|vins|rviz
#    ./run.sh stop           停止全部
#    ./run.sh doctor         环境自检（不启动任何东西）
#
#  ---------------------------------------------------------------------------
#  这个脚本把调试中踩过的坑都固化了，改动前先读注释：
#
#  1) NVIDIA 渲染必须指定 EGL vendor
#     Gazebo 的 Ogre2 走 EGL 而非 GLX。混合显卡上不指定 NVIDIA 的 EGL vendor
#     就会黑屏/窗口不出现，日志报 "failed to create dri2 screen"。
#
#  2) 必须先启 gz server，再启 PX4
#     PX4 自启 gz 时会 source 它编译期生成的 gz_env.sh，覆盖
#     GZ_SIM_SERVER_CONFIG_PATH → Gazebo 不加载 Imu 系统插件 →
#     IMU 话题存在但【没有发布者】→ VINS 永远等不到数据。
#
#  3) 自定义 world 必须有 <spherical_coordinates>
#     缺了它 GPS 报 lat/lon=0 → EKF2 偏航无法对齐(cs_yaw_align=False)
#     → 水平位置无效 → 无法解锁起飞。
#
#  4) 千万别用 SYS_HAS_MAG=0 / EKF2_MAG_TYPE=5 "关掉磁力计"
#     EKF2 会失去偏航基准，症状和 (3) 一模一样但更难查。
#     只需 COM_ARM_MAG_STR=0 关掉磁强度检查即可。
#
#  5) 跨容器 DDS 必须用 UDP-only profile
#     宿主机的 bridge 与容器内的 VINS，默认走共享内存传输，跨容器不通：
#     表现为「容器能 ros2 topic list 看到话题，但收不到任何数据」。
#
#  6) VINS 发布的话题在根命名空间：/odometry /path /point_cloud /image_track
#     不是 /vins_estimator/odometry（ROS2 里无命名空间节点的相对话题解析到根）。
#     且发布端 QoS 是 BEST_EFFORT，订阅端要匹配。
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------- 可配置项
WORLD=${WORLD:-vins_warehouse}
MODEL=${MODEL:-x500_depth_vinsfusion}
AIRFRAME=${AIRFRAME:-4002}
IMAGE=${IMAGE:-vins-ros2:humble-amd64}
CONTAINER=${CONTAINER:-vins-ros2-local}
# PX4 位置：优先用环境变量，其次探测几个常见路径
#   用法: PX4_DIR=/your/path/PX4-Autopilot ./run.sh
if [ -z "${PX4_DIR:-}" ]; then
    for _d in "$HOME/workspace/GroundControl/PX4-Autopilot" \
              "$HOME/PX4-Autopilot" "$HOME/src/PX4-Autopilot" "/opt/PX4-Autopilot"; do
        [ -d "$_d" ] && { PX4_DIR="$_d"; break; }
    done
    PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
fi
ROOTFS=${ROOTFS:-$HOME/.vins_gz_rootfs}
LOG=${LOG:-/tmp/vins_sim}
VINS_CFG=/ros2_ws/vins_config/gz_sim/gz_mono_imu.yaml
DDS_PROFILE=/ros2_ws/vins_config/fastdds_profile.xml

# 负载安全阈值（超过就警告；核数的 75%）
NPROC=$(nproc)
LOAD_WARN=$(awk -v n="$NPROC" 'BEGIN{printf "%.0f", n*0.75}')

C_G='\033[0;32m'; C_Y='\033[1;33m'; C_R='\033[0;31m'; C_B='\033[0;36m'; C_N='\033[0m'
ok()   { echo -e "${C_G}✓${C_N} $*"; }
info() { echo -e "${C_B}▸${C_N} $*"; }
warn() { echo -e "${C_Y}!${C_N} $*"; }
err()  { echo -e "${C_R}✗${C_N} $*" >&2; }
die()  { err "$*"; exit 1; }

load_now() { cut -d' ' -f1 /proc/loadavg; }

check_load() {
    local l; l=$(load_now)
    if awk -v l="$l" -v w="$LOAD_WARN" 'BEGIN{exit !(l+0 > w)}'; then
        warn "系统负载偏高: $l / ${NPROC}核（阈值 $LOAD_WARN）"
        return 1
    fi
    return 0
}

# ROS2 的 setup.bash 引用未定义变量，与 set -u 冲突
source_ros() { set +u; source /opt/ros/humble/setup.bash; set -u; }

inc() { docker exec "$CONTAINER" /ros_entrypoint.sh bash -c "$1"; }

# ============================================================ 环境自检
doctor() {
    local fail=0
    echo "═══════════ 环境自检 ═══════════"

    command -v gz >/dev/null && ok "Gazebo: $(gz sim --version 2>/dev/null | head -1)" \
        || { err "未安装 Gazebo (gz)"; fail=1; }

    [ -d /opt/ros/humble ] && ok "ROS2 Humble: /opt/ros/humble" \
        || { err "未找到 ROS2 Humble"; fail=1; }

    source_ros 2>/dev/null
    ros2 pkg executables ros_gz_bridge >/dev/null 2>&1 && ok "ros_gz_bridge 可用" \
        || { err "缺少 ros_gz_bridge: sudo apt install ros-humble-ros-gz"; fail=1; }

    [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] && ok "PX4 SITL 已编译" \
        || { err "PX4 未编译: cd $PX4_DIR && make px4_sitl_default"; fail=1; }

    [ -f "$PX4_DIR/src/modules/simulation/gz_bridge/server.config" ] \
        && ok "PX4 gz server.config 存在" \
        || { err "缺少 server.config（Imu 插件靠它加载）"; fail=1; }

    command -v docker >/dev/null && docker info >/dev/null 2>&1 && ok "Docker 可用" \
        || { err "Docker 不可用或无权限"; fail=1; }

    docker image inspect "$IMAGE" >/dev/null 2>&1 && ok "镜像 $IMAGE 存在" \
        || { err "镜像不存在，先构建: docker build -t $IMAGE build_ctx"; fail=1; }

    [ -f "$HERE/worlds/$WORLD.sdf" ] && ok "World: $WORLD.sdf" \
        || { err "缺少 worlds/$WORLD.sdf"; fail=1; }
    grep -q "spherical_coordinates" "$HERE/worlds/$WORLD.sdf" 2>/dev/null \
        && ok "World 含 GPS 原点" \
        || { err "World 缺 <spherical_coordinates> → EKF2 偏航无法对齐，无法起飞"; fail=1; }

    [ -d "$HERE/models/$MODEL" ] && ok "机型: $MODEL" \
        || { err "缺少 models/$MODEL"; fail=1; }

    [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && ok "NVIDIA EGL vendor 存在" \
        || warn "无 NVIDIA EGL vendor，Gazebo 将用软件渲染（慢）"

    [ -f "$HERE/config/fastdds_profile.xml" ] && ok "DDS UDP-only profile 存在" \
        || { err "缺少 config/fastdds_profile.xml（跨容器通信必需）"; fail=1; }

    echo "── 资源 ──"
    echo "  CPU: ${NPROC} 核，当前负载 $(load_now)"
    echo "  内存: $(free -h | awk '/^Mem:/{print $3" / "$2" (可用 "$7")"}')"
    command -v nvidia-smi >/dev/null && \
        echo "  GPU: $(nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader 2>/dev/null | head -1)"

    echo "════════════════════════════════"
    [ $fail -eq 0 ] && ok "环境检查通过" || err "环境检查未通过，请先解决上面的问题"
    return $fail
}

# ============================================================ 停止
stop_all() {
    info "停止仿真..."
    # 用 [x] 写法：否则 pkill -f 会匹配到承载它自己的命令行，把自己先杀掉
    ps -eo pid,args --no-headers 2>/dev/null \
      | grep -E "[g]z sim|[p]x4 |[p]arameter_bridge|[s]eeker_node" \
      | awk '{print $1}' | while read -r p; do kill -9 "$p" 2>/dev/null; done

    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER" && \
      docker exec "$CONTAINER" bash -c '
        ps -eo pid,args --no-headers \
          | grep -E "[v]ins_node|[r]viz2|[r]ealsense2_camera|[d]435i_publisher" \
          | awk "{print \$1}" | while read p; do kill -9 $p 2>/dev/null; done
        exit 0' >/dev/null 2>&1

    sleep 3
    local left
    left=$(ps -eo args --no-headers 2>/dev/null | grep -cE "[g]z sim|[p]x4 " || true)
    if [ "${left:-0}" -gt 0 ]; then
        warn "仍有 $left 个残留进程，强制清理"
        ps -eo pid,args --no-headers | grep -E "[g]z sim|[p]x4 " \
          | awk '{print $1}' | while read -r p; do kill -9 "$p" 2>/dev/null; done
        sleep 2
    fi
    ok "已停止（负载 $(load_now)）"
}

# ============================================================ 状态
show_status() {
    echo "═══════════ 仿真状态 ═══════════"
    echo "── 进程 ──"
    for n in "gz sim server" "gz sim gui" "px4 " "parameter_bridge"; do
        local c; c=$(ps -eo args --no-headers 2>/dev/null | grep -cF "$n" || true)
        [ "${c:-0}" -gt 0 ] && echo -e "  ${C_G}✓${C_N} $n" || echo -e "  ${C_Y}-${C_N} $n"
    done
    for n in vins_node rviz2; do
        if docker exec "$CONTAINER" pgrep -f "$n" >/dev/null 2>&1; then
            echo -e "  ${C_G}✓${C_N} $n (容器内)"
        else
            echo -e "  ${C_Y}-${C_N} $n (容器内)"
        fi
    done

    echo "── Gazebo ──"
    echo "  话题数: $(timeout 8 gz topic -l 2>/dev/null | wc -l)"
    local imu_t="/world/$WORLD/model/${MODEL}_0/link/base_link/sensor/imu_sensor/imu"
    local pub; pub=$(timeout 6 gz topic -i -t "$imu_t" 2>/dev/null | grep -c 'tcp://' || true)
    [ "${pub:-0}" -gt 0 ] && echo -e "  ${C_G}✓${C_N} IMU 有发布者" \
                          || echo -e "  ${C_R}✗${C_N} IMU 无发布者（Imu 插件未加载）"

    echo "── ROS2 话题 ──"
    source_ros 2>/dev/null
    for t in /cam0/image_raw /imu0 /odometry; do
        local r; r=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1)
        echo "  $t: ${r:-无数据}"
    done

    echo "── PX4 ──"
    if [ -d "$ROOTFS" ]; then
        (cd "$ROOTFS" && timeout 10 "$PX4_DIR/build/px4_sitl_default/bin/px4-commander" check 2>&1 \
          | grep -E "Preflight check" | head -1 | sed 's/^.*\] /  /') || echo "  (无法连接)"
    fi

    echo "── 资源 ──"
    echo "  负载: $(load_now) / ${NPROC}核    内存: $(free -h | awk '/^Mem:/{print $3"/"$2}')"
    echo "════════════════════════════════"
}

# ============================================================ 本地真实相机
ensure_container() {
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" && return 0
    info "创建容器 $CONTAINER ..."
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    docker run -d --name "$CONTAINER" --init --privileged --net=host --ipc=host \
        -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
        -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev:/dev -v /run/udev:/run/udev:ro \
        -v "$HOME/vins_output:/root/output" \
        "$IMAGE" sleep infinity >/dev/null
    sleep 3
    ok "容器已创建"
}

stop_local() {
    # 宿主机侧的相机驱动与去畸变节点。
    # 用 [x] 括号写法匹配自身进程名之外的进程：否则这条 grep 会匹配到承载它的
    # 那个 shell，把自己 kill 掉，表现成「相机突然坏了」，很难查。
    ps -eo pid,args --no-headers 2>/dev/null \
      | grep -E "[s]eeker_node|[s]eeker_split_node|[s]eeker1|[p]arameter_bridge" \
      | awk '{print $1}' | while read -r p; do kill -9 "$p" 2>/dev/null; done
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER" && \
      docker exec "$CONTAINER" bash -c '
        ps -eo pid,args --no-headers \
          | grep -E "[v]ins_node|[r]viz2|[r]ealsense2_camera|[d]435i_publisher" \
          | awk "{print \$1}" | while read p; do kill -9 $p 2>/dev/null; done
        exit 0' >/dev/null 2>&1
    sleep 2
}

# seeker1 四目鱼眼环视相机
#   驱动 + 去畸变节点跑在宿主机(都是 ROS2 包，colcon build 即可)，VINS 跑在容器里。
#
#   两个反直觉但实测确认的点，改之前先读：
#   1) 宿主机侧【绝不能】设 FASTRTPS_DEFAULT_PROFILES_FILE。
#      那个 profile 关掉了共享内存，同机进程通信会腰斩：20.1Hz -> 9.2Hz。
#      它只该在容器内设(跨容器共享内存不通，必须走 UDP)。
#   2) 中间必须有 seeker_split_node。驱动原始的 /fisheye/*/image_raw 是
#      bgr8 1088x1280 单帧 4.18MB，DDS 传不动(2~8Hz)；而且是鱼眼，
#      VINS 的 MEI 反投影在 xi=3.22 时会开负数根输出 NaN。
#      该节点从 /all/compressed 解码、切片、去畸变成针孔，发 /cam0 /cam1。
do_local_seeker() {
    local use_rviz=true
    for a in "$@"; do [ "$a" = "--no-rviz" ] && use_rviz=false; done

    mkdir -p "$LOG"
    lsusb 2>/dev/null | grep -q "2207:0000" || die "未检测到 seeker1 相机(USB 2207:0000)"
    ok "seeker1 相机已连接"

    # 工作区用软链指向项目源码，避免源码两份不同步
    local ws="$HERE/.seeker_ws"
    mkdir -p "$ws/src"
    [ -L "$ws/src/seeker1" ] || { rm -rf "$ws/src/seeker1"; \
        ln -s ../../ros2_ws_src/seeker1 "$ws/src/seeker1"; }

    # 源码比产物新就重编，否则改了代码不生效很难查
    local bin="$ws/install/seeker/lib/seeker/seeker_split_node"
    if [ ! -x "$bin" ] || \
       [ -n "$(find "$HERE/ros2_ws_src/seeker1" -newer "$bin" -name '*.cpp' -o \
               -newer "$bin" -name '*.h' -o -newer "$bin" -name 'CMakeLists.txt' 2>/dev/null)" ]; then
        info "编译 seeker 驱动与去畸变节点..."
        ( cd "$ws" && set +u && source /opt/ros/humble/setup.bash && set -u && \
          colcon build --packages-select seeker --cmake-args -DCMAKE_BUILD_TYPE=Release ) \
          > "$LOG/seeker_build.log" 2>&1 \
          || { err "编译失败，见 $LOG/seeker_build.log"; tail -10 "$LOG/seeker_build.log"; exit 1; }
        ok "编译完成"
    fi

    # 标定：直接从相机读出厂值(Kalibr 格式)，不是估计
    if [ ! -f "$HERE/config/seeker/seeker_remap.yaml" ]; then
        info "生成标定与去畸变配置..."
        python3 "$HERE/scripts/gen_seeker_config.py" \
          || { err "标定生成失败"; exit 1; }
    fi

    ensure_container
    stop_local >/dev/null 2>&1
    xhost +local:docker >/dev/null 2>&1 || true

    # 见函数头注释第 1 点：宿主机必须【不带】profile
    unset FASTRTPS_DEFAULT_PROFILES_FILE

    info "启动 seeker 驱动..."
    ( cd "$ws" && set +u && source /opt/ros/humble/setup.bash && source install/setup.bash && set -u && \
      nohup ros2 launch seeker 1seeker.launch.py > "$LOG/seeker.log" 2>&1 & )

    source_ros
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    info "等待相机出图与 IMU..."
    local n=0 img_ok=false imu_ok=false
    while [ $n -lt 60 ]; do
        sleep 3; n=$((n+3))
        $img_ok || timeout 6 ros2 topic echo /all/compressed --once --field format \
            >/dev/null 2>&1 && img_ok=true
        $imu_ok || timeout 6 ros2 topic echo /imu_data_raw --once --field header \
            >/dev/null 2>&1 && imu_ok=true
        $img_ok && $imu_ok && break
    done
    $img_ok && ok "相机出图正常" || { err "无图像数据"; tail -10 "$LOG/seeker.log"; exit 1; }
    $imu_ok && ok "IMU 正常"  || warn "无 IMU 数据 —— VINS 需要 IMU，请检查驱动 pub_imu 参数"

    info "启动去畸变节点（鱼眼 → 矫正双目针孔）..."
    ( cd "$ws" && set +u && source /opt/ros/humble/setup.bash && source install/setup.bash && set -u && \
      nohup ros2 run seeker seeker_split_node --ros-args \
        -p remap_file:="$HERE/config/seeker/seeker_remap.yaml" \
        > "$LOG/seeker_split.log" 2>&1 & )
    sleep 5
    timeout 8 ros2 topic echo /cam0/image_raw --once --field encoding >/dev/null 2>&1 \
        && ok "/cam0 /cam1 已就绪" \
        || { err "去畸变节点无输出"; tail -15 "$LOG/seeker_split.log"; exit 1; }

    docker cp "$HERE/config/seeker/." "$CONTAINER:/ros2_ws/vins_config/seeker/" >/dev/null 2>&1 || true
    docker cp "$HERE/config/fastdds_profile.xml" "$CONTAINER:/ros2_ws/vins_config/" >/dev/null 2>&1 || true
    docker cp "$HERE/config/vins_rviz2.rviz" "$CONTAINER:/ros2_ws/vins_config/" >/dev/null 2>&1 || true

    # VINS 存在一个尚未定位的【间歇性坏初始化】: 同一份配置、同一个二进制，
    # 有时会进入加速度计零偏不参与优化的状态(调试输出里 Ba 恒为 0.00000、
    # solver 耗时明显偏低)，随后位置单方向发散且无法自行恢复。
    # 根因未找到，这里用「自检 + 重试」绕过 —— 坏状态在启动十几秒内就能判定。
    local vins_ok=false
    for attempt in 1 2 3; do
        info "启动 VINS-Fusion（矫正双目 + IMU）第 $attempt 次..."
        docker exec "$CONTAINER" bash -c \
            "ps -eo pid,args --no-headers | grep '[v]ins_node' | awk '{print \$1}' | xargs -r kill -9" \
            >/dev/null 2>&1
        sleep 2
        docker exec -d -e FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE "$CONTAINER" \
            /ros_entrypoint.sh bash -c \
            "ros2 run vins vins_node /ros2_ws/vins_config/seeker/seeker_stereo_imu.yaml > /root/output/vins.log 2>&1"

        local n=0
        while [ $n -lt 60 ]; do
            sleep 3; n=$((n+3))
            # 注意 || true 不能写成 || echo 0: grep -c 计数为 0 时退出码是 1，
            # 那样会拼出 "0\n0" 让 [ 报 integer expression expected
            local cnt
            cnt=$(grep -c 'DBG V' "$HOME/vins_output/vins.log" 2>/dev/null || true)
            [ "${cnt:-0}" -gt 100 ] && break
        done

        # 健康判据: 加速度计零偏必须在被优化(取值有变化)。坏状态下它恒为 0.00000。
        local uniq
        uniq=$(grep 'DBG V' "$HOME/vins_output/vins.log" 2>/dev/null \
               | sed 's/.*| Ba/Ba/;s/| Bg.*//' | sort -u | wc -l)
        if [ "${uniq:-0}" -gt 3 ]; then
            ok "VINS 运行中（零偏正常收敛，Ba 取值数 $uniq）"
            vins_ok=true; break
        fi
        warn "第 $attempt 次落入坏初始化（Ba 恒定不变），重试"
    done
    $vins_ok || warn "3 次均落入坏初始化，请查看 ./run.sh logs vins"

    if $use_rviz; then
        info "启动 rviz2..."
        docker exec -d -e DISPLAY="${DISPLAY:-:0}" -e FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE \
            -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia "$CONTAINER" \
            /ros_entrypoint.sh bash -c "rviz2 -d /ros2_ws/vins_config/vins_rviz2.rviz > /root/output/rviz.log 2>&1"
        sleep 8
    fi

    echo
    echo "────────────────────────────────────────────────────────"
    ok "seeker 环视相机 VINS 已启动（负载 $(load_now)/${NPROC}核）"
    echo
    echo -e "  ${C_Y}下一步${C_N}：拿起相机【缓慢平移】几秒，轨迹才会出现"
    echo "     别原地纯旋转 —— 纯旋转没有视差，三角化不出深度"
    echo
    echo "  相机: front 矫正双目(物理鱼眼 left+right，基线 4.6cm) + IMU"
    echo "        鱼眼已去畸变为 640x480 针孔，视场 90°x74°"
    echo "  状态: ./run.sh status   日志: ./run.sh logs vins   停止: ./run.sh stop"
    echo "────────────────────────────────────────────────────────"
}

do_local() {
    local target="${1:-}"; shift 2>/dev/null || true
    case "$target" in
        seeker|seeker1)        do_local_seeker "$@" ;;
        d435i|realsense_d435i) exec "$HERE/run_local_d435i.sh" "$@" ;;
        *) die "用法: ./run.sh local <seeker|d435i> [--no-rviz]" ;;
    esac
}

# ============================================================ 起飞
do_takeoff() {
    local B="$PX4_DIR/build/px4_sitl_default/bin"
    [ -d "$ROOTFS" ] || die "仿真未运行，先执行 ./run.sh"

    info "检查预检状态..."
    if ! (cd "$ROOTFS" && timeout 12 "$B/px4-commander" check 2>&1 | grep -q "Preflight check: OK"); then
        warn "预检未通过，尝试重新配置参数..."
        configure_px4_params
    fi

    info "起飞..."
    (cd "$ROOTFS" && timeout 15 "$B/px4-commander" takeoff >/dev/null 2>&1)
    sleep 20
    local z; z=$(cd "$ROOTFS" && timeout 10 "$B/px4-listener" vehicle_local_position 2>/dev/null \
                 | grep -oE "^    z: -?[0-9.]+" | head -1 | grep -oE "\-?[0-9.]+")
    if [ -n "$z" ] && awk -v z="$z" 'BEGIN{exit !(z < -0.5)}'; then
        ok "已起飞，高度 $(awk -v z="$z" 'BEGIN{printf "%.2f", -z}') m"
        echo
        echo "  提示: 单目 VIO 的尺度靠 IMU 激励恢复，纯垂直起飞后悬停时"
        echo "        尺度收敛不充分。要提高精度需让无人机做水平机动。"
    else
        warn "似乎未起飞（z=$z）。查看原因: ./run.sh logs px4"
    fi
}

configure_px4_params() {
    local B="$PX4_DIR/build/px4_sitl_default/bin"
    # 仿真里没有地面站、没有遥控器，这些检查必须放行。
    # 注意：不要动磁力计的 SYS_HAS_MAG / EKF2_MAG_TYPE（见文件头注释 4）
    for prm in "NAV_DLL_ACT 0" "COM_RCL_EXCEPT 4" "NAV_RCL_ACT 0" "COM_RC_IN_MODE 4" \
               "COM_ARM_MAG_STR 0" "CBRK_SUPPLY_CHK 894281"; do
        (cd "$ROOTFS" && timeout 8 "$B/px4-param" set $prm >/dev/null 2>&1) || true
    done
    sleep 5
}

# ============================================================ 日志
show_logs() {
    case "${1:-vins}" in
        gz)     tail -40 "$LOG/gz.log" 2>/dev/null ;;
        px4)    tail -40 "$LOG/px4.log" 2>/dev/null ;;
        bridge) tail -40 "$LOG/bridge.log" 2>/dev/null ;;
        vins)   docker exec "$CONTAINER" tail -40 /root/output/vins.log 2>/dev/null ;;
        rviz)   docker exec "$CONTAINER" tail -40 /root/output/rviz.log 2>/dev/null ;;
        *)      err "未知日志: $1 (可选 gz|px4|bridge|vins|rviz)"; exit 1 ;;
    esac
}

# ============================================================ 启动
do_start() {
    local gui=true
    for a in "$@"; do
        case "$a" in
            --headless|-H) gui=false ;;
            *) warn "未知参数: $a" ;;
        esac
    done

    doctor >/dev/null 2>&1 || { doctor; die "环境检查未通过"; }
    check_load || warn "负载偏高仍继续启动，如卡顿请 ./run.sh stop"

    stop_all >/dev/null 2>&1
    mkdir -p "$LOG"

    # ---------- 渲染与资源路径 ----------
    export DISPLAY=${DISPLAY:-:0}
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    unset LIBGL_ALWAYS_SOFTWARE
    export GZ_SIM_SERVER_CONFIG_PATH=$PX4_DIR/src/modules/simulation/gz_bridge/server.config
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins
    export GZ_SIM_RESOURCE_PATH=$HERE/models:$HERE/models/aws_warehouse:$HERE/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
    xhost +local: >/dev/null 2>&1 || true

    # ---------- 1. Gazebo ----------
    info "启动 Gazebo ($WORLD, $($gui && echo 'NVIDIA GUI' || echo '无头'))..."
    local gzopt="-r -v 2"; $gui || gzopt="-s -r -v 2"
    (cd /tmp && nohup gz sim $gzopt "$HERE/worlds/$WORLD.sdf" > "$LOG/gz.log" 2>&1 &)
    local n=0
    until [ "$(timeout 6 gz topic -l 2>/dev/null | wc -l)" -gt 5 ]; do
        sleep 3; n=$((n+3))
        [ $n -gt 120 ] && { err "Gazebo 启动超时"; show_logs gz | tail -5; exit 1; }
    done
    ok "Gazebo 就绪"

    # ---------- 2. PX4 ----------
    info "启动 PX4 (spawn $MODEL)..."
    mkdir -p "$ROOTFS"      # 不删：参数存在 parameters.bson，删了每次都要重设
    (cd "$ROOTFS" && PX4_SYS_AUTOSTART=$AIRFRAME PX4_SIM_MODEL=$MODEL PX4_GZ_WORLD=$WORLD \
      nohup "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -d "$PX4_DIR/build/px4_sitl_default/etc" -w "$ROOTFS" > "$LOG/px4.log" 2>&1 &)
    n=0
    until timeout 6 gz topic -l 2>/dev/null | grep -q "IMX214/image"; do
        sleep 3; n=$((n+3))
        [ $n -gt 120 ] && { err "PX4 spawn 超时"; show_logs px4 | tail -5; exit 1; }
    done
    ok "无人机已 spawn"

    # 验证 IMU 真的有发布者（话题名存在 ≠ 有数据，见文件头注释 2）
    local imu_t="/world/$WORLD/model/${MODEL}_0/link/base_link/sensor/imu_sensor/imu"
    n=0
    until [ "$(timeout 6 gz topic -i -t "$imu_t" 2>/dev/null | grep -c 'tcp://')" -gt 0 ]; do
        sleep 3; n=$((n+3))
        [ $n -gt 45 ] && die "IMU 无发布者 → Imu 系统插件未加载，检查 GZ_SIM_SERVER_CONFIG_PATH"
    done
    ok "IMU 正在发布"

    info "配置 PX4 仿真参数..."
    configure_px4_params
    if (cd "$ROOTFS" && timeout 12 "$PX4_DIR/build/px4_sitl_default/bin/px4-commander" check 2>&1 \
        | grep -q "Preflight check: OK"); then
        ok "PX4 预检通过，可起飞"
    else
        warn "PX4 预检未通过（参数需重启生效，再跑一次本脚本即可）"
    fi

    # ---------- 3. ROS2 桥 ----------
    info "桥接 Gazebo → ROS2..."
    source_ros
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/config/fastdds_profile.xml
    local IMG=/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/image
    local INFO=/world/$WORLD/model/${MODEL}_0/link/camera_link/sensor/IMX214/camera_info
    nohup ros2 run ros_gz_bridge parameter_bridge \
      "$IMG@sensor_msgs/msg/Image[gz.msgs.Image" \
      "$INFO@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
      "$imu_t@sensor_msgs/msg/Imu[gz.msgs.IMU" \
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
      --ros-args -r "$IMG:=/cam0/image_raw" -r "$INFO:=/cam0/camera_info" \
                 -r "$imu_t:=/imu0" > "$LOG/bridge.log" 2>&1 &
    sleep 12
    local okcnt=0
    for t in /cam0/image_raw /imu0; do
        local r; r=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1)
        if [ -n "$r" ]; then ok "$t : $r"; okcnt=$((okcnt+1)); else warn "$t : 无数据"; fi
    done
    [ $okcnt -lt 2 ] && warn "桥接不完整，VINS 可能无法初始化"

    # ---------- 4. VINS + rviz2 ----------
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || {
        info "创建容器 $CONTAINER ..."
        docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
        docker run -d --name "$CONTAINER" --init --privileged --net=host --ipc=host \
            -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
            -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev:/dev \
            -v "$HOME/vins_output:/root/output" \
            "$IMAGE" sleep infinity >/dev/null
        sleep 3
    }
    # 每次同步配置，避免容器里是旧版本
    docker cp "$HERE/config/gz_sim/." "$CONTAINER:/ros2_ws/vins_config/gz_sim/" >/dev/null 2>&1 || true
    docker cp "$HERE/config/fastdds_profile.xml" "$CONTAINER:/ros2_ws/vins_config/" >/dev/null 2>&1 || true
    docker cp "$HERE/config/vins_rviz2.rviz" "$CONTAINER:/ros2_ws/vins_config/" >/dev/null 2>&1 || true

    info "启动 VINS-Fusion..."
    docker exec -d -e FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE "$CONTAINER" \
        /ros_entrypoint.sh bash -c "ros2 run vins vins_node $VINS_CFG > /root/output/vins.log 2>&1"
    sleep 10
    if docker exec "$CONTAINER" pgrep -f vins_node >/dev/null 2>&1; then
        ok "VINS 运行中"
    else
        warn "VINS 未启动，查看: ./run.sh logs vins"
    fi

    if $gui; then
        info "启动 rviz2..."
        docker exec -d -e DISPLAY="$DISPLAY" -e FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE \
            -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia "$CONTAINER" \
            /ros_entrypoint.sh bash -c "rviz2 -d /ros2_ws/vins_config/vins_rviz2.rviz > /root/output/rviz.log 2>&1"
        sleep 8
    fi

    echo
    echo "────────────────────────────────────────────────────────"
    ok "仿真已启动（负载 $(load_now) / ${NPROC}核）"
    echo
    echo -e "  ${C_Y}下一步${C_N}：VINS 需要【运动】才能初始化（单目 VIO 靠视差三角化）"
    echo "     ./run.sh takeoff      让无人机起飞"
    echo
    echo "  查看状态: ./run.sh status"
    echo "  查看日志: ./run.sh logs vins"
    echo "  停止:     ./run.sh stop"
    echo "────────────────────────────────────────────────────────"
}

# ============================================================ 入口
case "${1:-start}" in
    start)          shift 2>/dev/null || true; do_start "$@" ;;
    --headless|-H)  do_start --headless ;;
    local)          shift 2>/dev/null || true; do_local "$@" ;;
    takeoff|fly)    do_takeoff ;;
    stop)           stop_all ;;
    restart)        stop_all; sleep 2; do_start ;;
    status)         show_status ;;
    logs)           shift 2>/dev/null || true; show_logs "$@" ;;
    doctor|check)   doctor ;;
    shell)          docker exec -it "$CONTAINER" /ros_entrypoint.sh bash ;;
    -h|--help|help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}" ;;
    *)              err "未知命令: $1"; echo "运行 ./run.sh --help 查看用法"; exit 1 ;;
esac
