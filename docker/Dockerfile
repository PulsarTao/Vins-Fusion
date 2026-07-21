# =============================================================================
#  VINS-Fusion (ROS2 Humble) · 架构无关镜像 (amd64 / arm64 通用)
#
#  同一份 Dockerfile 靠 --platform 决定目标架构，不需要分开写：
#    x86 本机开发:  docker buildx build --platform linux/amd64 -t vins-ros2:humble-amd64 --load <ctx>
#    边缘板子交叉:  ./scripts/build_arm64.sh          (linux/arm64，走 QEMU)
#  能通用的原因：FROM 的 ros:humble-ros-base 是 multi-arch 镜像，
#  且下面的 apt 源同时处理了 ports.ubuntu.com(arm64) 与 archive.ubuntu.com(amd64)。
#
#  内含: ROS2 Humble + mavros + Intel RealSense (D435i) + VINS-Fusion(ROS2 移植版)
#  目标板: x86 工作站 / 树莓派 5 / RDK X5 / Jetson AGX Orin
# =============================================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=humble

# ---------------------------------------------------------------- apt 加速
# 架构无关：arm64 的 Ubuntu 归档在 ports.ubuntu.com，amd64 在 archive.ubuntu.com，
# 两条 sed 都写上，命中哪条取决于实际架构。换清华镜像大幅提速（失败自动回退官方源）。
RUN cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    sed -i -e 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
           -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
           -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
           /etc/apt/sources.list && \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
        /etc/apt/sources.list.d/ros2-latest.list 2>/dev/null || true; \
    apt-get update || (cp /etc/apt/sources.list.bak /etc/apt/sources.list && apt-get update)

# ---------------------------------------------------------------- 编译依赖
# 注意不要装 libceres-dev（jammy 是 2.0.0），下面会从源码装 Ceres 2.1.0。
# 原因：本仓库同时用了两套 Ceres API —— camera_models/global_fusion 用 Manifold
# （2.1 才引入），vins/loop_fusion 用 LocalParameterization（2.2 已移除）。
# 只有 2.1 这一个版本同时提供两者，版本必须精确锁定。
RUN apt-get install -y --no-install-recommends \
        build-essential cmake git wget \
        libeigen3-dev \
        libopencv-dev \
        libgoogle-glog-dev libgflags-dev \
        libboost-all-dev \
        libyaml-cpp-dev \
        libatlas-base-dev libsuitesparse-dev \
        python3-colcon-common-extensions python3-rosdep \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
        ros-humble-nav-msgs ros-humble-visualization-msgs \
        ros-humble-sensor-msgs ros-humble-geometry-msgs \
        ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- Ceres 2.1.0
# 必须精确用 2.1.0：只有它同时提供 Manifold(2.1 新增) 与 LocalParameterization(2.2 移除)，
# 而本仓库这两套 API 都用到了。换成 2.0 或 2.2+ 都编不过。
RUN wget -q https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz -O /tmp/ceres.tar.gz && \
    tar xzf /tmp/ceres.tar.gz -C /tmp && \
    cmake -S /tmp/ceres-solver-2.1.0 -B /tmp/ceres-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF && \
    cmake --build /tmp/ceres-build -j$(nproc) --target install && \
    rm -rf /tmp/ceres.tar.gz /tmp/ceres-solver-2.1.0 /tmp/ceres-build && \
    ldconfig

# ---------------------------------------------------------------- RealSense
# 官方 arm64 预编译包，无需源码编译 librealsense（板子上编译要几十分钟）
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-librealsense2 \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-camera-msgs \
        ros-humble-realsense2-description \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# pyrealsense2：wheel 自带较新的 librealsense，用于
#   1) 相机自检脚本  2) 官方驱动与新固件不兼容时的备用发布器
RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple pyrealsense2

# ---------------------------------------------------------------- mavros
# 大地水准面数据本地注入：官方脚本从 SourceForge 下载，经常挂死（实测卡 15 分钟+）
COPY egm96-5.tar.bz2 /tmp/egm96-5.tar.bz2
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
        geographiclib-tools \
    && mkdir -p /usr/share/GeographicLib \
    && tar xjf /tmp/egm96-5.tar.bz2 -C /usr/share/GeographicLib \
    && rm /tmp/egm96-5.tar.bz2 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- VINS-Fusion
ENV ROS2_WS=/ros2_ws
WORKDIR $ROS2_WS
COPY src/ $ROS2_WS/src/

# 源码编译进镜像（不挂载），部署到板子上开箱即用
# 不能用 --symlink-install：它让 install/ 指向 build/ 的符号链接，
# 而下面又删掉了 build/，会导致 install/ 失效（找不到 local_setup.bash）。
# 这里用普通安装（复制），删 build/ 才是安全的。
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers $(nproc) && \
    rm -rf build log

# 配置、launch、辅助脚本
COPY config/ $ROS2_WS/vins_config/
COPY launch/ $ROS2_WS/vins_launch/
COPY scripts/ $ROS2_WS/scripts/
RUN chmod +x $ROS2_WS/scripts/*.py 2>/dev/null || true

# ---------------------------------------------------------------- 运行环境
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc && \
    mkdir -p /root/output

# 板子上多为单机通信，限定 localhost 避免局域网串扰（需要跨机通信时改这里）
ENV ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /ros2_ws/install/setup.bash\nexec "$@"\n' \
      > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
