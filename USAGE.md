# VINS-Fusion 跨平台运行指南（ROS2 Humble）

本文档教你在不同平台把 VINS-Fusion 跑起来。所有内容基于 **ROS2 Humble**。

## 验证状态说明

不同路线的实测情况不同，先说清楚，避免你踩坑时误判：

| 路线 | 平台 | 状态 |
|---|---|---|
| A · 本机实时 | x86_64 + D435i | ✅ **已实测跑通**（IMU 200Hz / 双目 30Hz / 里程计输出正常） |
| C · 数据集回放 | x86_64（无需相机） | ✅ 镜像内含 EuRoC launch，可离线验证 |
| B · 边缘板子 | Jetson / RDK X5 / 树莓派5 | ⚠️ **镜像可交叉构建，但未在真实板子 + 真实相机上验证**，首次上板可能需针对板型微调 |

---

## 目录

- [0. 平台支持矩阵](#0-平台支持矩阵)
- [1. 名词与数据流](#1-名词与数据流)
- [路线 A：x86 开发机 + D435i（推荐先跑这个）](#路线-ax86-开发机--d435i推荐先跑这个)
- [路线 B：ARM64 边缘板子部署](#路线-barm64-边缘板子部署)
- [路线 C：EuRoC 数据集回放（无需相机）](#路线-ceuroc-数据集回放无需相机)
- [2. 相机标定（换相机/追求精度时）](#2-相机标定换相机追求精度时)
- [3. 接入飞控](#3-接入飞控)
- [4. 话题与命令速查](#4-话题与命令速查)
- [5. 按平台调参](#5-按平台调参)
- [6. 故障排查](#6-故障排查)
- [7. 已知限制与设计约束](#7-已知限制与设计约束)

---

## 0. 平台支持矩阵

| 平台 | 架构 | 用哪个镜像 | 加速 | 构建方式 |
|---|---|---|---|---|
| x86 开发机 / 工控机 | amd64 | `vins-ros2:humble-amd64` | CPU | 本机原生构建 |
| NVIDIA Jetson AGX Orin | arm64 | `vins-ros2:humble-jetson` | CUDA | x86 交叉构建 |
| 地平线 RDK X5 | arm64 | `vins-ros2:humble-arm64` | CPU¹ | x86 交叉构建 |
| 树莓派 5 | arm64 | `vins-ros2:humble-arm64` | CPU | x86 交叉构建 |

¹ RDK X5 的 BPU 未被利用（需专有工具链改写前端，本包未涉及）。

**共同点**：三块板子和 x86 都用同一套代码、同一套配置。区别只在基础镜像和调参。

---

## 1. 名词与数据流

```
 相机(双目+IMU)  ──►  VINS-Fusion  ──►  里程计/轨迹/点云  ──►  rviz2 可视化
                        (vins_node)                        └►  飞控(可选,经 mavros)
```

- **VIO**：视觉惯性里程计。靠双目相机看环境 + IMU 测运动，实时估计自身位姿，**不依赖 GPS**。
- **红外双目**：D435i 的 `infra1/infra2`，全局快门、硬件同步，比 RGB 适合 VIO。
- 本部署包的 VINS 发布的话题在**根命名空间**：`/odometry`、`/path`、`/point_cloud`、`/image_track`
  （不是 `/vins_estimator/odometry`——ROS2 里无命名空间节点的相对话题名解析到根路径）。

---

## 路线 A：x86 开发机 + D435i（推荐先跑这个）

最简单、已实测。适合开发、调试、验证算法。

### A.0 前置条件

```bash
# 1) Docker
docker --version || (curl -fsSL https://get.docker.com | sh)

# 2) NVIDIA 容器运行时（如果要 GPU 硬件渲染 rviz，可选）
#    没有独显也能跑，rviz 会用软件渲染

# 3) D435i 插在 USB3 口（蓝色），用 USB3 数据线
lsusb | grep 8086     # 应看到 Intel RealSense
```

### A.1 构建镜像（一次性，约 10~20 分钟）

```bash
cd ~/workspace/vins_ros2_deploy

# 准备构建上下文
rm -rf build_ctx && mkdir build_ctx
cp -r src_upstream build_ctx/src && rm -rf build_ctx/src/.git build_ctx/src/docker
cp -r config launch scripts build_ctx/
cp docker/egm96-5.tar.bz2 docker/Dockerfile.arm64 build_ctx/
mv build_ctx/Dockerfile.arm64 build_ctx/Dockerfile

# 国内网络：buildx 走宿主代理（脚本已自动处理，或手动构建如下）
docker build -t vins-ros2:humble-amd64 build_ctx
```

> 如果 `docker build` 卡在拉取 `ros:humble-ros-base`，是 Docker Hub 直连超时。
> 解决：用 `scripts/build_arm64.sh` 里的 buildx 逻辑（会自动透传宿主代理），
> 或给 Docker daemon 配好 registry-mirror。

### A.2 一键运行

```bash
./run_local_d435i.sh            # 自检相机 → 启动驱动 → VINS → rviz2
```

脚本会依次：清理残留进程 → USB 复位 → 相机自检 → 启动 realsense 驱动
→ 等图像数据 → 启动 vins_node + rviz2。

**rviz2 窗口出现后**，拿起相机对着**有纹理、光线好**的地方（书架、键盘、桌面杂物），
缓慢平移走动几秒。绿色轨迹线（VIO Path）和白色特征点云就会出现。

> ⚠️ 不要对着白墙、纯色桌面、暗光环境——VIO 需要视觉特征。
> ⚠️ 不要原地纯旋转——纯旋转没有视差，三角化不出深度，无法初始化。

### A.3 其他命令

```bash
./run_local_d435i.sh --status     # 看话题频率 + VINS 是否在输出
./run_local_d435i.sh --check      # 只做相机自检
./run_local_d435i.sh --stop       # 停止
./run_local_d435i.sh --fallback   # 官方驱动异常时，改用 pyrealsense2 发布器
```

### A.4 确认真的跑起来了

```bash
docker exec vins-ros2-local /ros_entrypoint.sh bash -c '
  ros2 topic hz /camera/camera/imu                        # ≈200 Hz
  ros2 topic hz /camera/camera/infra1/image_rect_raw      # ≈30 Hz
  ros2 topic hz /odometry                                 # 有输出 = VINS 已初始化
'
```

---

## 路线 B：ARM64 边缘板子部署

在 x86 开发机上**交叉构建**镜像，导出成 tar，拷到板子上一键部署。

### B.1 在 x86 开发机上构建

```bash
cd ~/workspace/vins_ros2_deploy

./scripts/build_arm64.sh            # 通用 arm64 镜像（Pi5 / RDK X5 / Jetson CPU 通用）
# 或
./scripts/build_arm64.sh --jetson   # Jetson CUDA 加速变体
```

- 用 QEMU 模拟 aarch64 编译，**首次约 40~90 分钟**（含 Ceres 2.1 源码编译）。
- 脚本自动注册 QEMU、透传宿主代理、构建、导出。
- 产物在 `dist/`：镜像 tar 包 + `deploy.sh`。

> Jetson 变体要选对 L4T 版本。查板子：`cat /etc/nv_tegra_release`，
> 改 `docker/Dockerfile.jetson` 的 `FROM` 行匹配你的 JetPack。

### B.2 拷贝到板子

```bash
scp dist/vins-ros2-humble-arm64.tar.gz dist/deploy.sh <用户>@<板子IP>:~/
```

### B.3 在板子上部署

```bash
ssh <用户>@<板子IP>
bash ~/deploy.sh ~/vins-ros2-humble-arm64.tar.gz
```

`deploy.sh` 自动：检查 docker → 导入镜像 → **识别板型**（Jetson 自动加 `--runtime nvidia`）
→ 建容器 → 装 `vins` 快捷命令。

### B.4 板子上使用

```bash
vins calib          # 首次必做：从相机读出厂标定生成配置
vins start          # 启动相机 + VINS
vins status         # 看节点与频率
vins logs           # 实时日志
vins stop           # 停止

# 带飞控回灌
vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600
```

### B.5 各板子注意事项

| 板子 | 注意 |
|---|---|
| **Jetson AGX Orin** | 确认 `docker info` 有 nvidia runtime；镜像 L4T 版本要匹配 JetPack |
| **RDK X5** | 纯 CPU 跑；`max_cnt` 调到 120；确认 USB3 供电充足 |
| **树莓派 5** | 算力最弱：`max_cnt` 降到 80~100，图像降到 424×240；**务必用带独立供电的 USB Hub**，相机峰值电流大 |

---

## 路线 C：EuRoC 数据集回放（无需相机）

没有相机、或想先验证算法本身，用公开数据集最快。

### C.1 下载数据集

EuRoC MAV 数据集（ROS bag 格式）：
<https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>
下载任一 `.bag`，例如 `MH_01_easy.bag`。ROS2 需要先转成 rosbag2 格式，或下载 ros2 版。

### C.2 启动 VINS（容器内）

```bash
# 挂载数据集目录进容器
docker run -it --rm --net=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/datasets:/data \
  vins-ros2:humble-amd64 bash

# 容器内：用自带的 EuRoC launch（配置已内置）
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 launch vins euroc.launch.py \
  config_path:=/ros2_ws/src/vins/../../vins_config/... # 见下方说明
```

> 镜像里自带 EuRoC 配置：`/opt/ros/humble` 环境下
> `ros2 pkg prefix vins`/share 里有 `launch/euroc.launch.py`。
> 源码配置在构建上下文的 `src/config/euroc/euroc_stereo_imu_config.yaml`。

### C.3 回放 bag

另开一个终端进同一容器：

```bash
ros2 bag play /data/MH_01_easy   # rosbag2 目录
```

VINS 会订阅 bag 里的双目 + IMU 话题输出轨迹。rviz2 里可看到完整飞行轨迹。

> EuRoC 的话题名与 D435i 不同（`/cam0/image_raw` 等），
> 用 `euroc_stereo_imu_config.yaml`，别用 d435i 的配置。

---

## 2. 相机标定（换相机/追求精度时）

**每台相机的内参、IMU-相机外参都不同**，照抄别人的配置会明显掉精度甚至发散。

### D435i：从出厂标定自动生成

```bash
# 本机
docker exec -it vins-ros2-local /ros_entrypoint.sh \
  python3 /ros2_ws/scripts/gen_d435i_config.py --out /ros2_ws/vins_config/d435i
# 板子
vins calib
```

脚本从相机读真实内参和外参，生成 `left.yaml` / `right.yaml` / `extrinsics_from_device.yaml`。
拿到外参后编辑 `d435i_stereo_imu.yaml`：填入 `body_T_cam0/1`，把 `estimate_extrinsic` 改成 `0`。

### 其他相机

用 [Kalibr](https://github.com/ethz-asl/kalibr) 做完整的相机-IMU 联合标定，
IMU 噪声参数用 [imu_utils](https://github.com/gaowenliang/imu_utils) 跑 2 小时静置数据。
把结果填进对应的 config yaml。

### 内参换算（降分辨率时）

```
fx' = fx × (新宽/原宽)    cx' = cx × (新宽/原宽)
fy' = fy × (新高/原高)    cy' = cy × (新高/原高)
```

核对相机实际输出：`ros2 topic echo --once /camera/camera/infra1/camera_info`

---

## 3. 接入飞控

让飞控用 VINS 的位姿定位飞行（无 GPS 场景）。

### 飞控侧参数（PX4）

```
EKF2_EV_CTRL  = 15      # 融合视觉的位置+速度+偏航
EKF2_HGT_REF  = 3       # 高度基准用视觉
EKF2_GPS_CTRL = 0       # 室内无 GPS 时关闭
EKF2_EV_DELAY = 5       # 视觉相对 IMU 的延迟(ms)，按实测调
```

### 连接方式

```bash
vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600   # USB 串口
vins start mavros:=true fcu_url:=/dev/ttyTHS1:921600   # Jetson 硬件串口
vins start mavros:=true fcu_url:=udp://:14540@         # 网络
```

### 验证链路

```bash
ros2 topic echo /mavros/vision_pose/pose      # VINS 位姿是否回灌
ros2 topic echo /mavros/local_position/pose   # 飞控融合后位姿，应跟随 VINS
```

> **纯视觉（无 GPS）必须用 OFFBOARD 模式起飞。** `AUTO.*` 模式都要求全局位置(GPS)，
> 纯视觉只有局部位置，解锁会被拒（报 `Arming denied: Resolve system health failures first`）。

---

## 4. 话题与命令速查

### VINS 输出话题（根命名空间）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/odometry` | `nav_msgs/Odometry` | 位姿 + 速度（主输出） |
| `/path` | `nav_msgs/Path` | 历史轨迹 |
| `/point_cloud` | `sensor_msgs/PointCloud` | 稀疏特征点云 |
| `/image_track` | `sensor_msgs/Image` | 特征跟踪可视化 |
| `/camera_pose` | `nav_msgs/Odometry` | 相机位姿 |
| `/extrinsic` | `nav_msgs/Odometry` | 在线估计的外参 |

### 相机输入话题（官方驱动，双 camera 命名空间）

| 话题 | 频率 |
|---|---|
| `/camera/camera/infra1/image_rect_raw` | 30 Hz |
| `/camera/camera/infra2/image_rect_raw` | 30 Hz |
| `/camera/camera/imu` | 200 Hz |

> ⚠️ VINS 发布端 QoS 是 **BEST_EFFORT**。用 `ros2 topic echo` 收不到时，
> 加 `--qos-reliability best_effort`；自己写订阅节点也要用 BEST_EFFORT。

### 常用命令

```bash
# 本机
./run_local_d435i.sh [--check|--status|--stop|--fallback]
# 板子
vins [start|stop|status|logs|calib|shell|restart]
# 通用
ros2 node info /vins_estimator          # 看节点真实发布/订阅的话题
ros2 topic hz /odometry                 # 确认 VINS 在输出
```

---

## 5. 按平台调参

改 `config/d435i/d435i_stereo_imu.yaml`，最影响性能的几项：

| 参数 | 树莓派5 | RDK X5 | Jetson Orin | x86 | 说明 |
|---|---|---|---|---|---|
| `max_cnt` | 80~100 | 120 | 150 | 150~200 | 跟踪特征数，**最影响 CPU** |
| `image_width/height` | 424×240 | 640×480 | 640×480 | 848×480 | 降分辨率是提速最有效手段 |
| `max_solver_time` | 0.06 | 0.05 | 0.04 | 0.04 | 后端时间预算 |
| `max_num_iterations` | 6 | 8 | 8 | 8 | 优化迭代次数 |
| `show_track` | 0 | 0 | 0 | 1 | 可视化吃 CPU，板子上关 |
| `use_gpu` | 0 | 0 | 1 | 0 | 仅 Jetson 变体支持 |

判断是否跟得上：`ros2 topic hz /odometry` 明显低于图像帧率，或 `top` 里 vins_node 长期
单核 100%+，就该降 `max_cnt` 或分辨率。

其他关键参数：
- `estimate_extrinsic: 1` 在线估计外参（安全默认）；标定后改 `0` 更准
- `estimate_td: 1` 在线估计相机-IMU 时间偏移（真机建议开）
- `g_norm` 当地重力（北京 9.801，广州 9.788）
- IMU 噪声 `acc_n/gyr_n/acc_w/gyr_w`：**宁大勿小**，设太小机动时易发散

---

## 6. 故障排查

### 相机相关

**症状：取不到帧 / `VIDIOC_S_FMT failed` / `IR stream start failure`**

按顺序排查（**第 1 条最常见，本项目多次栽在这**）：

1. **残留进程占着相机** —— 之前的 realsense/vins 进程没退干净，会让相机所有操作都失败，
   表现得像硬件坏了。先 `./run_local_d435i.sh --stop`，脚本会清理并 USB 复位。
2. **USB 线/口** —— 必须 USB3 数据线（很多线只能充电），直插机身蓝色口，别用 Hub/扩展坞。
3. **供电不足** —— 树莓派上尤其明显，用带独立供电的 Hub。

**症状：IMU 无数据 / `iio_hid_sensor: Frames didn't arrive`**

D435i 的 IMU 走 Linux HID/IIO 内核接口，重启后可能卡住。**普通 USB 复位不够，需要连续两次深度复位**：

```bash
L=$(lsusb | grep -i 8086:0b3a)
BUS=$(printf "%03d" $(echo "$L"|sed 's/Bus 0*\([0-9]*\).*/\1/'))
DEV=$(printf "%03d" $(echo "$L"|sed 's/.*Device 0*\([0-9]*\).*/\1/'))
for i in 1 2; do
  python3 -c "import fcntl;f=open('/dev/bus/usb/$BUS/$DEV','wb');fcntl.ioctl(f,ord('U')<<8|20,0)"
  sleep 5
done
```

### VINS 相关

**症状：rviz 里什么都没有 / 里程计无数据**

1. **确认话题名对不对** —— VINS 发布 `/odometry` 不是 `/vins_estimator/odometry`。
   用 `ros2 node info /vins_estimator` 看真实话题。
2. **QoS 不匹配** —— 发布端 BEST_EFFORT，订阅端默认 RELIABLE 收不到。
   `ros2 topic echo --qos-reliability best_effort /odometry`
3. **VINS 还没初始化** —— 看日志有没有 `Initialization finish!`。
   没有的话：给相机运动（平移，别纯旋转）、确认场景有纹理。

**症状：位置漂到几百上千米（发散）**

1. **特征太少** —— 对着有纹理的场景，看 `n_pts size` 应有几十个。
2. **IMU 频率不足** —— `ros2 topic hz /camera/camera/imu` 应 ≈200Hz，掉到几十会发散。
3. **标定不对** —— 内参、基线、外参是否与实际相机一致。
4. **纯静止初始化** —— VIO 对纯静止是病态的，起来后给点运动帮助收敛。

**症状：日志刷 `throw img1`**

左右目时间戳对不齐（VINS 容差 3ms），双目退化成单目。检查两目帧率是否一致、时间戳是否同步。

### 构建相关

**症状：`Could NOT find Boost` / `ceres::Manifold is not a member`**

依赖版本问题，本包 Dockerfile 已处理（Boost 全家桶 + **Ceres 精确锁 2.1.0**）。
如果你改了 Dockerfile：VINS 同时用了 `Manifold`(≥2.1) 和 `LocalParameterization`(<2.2)，
**只有 Ceres 2.1.0 同时提供两者**，2.0 和 2.2+ 都编不过。

**症状：`docker build` 卡在拉取基础镜像**

Docker Hub 直连超时。用 `scripts/build_arm64.sh`（自动透传宿主代理），
或给 Docker daemon 配 registry-mirror。

---

## 7. 已知限制与设计约束

- **本包 VINS 源码是第三方 ROS2 移植版**（zinuok/VINS-Fusion-ROS2），
  与 HKUST 官方版、以及 `../vins_docs/DATABASE.md` 知识库讲解的代码有少量差异。
- **Ceres 必须 2.1.0**：见上文，不要动这个版本。
- **arm64 板子路线未在真实硬件验证**：镜像能交叉构建，但 Pi5/RDK X5/Jetson + 真实相机
  的端到端未实测，realsense 参数名在不同驱动版本间有差异，首次上板可能要微调。
- **RDK X5 的 BPU 未利用**：通用镜像是纯 CPU 的。
- **官方 realsense 驱动默认双 camera 命名空间**：话题是 `/camera/camera/...`，
  配置文件已按此填写；若你改了 `camera_name`，记得同步改 VINS 配置。
- **D435i 出厂可能无 IMU 标定**：本项目用的这台就没有（`estimate_extrinsic: 1` 在线估计），
  精度够日常用；追求最佳精度请自己用 Kalibr 标定。

---

## 附：最短路径速查

```bash
# ═══ x86 + D435i（已验证）═══
cd ~/workspace/vins_ros2_deploy
docker build -t vins-ros2:humble-amd64 build_ctx    # 首次
./run_local_d435i.sh                                # 运行，然后拿起相机移动

# ═══ 边缘板子 ═══
./scripts/build_arm64.sh                            # x86 上构建
scp dist/*.tar.gz dist/deploy.sh 板子:~/            # 拷过去
ssh 板子 'bash ~/deploy.sh ~/*.tar.gz && vins calib && vins start'
```
