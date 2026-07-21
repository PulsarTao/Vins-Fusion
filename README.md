# VINS-Fusion · ROS2 Humble

VINS-Fusion 视觉惯性里程计（VIO）的 **ROS2 Humble** 完整工程：从 Gazebo 仿真、
真机 RealSense D435i，到树莓派/RDK X5/Jetson 边缘板部署，一套代码全覆盖。

```
Gazebo Harmonic (NVIDIA 渲染)          Intel RealSense D435i
  x500 无人机 + 相机 + IMU                红外双目 + IMU
        │                                      │
        └──────────► VINS-Fusion ◄─────────────┘
                          │
              ┌───────────┼───────────┐
           rviz2      /odometry    mavros → PX4 飞控
          可视化      位姿/轨迹      视觉定位飞行(无 GPS)
```

## 验证状态

不同路线的实测情况不同，先说清楚，避免你踩坑时误判：

| 路线 | 平台 | 状态 |
|---|---|---|
| **Gazebo 仿真** | x86_64 | ✅ **已跑通** — VINS 初始化成功、输出位姿与点云 |
| **D435i 真机** | x86_64 | ✅ **已跑通** — 图像 30Hz / IMU 198Hz / 位姿正常 |
| 边缘板部署 | Jetson / RDK X5 / 树莓派5 | ⚠️ **镜像可交叉构建，但未在真实板子验证** |

---

## Quick Start（仿真，5 条命令）

```bash
git clone git@github.com:PulsarTao/Vins-Fusion.git && cd Vins-Fusion

./scripts/fetch_upstream.sh    # 1. 拉取上游 VINS 源码（锁定版本，本仓库不含它）
./scripts/build_amd64.sh       # 2. 构建镜像（首次约 10~20 分钟）
./run.sh doctor                # 3. 环境自检（缺什么会直接告诉你怎么装）
./run.sh                       # 4. 启动仿真（Gazebo + PX4 + VINS + rviz2）
./run.sh takeoff               # 5. 起飞 —— VINS 需要运动才能初始化
```

起飞后 rviz2 里会出现：**绿色轨迹**（VIO 估计的运动路径）、**坐标轴**（当前位姿）、
**白色点云**（三角化出的 3D 特征）。

> **为什么必须起飞**：单目 VIO 靠视差三角化恢复深度、靠 IMU 激励恢复尺度。
> 静止时既无视差也无激励，VINS 会一直停在 `Not enough features or parallax`。

停止：`./run.sh stop`

---

## 目录

- [环境要求](#环境要求)
- [路线 A：Gazebo 仿真](#路线-agazebo-仿真)
- [路线 B：D435i 真机](#路线-bd435i-真机)
- [路线 C：边缘板部署](#路线-c边缘板部署)
- [项目结构](#项目结构)
- [关键设计与踩坑记录](#关键设计与踩坑记录)
- [调参](#调参)
- [故障排查](#故障排查)
- [已知限制](#已知限制)

---

## 环境要求

| 组件 | 版本 | 说明 |
|---|---|---|
| Ubuntu | 22.04 | ROS2 Humble 的目标系统 |
| ROS2 | Humble | `/opt/ros/humble` |
| Gazebo | Harmonic (gz-sim 8.x) | 仿真用；非 Gazebo Classic |
| ros_gz | `ros-humble-ros-gz` | Gazebo ↔ ROS2 桥接 |
| PX4-Autopilot | v1.15+ | 仿真用，需已 `make px4_sitl_default` |
| Docker | 20.10+ | VINS 跑在容器里 |
| NVIDIA 驱动 | 可选 | 有则 Gazebo/rviz2 走硬件渲染，无则软件渲染（慢） |

一条命令检查全部：`./run.sh doctor`

---

## 路线 A：Gazebo 仿真

### 启动

```bash
./run.sh                # Gazebo GUI + PX4 + 桥接 + VINS + rviz2
./run.sh --headless     # 无 GUI，省资源（适合批量测试）
```

脚本会依次完成并**逐项验证**：

1. 启动 Gazebo（NVIDIA 渲染）→ 等话题就绪
2. 启动 PX4 spawn 无人机 → 等相机话题出现
3. **验证 IMU 真有发布者**（话题名存在 ≠ 有数据，这是个大坑）
4. 配置 PX4 仿真参数（放行无地面站/无遥控器的预检）
5. 桥接到 ROS2 → 验证话题实际频率
6. 启动 VINS + rviz2

### 常用命令

```bash
./run.sh takeoff        # 起飞（VINS 初始化需要运动）
./run.sh status         # 各环节状态：进程/Gazebo话题/ROS2频率/PX4预检/负载
./run.sh logs vins      # 日志：gz | px4 | bridge | vins | rviz
./run.sh stop           # 停止全部（含容器内进程）
./run.sh doctor         # 环境自检，不启动任何东西
```

### 仿真环境构成

| 项 | 内容 |
|---|---|
| 场景 | AWS RoboMaker Small Warehouse（货架/纸箱/杂物，纹理丰富） |
| 机型 | `x500_depth_vinsfusion`（x500 四旋翼 + 640×480@30Hz 相机 + 250Hz IMU） |
| VINS 模式 | 单目 + IMU（该机型无双目左右目对） |
| 数据链路 | Gazebo → `ros_gz_bridge` → `/cam0/image_raw` + `/imu0` → VINS |

实测：图像 ~21Hz、IMU 248Hz、系统负载 4~5/32 核。

### VINS 输出话题

⚠️ 在**根命名空间**，不是 `/vins_estimator/*`（ROS2 里无命名空间节点的相对话题解析到根路径），
且 QoS 是 **BEST_EFFORT**，订阅端必须匹配否则收不到：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/odometry` | `nav_msgs/Odometry` | 位姿 + 速度（主输出） |
| `/path` | `nav_msgs/Path` | 历史轨迹 |
| `/point_cloud` | `sensor_msgs/PointCloud` | 稀疏特征点云 |
| `/image_track` | `sensor_msgs/Image` | 特征跟踪可视化 |

```bash
# 命令行查看（注意加 QoS 参数）
ros2 topic echo --qos-reliability best_effort /odometry
```

---

## 路线 B：D435i 真机

```bash
./run_local_d435i.sh --check      # 相机自检（含 USB 复位）
./run_local_d435i.sh              # 启动：相机 + VINS + rviz2
./run_local_d435i.sh --status     # 话题频率
./run_local_d435i.sh --stop       # 停止
./run_local_d435i.sh --fallback   # 官方驱动与固件不兼容时，用备用发布器
```

跑起来后**拿起相机缓慢平移**（别原地纯旋转，纯旋转无视差三角化不出深度），
对着有纹理、光线好的场景，轨迹就会画出来。

### 相机标定（换相机时必做）

每台 D435i 的内参和 IMU-相机外参都不同，照抄配置会明显掉精度：

```bash
docker exec -it vins-ros2-local /ros_entrypoint.sh \
  python3 /ros2_ws/scripts/gen_d435i_config.py --out /ros2_ws/vins_config/d435i
```

脚本从相机固件读真实参数生成 `left.yaml` / `right.yaml` / `extrinsics_from_device.yaml`。
拿到外参后把 `config/d435i/d435i_stereo_imu.yaml` 的 `estimate_extrinsic` 改成 `0` 精度更好。

---

## 路线 C：边缘板部署

在 x86 上交叉构建，拷到板子一键部署。

```bash
# x86 开发机
./scripts/build_arm64.sh              # 通用 arm64 镜像（Pi5 / RDK X5 / Jetson）
./scripts/build_arm64.sh --jetson     # Jetson CUDA 加速变体
scp dist/*.tar.gz dist/deploy.sh 用户@板子IP:~/

# 板子上
bash ~/deploy.sh ~/vins-ros2-humble-arm64.tar.gz
vins calib && vins start              # 标定后启动
```

`deploy.sh` 会自动识别板型（Jetson 自动加 `--runtime nvidia`）、导入镜像、
建容器、装 `vins` 快捷命令（`start/stop/status/logs/calib/shell`）。

### 接入飞控（视觉定位飞行）

```bash
vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600
```

飞控侧 PX4 参数：

```
EKF2_EV_CTRL  = 15    # 融合视觉的位置+速度+偏航
EKF2_HGT_REF  = 3     # 高度基准用视觉
EKF2_GPS_CTRL = 0     # 室内无 GPS 时关闭
EKF2_EV_DELAY = 5     # 视觉相对 IMU 的延迟(ms)，按实测调
```

> 纯视觉（无 GPS）**必须用 OFFBOARD 模式起飞**。所有 `AUTO.*` 模式都要求全局位置，
> 纯视觉只有局部位置，解锁会被拒（报 `Arming denied`）。

---

## 项目结构

```
run.sh                      ★ 仿真一键启动（含 doctor 自检、负载守护）
run_local_d435i.sh          ★ D435i 真机一键启动
docker/
  Dockerfile                架构无关镜像（--platform 决定 amd64/arm64）
  Dockerfile.jetson         Jetson CUDA 变体（基于 L4T）
  egm96-5.tar.bz2           mavros 大地水准面数据（本地注入避免下载挂死）
scripts/
  fetch_upstream.sh         拉取上游 VINS 源码（锁定提交）
  build_amd64.sh            x86 原生构建
  build_arm64.sh            QEMU 交叉构建 + 导出 tar
  deploy.sh                 板子上一键部署
  gen_d435i_config.py       从相机出厂标定生成配置
  d435i_publisher_ros2.py   备用相机发布器
  vins_to_mavros.py         VINS 位姿 → 飞控
config/
  gz_sim/                   仿真配置（单目+IMU）
  d435i/                    真机配置（双目+IMU）
  fastdds_profile.xml       DDS UDP-only（跨容器通信必需）
  vins_rviz2.rviz           可视化配置
models/
  x500_depth_vinsfusion/    定制机型
  OakD-VINS/                640×480 相机
  aws_warehouse/            仓库场景模型
worlds/vins_warehouse.sdf   适配后的仓库场景
docs/UPSTREAM.md            上游源码来源、版本、依赖约束
```

---

## 关键设计与踩坑记录

这些是调通过程中的真实坑，改动前务必读：

### 1. NVIDIA 渲染必须指定 EGL vendor

Gazebo 的 Ogre2 走 **EGL** 而非 GLX。混合显卡（Intel + NVIDIA）上只设
`__NV_PRIME_RENDER_OFFLOAD` 和 `__GLX_VENDOR_LIBRARY_NAME` **不够**，会黑屏、
窗口不出现，日志报 `libEGL: failed to create dri2 screen`。必须加：

```bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

验证真的在用独显（而非悄悄回落软件渲染）：

```bash
PID=$(pgrep -f "gz sim gui" | head -1)
ls -l /proc/$PID/fd | grep nvidia          # 应看到 /dev/nvidia0
grep -o "libEGL_nvidia[^ ]*" /proc/$PID/maps | head -1
```

### 2. 必须先启 gz server 再启 PX4

PX4 自己启动 Gazebo 时会 `source` 它编译期生成的 `gz_env.sh`，其中路径指向
**编译时的源码目录**，会覆盖 `GZ_SIM_SERVER_CONFIG_PATH` → Gazebo 不加载
`gz::sim::systems::Imu` 插件 → **IMU 话题存在但没有发布者** → VINS 永远等不到数据。

### 3. 自定义 world 必须有 `<spherical_coordinates>`

缺了它 GPS 报 `lat/lon = 0` → EKF2 无法确定地理方位 → 偏航对齐失败
（`cs_yaw_align=False`）→ 水平位置无效 → **无法解锁起飞**。
这条链路很长，很容易误判成别的问题。

### 4. 千万别用 `SYS_HAS_MAG=0` 关磁力计

为绕过"强磁干扰"报警而关掉磁力计，会让 EKF2 失去偏航基准，症状与第 3 条
一模一样但更难查。正确做法是只关强度检查：`COM_ARM_MAG_STR=0`。

### 5. 跨容器 DDS 必须用 UDP-only

宿主机的 bridge 与容器内的 VINS，默认走共享内存传输，跨容器不通。
症状是「容器能 `ros2 topic list` 看到话题，但收不到任何数据」。
用 `config/fastdds_profile.xml` 禁用 builtin transports 只留 UDPv4。

### 6. Ceres 必须锁 2.1.0

上游同时用了 `ceres::Manifold`（2.1 引入）和 `LocalParameterization`（2.2 移除），
**只有 2.1.0 同时提供两者**。Ubuntu 自带的 2.0 和最新的 2.2+ 都编不过。

### 7. 仿真里别用 `set_pose` 驱动无人机

直接用 Gazebo 的 `set_pose` 服务"瞬移"模型来制造运动，看似能绕开飞控，
但 IMU 测的是真实物理加速度，瞬移会产生巨大的不连续跳变，
VINS 的预积分基于这种假数据**必然发散**（实测漂到 471m）。
VIO 仿真必须让飞控真正驱动飞行。

---

## 调参

改 `config/gz_sim/gz_mono_imu.yaml`（仿真）或 `config/d435i/d435i_stereo_imu.yaml`（真机）：

| 参数 | 树莓派5 | RDK X5 | Jetson Orin | x86 | 说明 |
|---|---|---|---|---|---|
| `max_cnt` | 80~100 | 120 | 150 | 150~200 | 跟踪特征数，**最影响 CPU** |
| 分辨率 | 424×240 | 640×480 | 640×480 | 640×480 | 降分辨率是提速最有效手段 |
| `max_solver_time` | 0.06 | 0.05 | 0.04 | 0.04 | 后端时间预算 |
| `show_track` | 0 | 0 | 0 | 1 | 可视化吃 CPU，板子上关 |
| `use_gpu` | 0 | 0 | 1 | 0 | 仅 Jetson 变体支持 |

降分辨率时记得**同步缩放内参**：`fx, fy, cx, cy` 全部乘以缩放比例。

其他关键项：
- `estimate_extrinsic` — 仿真中外参精确已知设 `0`；真机没标定过设 `1` 在线优化
- `estimate_td` — 真机建议 `1`（在线估计相机-IMU 时间偏移）；仿真共用 `/clock` 设 `0`
- IMU 噪声 `acc_n/gyr_n/acc_w/gyr_w` — **宁大勿小**，设太小机动时易发散
- `g_norm` — 当地重力（北京 9.801，广州 9.788）

---

## 故障排查

### VINS 不初始化（`Not enough features or parallax`）

1. **给它运动** — 仿真里 `./run.sh takeoff`；真机拿起相机平移。纯静止或纯旋转都不行
2. **看特征够不够** — rviz2 的 track_image 面板，或 `ros2 topic echo /image_track`
3. **场景纹理** — 白墙、暗光环境会失效。仿真里默认的仓库场景可提取角点 183 个

### rviz2 里什么都没有

1. **话题名** — VINS 发布 `/odometry` 不是 `/vins_estimator/odometry`，
   用 `ros2 node info /vins_estimator` 看真实话题
2. **QoS** — 发布端 BEST_EFFORT，订阅端默认 RELIABLE 收不到
3. **VINS 是否初始化** — `./run.sh logs vins` 看有没有 `Initialization finish!`

### 位置发散到几百米

1. IMU 频率是否够（`ros2 topic hz /imu0` 应 ~248Hz，掉到几十会发散）
2. 标定是否正确（内参、基线、外参）
3. 特征数是否太少

### 相机没数据 / 取帧失败

按顺序排查（**第 1 条最常见**）：

1. **残留进程占着相机** — 之前的进程没退干净时，一切表现都像硬件坏了。
   先 `./run_local_d435i.sh --stop`（脚本会清理并 USB 复位）
2. **USB 线/口** — 必须 USB3 数据线（很多线只能充电），直插机身蓝色口
3. **供电不足** — 树莓派上尤其明显，用带独立供电的 Hub

### PX4 无法解锁

`./run.sh logs px4` 看具体原因。常见：
- 仿真无地面站/遥控器 → `run.sh` 已自动设参数放行
- `cs_yaw_align=False` → 检查 world 有没有 `<spherical_coordinates>`（见踩坑记录 3）

### Gazebo 黑屏

见[踩坑记录 1](#1-nvidia-渲染必须指定-egl-vendor)。

---

## 已知限制

- **仿真是单目 VIO** — PX4 自带机型只有单目 RGB + 深度相机，没有双目左右目对。
  单目的尺度靠 IMU 激励恢复，垂直起飞后悬停时尺度收敛不充分（实测高度误差约 33%）。
  要提高精度需让无人机做水平机动。
- **边缘板路线未在真实硬件验证** — 镜像能交叉构建，但 Pi5/RDK X5/Jetson +
  真实相机的端到端未实测，realsense 参数名在不同驱动版本间有差异。
- **RDK X5 的 BPU 未利用** — 通用镜像是纯 CPU 的。
- **VINS 源码是第三方 ROS2 移植版**（[zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2)，
  GPL-3.0），与 HKUST 官方版有少量差异，详见 `docs/UPSTREAM.md`。

---

## 致谢与许可

- VINS-Fusion 原作者：[HKUST-Aerial-Robotics](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)（GPL-3.0）
- ROS2 移植：[zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2)（GPL-3.0）
- 仿真场景：[AWS RoboMaker Small Warehouse](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)（MIT-0）

本仓库的脚本、配置、模型与文档遵循上游 GPL-3.0。
