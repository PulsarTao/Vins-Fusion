# VINS-Fusion · ROS2 Humble · aarch64 边缘部署包

把 VINS-Fusion 视觉惯性里程计打包成 **一个 aarch64 Docker 镜像**，
在 x86 开发机上交叉构建，拷到板子上一条命令部署运行。

**目标板**：树莓派 5 / 地平线 RDK X5 / NVIDIA Jetson AGX Orin
**相机**：Intel RealSense D435i（红外双目 + IMU）
**中间件**：ROS2 Humble + mavros（可把位姿回灌给 PX4/ArduPilot 飞控）

---

## 目录

- [三步跑起来](#三步跑起来)
- [板子上的日常操作](#板子上的日常操作)
- [镜像里有什么](#镜像里有什么)
- [为什么这么设计](#为什么这么设计)
- [相机标定（首次必做）](#相机标定首次必做)
- [接入飞控](#接入飞控)
- [按板子调优](#按板子调优)
- [故障排查](#故障排查)
- [已知限制](#已知限制)

---

## 三步跑起来

### 第 1 步：在 x86 开发机上构建

```bash
cd ~/workspace/vins_ros2_deploy

./scripts/build_arm64.sh            # 通用镜像（Pi5 / RDK X5 / Jetson 都能跑）
# 或
./scripts/build_arm64.sh --jetson   # Jetson CUDA 加速变体
```

用 QEMU 模拟 aarch64 编译，**首次约 40–90 分钟**（之后有缓存会快很多）。
产物在 `dist/`：

```
dist/vins-ros2-humble-arm64.tar.gz    # 镜像包
dist/deploy.sh                        # 板子上的部署脚本
```

### 第 2 步：拷到板子

```bash
scp dist/vins-ros2-humble-arm64.tar.gz dist/deploy.sh pi@<板子IP>:~/
```

### 第 3 步：在板子上部署

```bash
ssh pi@<板子IP>
bash ~/deploy.sh ~/vins-ros2-humble-arm64.tar.gz
```

脚本会自动：检查 docker → 导入镜像 → **识别板型**（Jetson 自动启用 `--runtime nvidia`）
→ 创建容器 → 安装 `vins` 快捷命令。

---

## 板子上的日常操作

部署后可用 `vins` 命令：

```bash
vins calib          # 从相机读出厂标定生成配置（首次必做，见下文）
vins start          # 启动：相机 + VINS
vins status         # 看节点列表与里程计频率
vins logs           # 实时日志
vins stop           # 停止
vins shell          # 进容器
```

带飞控回灌：

```bash
vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600
```

其他可选参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `camera:=false` | true | 不启动相机（用已有话题或回放 bag 时） |
| `mavros:=true` | false | 启动 mavros 并把 VINS 位姿回灌飞控 |
| `loop_fusion:=true` | false | 启用回环检测（消除长时间漂移，但吃 CPU） |
| `rviz:=true` | false | 启动 rviz2（需要板子接显示器或配好 X11 转发） |
| `config:=<路径>` | 内置 | 换用自定义 VINS 配置 |

输出的位姿在容器外也能拿到：容器把 `/root/output` 映射到板子的 `~/vins_output`。

---

## 镜像里有什么

```
ros:humble-ros-base (arm64, Ubuntu 22.04)
├── ROS2 Humble 基础
├── VINS-Fusion (ROS2 移植版, 源码编译进镜像)
│     ├── vins            视觉惯性里程计主节点
│     ├── loop_fusion     回环检测与位姿图优化
│     ├── global_fusion   GPS 等全局传感器融合
│     └── camera_models   相机模型库
├── librealsense2 + realsense2_camera   (官方 arm64 预编译包)
├── mavros + mavros_extras              (含大地水准面数据)
├── Ceres 2.0 / Eigen3 / OpenCV 4.5
└── /ros2_ws/
      ├── vins_config/d435i/    配置文件
      ├── vins_launch/          launch 文件
      └── scripts/              标定生成 + 位姿回灌脚本
```

VINS 源码取自 [zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2)（GPL-3.0），
在本项目中适配到 ROS2 Humble 与 aarch64。

---

## 为什么这么设计

几个不那么显然但很关键的决定：

**用红外双目，不用 RGB。**
D435i 的 `infra1`/`infra2` 是真正的全局快门双目对，基线 50 mm，硬件同步。
RGB 是卷帘快门且与深度模组不同步 —— 卷帘快门在快速运动时会让特征位置扭曲，
对 VIO 是灾难。

**必须关闭红外发射器（emitter）。**
D435i 默认投射红外散斑帮助深度计算，但散斑是**投射在场景上的**：
相机一动，散斑图案跟着动，特征跟踪器会把它们当成静止特征去跟，直接污染估计。
launch 文件里已设 `depth_module.emitter_enabled: 0`。

**镜像自带源码编译产物，不做挂载。**
板子上开箱即用，不需要再拉代码、装依赖、编译。代价是镜像大一些（约 3–4 GB）。

**RealSense 用官方 arm64 预编译包。**
`librealsense` 源码编译在树莓派上要 40 分钟以上，用 apt 包几十秒搞定。

**Ceres 锁定 2.0（jammy 自带版本）。**
VINS 用的 `LocalParameterization` API 在 Ceres 2.2+ 已被移除。
不要手动升级 Ceres，会编译失败。

**Jetson 变体不装 `libopencv-dev`。**
L4T 镜像自带**带 CUDA 的** OpenCV，apt 版会覆盖它并丢掉 CUDA 支持 ——
这是 Jetson 上最常见的踩坑点。

---

## 相机标定（首次必做）

**每台 D435i 的内参和 IMU-相机外参都不同**，出厂标定值存在相机固件里。
照抄别人的配置会明显掉精度甚至发散。

```bash
vins calib
```

脚本会从相机读出真实参数，生成：

- `left.yaml` / `right.yaml` —— 左右红外相机内参
- `extrinsics_from_device.yaml` —— IMU→相机外参

拿到外参后，编辑 `d435i_stereo_imu.yaml`：把 `body_T_cam0/1` 换成生成的矩阵，
并把 `estimate_extrinsic` 从 `1` 改成 `0`（完全信任外参，精度更好）：

```bash
vins shell
vi /ros2_ws/vins_config/d435i/d435i_stereo_imu.yaml
```

> 出厂标定一般是 mm 级精度，足够日常使用。
> 追求最佳精度可以用 [Kalibr](https://github.com/ethz-asl/kalibr) 做一次完整标定，
> IMU 噪声参数可用 [imu_utils](https://github.com/gaowenliang/imu_utils) 跑 2 小时静置数据标定。

---

## 接入飞控

### 飞控侧参数（PX4）

```
EKF2_EV_CTRL  = 15      # 融合视觉的位置 + 速度 + 偏航
EKF2_HGT_REF  = 3       # 高度基准用视觉
EKF2_GPS_CTRL = 0       # 室内无 GPS 时关闭
EKF2_EV_DELAY = 5       # 视觉相对 IMU 的延迟(ms)，按实测调
```

### 连接

```bash
vins start mavros:=true fcu_url:=/dev/ttyUSB0:921600   # USB 串口
vins start mavros:=true fcu_url:=/dev/ttyTHS1:921600   # Jetson 硬件串口
vins start mavros:=true fcu_url:=udp://:14540@         # 网络（PX4 SITL 或 UDP 电台）
```

### 验证链路

```bash
vins shell
ros2 topic hz /vins_estimator/odometry      # VINS 输出，应接近相机帧率
ros2 topic hz /mavros/vision_pose/pose      # 回灌链路
ros2 topic echo /mavros/local_position/pose # 飞控融合后的位姿，应跟随 VINS
```

> **纯视觉（无 GPS）必须用 OFFBOARD 模式起飞。**
> `AUTO.*` 模式都要求全局位置（GPS），纯视觉只有局部位置，解锁会被拒绝
> （报 `Arming denied: Resolve system health failures first`）。

---

## 按板子调优

三块板子算力差距很大，`d435i_stereo_imu.yaml` 里最值得调的参数：

| 参数 | 树莓派 5 | RDK X5 | Jetson AGX Orin | 说明 |
|---|---|---|---|---|
| `max_cnt` | 80–100 | 120 | 150 | 跟踪特征数，**最影响 CPU** |
| `image_width/height` | 424×240 | 640×480 | 640×480 | 降分辨率是提速最有效手段 |
| `max_solver_time` | 0.06 | 0.05 | 0.04 | 后端时间预算 |
| `max_num_iterations` | 6 | 8 | 8 | 优化迭代次数 |
| `show_track` | 0 | 0 | 0/1 | 可视化很吃 CPU，板子上建议关 |
| `use_gpu` | 0 | 0 | 1 | 仅 Jetson 变体支持 |

降分辨率时记得**同步缩放内参**：`fx, fy, cx, cy` 全部乘以缩放比例。

判断是否跟得上：

```bash
vins shell
ros2 topic hz /vins_estimator/odometry     # 明显低于相机帧率就是算力不够
top                                        # vins_node 长期 100%+ 单核占用要调参
```

---

## 故障排查

### 相机没数据

```bash
vins shell
ros2 topic list | grep camera             # 应看到 infra1/infra2/imu
rs-enumerate-devices                       # 相机是否被识别
```

- D435i **必须插 USB 3.0**（蓝色口）。USB 2.0 带宽不足，双目 640×480@30 跑不动
- 树莓派建议用**带外部供电的 USB Hub**，相机峰值电流较大，供电不足会随机掉线
- 权限问题：`deploy.sh` 已用 `--privileged` + `-v /dev:/dev`，一般不会遇到

### VINS 不初始化 / 发散

按这个顺序排查：

1. **标定做了吗** —— `vins calib`，这是最常见原因
2. **特征够不够** —— 对着白墙、暗光环境会失效。VIO 需要有纹理的静止场景
3. **初始化时给点运动** —— 纯静止启动对 VIO 是病态问题（加速度计 bias 与重力不可分离）。
   起来后轻轻平移、旋转几秒帮助收敛
4. **时间戳质量** —— `ros2 topic hz /camera/imu` 和 `/camera/infra1/image_rect_raw`，
   频率应稳定；抖动大说明 USB 带宽或 CPU 不足

### 位姿漂移

- 开回环检测：`vins start loop_fusion:=true`
- 检查 IMU 噪声参数是否合理（宁大勿小）
- 确认 `g_norm` 与当地重力一致（北京 9.801，广州 9.788）

### Jetson 上 CUDA 没生效

```bash
docker info | grep -i runtime          # 应有 nvidia
cat /etc/nv_tegra_release              # 确认 JetPack 版本与镜像匹配
```

镜像的 L4T 版本必须与板子的 JetPack 对应，不匹配就改 `Dockerfile.jetson` 的 `FROM` 行重新构建。

---

## 已知限制

- **未在真实硬件上验证。** 本部署包在 x86 上交叉构建完成并通过镜像内自检，
  但**没有在实际的 Pi5 / RDK X5 / Jetson + 真实 D435i 上跑过** ——
  首次上板可能仍需针对具体板型微调（尤其是 realsense 参数名在不同版本间有差异）。
- **RDK X5 的 BPU 未被利用。** 通用镜像是纯 CPU 的。要用地平线 BPU 加速需要
  用其专有工具链改写前端，本包未涉及。
- **VINS 源码来自第三方移植版**（zinuok），与官方 HKUST 版本、
  以及 `VINS-Fusion/DATABASE.md` 知识库所讲解的代码存在少量差异。
- **Jetson 变体的 GPU 加速依赖移植版自带的 CUDA 实现**，其效果未经本项目实测。
