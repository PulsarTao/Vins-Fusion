# seeker1 四目鱼眼环视相机接入 VINS-Fusion

本文记录接入过程中定位到的问题与结论。**每一条都有实测数据支撑**，不是推测。
排查过程走了不少弯路，这里把错误的判断也一并保留，避免下次重复。

---

## 1. 数据链路

```
seeker 驱动 ──/all/compressed──> seeker_split_node ──/cam0 /cam1──> VINS-Fusion
  (宿主机)      JPEG 四路堆叠         (宿主机)         mono8 640x480    (容器内)
  20 Hz         约 346 KB           解码/切片/去畸变                    10 Hz 输出
              ──/imu_data_raw───────────────────────────────────────>
                200 Hz
```

中间这一层不是多余的，两个理由缺一不可：

**理由一：带宽。** 驱动的 `/fisheye/*/image_raw` 是 bgr8 1088×1280，单帧 4.18 MB，
DDS 传这种大消息严重掉速，实测四路合计只有 2.2 ~ 8.0 Hz 且帧率极不均匀。
走 `/all/compressed` 压缩流本地拆分，可以跑满相机的 20 Hz。

**理由二：相机模型。** 出厂标定是 omni(MEI) 模型，镜面参数 `xi = 3.22`。
VINS 的 `CataCamera::liftProjective` 里有这么一句（`CataCamera.cc:624`）：

```cpp
P << mx_u, my_u, 1.0 - xi*(rho2_d+1.0) / (xi + sqrt(1.0 + (1.0-xi*xi)*rho2_d));
//                                                     ~~~~~~~~~~~~~~~~~~~~~~
// xi=3.22 -> (1-xi²) = -9.37 -> 只要 rho2_d > 0.107 就是负数开方 -> NaN
```

标准 MEI 模型的 `xi` 定义域是 `[0,1]`，Kalibr 用的是允许 `xi>1` 的扩展形式，
两者不兼容。直接把鱼眼标定喂给 VINS，输出的位姿全是 `nan`。

**正向**投影没有这个问题（`z = Pz + xi·‖P‖`，`xi>1` 时恒为正），
所以预计算一张「虚拟针孔像素 → 鱼眼像素」的查找表是安全的。

---

## 2. 四个鱼眼怎么用

**VINS-Fusion 最多支持 2 个相机**，这是硬限制：

```cpp
// parameters.cpp:157
if (NUM_OF_CAM != 1 && NUM_OF_CAM != 2) { printf("num_of_cam should be 1 or 2\n"); assert(0); }
```

厂家脚本 `3_get_undistort_kalibr_info.py` 定义了正确用法：把**相邻两个鱼眼
配成一对矫正双目**，四个鱼眼刚好组成四对。每个虚拟针孔图只来自一个物理鱼眼，
且**光心不变、只做旋转**，所以重映射与深度无关，是纯方向重采样。

| 双目对 | 物理鱼眼 | 基线 |
|---|---|---|
| `front`（默认） | 0(left) + 1(right) | 4.61 cm |
| `right` | 1(right) + 2(bright) | ~3.2 cm |
| `back` | 2(bright) + 3(bleft) | ~3.2 cm |
| `left` | 3(bleft) + 0(left) | ~3.2 cm |

切换朝向：`python3 scripts/gen_seeker_config.py --pair back`

`gen_seeker_config.py` 复现了厂家的矫正数学，并与厂家自己的输出逐项比对，
**最大差 4.44e-16**（机器精度），所以外参可以设 `estimate_extrinsic: 0` 完全信任。

虚拟相机规格与厂家脚本写死的值一致：640×480，`fx=fy=320`，`cx=320 cy=240`，
零畸变，视场 90°×74°。视场覆盖检查显示 **100% 像素落在鱼眼成像圆内**，无黑边。

---

## 3. 三个把系统打垮的坑

相机静置桌面时位置发散到**几千米**。逐个排除后定位到三个独立问题，
**必须全部修好**，只修其中任意一两个仍然会炸。

### 坑一：`estimate_td: 1` 会死锁（危害最大）

`td` 是相机与 IMU 的时间偏移。它**只有在有运动时才可观测** ——
时间偏移造成的特征位移正比于运动速度，静止时该项恒为 0，`td` 完全不可观。
于是优化器可以随意推动它去吸收别的误差。实测静置时 `td` 的演化：

```
0.000 → 0.463 → 0.926 → 1.387 → 140.885 → 558.826 秒
```

一旦 `td` 变大，VINS 内部 `curTime = 图像时间 + td` 就远远领先 IMU 流，
`IMUAvailable()` 永远返回 false，估计器彻底卡死在 `wait for imu`
（实测刷了 **47478 次**），而位置在死锁前已被打飞到千米量级。

**解法**：`estimate_td: 0`。厂家标定给出 `timeshift_cam_imu = 0.0`，
且驱动的图像与 IMU 时间戳来自设备同一时钟（实测两者 dt 都干净、无重复无倒退）。

### 坑二：滑窗停滞导致特征饥饿（需要改 VINS 源码）

上游只用视差决定关键帧。相机静止时视差 ≈ 0，每帧都走 `MARGIN_SECOND_NEW` ——
这个分支只丢弃「次新帧」，窗口里 `0 .. frame_count-2` 这些帧被**冻结**，
永远不再接收新观测。后果有两个，第二个才是致命的：

1. IMU 预积分无界增长（`sum_dt` 一路涨，jacobian 超过 1e8，
   触发 `numerical unstable in preintegration`）
2. **特征饥饿**：新特征只能落在最后一个槽位，攒不够 4 次观测；
   而优化器只用 `used_num >= 4` 的特征（`estimator.cpp:1150`）。
   老特征随跟踪丢失逐个消亡后，**参与优化的特征数变成 0**，
   视觉约束彻底消失，VINS 退化为纯 IMU 递推。

调试输出里的 `feat` 一栏会稳定显示 `0`，这是最直接的判据。

**解法**：`patches/0001-force-keyframe-on-long-preintegration.patch`。
预积分区间超过 0.5 s 就强制产生关键帧，给 `sum_dt` 一个硬上界。

> **索引很关键**：真正无限增长的是 `pre_integrations[frame_count - 1]`
> （合并的目标，见 `estimator.cpp` 的 `slideWindow`），而不是
> `pre_integrations[frame_count]`（它每帧都被重建，恒等于一个图像周期）。
> 我第一版补丁检查错了索引，等于没检查，白白多走了一轮。

只调 `keyframe_parallax` 治不了本 —— 那等于指望像素噪声或场景里有人走动去
超过阈值。实测阈值降到 0.3 时，画面里有人活动就正常、真正静止时照样漂公里级。
不过把它调小仍然有益（真实运动时每帧视差本就远超阈值，调小不影响正常工况），
所以配置里同时设了 `keyframe_parallax: 0.3`。

### 坑三：宿主机侧不能加载 DDS profile

`config/fastdds_profile.xml` 禁用了共享内存（跨容器不通，必须走 UDP）。
但**宿主机侧绝不能加载它**，否则同机进程通信走 UDP 而非共享内存：

| | seeker_split_node 输出帧率 |
|---|---|
| 宿主机加载 profile | **9.2 Hz** |
| 宿主机不加载（默认共享内存） | **20.1 Hz** |

正确用法：宿主机不设 `FASTRTPS_DEFAULT_PROFILES_FILE`，只在 `docker exec` 时设。
容器侧还需把收发 buffer 提到 16 MB（一帧 mono8 要 200+ 个 UDP 分片），
并放开内核上限：

```bash
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.wmem_max=16777216
```

---

## 4. 修复后的实测结果

相机静置桌面，`./run.sh local seeker`：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| 静止位置漂移 | 244 m ~ 7545 m | **2.3 ~ 9.7 cm** |
| `wait for imu` 死锁 | 47478 次 | **0** |
| `numerical unstable` | 6 ~ 13 次 | **0** |
| 参与优化的特征数 | **0** | 50 ~ 150 |
| 图像帧率 | — | 20.0 Hz（宿主机与容器一致） |
| IMU 频率 | — | 200 Hz，零重复时间戳 |
| solver 耗时 | — | 3.5 ~ 12 ms（10 Hz 下余量充足） |

加速度计零偏 `Ba` 收敛并稳定，陀螺零偏 `Bg = (0.0187, 0.0118, -0.0169)`
与静止时实测的陀螺均值高度吻合 —— 这是估计器工作正常的有力旁证。

---

## 5. 排查时的有用手段

### 看 `feat` 一栏

`patches/0002-debug-print-estimator-state.patch` 给 VINS 加了一行调试输出：

```
DBG V: 0.0003 0.0003 0.0002 | Ba: -0.077 -0.002 0.131 | Bg: 0.0187 0.0118 -0.0169 | feat: 144/150 双目 19
```

* `feat` 为 **0** → 视觉约束完全失效，在做纯 IMU 递推（见坑二）
* `Ba` 恒为 **0.00000** → 优化器没在动它，同样说明视觉没起作用
* `Bg` 应当收敛到静止时的陀螺读数均值，对不上说明 IMU 链路或外参有问题
* `V` 在静止时应当 ≈ 0，持续增长说明正在发散

### 用 `imu: 0` 做隔离

把配置里 `imu` 改成 `0` 跑纯双目。如果纯双目稳定（实测静止 16 秒漂移
**0.0001 m**）而加上 IMU 就发散，就能把问题干净地隔离到 IMU 融合侧，
不必再怀疑去畸变、内参、双目外参、三角化。这一步省了大量时间。

### 别只测一个指标

排查双目匹配率时我一度只测了「双目匹配成功率」，据此判断需要做曝光对齐。
补测「帧间跟踪成功率」后发现两者结论相反 —— VINS **两个都要**：
帧间跟踪决定特征能否攒够 4 次观测进入优化，双目匹配决定有没有深度约束。
只优化其中一个会得出错误结论。

### `pkill -f` 会杀掉自己

`pkill -f seeker_split_node` 在一条同时含有该字符串的命令行里执行时，
会匹配到承载它自己的 shell。表现是「命令莫名其妙中断」或「相机突然坏了」，
非常难查。用 `[s]eeker_split_node` 这种括号写法，或者直接按 PID 杀。
这个坑我在 `run.sh` 里写了注释，之后自己在临时命令里又踩了一次。

---

## 6. 可选：曝光对齐

四个鱼眼各自跑独立自动曝光，物理朝向差 90°，看到的光照不同，AE 会收敛到
不同的值。实测某时刻两目均值差 25 灰阶（cam0 有 9.9% 像素饱和）。
驱动没有开放曝光控制（`seeker.hpp` 只有 init/open/流控/标定/重启），
只能在 `seeker_split_node` 里补偿。

两次对照实验（各 40~50 对同步帧，完整复现 VINS 的 LK + 反向校验）：

| 方案 | 强逆光场景<br>双目匹配 | 光照均匀场景<br>帧间跟踪 / 双目匹配 |
|---|---|---|
| 原始 | 53.8% | 99.9% / 59.3% |
| CLAHE 双边 | 64.0% | 99.9% / 59.9% |
| 直方图匹配 r→l | 67.2% | 99.9% / 59.5% |
| 两者叠加 | 67.8% | 99.9% / 56.3% |

**结论是收益不确定，所以默认关闭** —— 只在两目曝光差很大时才有明显收益，
光照均匀时反而可能略微变差。需要时打开：

```bash
ros2 run seeker seeker_split_node --ros-args \
  -p remap_file:=.../seeker_remap.yaml -p hist_match:=true -p clahe:=true
```

直方图匹配方向选 `r→l`（让 cam1 适配 cam0）而不是反过来：
cam0 是做特征**检测**的那幅图，不该改动它。

---

## 7. 相关文件

| 路径 | 作用 |
|---|---|
| `scripts/gen_seeker_config.py` | 读标定 → 算矫正双目 → 生成 VINS 配置与重映射参数 |
| `ros2_ws_src/seeker1/src/seeker_split_node.cpp` | 解码/切片/去畸变，发 `/cam0` `/cam1` |
| `config/seeker/seeker_remap.yaml` | 给上面那个节点的重映射参数 |
| `config/seeker/seeker_stereo_imu.yaml` | VINS 主配置 |
| `config/seeker/kalibr_raw_fisheye.yaml` | 从设备读出的原始鱼眼标定（存档） |
| `config/seeker/kalibr_undistorted.yaml` | 厂家算的矫正后标定（用于交叉验证） |
| `patches/` | 对 VINS 上游的必要改动，由 `fetch_upstream.sh` 自动应用 |
