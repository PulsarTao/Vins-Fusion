# 源码出处

本目录是第三方 VINS-Fusion ROS2 移植版，**非本项目编写**。

- 上游仓库: https://github.com/zinuok/VINS-Fusion-ROS2.git
- 使用版本: 72023bc (Merge pull request #25 from WhatWhatz/main)
- 许可证:   GPL-3.0（见 LICENCE）
- 原始出处: HKUST-Aerial-Robotics/VINS-Fusion（本仓库是其 ROS2 移植）

## 获取源码

本仓库**不包含**这份第三方源码（见 .gitignore），首次使用请执行：

```bash
./scripts/fetch_upstream.sh
```

它会 clone 上游并 checkout 到锁定的提交 `72023bc`。

## 本项目对它的依赖约束

- **Ceres 必须是 2.1.0**：本仓库同时用了 `ceres::Manifold`(2.1 引入) 和
  `LocalParameterization`(2.2 移除)，只有 2.1.0 同时提供两者。
  docker/Dockerfile 里是源码编译 2.1.0，不要改成 apt 的 libceres-dev(2.0)。
- **package.xml 声明不全**：缺 Boost、yaml-cpp，已在 Dockerfile 里补齐。
- **发布话题在根命名空间**：`/odometry` `/path` `/point_cloud` `/image_track`
  （不是 `/vins_estimator/odometry`），QoS 为 BEST_EFFORT。
