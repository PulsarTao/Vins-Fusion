#!/usr/bin/env python3
"""从【正在运行的 D435i】读取出厂标定，生成 VINS 配置文件。

为什么需要它:
  每台 D435i 的内参和 IMU-相机外参都不同(出厂标定值存在相机固件里)。
  照抄别人的配置会让 VINS 精度大打折扣甚至发散。本脚本直接从相机读真值。

用法(容器内,相机已插好且 realsense 节点已启动):
    ros2 launch /ros2_ws/vins_launch/vins_d435i.launch.py camera:=true &
    python3 gen_d435i_config.py --out /ros2_ws/vins_config/d435i

生成: left.yaml / right.yaml / d435i_stereo_imu.yaml

注意:
  出厂 IMU-相机外参精度一般够用(mm 级)。若追求最佳精度,
  用 Kalibr 做一次标定,再把结果填进 d435i_stereo_imu.yaml 的 body_T_cam0/1。
"""
import argparse
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


CAM_YAML = """%YAML:1.0
---
# 由 gen_d435i_config.py 从相机出厂标定自动生成
model_type: PINHOLE
camera_name: {name}
image_width: {w}
image_height: {h}
distortion_parameters:
   k1: {k1}
   k2: {k2}
   p1: {p1}
   p2: {p2}
projection_parameters:
   fx: {fx}
   fy: {fy}
   cx: {cx}
   cy: {cy}
"""


def mat_yaml(name, R, t):
    """输出 OpenCV 4x4 矩阵格式（VINS 用 cv::FileStorage 读取）。"""
    rows = []
    for i in range(3):
        rows.append('          %.9f, %.9f, %.9f, %.9f,' % (R[i, 0], R[i, 1], R[i, 2], t[i]))
    body = '\n'.join(rows)
    return ('%s: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: [\n'
            '%s\n          0.0, 0.0, 0.0, 1.0 ]\n' % (name, body))


class Grabber(Node):
    def __init__(self):
        super().__init__('gen_d435i_config')
        self.infos = {}
        for key, topic in (('left', '/camera/infra1/camera_info'),
                           ('right', '/camera/infra2/camera_info')):
            self.create_subscription(CameraInfo, topic,
                                     lambda m, k=key: self.infos.setdefault(k, m), 10)


def grab_camera_infos(timeout=20.0):
    rclpy.init()
    node = Grabber()
    import time
    t0 = time.time()
    while rclpy.ok() and len(node.infos) < 2 and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    infos = dict(node.infos)
    node.destroy_node()
    rclpy.shutdown()
    return infos


def read_extrinsics_from_sdk():
    """直接用 pyrealsense2 读 IMU→左红外相机外参（比 ROS 话题更完整）。"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        return None
    try:
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            return None
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
        cfg.enable_stream(rs.stream.accel)
        prof = pipe.start(cfg)
        ir1 = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        ir2 = prof.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        acc = prof.get_stream(rs.stream.accel)
        e_imu_to_ir1 = acc.get_extrinsics_to(ir1)
        e_ir1_to_ir2 = ir1.get_extrinsics_to(ir2)
        pipe.stop()
        return {
            'imu_to_cam0': (np.array(e_imu_to_ir1.rotation).reshape(3, 3).T,
                            np.array(e_imu_to_ir1.translation)),
            'cam0_to_cam1': (np.array(e_ir1_to_ir2.rotation).reshape(3, 3).T,
                             np.array(e_ir1_to_ir2.translation)),
        }
    except Exception as exc:  # noqa: BLE001
        print('  (pyrealsense2 读取外参失败: %s)' % exc, file=sys.stderr)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/ros2_ws/vins_config/d435i')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    print('读取 camera_info ...')
    infos = grab_camera_infos()
    if len(infos) < 2:
        print('✗ 未收到 infra1/infra2 的 camera_info。请确认相机已启动:', file=sys.stderr)
        print('  ros2 launch /ros2_ws/vins_launch/vins_d435i.launch.py camera:=true',
              file=sys.stderr)
        sys.exit(1)

    for key, fname in (('left', 'left.yaml'), ('right', 'right.yaml')):
        m = infos[key]
        K, D = m.k, list(m.d) + [0.0] * 5
        path = os.path.join(a.out, fname)
        with open(path, 'w') as f:
            f.write(CAM_YAML.format(name='cam0' if key == 'left' else 'cam1',
                                    w=m.width, h=m.height,
                                    fx=K[0], fy=K[4], cx=K[2], cy=K[5],
                                    k1=D[0], k2=D[1], p1=D[2], p2=D[3]))
        print('✓ %s  fx=%.2f cx=%.2f cy=%.2f' % (path, K[0], K[2], K[5]))

    print('\n读取 IMU-相机外参 ...')
    ext = read_extrinsics_from_sdk()
    if ext is None:
        print('! 无法读取外参(pyrealsense2 不可用或相机被占用)。')
        print('  d435i_stereo_imu.yaml 将保留 estimate_extrinsic: 1(在线优化初值)。')
        print('  建议:先停掉 realsense 节点再运行本脚本,以便独占相机读取外参。')
        return

    R_ic0, t_ic0 = ext['imu_to_cam0']
    R_c0c1, t_c0c1 = ext['cam0_to_cam1']
    # body(=IMU) → cam1 = (body→cam0) ∘ (cam0→cam1)
    R_ic1 = R_ic0 @ R_c0c1
    t_ic1 = R_ic0 @ t_c0c1 + t_ic0

    ext_path = os.path.join(a.out, 'extrinsics_from_device.yaml')
    with open(ext_path, 'w') as f:
        f.write('# 由 gen_d435i_config.py 从相机出厂标定读出\n')
        f.write('# 把这两个矩阵粘贴进 d435i_stereo_imu.yaml，并设 estimate_extrinsic: 0\n\n')
        f.write(mat_yaml('body_T_cam0', R_ic0, t_ic0))
        f.write('\n')
        f.write(mat_yaml('body_T_cam1', R_ic1, t_ic1))
    print('✓ %s' % ext_path)
    print('  基线 |t_cam0→cam1| = %.4f m (D435i 标称 0.05 m)' % np.linalg.norm(t_c0c1))


if __name__ == '__main__':
    main()
