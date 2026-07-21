#!/usr/bin/env python3
"""D435i → ROS2 话题发布器（备用方案）。

正常情况下请直接用 realsense2_camera 官方节点。
本脚本仅在官方节点与相机固件版本不兼容时作为后备
（症状：控制传输失败、"IR stream start failure"、取不到帧）。
它用 pip 装的 pyrealsense2（wheel 自带较新 librealsense），兼容新固件。

发布话题（与 VINS 配置对应）：
    /camera/infra1/image_rect_raw   左红外 mono8
    /camera/infra2/image_rect_raw   右红外 mono8
    /camera/imu                     陀螺 + 加速度计（200 Hz）

用法:
    python3 d435i_publisher_ros2.py --width 640 --height 480 --fps 30
"""
import argparse
import threading

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, CameraInfo


class D435iPublisher(Node):
    def __init__(self, w, h, fps):
        super().__init__('d435i_publisher')
        self.w, self.h, self.fps = w, h, fps

        # 传感器数据用 BEST_EFFORT：丢一帧也不要阻塞，实时性优先
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_imu = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=200)

        self.pub_ir1 = self.create_publisher(Image, '/camera/infra1/image_rect_raw', qos)
        self.pub_ir2 = self.create_publisher(Image, '/camera/infra2/image_rect_raw', qos)
        self.pub_info1 = self.create_publisher(CameraInfo, '/camera/infra1/camera_info', qos)
        self.pub_info2 = self.create_publisher(CameraInfo, '/camera/infra2/camera_info', qos)
        self.pub_imu = self.create_publisher(Imu, '/camera/imu', qos_imu)

        # 两条独立 pipeline —— 这是本脚本最关键的设计。
        # librealsense 默认让一条 pipeline 的所有帧共用一个回调线程，
        # 图像发布（640x480 双目 30fps ≈ 18MB/s 的拷贝 + DDS 序列化）会把
        # IMU 回调饿死：实测 IMU 名义 200Hz，实际投递只有 46Hz 且出现 2 秒空档，
        # 直接导致 VINS 预积分崩掉、位置漂到上万米。分成两条 pipeline 后，
        # 运动模组有自己的回调线程，不受图像发布影响。
        self.pipe_img = rs.pipeline()
        self.pipe_imu = rs.pipeline()
        self.info1 = self.info2 = None
        self.last_accel = None
        self.lock = threading.Lock()
        self.seq = 0
        self.imu_n = 0

    # ---------------------------------------------------------------- 启动
    def start(self):
        # --- 运动模组（独立线程，优先启动，保证 IMU 先于图像就绪）---
        cfg_imu = rs.config()
        # D435i 运动模组只支持 gyro 400/200 Hz、accel 200/100 Hz。
        # 请求不支持的组合（比如 accel 250）会直接报 "Couldn't resolve requests"。
        cfg_imu.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        cfg_imu.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        self.pipe_imu.start(cfg_imu, self.on_motion)

        # --- 立体模组（独立线程）---
        cfg_img = rs.config()
        cfg_img.enable_stream(rs.stream.infrared, 1, self.w, self.h, rs.format.y8, self.fps)
        cfg_img.enable_stream(rs.stream.infrared, 2, self.w, self.h, rs.format.y8, self.fps)
        prof = self.pipe_img.start(cfg_img, self.on_image)

        for s in prof.get_device().query_sensors():
            if s.supports(rs.option.emitter_enabled):
                s.set_option(rs.option.emitter_enabled, 0)
                self.get_logger().info('已关闭红外发射器（VIO 必须关闭：投射散斑不是场景真实特征）')

        self.info1 = self._camera_info(prof, 1)
        self.info2 = self._camera_info(prof, 2)
        i = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile().get_intrinsics()
        self.get_logger().info('左目内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f' % (i.fx, i.fy, i.ppx, i.ppy))
        self.get_logger().info('建议把这些值填进 VINS 的 left.yaml / right.yaml')

    def _camera_info(self, prof, idx):
        i = prof.get_stream(rs.stream.infrared, idx).as_video_stream_profile().get_intrinsics()
        m = CameraInfo()
        m.width, m.height = i.width, i.height
        m.distortion_model = 'plumb_bob'
        # ROS2 的 CameraInfo 字段是小写 d/k/r/p（ROS1 才是大写）
        m.d = [float(x) for x in i.coeffs]
        m.k = [i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [i.fx, 0.0, i.ppx, 0.0, 0.0, i.fy, i.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    # ---------------------------------------------------------------- 回调
    def _stamp(self, ms):
        t = rclpy.time.Time(nanoseconds=int(ms * 1e6))
        return t.to_msg()

    def on_motion(self, frame):
        """运动模组回调（独立线程，不受图像发布阻塞）。"""
        try:
            st = frame.get_profile().stream_type()
            if st == rs.stream.accel:
                with self.lock:
                    self.last_accel = frame.as_motion_frame().get_motion_data()
            elif st == rs.stream.gyro:
                self._pub_imu(frame)
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('IMU 回调异常: %s' % exc, throttle_duration_sec=5.0)

    def on_image(self, frame):
        """立体模组回调（独立线程）。"""
        try:
            if not frame.is_frameset():
                return
            fs = frame.as_frameset()
            ir1, ir2 = fs.get_infrared_frame(1), fs.get_infrared_frame(2)
            if not (ir1 and ir2):
                return
            stamp = self._stamp(ir1.get_timestamp())
            self._pub_img(self.pub_ir1, ir1, stamp, 'camera_infra1_optical_frame')
            self._pub_img(self.pub_ir2, ir2, stamp, 'camera_infra2_optical_frame')
            for pub, info, fid in ((self.pub_info1, self.info1, 'camera_infra1_optical_frame'),
                                   (self.pub_info2, self.info2, 'camera_infra2_optical_frame')):
                if info is None:
                    continue
                info.header.stamp = stamp
                info.header.frame_id = fid
                pub.publish(info)
            self.seq += 1
            if self.seq % 300 == 1:
                self.get_logger().info('已发布 %d 帧图像 / %d 条 IMU' % (self.seq, self.imu_n))
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('图像回调异常: %s' % exc, throttle_duration_sec=5.0)

    def _pub_img(self, pub, frame, stamp, frame_id):
        data = np.asanyarray(frame.get_data())
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.height, m.width = int(data.shape[0]), int(data.shape[1])
        m.encoding = 'mono8'
        m.is_bigendian = 0
        m.step = m.width
        m.data = data.tobytes()
        pub.publish(m)

    def _pub_imu(self, gyro_frame):
        with self.lock:
            accel = self.last_accel
        if accel is None:
            return                       # 开头几帧还没收到加速度，跳过
        g = gyro_frame.as_motion_frame().get_motion_data()
        m = Imu()
        m.header.stamp = self._stamp(gyro_frame.get_timestamp())
        m.header.frame_id = 'camera_imu_optical_frame'
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g.x, g.y, g.z
        m.linear_acceleration.x = accel.x
        m.linear_acceleration.y = accel.y
        m.linear_acceleration.z = accel.z
        m.orientation_covariance[0] = -1.0            # 不提供姿态
        self.pub_imu.publish(m)
        self.imu_n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    a = ap.parse_args()

    rclpy.init()
    node = D435iPublisher(a.width, a.height, a.fps)
    try:
        node.start()
    except RuntimeError as exc:
        node.get_logger().error('相机启动失败: %s' % exc)
        node.get_logger().error('排查顺序：1) 是否有残留进程占用相机（最常见）'
                                ' 2) USB3 数据线 3) 直插机身 USB3 口')
        rclpy.shutdown()
        return
    node.get_logger().info('D435i 发布器已启动 %dx%d@%dfps' % (a.width, a.height, a.fps))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipe_img.stop()
        node.pipe_imu.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
