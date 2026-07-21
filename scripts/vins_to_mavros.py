#!/usr/bin/env python3
"""把 VINS-Fusion 里程计转发给 PX4/ArduPilot (经 mavros vision_pose)。ROS2 版。

  /vins_estimator/odometry  (nav_msgs/Odometry)
      → /mavros/vision_pose/pose  (geometry_msgs/PoseStamped)

飞控侧需要的设置(PX4):
    EKF2_EV_CTRL   = 15     # 融合视觉的位置+速度+偏航
    EKF2_HGT_REF   = 3      # 高度基准用视觉
    EKF2_GPS_CTRL  = 0      # 室内无 GPS 时关闭
    EKF2_EV_DELAY  ≈ 5 ms   # 视觉数据相对 IMU 的延迟,按实测调

坐标系: VINS 世界系已由重力对齐(z 向上),与 mavros ENU 一致,可直接透传。
若你的机体安装方式导致偏航起始不一致,EKF2 会在初始化时自行对齐。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VinsToMavros(Node):
    def __init__(self):
        super().__init__('vins_to_mavros')
        # mavros 的 vision_pose 插件默认用 BEST_EFFORT，需匹配否则收不到
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', qos)
        self.create_subscription(Odometry, '/vins_estimator/odometry', self.cb, 10)
        self.n = 0
        self.get_logger().info('vins_to_mavros 已启动: /vins_estimator/odometry → /mavros/vision_pose/pose')

    def cb(self, odom):
        msg = PoseStamped()
        msg.header.stamp = odom.header.stamp
        msg.header.frame_id = 'map'
        msg.pose = odom.pose.pose
        self.pub.publish(msg)
        self.n += 1
        if self.n % 200 == 1:
            p = odom.pose.pose.position
            self.get_logger().info('转发 #%d: (%.2f, %.2f, %.2f)' % (self.n, p.x, p.y, p.z))


def main():
    rclpy.init()
    node = VinsToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
