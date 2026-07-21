#!/usr/bin/env python3
"""D435i + VINS-Fusion (ROS2 Humble) 一体化启动。

启动内容:
  1. realsense2_camera —— 红外双目 (infra1/infra2) + IMU,红外发射器关闭
  2. vins_node        —— 双目+IMU 紧耦合里程计
  3. (可选) mavros    —— 把 VINS 位姿回灌给飞控
  4. (可选) rviz2

为什么用红外双目而不是 RGB:
  - D435i 的 infra1/infra2 是真正的全局快门双目对,基线 50mm,已硬件同步
  - RGB 是卷帘快门且与深度模组不同步,不适合 VIO
  - 必须关闭红外发射器(emitter):投射的散斑点不是场景真实特征,
    相机一动散斑不动,会严重污染特征跟踪
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

CONFIG_DIR = '/ros2_ws/vins_config/d435i'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('config', default_value=os.path.join(CONFIG_DIR, 'd435i_stereo_imu.yaml'),
                              description='VINS 配置文件路径'),
        DeclareLaunchArgument('camera', default_value='true', description='是否启动 realsense 相机'),
        DeclareLaunchArgument('mavros', default_value='false', description='是否启动 mavros 并回灌位姿'),
        DeclareLaunchArgument('rviz', default_value='false', description='是否启动 rviz2'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyUSB0:921600',
                              description='飞控连接串口(或 udp://:14540@)'),
        DeclareLaunchArgument('loop_fusion', default_value='false', description='是否启用回环检测'),
    ]

    # ---------------- 1. RealSense D435i ----------------
    # 通过官方 rs_launch.py 传参,避免不同 realsense2_camera 版本的参数名差异
    realsense = GroupAction(
        condition=IfCondition(LaunchConfiguration('camera')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': '',
                # 红外双目 640x480@30
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'depth_module.profile': '640x480x30',
                # 关闭发射器:散斑会污染 VIO 特征跟踪
                'depth_module.emitter_enabled': '0',
                # 关掉用不上的流,省 USB 带宽和 CPU
                'enable_color': 'false',
                'enable_depth': 'false',
                'pointcloud.enable': 'false',
                # IMU: 陀螺+加速度计,线性插值合成到单一 /camera/imu 话题
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',       # 2 = linear_interpolation
                'gyro_fps': '200',
                'accel_fps': '250',
                # 用相机硬件时间戳,VIO 对时间戳质量极敏感
                'enable_sync': 'true',
            }.items())]
    )

    # ---------------- 2. VINS-Fusion ----------------
    vins = Node(
        package='vins', executable='vins_node', name='vins_estimator',
        output='screen',
        arguments=[LaunchConfiguration('config')],
    )

    # ---------------- 3. 回环检测(可选) ----------------
    loop = Node(
        package='loop_fusion', executable='loop_fusion_node', name='loop_fusion',
        output='screen',
        arguments=[LaunchConfiguration('config')],
        condition=IfCondition(LaunchConfiguration('loop_fusion')),
    )

    # ---------------- 4. mavros + 位姿回灌(可选) ----------------
    mavros_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('mavros')),
        actions=[
            Node(package='mavros', executable='mavros_node', name='mavros', output='screen',
                 parameters=[{
                     'fcu_url': LaunchConfiguration('fcu_url'),
                     'gcs_url': '',
                     'target_system_id': 1,
                     'target_component_id': 1,
                 }]),
            ExecuteProcess(cmd=['python3', '/ros2_ws/scripts/vins_to_mavros.py'],
                           output='screen'),
        ]
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [realsense, vins, loop, mavros_group, rviz])
