#!/usr/bin/env python3
"""
SLAM建图启动文件 - 适配ROS2 Humble
启动激光雷达、SLAM工具箱和建图监控节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 visualization'
    )
    
    auto_start_mapping_arg = DeclareLaunchArgument(
        'auto_start_mapping',
        default_value='false',
        description='Automatically start mapping on launch'
    )
    
    # 获取包路径
    bottle_detection_pkg = FindPackageShare('bottle_detection_ros2')
    
    # 参数文件路径
    slam_params_file = PathJoinSubstitution([
        bottle_detection_pkg,
        'config',
        'slam_mapping_params.yaml'
    ])
    
    # ==================== 激光雷达节点 ====================
    # 激光雷达驱动节点（MS200）
    laser_driver_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='laser_driver',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/oradar'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # ==================== 坐标变换 ====================
    # base_link到lidar的静态变换
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'lidar']
    )
    
    # odom到base_link的变换（如果没有里程计）
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # ==================== SLAM工具箱 ====================
    # SLAM Toolbox 节点（适配ROS2 Humble）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # ==================== 建图监控节点 ====================
    # 自定义SLAM建图监控节点
    slam_mapping_node = Node(
        package='bottle_detection_ros2',
        executable='slam_mapping_node',
        name='slam_mapping_node',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'auto_start_mapping': LaunchConfiguration('auto_start_mapping')}
        ]
    )
    
    # ==================== 可视化 ====================
    # RViz2可视化
    rviz_config_file = PathJoinSubstitution([
        bottle_detection_pkg,
        'rviz',
        'slam_mapping.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ==================== 组装启动描述 ====================
    return LaunchDescription([
        # 启动参数
        use_sim_time_arg,
        enable_rviz_arg,
        auto_start_mapping_arg,
        
        # 硬件节点
        laser_driver_node,
        
        # 坐标变换
        base_to_laser_tf,
        odom_to_base_tf,
        
        # SLAM节点
        slam_toolbox_node,
        slam_mapping_node,
        
        # 可视化
        rviz_node,
    ]) 