#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试启动文件
可以单独启动各个节点进行调试
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """生成启动描述"""
    
    pkg_dir = get_package_share_directory('bottle_detection_ros2')
    
    # 声明启动哪些节点的参数
    launch_detection_arg = DeclareLaunchArgument(
        'launch_detection',
        default_value='true',
        description='是否启动瓶子检测节点'
    )
    
    launch_websocket_arg = DeclareLaunchArgument(
        'launch_websocket',
        default_value='false',
        description='是否启动WebSocket节点'
    )
    
    launch_robot_control_arg = DeclareLaunchArgument(
        'launch_robot_control',
        default_value='false',
        description='是否启动机器人控制节点'
    )
    
    launch_servo_control_arg = DeclareLaunchArgument(
        'launch_servo_control',
        default_value='false',
        description='是否启动舵机控制节点'
    )
    
    launch_auto_harvest_arg = DeclareLaunchArgument(
        'launch_auto_harvest',
        default_value='false',
        description='是否启动自动采摘控制器'
    )
    
    # 调试模式
    debug_mode_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='调试模式'
    )
    
    # 瓶子检测节点（调试版本）
    bottle_detection_node = Node(
        package='bottle_detection_ros2',
        executable='integrated_bottle_detection_node',
        name='bottle_detection_debug',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_detection')),
        parameters=[{
            'camera_id': 21,
            'show_display': True,
            'thread_num': 1,  # 调试时使用单线程
            'publish_rate': 10.0,  # 降低发布频率
        }],
        arguments=['--ros-args', '--log-level', 'debug'] if LaunchConfiguration('debug') else []
    )
    
    # WebSocket节点（离线测试版本）
    websocket_bridge_node = Node(
        package='bottle_detection_ros2',
        executable='websocket_bridge_node',
        name='websocket_bridge_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_websocket')),
        parameters=[{
            'server_url': 'ws://localhost:8080/ws/test',  # 本地测试服务器
            'robot_id': 'test_robot',
        }]
    )
    
    # 机器人控制节点（模拟模式）
    robot_control_node = Node(
        package='bottle_detection_ros2',
        executable='robot_control_node',
        name='robot_control_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_robot_control')),
        parameters=[{
            'serial_port': '/dev/null',  # 模拟串口
            'baudrate': 115200,
        }]
    )
    
    # 舵机控制节点（模拟模式）
    servo_control_node = Node(
        package='bottle_detection_ros2',
        executable='servo_control_node',
        name='servo_control_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_servo_control')),
        parameters=[{
            'serial_port': '/dev/null',  # 模拟串口
            'enable_tracking': True,  # 启用跟踪功能
        }]
    )
    
    # 自动采摘控制器
    auto_harvest_node = Node(
        package='bottle_detection_ros2',
        executable='auto_harvest_controller',
        name='auto_harvest_debug',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_auto_harvest')),
        parameters=[{
            'control_rate': 5.0,  # 降低控制频率
        }]
    )
    
    # RViz2可视化（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'debug.rviz')],
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    return LaunchDescription([
        # 参数声明
        launch_detection_arg,
        launch_websocket_arg,
        launch_robot_control_arg,
        launch_servo_control_arg,
        launch_auto_harvest_arg,
        debug_mode_arg,
        
        # 节点
        bottle_detection_node,
        websocket_bridge_node,
        robot_control_node,
        servo_control_node,
        auto_harvest_node,
        # rviz_node,  # 取消注释以启用RViz
    ])