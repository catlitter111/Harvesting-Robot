#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地控制系统启动文件
启动本地控制界面和必要的系统节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    # 获取功能包路径
    pkg_dir = get_package_share_directory('bottle_detection_ros2')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # 声明启动参数
    # 相机参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='21',
        description='双目相机设备ID'
    )
    
    # 模型参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value="/home/elf/Downloads/Harvesting-Robot/robot_ROS2-6_9/src/bottle_detection_ros2/data/yolo11n.rknn",
        description='RKNN模型文件路径'
    )
    
    # 串口参数
    robot_serial_arg = DeclareLaunchArgument(
        'robot_serial_port',
        default_value='/dev/ttyS3',
        description='机器人控制串口'
    )
    
    servo_serial_arg = DeclareLaunchArgument(
        'servo_serial_port',
        default_value='/dev/ttyS9',
        description='舵机控制串口'
    )
    
    # 显示参数
    show_display_arg = DeclareLaunchArgument(
        'show_display',
        default_value='true',
        description='是否显示图像窗口'
    )
    
    # 瓶子检测节点
    bottle_detection_node = Node(
        package='bottle_detection_ros2',
        executable='integrated_bottle_detection_node',
        name='bottle_detection',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'model_path': LaunchConfiguration('model_path'),
            'show_display': LaunchConfiguration('show_display'),
            'camera_width': 1280,
            'camera_height': 480,
            'model_size': [640, 640],
            'thread_num': 3,
            'publish_rate': 30.0,
            'min_distance': 0.2,
            'max_distance': 15.0,
            'confidence_threshold': 0.05,
            'enable_servo_tracking': True,
        }]
    )
    
    # 机器人控制节点
    robot_control_node = Node(
        package='bottle_detection_ros2',
        executable='robot_control_node',
        name='robot_control',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('robot_serial_port'),
            'baudrate': 115200,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'status_publish_rate': 2.0,
        }]
    )
    
    # 舵机控制节点
    servo_control_node = Node(
        package='bottle_detection_ros2',
        executable='servo_control_node',
        name='servo_control',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('servo_serial_port'),
            'baudrate': 115200,
            'tracking_deadzone': 30,
            'tracking_speed': 7.5,
            'enable_tracking': True,
        }]
    )
    
    # 自动采摘控制器
    auto_harvest_node = Node(
        package='bottle_detection_ros2',
        executable='auto_harvest_controller',
        name='auto_harvest',
        output='screen',
        parameters=[{
            'control_rate': 10.0,
            'search_timeout': 5.0,
            'approach_speed': 30.0,
            'turn_speed': 20.0,
            'fine_approach_speed': 10.0,
            'fine_turn_speed': 15.0,
        }]
    )
    
    # 本地控制GUI界面
    local_control_gui_node = Node(
        package='bottle_detection_ros2',
        executable='local_control_gui',
        name='local_control_gui',
        output='screen',
        parameters=[{
            'display_width': 1200,
            'display_height': 800,
            'update_rate': 30.0,
        }]
    )
    
    # 日志信息
    log_info = LogInfo(
        msg=[
            '\n========================================\n',
            '正在启动本地控制系统...\n',
            '相机ID: ', LaunchConfiguration('camera_id'), '\n',
            '模型路径: ', LaunchConfiguration('model_path'), '\n',
            '机器人串口: ', LaunchConfiguration('robot_serial_port'), '\n',
            '舵机串口: ', LaunchConfiguration('servo_serial_port'), '\n',
            '显示窗口: ', LaunchConfiguration('show_display'), '\n',
            '========================================\n'
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        camera_id_arg,
        model_path_arg,
        robot_serial_arg,
        servo_serial_arg,
        show_display_arg,
        
        # 日志
        log_info,
        
        # 核心系统节点
        bottle_detection_node,
        robot_control_node,
        servo_control_node,
        auto_harvest_node,
        
        # 本地控制GUI
        local_control_gui_node,
    ]) 