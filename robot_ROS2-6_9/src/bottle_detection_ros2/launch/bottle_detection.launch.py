#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测功能包启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取功能包路径
    pkg_dir = get_package_share_directory('bottle_detection_ros2')
    
    # 声明启动参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='21',
        description='双目相机设备ID'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/elf/Desktop/robot_ROS2/src/bottle_detection_ros2/data/yolo11n.rknn',
        description='RKNN模型文件路径'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='图像发布频率(Hz)'
    )
    
    show_display_arg = DeclareLaunchArgument(
        'show_display',
        default_value='True',
        description='是否显示图像窗口'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='/home/elf/Desktop/robot_ROS2/src/bottle_detection_ros2/data/out.xls',
        description='相机标定文件路径（留空使用默认参数）'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.2',
        description='最小有效检测距离(米)'
    )
    
    max_distance_arg = DeclareLaunchArgument(
        'max_distance',
        default_value='5.0',
        description='最大有效检测距离(米)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='瓶子检测置信度阈值'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG压缩质量(1-100)'
    )
    
    # 创建瓶子检测节点
    bottle_detection_node = Node(
        package='bottle_detection_ros2',
        executable='bottle_detection_node',
        name='bottle_detection_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'model_path': LaunchConfiguration('model_path'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'show_display': LaunchConfiguration('show_display'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'min_distance': LaunchConfiguration('min_distance'),
            'max_distance': LaunchConfiguration('max_distance'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'camera_width': 1280,
            'camera_height': 480,
            'model_size': [640, 640],
            'publish_compressed': True,
        }],
        remappings=[
            # 可以在这里添加话题重映射
            # ('/camera/left/image_raw', '/stereo/left/image_raw'),
        ]
    )
    
    # 日志信息
    log_info = LogInfo(
        msg=['正在启动瓶子检测节点...\n',
             '相机ID: ', LaunchConfiguration('camera_id'), '\n',
             '模型路径: ', LaunchConfiguration('model_path'), '\n',
             '发布频率: ', LaunchConfiguration('publish_rate'), ' Hz\n',
             '显示窗口: ', LaunchConfiguration('show_display')]
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 启动参数
        camera_id_arg,
        model_path_arg,
        publish_rate_arg,
        show_display_arg,
        calibration_file_arg,
        min_distance_arg,
        max_distance_arg,
        confidence_threshold_arg,
        jpeg_quality_arg,
        
        # 日志
        log_info,
        
        # 节点
        bottle_detection_node,
    ])