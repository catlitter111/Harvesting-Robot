#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
水果识别测试系统Launch文件
启动水果图片发布节点和WebSocket桥接节点进行测试
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 声明launch参数
    image_folder_arg = DeclareLaunchArgument(
        'image_folder_path',
        default_value='/home/robot/fruit_images',
        description='水果图片文件夹路径'
    )
    
    publish_interval_arg = DeclareLaunchArgument(
        'publish_interval',
        default_value='10.0',
        description='图片发布间隔（秒）'
    )
    
    loop_images_arg = DeclareLaunchArgument(
        'loop_images',
        default_value='true',
        description='是否循环发布图片'
    )
    
    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='ws://101.201.150.96:1234/ws/robot/robot_123',
        description='WebSocket服务器URL'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_123',
        description='机器人ID'
    )

    # 水果图片发布节点
    fruit_image_publisher_node = Node(
        package='bottle_detection_ros2',
        executable='fruit_image_publisher_node',
        name='fruit_image_publisher',
        parameters=[{
            'image_folder_path': LaunchConfiguration('image_folder_path'),
            'publish_interval': LaunchConfiguration('publish_interval'),
            'loop_images': LaunchConfiguration('loop_images'),
            'image_quality': 85,
            'max_image_size': 1024,
            'supported_formats': ['jpg', 'jpeg', 'png', 'bmp']
        }],
        output='screen',
        emulate_tty=True
    )

    # WebSocket桥接节点
    websocket_bridge_node = Node(
        package='bottle_detection_ros2',
        executable='websocket_bridge_node',
        name='websocket_bridge',
        parameters=[{
            'server_url': LaunchConfiguration('server_url'),
            'robot_id': LaunchConfiguration('robot_id'),
            'reconnect_attempts': 5,
            'reconnect_interval': 3.0,
            'ai_enabled': True,
            'ai_base_url': 'https://ai-gateway.vei.volces.com/v1',
            'ai_api_key': 'sk-1b880a05df7249d3927443d4872e2839oklzor2ja52wf1eu',  # 文本模型API key
            'ai_vision_api_key': 'sk-41995897b2aa4a6595f155f9abe700e6utiiwrjgtvnzod30',  # 视觉模型API key
            'ai_vision_model': 'doubao-1.5-thinking-pro-vision',  # 视觉模型，用于图片识别
            'ai_text_model': 'doubao-1.5-lite-32k',              # 文本模型，用于聊天
            'ai_max_tokens': 800
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        # Launch参数
        image_folder_arg,
        publish_interval_arg,
        loop_images_arg,
        server_url_arg,
        robot_id_arg,
        
        # 节点
        fruit_image_publisher_node,
        websocket_bridge_node,
    ]) 