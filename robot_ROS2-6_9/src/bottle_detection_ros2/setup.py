#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'bottle_detection_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 安装package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        
        # 安装RViz配置文件
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        
        # 安装消息文件
        (os.path.join('share', package_name, 'msg'), 
            glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 nodes for bottle detection and harvesting system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 检测节点
            'bottle_detection_node = bottle_detection_ros2.nodes.detection.bottle_detection_node:main',
            'bottle_detection_node_async = bottle_detection_ros2.nodes.detection.bottle_detection_node_async:main',
            'integrated_bottle_detection_node = bottle_detection_ros2.nodes.detection.integrated_bottle_detection_node:main',
            
            # 控制节点
            'robot_control_node = bottle_detection_ros2.nodes.control.robot_control_node:main',
            'servo_control_node = bottle_detection_ros2.nodes.control.servo_control_node:main',
            'auto_harvest_controller = bottle_detection_ros2.nodes.control.auto_harvest_controller:main',
            
            # 通信节点
            'websocket_bridge_node = bottle_detection_ros2.nodes.communication.websocket_bridge_node:main',
            
            # 硬件节点
            'laser_obstacle_avoidance = bottle_detection_ros2.core.hardware.laser_obstacle_avoidance:main',
            
            # 导航节点
            'slam_mapping_node = bottle_detection_ros2.nodes.navigation.slam_mapping_node:main',
            
            # GUI和调试工具
            'debug_visualizer_gui = bottle_detection_ros2.gui.debug_visualizer_gui:main',
            
            # 测试脚本（如果存在main函数）
            'test_lidar_integration = bottle_detection_ros2.test_lidar_integration:main',
        ],
    },
)