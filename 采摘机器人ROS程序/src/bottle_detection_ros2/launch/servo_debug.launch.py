#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机调试launch文件
同时启动舵机控制节点和调试节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成launch描述"""
    
    # 获取包的共享目录
    pkg_share = get_package_share_directory('bottle_detection_ros2')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'servo_debug_params.yaml')
    
    # 检查配置文件是否存在
    if not os.path.exists(config_file):
        config_file = None
    
    # 声明launch参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS9',
        description='舵机串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='串口波特率'
    )
    
    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='50',
        description='调试节点的步长大小（PWM）'
    )
    
    time_ms_arg = DeclareLaunchArgument(
        'time_ms',
        default_value='200',
        description='舵机移动时间（毫秒）'
    )
    
    tracking_enabled_arg = DeclareLaunchArgument(
        'enable_tracking',
        default_value='false',
        description='是否启用自动跟踪功能（调试时建议设为false）'
    )
    
    # 构建参数列表
    servo_control_params = []
    if config_file:
        servo_control_params.append(config_file)
    servo_control_params.append({
        'serial_port': LaunchConfiguration('serial_port'),
        'baudrate': LaunchConfiguration('baudrate'),
        'enable_tracking': LaunchConfiguration('enable_tracking'),
    })
    
    servo_debug_params = []
    if config_file:
        servo_debug_params.append(config_file)
    servo_debug_params.append({
        'step_size': LaunchConfiguration('step_size'),
        'time_ms': LaunchConfiguration('time_ms'),
    })
    
    # 舵机控制节点
    servo_control_node = Node(
        package='bottle_detection_ros2',
        executable='servo_control_node',
        name='servo_control_node',
        output='screen',
        parameters=servo_control_params,
        remappings=[
            ('servo/command', 'servo/command'),
            ('servo/status', 'servo/status'),
            ('servo/tracking_target', 'servo/tracking_target'),
            ('robot/harvest_command', 'robot/harvest_command'),
            ('harvest/status', 'harvest/status'),
        ]
    )
    
    # 舵机调试节点
    servo_debug_node = Node(
        package='bottle_detection_ros2',
        executable='servo_debug_node',
        name='servo_debug_node',
        output='screen',
        parameters=servo_debug_params,
        remappings=[
            ('servo/command', 'servo/command'),
            ('servo/status', 'servo/status'),
            ('robot/harvest_command', 'robot/harvest_command'),
        ]
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            '舵机调试系统已启动\n',
            '='*60, '\n',
            '节点信息:\n',
            '  - servo_control_node: 舵机控制节点\n',
            '  - servo_debug_node: 机械臂调试节点\n\n',
            '调试控制说明:\n',
            '  方向键: 控制舵机移动\n',
            '  数字键1-9: 快捷位置\n',
            '  Space: 回到中心位置\n',
            '  h: 显示帮助\n',
            '  t: 测试采摘动作\n',
            '  q: 退出程序\n',
            '='*60, '\n',
            '串口设备: ', LaunchConfiguration('serial_port'), '\n',
            '波特率: ', LaunchConfiguration('baudrate'), '\n',
            '步长大小: ', LaunchConfiguration('step_size'), ' PWM\n',
            '自动跟踪: ', LaunchConfiguration('enable_tracking'), '\n',
            '='*60, '\n'
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        serial_port_arg,
        baudrate_arg,
        step_size_arg,
        time_ms_arg,
        tracking_enabled_arg,
        
        # 启动信息
        start_info,
        
        # 节点启动
        servo_control_node,
        servo_debug_node,
    ]) 