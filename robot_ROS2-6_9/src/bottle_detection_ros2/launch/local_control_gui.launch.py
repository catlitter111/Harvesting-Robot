#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地控制GUI启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 包目录
    pkg_dir = get_package_share_directory('bottle_detection_ros2')
    
    # 设置DISPLAY环境变量
    display_env = SetEnvironmentVariable(
        name='DISPLAY',
        value=EnvironmentVariable('DISPLAY', default_value=':0')
    )
    
    # 启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 日志级别参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # 本地控制GUI节点
    local_control_gui_node = Node(
        package='bottle_detection_ros2',
        executable='local_control_gui',
        name='local_control_gui_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
        # 设置环境变量
        env={'DISPLAY': EnvironmentVariable('DISPLAY', default_value=':0')},
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg="启动本地控制GUI界面..."
    )
    
    return LaunchDescription([
        display_env,
        use_sim_time_arg,
        log_level_arg,
        launch_info,
        local_control_gui_node,
    ]) 