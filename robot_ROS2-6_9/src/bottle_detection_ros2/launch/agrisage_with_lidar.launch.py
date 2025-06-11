#!/usr/bin/env python3
"""
AgriSage3 完整系统启动文件（包含激光雷达避障）
启动所有必要的节点：瓶子检测、自动采摘控制、舵机控制、激光雷达避障等
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_avoidance_arg = DeclareLaunchArgument(
        'enable_avoidance',
        default_value='true',
        description='Enable laser obstacle avoidance'
    )
    
    enable_display_arg = DeclareLaunchArgument(
        'enable_display',
        default_value='true',
        description='Enable camera display windows'
    )
    
    # 获取包路径
    bottle_detection_pkg = FindPackageShare('bottle_detection_ros2')
    
    # 参数文件路径
    params_file = PathJoinSubstitution([
        bottle_detection_pkg,
        'config',
        'laser_avoidance_params.yaml'
    ])
    
    # ==================== 激光雷达节点 ====================
    # 激光雷达驱动节点（来自order-humble-main包）
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
    
    # base_link到lidar的TF变换
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'lidar']
    )
    
    # ==================== 避障控制节点 ====================
    # 激光雷达避障控制器
    laser_avoidance_node = Node(
        package='bottle_detection_ros2',
        executable='laser_obstacle_avoidance',
        name='laser_obstacle_avoidance',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'avoidance_enabled': LaunchConfiguration('enable_avoidance')}
        ]
    )
    
    # ==================== AgriSage3 核心节点 ====================
    # 集成瓶子检测节点
    bottle_detection_node = Node(
        package='bottle_detection_ros2',
        executable='integrated_bottle_detection_node',
        name='integrated_bottle_detection_node',
        output='screen',
        parameters=[
            {'camera_id': 21},
            {'model_path': '/home/elf/Desktop/robot_ROS2/src/bottle_detection_ros2/data/yolo11n.rknn'},
            {'thread_num': 2},
            {'queue_size': 3},
            {'publish_rate': 15.0},
            {'show_display': LaunchConfiguration('enable_display')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 自动采摘控制器
    auto_harvest_node = Node(
        package='bottle_detection_ros2',
        executable='auto_harvest_controller',
        name='auto_harvest_controller',
        output='screen',
        parameters=[
            {'control_rate': 10.0},
            {'search_timeout': 5.0},
            {'approach_speed': 30.0},
            {'turn_speed': 20.0},
            {'fine_approach_speed': 10.0},
            {'fine_turn_speed': 15.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 舵机控制节点
    servo_control_node = Node(
        package='bottle_detection_ros2',
        executable='servo_control_node',
        name='servo_control_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyS9'},
            {'baudrate': 115200},
            {'timeout': 1.0},
            {'tracking_deadzone': 30},
            {'tracking_speed': 7.5},
            {'enable_tracking': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 机器人控制节点
    robot_control_node = Node(
        package='bottle_detection_ros2',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyS3'},
            {'baudrate': 115200},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # WebSocket桥接节点
    websocket_bridge_node = Node(
        package='bottle_detection_ros2',
        executable='websocket_bridge_node',
        name='websocket_bridge_node',
        output='screen',
        parameters=[
            {'websocket_port': 8765},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # ==================== 组装启动描述 ====================
    return LaunchDescription([
        # 启动参数
        use_sim_time_arg,
        enable_avoidance_arg,
        enable_display_arg,
        
        # 激光雷达相关节点
        laser_driver_node,
        base_to_laser_tf,
        laser_avoidance_node,
        
        # AgriSage3 核心节点
        bottle_detection_node,
        auto_harvest_node,
        servo_control_node,
        robot_control_node,
        websocket_bridge_node,
    ]) 