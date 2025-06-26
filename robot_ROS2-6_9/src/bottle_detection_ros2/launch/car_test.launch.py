 #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车测试Launch文件
同时启动：
1. robot_control_node - 机器人控制节点
2. websocket_bridge_node - WebSocket桥接节点  
3. car_test_node - 小车测试节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """生成Launch描述"""
    
    # 声明Launch参数
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS3',
        description='机器人控制节点串口设备路径'
    )
    
    declare_server_url = DeclareLaunchArgument(
        'server_url', 
        default_value='ws://101.201.150.96:1234/ws/robot/robot_123',
        description='WebSocket服务器URL'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_123',
        description='机器人ID'
    )
    
    # 启动信息
    start_info = LogInfo(
        msg="🚀 启动小车测试系统..."
    )
    
    # 1. 机器人控制节点
    robot_control_node = Node(
        package='bottle_detection_ros2',
        executable='robot_control_node.py',
        name='robot_control_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': 115200,
            'timeout': 1.0,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'status_publish_rate': 2.0
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 2. WebSocket桥接节点
    websocket_bridge_node = Node(
        package='bottle_detection_ros2',
        executable='websocket_bridge_node.py',
        name='websocket_bridge_node',
        parameters=[{
            'server_url': LaunchConfiguration('server_url'),
            'robot_id': LaunchConfiguration('robot_id'),
            'reconnect_attempts': 5,
            'reconnect_interval': 3.0,
            'ai_enabled': True,
            'ai_base_url': 'https://ai-gateway.vei.volces.com/v1',
            'ai_api_key': 'sk-1b880a05df7249d3927443d4872e2839oklzor2ja52wf1eu',
            'ai_vision_api_key': 'sk-41995897b2aa4a6595f155f9abe700e6utiiwrjgtvnzod30',
            'ai_vision_model': 'doubao-1.5-thinking-pro-vision',
            'ai_text_model': 'doubao-1.5-lite-32k',
            'ai_max_tokens': 300
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 3. 小车测试节点 - 延迟启动以确保其他节点就绪
    car_test_node = TimerAction(
        period=3.0,  # 延迟3秒启动
        actions=[
            LogInfo(msg="🧪 启动小车测试节点..."),
            Node(
                package='bottle_detection_ros2',
                executable='car_test_node.py',
                name='car_test_node',
                output='screen',
                respawn=False  # 测试节点不需要重启
            )
        ]
    )
    
    # 系统信息
    system_info = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="🔧 系统组件说明："),
            LogInfo(msg="  📡 robot_control_node: 负责接收cmd_vel指令并控制机器人移动"),
            LogInfo(msg="  🌐 websocket_bridge_node: 负责WebSocket通信和AI功能"),
            LogInfo(msg="  🧪 car_test_node: 发送测试指令验证系统功能"),
            LogInfo(msg="📊 测试序列：前进1秒→停止→测试其他方向"),
            LogInfo(msg="🎯 观察日志输出查看测试结果")
        ]
    )
    
    # 使用说明
    usage_info = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="=" * 60),
            LogInfo(msg="🎮 小车测试系统使用说明："),
            LogInfo(msg="  1. 系统启动后会自动运行测试序列"),
            LogInfo(msg="  2. 测试节点会发送以下指令："),
            LogInfo(msg="     • 前进指令：linear.x = 0.5 m/s (100%速度)"),
            LogInfo(msg="     • 持续时间：1秒"),
            LogInfo(msg="     • 停止指令：所有速度归零"),
            LogInfo(msg="  3. 观察robot_control_node的日志确认指令接收"),
            LogInfo(msg="  4. 观察websocket_bridge_node的状态转发"),
            LogInfo(msg="  5. 按Ctrl+C退出测试"),
            LogInfo(msg="=" * 60)
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        declare_serial_port,
        declare_server_url,
        declare_robot_id,
        
        # 启动信息
        start_info,
        system_info,
        usage_info,
        
        # 节点启动
        robot_control_node,
        websocket_bridge_node,
        car_test_node,
    ])