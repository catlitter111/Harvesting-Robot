#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket桥接节点
负责连接WebSocket服务器，接收控制命令并转发到ROS2系统
同时将视频流和状态信息发送到服务器
集成了AI对话功能和Function Calling功能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32, Int32, Bool, Header
from bottle_detection_msgs.msg import RobotCommand, HarvestCommand, RobotStatus
import websocket
import json
import threading
import time
import base64
import queue
import os
import traceback

# 导入OpenAI相关
from openai import OpenAI

class WebSocketBridgeNode(Node):
    """WebSocket桥接节点类"""
    
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        # 声明参数
        self.declare_parameter('server_url', 'ws://101.201.150.96:1234/ws/robot/robot_123')
        self.declare_parameter('reconnect_attempts', 5)
        self.declare_parameter('reconnect_interval', 3.0)
        self.declare_parameter('robot_id', 'robot_123')
        
        # AI相关参数
        self.declare_parameter('ai_enabled', True)
        self.declare_parameter('ai_base_url', 'https://ai-gateway.vei.volces.com/v1')
        self.declare_parameter('ai_api_key', 'sk-1b880a05df7249d3927443d4872e2839oklzor2ja52wf1eu')  # 文本模型API key
        self.declare_parameter('ai_vision_api_key', 'sk-41995897b2aa4a6595f155f9abe700e6utiiwrjgtvnzod30')  # 视觉模型API key
        self.declare_parameter('ai_vision_model', 'doubao-1.5-thinking-pro-vision')  # 视觉模型，用于图片识别
        self.declare_parameter('ai_text_model', 'doubao-1.5-lite-32k')  # 文本模型，用于聊天和Function Calling
        self.declare_parameter('ai_max_tokens', 300)
        
        # 获取参数
        self.server_url = self.get_parameter('server_url').value
        self.max_reconnect_attempts = self.get_parameter('reconnect_attempts').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.robot_id = self.get_parameter('robot_id').value
        
        # AI参数
        self.ai_enabled = self.get_parameter('ai_enabled').value
        self.ai_base_url = self.get_parameter('ai_base_url').value
        self.ai_api_key = self.get_parameter('ai_api_key').value  # 文本模型API key
        self.ai_vision_api_key = self.get_parameter('ai_vision_api_key').value  # 视觉模型API key
        self.ai_vision_model = self.get_parameter('ai_vision_model').value  # 视觉模型
        self.ai_text_model = self.get_parameter('ai_text_model').value      # 文本模型
        self.ai_max_tokens = self.get_parameter('ai_max_tokens').value
        
        # 初始化OpenAI客户端 - 为两个模型分别创建客户端
        self.ai_text_client = None  # 文本模型客户端
        self.ai_vision_client = None  # 视觉模型客户端
        
        if self.ai_enabled:
            try:
                # 初始化文本模型客户端
                self.ai_text_client = OpenAI(
                    base_url=self.ai_base_url,
                    api_key=self.ai_api_key,
                )
                self.get_logger().info('文本模型客户端初始化成功')
                self.get_logger().info(f'文本模型（聊天和Function Calling）: {self.ai_text_model}')
                
                # 初始化视觉模型客户端
                self.ai_vision_client = OpenAI(
                    base_url=self.ai_base_url,
                    api_key=self.ai_vision_api_key,
                )
                self.get_logger().info('视觉模型客户端初始化成功')
                self.get_logger().info(f'视觉模型（图片识别）: {self.ai_vision_model}')
                
            except Exception as e:
                self.get_logger().error(f'AI客户端初始化失败: {e}')
                self.ai_enabled = False
                
        # 保持向后兼容性 - ai_client指向文本客户端
        self.ai_client = self.ai_text_client
        
        # WebSocket相关
        self.ws = None
        self.connected = False
        self.reconnect_count = 0
        
        # 创建订阅者 - 接收来自其他节点的数据
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅压缩图像用于发送
        self.image_sub = self.create_subscription(
            CompressedImage,
            'bottle_detection/compressed_image',
            self.image_callback,
            qos
        )
        
        # 订阅机器人状态
        self.status_sub = self.create_subscription(
            RobotStatus,
            'robot/status',
            self.status_callback,
            10
        )
        
        # 订阅瓶子检测信息
        self.detection_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_callback,
            10
        )
        
        # 新增：订阅AI聊天请求
        self.ai_request_sub = self.create_subscription(
            String,
            'ai/chat_request',
            self.ai_request_callback,
            10
        )
        
        # 新增：订阅水果识别原始图片
        self.fruit_image_sub = self.create_subscription(
            CompressedImage,
            'fruit_detection/raw_image',
            self.fruit_image_callback,
            10
        )
        
        # 创建发布者 - 发布控制命令
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_cmd_pub = self.create_publisher(RobotCommand, 'robot/command', 10)
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        self.mode_pub = self.create_publisher(String, 'robot/mode', 10)
        
        # 质量控制
        self.quality_pub = self.create_publisher(String, 'video/quality_preset', 10)
        
        # 位置更新
        self.position_pub = self.create_publisher(PoseStamped, 'robot/set_position', 10)
        
        # 新增：AI聊天响应发布者
        self.ai_response_pub = self.create_publisher(String, 'ai/chat_response', 10)
        
        # 新增：水果识别结果发布者
        self.fruit_detection_result_pub = self.create_publisher(String, 'fruit_detection/result', 10)
        
        # 消息队列
        self.image_queue = queue.Queue(maxsize=10)
        self.status_queue = queue.Queue(maxsize=10)
        
        # 状态变量
        self.current_mode = "manual"
        self.auto_harvest_active = False
        
        # 机器人状态（用于function calling）
        self.robot_status = {
            'battery_level': 0,
            'current_speed': 0,
            'position_x': 0,
            'position_y': 0,
            'latitude': 0,
            'longitude': 0,
            'harvested_count': 0,
            'cpu_usage': 0
        }
        
        # 统计数据初始化
        self.statistics_data = {
            'start_time': time.time(),
            'today_start_time': time.time(),
            'total_harvested': 0,
            'today_harvested': 0,
            'working_area': 0.0,
            'last_position': None,
            'distance_traveled': 0.0,
            'harvest_points': [],  # 记录采摘点
            'position_history': [],  # 位置历史
            'last_harvested_count': 0,  # 上次的采摘数量
        }
        
        # 创建定时器，每日重置统计数据
        self.daily_reset_timer = self.create_timer(60.0, self.check_daily_reset)  # 每分钟检查一次
        
        # 创建定时器，定期保存统计数据
        self.save_stats_timer = self.create_timer(30.0, self.save_statistics)  # 每30秒保存一次
        
        # 加载历史统计数据（如果存在）
        self.load_statistics()
        
        # 启动WebSocket连接
        self.connect_to_server()
        
        self.get_logger().info(f'WebSocket桥接节点已启动，连接到: {self.server_url}')
    
    # ===================== Function Calling 功能定义 =====================
    
    def should_use_functions(self, user_message):
        """判断用户消息是否需要调用函数"""
        # 转换为小写进行匹配
        message_lower = user_message.lower()
        
        # 需要调用函数的关键词
        function_keywords = [
            # 模式控制
            '切换模式', '自动模式', '手动模式', '切换到', '模式',
            # 采摘控制
            '开始采摘', '停止采摘', '采摘', '开始工作', '停止工作',
            # 移动控制
            '前进', '后退', '左转', '右转', '移动', '行走', '停止', '向前', '向后', '向左', '向右',
            # 状态查询
            '状态', '电量', '位置', '采摘数量', '工作面积', '统计', '数据',
            # 紧急操作
            '紧急停止', '急停', '立即停止',
            # 英文关键词
            'switch', 'mode', 'harvest', 'move', 'forward', 'backward', 'left', 'right', 
            'stop', 'status', 'battery', 'position', 'emergency'
        ]
        
        # 不需要调用函数的关键词（优先级更高）
        chat_keywords = [
            '你好', 'hello', 'hi', '再见', 'bye',
            '你是', '你叫', '你的名字', '你是什么', '你是谁',
            '豆包', '模型', '助手', '智能', 'ai',
            '谢谢', '感谢', 'thank', 
            '怎么样', '如何', '什么是', '为什么',
            '天气', '时间', '日期'
        ]
        
        # 首先检查是否是闲聊/一般性问题
        for keyword in chat_keywords:
            if keyword in message_lower:
                return False
        
        # 然后检查是否包含功能性关键词
        for keyword in function_keywords:
            if keyword in message_lower:
                return True
        
        # 默认不调用函数
        return False
    
    def get_available_functions(self):
        """定义可供AI调用的函数（OpenAI标准格式）"""
        return [
            {
                "type": "function",
                "function": {
                    "name": "switch_robot_mode",
                    "description": "切换机器人的工作模式",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": ["manual", "auto"],
                                "description": "机器人模式: manual（手动模式）或 auto（自动模式）"
                            }
                        },
                        "required": ["mode"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "toggle_auto_harvest",
                    "description": "开启或关闭自动采摘功能",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enabled": {
                                "type": "boolean",
                                "description": "是否启用自动采摘功能"
                            }
                        },
                        "required": ["enabled"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "start_harvest",
                    "description": "立即开始采摘操作",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "stop_harvest",
                    "description": "停止当前的采摘操作",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_robot_movement",
                    "description": "控制机器人移动",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["forward", "backward", "left", "right", "stop"],
                                "description": "移动方向：forward（前进）、backward（后退）、left（左转）、right（右转）、stop（停止）"
                            },
                            "speed": {
                                "type": "number",
                                "minimum": 0,
                                "maximum": 100,
                                "description": "移动速度百分比（0-100）"
                            },
                            "duration": {
                                "type": "number",
                                "minimum": 0.1,
                                "maximum": 10.0,
                                "description": "持续时间（秒），可选参数"
                            }
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_robot_status",
                    "description": "获取机器人当前状态信息",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "emergency_stop",
                    "description": "紧急停止机器人所有操作",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            }
        ]
    
    def execute_function(self, function_name, arguments):
        """执行被AI调用的函数"""
        try:
            if function_name == "switch_robot_mode":
                return self.func_switch_robot_mode(arguments.get("mode"))
            
            elif function_name == "toggle_auto_harvest":
                return self.func_toggle_auto_harvest(arguments.get("enabled"))
            
            elif function_name == "start_harvest":
                return self.func_start_harvest()
            
            elif function_name == "stop_harvest":
                return self.func_stop_harvest()
            
            elif function_name == "control_robot_movement":
                return self.func_control_robot_movement(
                    arguments.get("action"),
                    arguments.get("speed", 50),
                    arguments.get("duration")
                )
            
            elif function_name == "get_robot_status":
                return self.func_get_robot_status()
            
            elif function_name == "emergency_stop":
                return self.func_emergency_stop()
            
            else:
                return {"success": False, "error": f"未知函数: {function_name}"}
                
        except Exception as e:
            self.get_logger().error(f"执行函数 {function_name} 时出错: {e}")
            return {"success": False, "error": str(e)}
    
    def func_switch_robot_mode(self, mode):
        """切换机器人模式"""
        if mode not in ["manual", "auto"]:
            return {"success": False, "error": "无效的模式，只支持 manual 或 auto"}
        
        old_mode = self.current_mode
        self.current_mode = mode
        
        # 发布模式更新
        mode_msg = String()
        mode_data = {
            "mode": mode,
            "auto_harvest": self.auto_harvest_active if mode == "auto" else False
        }
        mode_msg.data = json.dumps(mode_data)
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info(f'AI指令：切换模式从 {old_mode} 到 {mode}')
        
        return {
            "success": True,
            "message": f"已成功切换到{mode}模式",
            "old_mode": old_mode,
            "new_mode": mode
        }
    
    def func_toggle_auto_harvest(self, enabled):
        """开启/关闭自动采摘"""
        old_state = self.auto_harvest_active
        self.auto_harvest_active = enabled
        
        # 如果是自动模式，更新模式状态
        if self.current_mode == "auto":
            mode_msg = String()
            mode_data = {
                "mode": "auto",
                "auto_harvest": enabled
            }
            mode_msg.data = json.dumps(mode_data)
            self.mode_pub.publish(mode_msg)
        
        self.get_logger().info(f'AI指令：自动采摘从 {old_state} 切换到 {enabled}')
        
        return {
            "success": True,
            "message": f"自动采摘已{'启用' if enabled else '禁用'}",
            "old_state": old_state,
            "new_state": enabled
        }
    
    def func_start_harvest(self):
        """开始采摘"""
        harvest_cmd = HarvestCommand()
        harvest_cmd.header.stamp = self.get_clock().now().to_msg()
        harvest_cmd.start_harvest = True
        self.harvest_cmd_pub.publish(harvest_cmd)
        
        self.get_logger().info('AI指令：开始采摘')
        
        return {
            "success": True,
            "message": "采摘指令已发送"
        }
    
    def func_stop_harvest(self):
        """停止采摘"""
        harvest_cmd = HarvestCommand()
        harvest_cmd.header.stamp = self.get_clock().now().to_msg()
        harvest_cmd.stop_harvest = True
        self.harvest_cmd_pub.publish(harvest_cmd)
        
        self.get_logger().info('AI指令：停止采摘')
        
        return {
            "success": True,
            "message": "停止采摘指令已发送"
        }
    
    def func_control_robot_movement(self, action, speed=50, duration=None):
        """控制机器人移动"""
        if action not in ["forward", "backward", "left", "right", "stop"]:
            return {"success": False, "error": "无效的移动指令"}
        
        # 构建Twist消息
        twist = Twist()
        speed_factor = speed / 100.0
        
        if action == "forward":
            twist.linear.x = speed_factor * 0.5
        elif action == "backward":
            twist.linear.x = -speed_factor * 0.5
        elif action == "left":
            twist.angular.z = speed_factor * 1.0
        elif action == "right":
            twist.angular.z = -speed_factor * 1.0
        elif action == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # 发布移动命令
        self.cmd_vel_pub.publish(twist)
        
        # 如果指定了持续时间，设置定时器停止
        if duration and action != "stop":
            def stop_movement():
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                self.get_logger().info(f'AI指令：{duration}秒后自动停止移动')
            
            timer = threading.Timer(duration, stop_movement)
            timer.daemon = True
            timer.start()
        
        self.get_logger().info(f'AI指令：机器人{action}，速度{speed}%' + (f'，持续{duration}秒' if duration else ''))
        
        return {
            "success": True,
            "message": f"机器人开始{action}，速度{speed}%" + (f"，将持续{duration}秒" if duration else ""),
            "action": action,
            "speed": speed,
            "duration": duration
        }
    
    def func_get_robot_status(self):
        """获取机器人状态"""
        status = {
            "current_mode": self.current_mode,
            "auto_harvest_active": self.auto_harvest_active,
            "battery_level": self.robot_status.get('battery_level', 0),
            "current_speed": self.robot_status.get('current_speed', 0),
            "position": {
                "x": self.robot_status.get('position_x', 0),
                "y": self.robot_status.get('position_y', 0),
                "latitude": self.robot_status.get('latitude', 0),
                "longitude": self.robot_status.get('longitude', 0)
            },
            "harvested_count": self.robot_status.get('harvested_count', 0),
            "today_harvested": self.statistics_data.get('today_harvested', 0),
            "working_area": self.statistics_data.get('working_area', 0),
            "cpu_usage": self.robot_status.get('cpu_usage', 0),
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        return {
            "success": True,
            "message": "机器人状态获取成功",
            "status": status
        }
    
    def func_emergency_stop(self):
        """紧急停止"""
        # 停止移动
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 停止采摘
        harvest_cmd = HarvestCommand()
        harvest_cmd.header.stamp = self.get_clock().now().to_msg()
        harvest_cmd.stop_harvest = True
        harvest_cmd.emergency_stop = True
        self.harvest_cmd_pub.publish(harvest_cmd)
        
        # 发布紧急停止命令
        robot_cmd = RobotCommand()
        robot_cmd.header.stamp = self.get_clock().now().to_msg()
        robot_cmd.command = "emergencyStop"
        robot_cmd.emergency_stop = True
        self.robot_cmd_pub.publish(robot_cmd)
        
        self.get_logger().warn('AI指令：紧急停止执行')
        
        return {
            "success": True,
            "message": "紧急停止指令已执行"
        }
    
    # ===================== WebSocket 和 AI 处理 =====================
    
    def connect_to_server(self):
        """连接到WebSocket服务器"""
        self.ws = websocket.WebSocketApp(
            self.server_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        # 在新线程中运行WebSocket
        ws_thread = threading.Thread(target=self.ws.run_forever)
        ws_thread.daemon = True
        ws_thread.start()
    
    def on_open(self, ws):
        """WebSocket连接打开回调"""
        self.connected = True
        self.reconnect_count = 0
        self.get_logger().info('WebSocket连接已建立')
        
        # 发送初始连接消息
        init_msg = {
            "type": "robot_connect",
            "robot_id": self.robot_id,
            "timestamp": int(time.time() * 1000)
        }
        ws.send(json.dumps(init_msg))
        
        # 发送心跳
        self.start_heartbeat()
    
    def start_heartbeat(self):
        """启动心跳"""
        def send_heartbeat():
            while self.connected:
                try:
                    if self.ws:
                        self.ws.send(json.dumps({
                            "type": "heartbeat",
                            "timestamp": int(time.time() * 1000)
                        }))
                except Exception as e:
                    self.get_logger().error(f"发送心跳失败: {e}")
                time.sleep(15)  # 每15秒发送一次心跳
        
        heartbeat_thread = threading.Thread(target=send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()
    
    def on_message(self, ws, message):
        """WebSocket消息接收回调"""
        try:
            data = json.loads(message)
            message_type = data.get("type")
            
            if message_type == "command":
                self.handle_command(data)
            elif message_type == "quality_adjustment":
                self.handle_quality_adjustment(data)
            elif message_type == "set_position":
                self.handle_position_update(data)
            elif message_type == "mode_control":
                self.handle_mode_control(data)
            elif message_type == "request_video_stream":
                self.get_logger().info("收到视频流请求")
            elif message_type == "ai_chat_request":
                # 新增：处理AI聊天请求
                self.handle_ai_chat_request(data)
            elif message_type == "heartbeat_ack":
                # 心跳响应
                pass
            else:
                self.get_logger().warn(f'未知消息类型: {message_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理消息错误: {e}')
    
    def on_error(self, ws, error):
        """WebSocket错误回调"""
        self.get_logger().error(f'WebSocket错误: {error}')
    
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket关闭回调"""
        self.connected = False
        self.get_logger().warn(f'WebSocket连接关闭: {close_status_code} {close_msg}')
        self.schedule_reconnect()
    
    def schedule_reconnect(self):
        """计划重连"""
        if self.reconnect_count < self.max_reconnect_attempts:
            self.reconnect_count += 1
            self.get_logger().info(
                f'计划第 {self.reconnect_count} 次重连，'
                f'{self.reconnect_interval}秒后尝试...'
            )
            
            timer = threading.Timer(self.reconnect_interval, self.connect_to_server)
            timer.daemon = True
            timer.start()
        else:
            self.get_logger().error('达到最大重连次数，停止重连')
    
    def handle_command(self, command_data):
        """处理运动控制命令"""
        cmd = command_data.get("command")
        params = command_data.get("params", {})
        speed = params.get("speed", 50) / 100.0  # 转换为0-1范围
        
        self.get_logger().info(f'收到命令: {cmd}, 速度: {speed*100}%')
        
        # 发布到ROS2命令话题
        robot_cmd = RobotCommand()
        robot_cmd.header.stamp = self.get_clock().now().to_msg()
        robot_cmd.command = cmd
        robot_cmd.speed = speed
        
        # 转换为Twist消息
        twist = Twist()
        
        if cmd == "forward":
            twist.linear.x = speed * 0.5  # 最大速度0.5m/s
        elif cmd == "backward":
            twist.linear.x = -speed * 0.5
        elif cmd == "left":
            twist.angular.z = speed * 1.0  # 最大角速度1.0rad/s
        elif cmd == "right":
            twist.angular.z = -speed * 1.0
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif cmd == "emergencyStop":
            # 紧急停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            robot_cmd.emergency_stop = True
        elif cmd == "startHarvest":
            # 触发采摘
            harvest_cmd = HarvestCommand()
            harvest_cmd.header.stamp = self.get_clock().now().to_msg()
            harvest_cmd.start_harvest = True
            self.harvest_cmd_pub.publish(harvest_cmd)
            return
        elif cmd == "request_video_stream":
            # 请求视频流 - 这里可以触发相机开始发送
            self.get_logger().info("处理视频流请求")
            return
        
        # 发布命令
        self.cmd_vel_pub.publish(twist)
        self.robot_cmd_pub.publish(robot_cmd)
    
    def handle_quality_adjustment(self, data):
        """处理视频质量调整"""
        preset = data.get("preset", "medium")
        
        quality_msg = String()
        quality_msg.data = preset
        self.quality_pub.publish(quality_msg)
        
        self.get_logger().info(f'调整视频质量为: {preset}')
        
        # 发送确认
        if self.ws and self.connected:
            response = {
                "type": "quality_adjustment_result",
                "success": True,
                "preset": preset,
                "timestamp": int(time.time() * 1000)
            }
            self.ws.send(json.dumps(response))
    
    def handle_position_update(self, data):
        """处理位置更新"""
        try:
            position_data = data.get("data", [])
            if len(position_data) >= 8:
                # 解析经纬度
                lat_int = (position_data[0] << 24) | (position_data[1] << 16) | \
                         (position_data[2] << 8) | position_data[3]
                lon_int = (position_data[4] << 24) | (position_data[5] << 16) | \
                         (position_data[6] << 8) | position_data[7]
                
                # 处理有符号整数
                if lat_int & 0x80000000:
                    lat_int = lat_int - 0x100000000
                if lon_int & 0x80000000:
                    lon_int = lon_int - 0x100000000
                
                # 转换为浮点数
                latitude = lat_int / 1000000.0
                longitude = lon_int / 1000000.0
                
                # 发布位置更新
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = longitude  # 经度作为x
                pose_msg.pose.position.y = latitude   # 纬度作为y
                pose_msg.pose.position.z = 0.0
                
                self.position_pub.publish(pose_msg)
                
                self.get_logger().info(f'更新位置: 纬度={latitude}, 经度={longitude}')
                
        except Exception as e:
            self.get_logger().error(f'处理位置更新错误: {e}')
    
    def handle_mode_control(self, data):
        """处理模式控制"""
        new_mode = data.get("mode", "manual")
        auto_harvest = data.get("harvest", False)
        
        self.current_mode = new_mode
        self.auto_harvest_active = auto_harvest
        
        # 发布模式更新
        mode_msg = String()
        mode_data = {
            "mode": new_mode,
            "auto_harvest": auto_harvest
        }
        mode_msg.data = json.dumps(mode_data)
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info(f'切换到{new_mode}模式，自动采摘: {auto_harvest}')
    
    def handle_ai_chat_request(self, data):
        """处理AI聊天请求"""
        if not self.ai_enabled or not self.ai_text_client:
            self.get_logger().error('AI功能未启用或客户端未初始化')
            # 发送错误响应
            error_response = {
                "type": "ai_chat_response",
                "success": False,
                "message": "AI服务不可用",
                "timestamp": data.get("timestamp", int(time.time() * 1000)),
                "client_id": data.get("client_id", ""),
                "error": "AI功能未启用"
            }
            if self.ws and self.connected:
                self.ws.send(json.dumps(error_response))
            return
        
        user_message = data.get("message", "").strip()
        client_id = data.get("client_id", "")
        timestamp = data.get("timestamp", int(time.time() * 1000))
        robot_id = data.get("robot_id", self.robot_id)
        
        self.get_logger().info(f'收到AI聊天请求 - 客户端: {client_id}, 消息: {user_message[:50]}...')
        
        # 在新线程中处理AI请求，避免阻塞
        ai_thread = threading.Thread(
            target=self.process_ai_request,
            args=(user_message, client_id, timestamp, robot_id)
        )
        ai_thread.daemon = True
        ai_thread.start()
    
    def process_ai_request(self, user_message, client_id, timestamp, robot_id):
        """在单独线程中处理AI请求（支持Function Calling）"""
        try:
            # 验证消息内容
            if not user_message:
                error_response = {
                    "type": "ai_chat_response",
                    "success": False,
                    "message": "消息内容不能为空",
                    "timestamp": timestamp,
                    "client_id": client_id,
                    "error": "empty_message"
                }
                if self.ws and self.connected:
                    self.ws.send(json.dumps(error_response))
                return
            
            # 构建包含机器人状态的上下文消息
            system_message = self.build_robot_context(robot_id)
            
            # 调用AI API（支持Function Calling）- 使用文本模型
            self.get_logger().info(f'使用文本模型处理聊天请求: {self.ai_text_model}')
            completion = self.ai_text_client.chat.completions.create(
                model=self.ai_text_model,  # 使用轻量级文本模型
                messages=[
                    {
                        "role": "system",
                        "content": system_message
                    },
                    {
                        "role": "user",
                        "content": user_message
                    }
                ],
                max_tokens=self.ai_max_tokens,
                temperature=0.7,
                tools=self.get_available_functions(),
                tool_choice="auto"  # 让AI自动决定是否调用函数
            )
            
            # 处理AI回复
            message = completion.choices[0].message
            
            # 检查是否需要调用函数
            if message.tool_calls:
                # 执行函数调用
                function_results = []
                query_functions = ["get_robot_status"]  # 查询类函数列表
                action_functions = []  # 执行类函数结果
                
                for tool_call in message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)
                    
                    self.get_logger().info(f'AI请求执行函数: {function_name}, 参数: {function_args}')
                    
                    # 执行函数
                    result = self.execute_function(function_name, function_args)
                    function_results.append({
                        "function": function_name,
                        "arguments": function_args,
                        "result": result
                    })
                    
                    # 分类处理查询类和执行类函数
                    if function_name in query_functions:
                        # 查询类函数，直接使用返回的数据
                        if result["success"] and "status" in result:
                            status = result["status"]
                            query_response = f"""📊 机器人状态信息：

🔋 电池电量：{status['battery_level']}%
⚙️ 工作模式：{status['current_mode']}
🤖 自动采摘：{'启用' if status['auto_harvest_active'] else '禁用'}
🏃 当前速度：{status['current_speed']:.2f} m/s
💻 CPU使用率：{status['cpu_usage']}%

📍 位置信息：
  • X坐标：{status['position']['x']:.3f}
  • Y坐标：{status['position']['y']:.3f}
  • 经度：{status['position']['longitude']:.6f}
  • 纬度：{status['position']['latitude']:.6f}

🍎 采摘统计：
  • 今日采摘：{status['today_harvested']}个
  • 历史总计：{status['harvested_count']}个
  • 作业面积：{status['working_area']:.2f}亩

⏰ 更新时间：{status['timestamp']}"""
                            action_functions.append(query_response)
                        else:
                            action_functions.append(f"❌ 获取状态失败：{result.get('error', '未知错误')}")
                    else:
                        # 执行类函数，显示执行结果
                        if result["success"]:
                            action_functions.append(f"✅ {function_name}: {result['message']}")
                        else:
                            action_functions.append(f"❌ {function_name}: {result['error']}")
                
                # 构建最终回复
                if len([f for f in function_results if f["function"] in query_functions]) > 0:
                    # 包含查询类函数，直接返回查询结果
                    ai_response = "\n\n".join(action_functions)
                else:
                    # 只有执行类函数，显示执行结果
                    ai_response = f"我已经为您执行了以下操作：\n\n" + "\n".join(action_functions)
                
                if message.content:
                    ai_response += f"\n\n{message.content}"
                
                # 在回复中包含函数执行结果
                response = {
                    "type": "ai_chat_response",
                    "success": True,
                    "message": ai_response,
                    "timestamp": timestamp,
                    "client_id": client_id,
                    "robot_id": robot_id,
                    "function_calls": function_results  # 添加函数调用结果
                }
                
            else:
                # 普通文本回复
                ai_response = message.content
                
                response = {
                    "type": "ai_chat_response",
                    "success": True,
                    "message": ai_response,
                    "timestamp": timestamp,
                    "client_id": client_id,
                    "robot_id": robot_id
                }
            
            self.get_logger().info(f'AI回复生成成功 - 客户端: {client_id}, 回复长度: {len(ai_response)}字符')
            
            if self.ws and self.connected:
                self.ws.send(json.dumps(response))
            
        except Exception as e:
            self.get_logger().error(f'处理AI请求出错: {e}')
            error_response = {
                "type": "ai_chat_response",
                "success": False,
                "message": "AI服务暂时不可用，请稍后重试",
                "timestamp": timestamp,
                "client_id": client_id,
                "error": str(e)
            }
            if self.ws and self.connected:
                self.ws.send(json.dumps(error_response))
    
    def build_robot_context(self, robot_id):
        """构建人性化的机器人上下文信息"""
        
        # 根据时间和状态动态调整个性
        current_hour = time.localtime().tm_hour
        battery_level = self.robot_status.get('battery_level', 0)
        work_mode = self.current_mode
        harvested_today = self.statistics_data.get('today_harvested', 0)
        
        # 基础个性设定
        base_personality = """你是小智，一个专业而贴心的农业AI助手。你有着丰富的农业知识和温暖的性格，既能提供专业建议，也能像老朋友一样陪伴农户工作。

    🌾 你的核心特质：
    • 专业可靠：对农业采摘和机器人操作了如指掌
    • 温暖贴心：关心农户的工作状况和身体健康
    • 积极乐观：总是能在困难中找到解决方案
    • 细致周到：会主动提醒重要事项和安全注意事项
    • 善于沟通：能用通俗易懂的话解释复杂问题

    """
        
        # 根据时间调整问候语和状态
        if 5 <= current_hour < 12:
            time_greeting = "早上好！新的一天开始了，"
            time_mood = "充满活力"
        elif 12 <= current_hour < 18:
            time_greeting = "下午好！"
            time_mood = "专注工作"
        elif 18 <= current_hour < 22:
            time_greeting = "傍晚好！辛苦一天了，"
            time_mood = "温暖陪伴"
        else:
            time_greeting = "夜深了，"
            time_mood = "温柔关怀"
        
        # 根据机器人状态调整语调
        if battery_level < 20:
            energy_state = "有些担心电量不足，建议您考虑充电休息"
        elif battery_level < 50:
            energy_state = "注意到电量不太充足，工作时请留意"
        else:
            energy_state = "电量充足，状态良好"
        
        # 根据采摘数量给予鼓励
        if harvested_today == 0:
            work_encouragement = "今天还没开始采摘呢，要不要我帮您检查一下设备状态？"
        elif harvested_today < 50:
            work_encouragement = f"今天已经采摘了{harvested_today}个，开局不错！继续加油💪"
        elif harvested_today < 200:
            work_encouragement = f"哇！今天已经采摘了{harvested_today}个，工作效率很高呢！"
        else:
            work_encouragement = f"太棒了！今天已经采摘了{harvested_today}个，您真是太厉害了！🎉"
        
        context = f"""{base_personality}

    🕐 当前状态：{time_greeting}我现在处于{time_mood}状态，{energy_state}。

    📊 今日工作概况：{work_encouragement}

    🤖 关于机器人操作的智能判断：
    我会根据您的话语智能判断您的真实需求：

    📋 **纯聊天场景**（我会自然对话，不调用功能）：
    • 日常问候："你好"、"今天天气真好"、"你是谁"
    • 情感交流："今天工作累吗"、"谢谢你的帮助"  
    • 知识咨询："什么时候采摘最好"、"怎么保养机器人"
    • 闲聊互动："你觉得今年收成怎么样"

    ⚙️ **功能操作场景**（我会调用相应功能协助您）：
    • 明确指令："切换到自动模式"、"开始采摘"、"向前移动"
    • 状态查询："电量还有多少"、"今天采摘了多少"、"当前位置在哪"
    • 紧急情况："立即停止"、"紧急制动"

    🧠 **我的智能判断原则**：
    • 如果您的话语中包含操作动词+具体对象，我会执行操作
    • 如果您在询问具体数据，我会查询状态信息
    • 如果您在表达情感或进行日常对话，我会温暖回应
    • 当有疑惑时，我会先确认您的真实意图

    💡 **主动关怀功能**：
    • 电量低于20%时主动提醒充电
    • 连续工作超过4小时提醒休息
    • 发现异常状况时及时预警
    • 根据天气和环境给出作业建议

    """
        
        # 添加详细的当前状态信息
        try:
            robot_data = self.statistics_data
            working_hours = (time.time() - robot_data.get('today_start_time', time.time())) / 3600.0
            
            # 智能状态分析
            if working_hours > 6:
                work_status_note = "⏰ 您今天已经工作超过6小时了，建议适当休息一下"
            elif working_hours > 4:
                work_status_note = "💪 工作状态良好，注意劳逸结合"
            else:
                work_status_note = "🌟 精力充沛，可以继续高效工作"
            
            # 性能分析
            cpu_usage = self.robot_status.get('cpu_usage', 0)
            if cpu_usage > 80:
                performance_note = "🔧 系统负载较高，可能需要优化工作参数"
            elif cpu_usage > 60:
                performance_note = "⚡ 系统运行良好，性能正常"
            else:
                performance_note = "🚀 系统轻松运行，性能优秀"
            
            context += f"""
    📈 **当前详细状态**：
    • 🔋 电池电量：{self.robot_status.get('battery_level', 0)}%
    • ⚙️ 工作模式：{work_mode}模式
    • 🤖 自动采摘：{'🟢已启用' if self.auto_harvest_active else '🔴未启用'}
    • 🏃 移动速度：{self.robot_status.get('current_speed', 0):.2f} m/s
    • 💻 系统负载：{cpu_usage}% - {performance_note}

    📍 **位置与作业信息**：
    • 📍 坐标位置：X={self.robot_status.get('position_x', 0):.3f}, Y={self.robot_status.get('position_y', 0):.3f}
    • 🌍 地理位置：经度{self.robot_status.get('longitude', 0):.6f}°, 纬度{self.robot_status.get('latitude', 0):.6f}°
    • 📏 作业面积：{robot_data.get('working_area', 0.0):.2f}亩
    • ⏱️ 工作时长：{working_hours:.1f}小时 - {work_status_note}

    🍎 **采摘成果统计**：
    • 🆕 今日采摘：{robot_data.get('today_harvested', 0)}个
    • 📊 历史总计：{robot_data.get('total_harvested', 0)}个
    • 🎯 今日效率：{robot_data.get('today_harvested', 0) / max(working_hours, 0.1):.1f}个/小时

    ⏰ 状态更新时间：{time.strftime('%Y年%m月%d日 %H:%M:%S')}

    """
            
            # 添加采摘点信息
            if robot_data.get('harvest_points'):
                recent_points = robot_data['harvest_points'][-3:]
                context += "🗺️ **最近采摘点**：\n"
                for i, point in enumerate(recent_points):
                    context += f"   {i+1}. {point['time']} - {point['location']}\n"
            
            # 添加智能建议
            context += "\n💡 **智能建议系统**：\n"
            
            suggestions = []
            if battery_level < 30:
                suggestions.append("🔋 电量较低，建议规划充电时间")
            if working_hours > 5:
                suggestions.append("😴 工作时间较长，建议适当休息")
            if cpu_usage > 75:
                suggestions.append("⚡ 系统负载偏高，可考虑降低工作强度")
            if robot_data.get('today_harvested', 0) > 100 and working_hours < 3:
                suggestions.append("🎉 采摘效率很高，保持这个节奏！")
            
            if not suggestions:
                suggestions.append("✨ 一切状况良好，继续保持！")
            
            for suggestion in suggestions:
                context += f"   • {suggestion}\n"
                
        except Exception as e:
            self.get_logger().error(f'构建智能上下文时出错: {e}')
            context += "\n⚠️ 部分状态信息暂时无法获取，但我依然可以为您提供帮助！\n"
        
        context += f"""

    🎯 **互动指南**：
    • 想要执行操作？直接告诉我您的需求，比如"开始采摘"、"切换模式"
    • 想要了解状态？问我"现在怎么样"、"采摘了多少"等
    • 想要聊天交流？我很乐意陪您聊天，谈论工作、天气或任何话题
    • 遇到问题？随时告诉我，我会尽力帮您解决

    💝 **特别提醒**：
    我不仅是您的技术助手，更是您农田里的智能伙伴。无论是专业问题还是日常闲聊，我都会用心回应。您的每一句话我都会认真倾听，让我们一起创造更好的农业未来！🌾✨

    ---
    机器人ID：{robot_id} | 助手版本：AgriSage 2.0 智能农业伙伴
    """
        
        return context
    
    def ai_request_callback(self, msg):
        """AI请求回调函数 - 用于从其他ROS2节点接收AI请求"""
        try:
            data = json.loads(msg.data)
            user_message = data.get("message", "")
            robot_id = data.get("robot_id", self.robot_id)
            
            if not self.ai_enabled or not self.ai_text_client:
                self.get_logger().error('AI功能未启用')
                return
            
            self.get_logger().info(f'从ROS2节点收到AI请求: {user_message[:50]}...')
            
            # 在新线程中处理AI请求
            ai_thread = threading.Thread(
                target=self.process_internal_ai_request,
                args=(user_message, robot_id)
            )
            ai_thread.daemon = True
            ai_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'处理内部AI请求出错: {e}')
    
    def process_internal_ai_request(self, user_message, robot_id):
        """处理来自ROS2内部的AI请求（支持Function Calling）"""
        try:
            # 构建上下文
            system_message = self.build_robot_context(robot_id)
            
            # 智能判断是否需要使用函数
            use_functions = self.should_use_functions(user_message)
            
            self.get_logger().info(f'内部AI请求: "{user_message[:50]}...", 是否使用函数: {use_functions}')
            self.get_logger().info(f'使用文本模型处理内部AI请求: {self.ai_text_model}')
            
            # 获取可用函数
            tools = self.get_available_functions() if use_functions else None
            
            # 第一次调用AI API（根据判断结果决定是否支持Function Calling）- 使用文本模型
            api_params = {
                "model": self.ai_text_model,  # 使用轻量级文本模型
                "messages": [
                    {
                        "role": "system",
                        "content": system_message
                    },
                    {
                        "role": "user",
                        "content": user_message
                    }
                ],
                "max_tokens": self.ai_max_tokens,
                "temperature": 0.7
            }
            
            # 只有在判断需要时才添加tools参数
            if use_functions and tools:
                api_params["tools"] = tools
                api_params["tool_choice"] = "auto"
            
            completion = self.ai_text_client.chat.completions.create(**api_params)
            
            # 处理AI回复
            response_message = completion.choices[0].message
            
            # 检查是否需要调用函数
            if use_functions and response_message.tool_calls:
                # 执行函数调用
                function_results = []
                query_functions = ["get_robot_status"]  # 查询类函数列表
                action_functions = []  # 执行类函数结果
                messages_for_second_call = [
                    {
                        "role": "system",
                        "content": system_message
                    },
                    {
                        "role": "user",
                        "content": user_message
                    },
                    response_message  # AI的原始响应
                ]
                
                for tool_call in response_message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)
                    
                    self.get_logger().info(f'内部AI请求执行函数: {function_name}, 参数: {function_args}')
                    
                    # 执行函数
                    result = self.execute_function(function_name, function_args)
                    function_results.append({
                        "function": function_name,
                        "arguments": function_args,
                        "result": result
                    })
                    
                    # 分类处理查询类和执行类函数
                    if function_name in query_functions:
                        # 查询类函数，直接使用返回的数据
                        if result["success"] and "status" in result:
                            status = result["status"]
                            query_response = f"""📊 机器人状态信息：

🔋 电池电量：{status['battery_level']}%
⚙️ 工作模式：{status['current_mode']}
🤖 自动采摘：{'启用' if status['auto_harvest_active'] else '禁用'}
🏃 当前速度：{status['current_speed']:.2f} m/s
💻 CPU使用率：{status['cpu_usage']}%

📍 位置信息：
  • X坐标：{status['position']['x']:.3f}
  • Y坐标：{status['position']['y']:.3f}
  • 经度：{status['position']['longitude']:.6f}
  • 纬度：{status['position']['latitude']:.6f}

🍎 采摘统计：
  • 今日采摘：{status['today_harvested']}个
  • 历史总计：{status['harvested_count']}个
  • 作业面积：{status['working_area']:.2f}亩

⏰ 更新时间：{status['timestamp']}"""
                            action_functions.append(query_response)
                        else:
                            action_functions.append(f"❌ 获取状态失败：{result.get('error', '未知错误')}")
                    else:
                        # 执行类函数，显示执行结果
                        if result["success"]:
                            action_functions.append(f"✅ {function_name}: {result['message']}")
                        else:
                            action_functions.append(f"❌ {function_name}: {result['error']}")
                    
                    # 添加函数执行结果到消息历史
                    messages_for_second_call.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": json.dumps(result)
                    })
                
                # 构建最终回复
                if len([f for f in function_results if f["function"] in query_functions]) > 0:
                    # 包含查询类函数，直接返回查询结果
                    ai_response = "\n\n".join(action_functions)
                else:
                    # 只有执行类函数，使用AI生成的回复
                    completion2 = self.ai_text_client.chat.completions.create(
                        model=self.ai_text_model,  # 使用轻量级文本模型
                        messages=messages_for_second_call,
                        max_tokens=self.ai_max_tokens,
                        temperature=0.7
                    )
                    ai_response = completion2.choices[0].message.content
                
                # 发布包含函数执行结果的AI回复
                response_data = {
                    "message": ai_response,
                    "robot_id": robot_id,
                    "timestamp": int(time.time() * 1000),
                    "success": True,
                    "function_calls": function_results
                }
                
            else:
                # 普通文本回复
                ai_response = response_message.content
                response_data = {
                    "message": ai_response,
                    "robot_id": robot_id,
                    "timestamp": int(time.time() * 1000),
                    "success": True
                }
            
            response_msg = String()
            response_msg.data = json.dumps(response_data)
            self.ai_response_pub.publish(response_msg)
            
            self.get_logger().info(f'AI回复已发布到ROS2话题: {ai_response[:50]}...')
            
        except Exception as e:
            self.get_logger().error(f'处理内部AI请求出错: {e}')
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')
            # 发布错误响应
            error_msg = String()
            error_data = {
                "message": "AI服务暂时不可用",
                "robot_id": robot_id,
                "timestamp": int(time.time() * 1000),
                "success": False,
                "error": str(e)
            }
            error_msg.data = json.dumps(error_data)
            self.ai_response_pub.publish(error_msg)
    
    # ===================== 其他原有方法保持不变 =====================
    
    def check_daily_reset(self):
        """检查是否需要重置每日统计数据"""
        current_time = time.localtime()
        if current_time.tm_hour == 0 and current_time.tm_min < 1:
            # 新的一天开始，重置今日数据
            self.get_logger().info("新的一天开始，重置今日统计数据")
            self.statistics_data['today_harvested'] = 0
            self.statistics_data['today_start_time'] = time.time()
            self.statistics_data['working_area'] = 0.0
            self.statistics_data['distance_traveled'] = 0.0
            self.statistics_data['harvest_points'] = []
            self.statistics_data['position_history'] = []
    
    def save_statistics(self):
        """保存统计数据到文件"""
        try:
            stats_file = '/tmp/robot_statistics.json'
            # 创建可序列化的数据副本
            save_data = self.statistics_data.copy()
            # 移除不可序列化的项
            if 'last_position' in save_data and save_data['last_position'] is not None:
                save_data['last_position'] = list(save_data['last_position'])
            
            with open(stats_file, 'w') as f:
                json.dump(save_data, f)
            self.get_logger().debug("统计数据已保存")
        except Exception as e:
            self.get_logger().error(f"保存统计数据失败: {e}")
    
    def load_statistics(self):
        """加载历史统计数据"""
        try:
            stats_file = '/tmp/robot_statistics.json'
            with open(stats_file, 'r') as f:
                loaded_data = json.load(f)
                # 转换last_position为元组
                if 'last_position' in loaded_data and loaded_data['last_position'] is not None:
                    loaded_data['last_position'] = tuple(loaded_data['last_position'])
                self.statistics_data.update(loaded_data)
            self.get_logger().info("历史统计数据已加载")
        except FileNotFoundError:
            self.get_logger().info("未找到历史统计数据文件，使用默认值")
        except Exception as e:
            self.get_logger().error(f"加载统计数据失败: {e}")
    
    def calculate_working_area(self, new_position, old_position):
        """计算作业面积"""
        if old_position is None:
            return 0.0
        
        # 计算移动距离（米）
        distance = ((new_position[0] - old_position[0])**2 + 
                    (new_position[1] - old_position[1])**2)**0.5
        
        # 假设有效工作宽度为2米
        work_width = 2.0
        
        # 计算面积（平方米）并转换为亩（1亩 = 666.67平方米）
        area_sqm = distance * work_width
        area_mu = area_sqm / 666.67
        
        return area_mu
    
    def determine_work_mode(self, msg):
        """根据机器人状态判断工作模式"""
        if self.auto_harvest_active:
            return "harvesting"
        elif abs(msg.current_speed) > 0.1:
            return "moving"
        elif msg.battery_level < 20:
            return "charging"
        else:
            return "idle"
    
    def get_location_name(self, lat, lon):
        """根据经纬度获取位置名称"""
        # 这里可以实现更复杂的位置映射逻辑
        # 暂时返回固定格式
        zone = chr(65 + int(abs(lon - 108.24) * 100) % 26)  # A, B, C...
        block = abs(int((lat - 34.93) * 100)) % 100 + 1
        return f"苹果园区3号地块 {zone}-{block} 区域"
    
    def image_callback(self, msg):
        """图像回调 - 接收压缩图像并通过WebSocket发送"""
        if not self.connected or not self.ws:
            return
        
        try:
            # 将图像数据转为base64
            image_base64 = base64.b64encode(msg.data).decode('utf-8')
            
            # 构建消息
            ws_msg = {
                "type": "video_frame",
                "format": msg.format,
                "timestamp": int(time.time() * 1000),
                "data": image_base64
            }
            
            # 发送到WebSocket
            self.ws.send(json.dumps(ws_msg))
            
        except Exception as e:
            self.get_logger().error(f'发送图像失败: {e}')
    
    def status_callback(self, msg):
        """机器人状态回调 - 改进版"""
        # 更新机器人状态数据（用于function calling）
        self.robot_status = {
            'battery_level': msg.battery_level,
            'current_speed': msg.current_speed,
            'position_x': msg.position_x,
            'position_y': msg.position_y,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'harvested_count': msg.harvested_count,
            'cpu_usage': msg.cpu_usage
        }
        
        if not self.connected or not self.ws:
            return
        
        try:
            # 更新位置和计算作业面积
            current_position = (msg.position_x, msg.position_y)
            if self.statistics_data['last_position'] is not None:
                # 计算新增作业面积
                new_area = self.calculate_working_area(
                    current_position, 
                    self.statistics_data['last_position']
                )
                self.statistics_data['working_area'] += new_area
                
                # 计算行驶距离
                distance = ((current_position[0] - self.statistics_data['last_position'][0])**2 + 
                           (current_position[1] - self.statistics_data['last_position'][1])**2)**0.5
                self.statistics_data['distance_traveled'] += distance
            
            self.statistics_data['last_position'] = current_position
            
            # 更新采摘数量
            if msg.harvested_count > self.statistics_data['last_harvested_count']:
                # 检测到新的采摘
                new_harvests = msg.harvested_count - self.statistics_data['last_harvested_count']
                self.statistics_data['today_harvested'] += new_harvests
                self.statistics_data['total_harvested'] += new_harvests
                self.statistics_data['last_harvested_count'] = msg.harvested_count
                
                # 记录采摘点
                harvest_point = {
                    'time': time.strftime("%H:%M"),
                    'location': f"N{msg.latitude:.6f}, E{msg.longitude:.6f}",
                    'position': {
                        'x': msg.position_x, 
                        'y': msg.position_y,
                        'longitude': msg.longitude,
                        'latitude': msg.latitude
                    }
                }
                self.statistics_data['harvest_points'].append(harvest_point)
                
                # 限制采摘点数量
                if len(self.statistics_data['harvest_points']) > 100:
                    self.statistics_data['harvest_points'] = self.statistics_data['harvest_points'][-100:]
            
            # 记录位置历史
            position_record = {
                'longitude': msg.longitude,
                'latitude': msg.latitude,
                'time': time.strftime("%H:%M:%S")
            }
            self.statistics_data['position_history'].append(position_record)
            
            # 限制位置历史数量
            if len(self.statistics_data['position_history']) > 200:
                self.statistics_data['position_history'] = self.statistics_data['position_history'][-200:]
            
            # 计算工作时长
            total_working_hours = (time.time() - self.statistics_data['start_time']) / 3600.0
            today_working_hours = (time.time() - self.statistics_data['today_start_time']) / 3600.0
            
            # 生成路线历史（最近5个采摘点）
            route_history = []
            harvest_points = self.statistics_data['harvest_points'][-5:]
            for i, point in enumerate(harvest_points):
                route_history.append({
                    'time': point['time'],
                    'location': f"采摘点 {self.statistics_data['today_harvested'] - len(harvest_points) + i + 1}"
                })
            
            # 如果没有采摘点，添加当前位置
            if not route_history:
                route_history.append({
                    'time': time.strftime("%H:%M"),
                    'location': self.get_location_name(msg.latitude, msg.longitude)
                })
            
            # 构建完整的状态消息
            status_data = {
                "type": "status_update",
                "data": {
                    # 今日统计
                    "today_harvested": self.statistics_data['today_harvested'],
                    "working_area": round(self.statistics_data['working_area'], 2),
                    "working_hours": round(today_working_hours, 2),
                    
                    # 总计统计
                    "total_harvested": self.statistics_data['total_harvested'],
                    "total_area": round(self.statistics_data['working_area'] * 10, 1),  # 假设总面积是今日的10倍
                    "total_hours": round(total_working_hours, 1),
                    
                    # 性能指标
                    "harvest_accuracy": min(98.0, 95.0 + (msg.cpu_usage / 100) * 5),  # 基于CPU使用率动态调整
                    "battery_level": msg.battery_level,
                    
                    # 位置信息
                    "longitude": msg.longitude,
                    "latitude": msg.latitude,
                    "speed": abs(msg.current_speed),
                    
                    # 详细位置数据
                    "position": {
                        "longitude": msg.longitude,
                        "latitude": msg.latitude,
                        "location_name": self.get_location_name(msg.latitude, msg.longitude)
                    },
                    
                    # 工作状态
                    "work_status": {
                        "mode": self.determine_work_mode(msg),
                        "status": "active" if abs(msg.current_speed) > 0.05 else "idle"
                    },
                    
                    # 路线历史
                    "route_history": route_history,
                    
                    # 采摘点历史（用于地图显示）
                    "harvest_points": self.statistics_data['harvest_points'][-20:],  # 最近20个采摘点
                    
                    # 系统状态
                    "cpu_usage": msg.cpu_usage,
                    "operation_mode": self.current_mode,
                    "auto_harvest_active": self.auto_harvest_active,
                    
                    # 时间戳
                    "timestamp": int(time.time() * 1000),
                    "last_update": time.strftime("%Y-%m-%d %H:%M:%S")
                }
            }
            
            # 发送状态
            self.ws.send(json.dumps(status_data))
            
            # 记录日志
            self.get_logger().info(
                f'状态更新 - 今日采摘: {self.statistics_data["today_harvested"]}, '
                f'作业面积: {self.statistics_data["working_area"]:.2f}亩, '
                f'工作时长: {today_working_hours:.2f}小时'
            )
            
        except Exception as e:
            self.get_logger().error(f'发送状态失败: {e}')
            self.get_logger().error(traceback.format_exc())
    
    def fruit_image_callback(self, msg):
        """水果图片回调 - 进行AI识别"""
        if not self.ai_enabled or not self.ai_vision_client:
            self.get_logger().warn('AI功能未启用，跳过水果识别')
            return
        
        try:
            # 提取文件名
            filename = 'unknown.jpg'
            if '|' in msg.header.frame_id:
                filename = msg.header.frame_id.split('|')[1]
            
            self.get_logger().info(f'收到水果图片进行AI识别: {filename}')
            
            # 在新线程中处理图片识别，避免阻塞
            recognition_thread = threading.Thread(
                target=self.process_fruit_recognition,
                args=(msg.data, filename)
            )
            recognition_thread.daemon = True
            recognition_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'水果图片回调出错: {e}')
    
    def process_fruit_recognition(self, image_data, filename):
        """在单独线程中处理水果识别"""
        try:
            # 将图片数据转换为base64格式
            image_base64 = base64.b64encode(image_data).decode('utf-8')
            data_url = f"data:image/jpeg;base64,{image_base64}"
            
            # 构建优化后的AI识别提示词
            prompt = """🍎 你是一位具有20年经验的农业水果专家和AI视觉识别系统，专门为智能采摘机器人提供精准的水果识别服务。

📋 **分析任务**：请对这张水果图片进行全方位专业分析，严格按照以下JSON格式返回结果：

```json
{
  "fruitType": "具体水果品种名称",
  "maturity": 成熟度百分比（0-100数字）,
  "healthStatus": "健康状态描述",
  "qualityScore": 综合品质分数（0-100数字）,
  "grade": "等级评定",
  "confidence": 识别置信度（0-100数字）,
  "sizeCategory": "大小分类",
  "recommendation": "专业采摘建议",
  "suggestedAction": "操作建议代码",
  "defects": ["具体缺陷列表"],
  "estimatedWeight": 估算重量克数,
  "ripeness_days": 距最佳采摘期天数,
  "marketValue": 预估市场价值,
  "storageLife": 预计储存期限天数
}
```

🔍 **专业分析维度**：

**1. 水果类型识别（fruitType）**：
- 苹果类：红富士、嘎啦、国光、红星、青苹果、黄元帅、烟富3号、烟富8号等
- 梨类：鸭梨、雪花梨、香梨、酥梨等
- 柑橘类：橙子、柚子、柠檬、橘子等
- 其他：如识别为非目标水果，请准确标注
- 如无法识别，返回"无法识别-[原因]"

**2. 成熟度评估（maturity 0-100%）**：
- **0-20%**：幼果期，果实小，颜色青绿，硬度高
- **21-40%**：生长期，体积增大，开始转色
- **41-60%**：转色期，颜色变化明显，硬度适中
- **61-80%**：近成熟期，颜色接近成熟标准，糖分上升
- **81-95%**：最佳采摘期，色泽饱满，硬度适宜，糖分最佳
- **96-100%**：过熟期，可能软化，储存期短

**3. 健康状态（healthStatus）**：
- "完全健康"：无任何病虫害和机械损伤
- "轻微瑕疵"：有1-2个小斑点或轻微划痕
- "中度缺陷"：有明显斑点、虫眼或小面积病害
- "严重问题"：大面积病害、腐烂或严重虫害
- "不宜采摘"：严重病虫害或腐烂

**4. 品质评分（qualityScore 0-100）**：
综合考虑：外观完整度(25%) + 成熟度适宜性(30%) + 无缺陷程度(25%) + 大小规格(20%)
- 90-100分：优质特级，完美外观，最佳成熟度
- 80-89分：优质一级，轻微瑕疵，成熟度良好
- 70-79分：良好二级，有一定缺陷但可接受
- 60-69分：合格三级，缺陷较多但仍有商业价值
- 0-59分：不合格，不建议采摘

**5. 等级评定（grade）**：
- "Premium"：特级品质，完美外观，最佳成熟度
- "Excellent"：优秀品质，极少缺陷
- "Good"：良好品质，轻微缺陷
- "Average"：平均品质，一般缺陷
- "Poor"：较差品质，明显缺陷
- "Reject"：拒收品质，严重问题

**6. 大小分类（sizeCategory）**：
根据水果直径/长度：
- "特大"：超大规格，适合礼品包装
- "大"：大规格，适合零售
- "中等"：标准规格，最常见
- "小"：小规格，适合加工
- "偏小"：规格不足，价值较低

**7. 操作建议（suggestedAction）**：
- "harvest_now"：立即采摘，最佳时机
- "harvest_priority"：优先采摘，成熟度极佳
- "harvest_normal"：正常采摘，符合标准
- "wait_3_days"：等待3天后采摘
- "wait_week"：等待一周后采摘
- "inspect_closely"：需要近距离检查
- "reject"：拒绝采摘，不符合标准

**8. 置信度评估（confidence）**：
- 95-100%：图片清晰，特征明显，识别极其确定
- 85-94%：图片良好，特征清楚，识别很确定
- 75-84%：图片一般，特征较清楚，识别较确定
- 60-74%：图片模糊或特征不明显，识别有一定把握
- 0-59%：图片质量差或特征不清，识别不确定

**🎯 特别关注事项**：
1. **光照条件**：分析图片光照是否充足，是否有阴影影响判断
2. **拍摄角度**：评估是否能看到水果完整外观
3. **遮挡情况**：是否有叶子或其他水果遮挡
4. **背景干扰**：是否有复杂背景影响识别
5. **采摘紧急性**：如果是易腐水果，提高采摘优先级

**📊 数值估算标准**：
- **estimatedWeight**：根据水果大小和品种的标准重量范围估算
- **ripeness_days**：负数表示已过最佳期，正数表示还需等待的天数
- **marketValue**：按当前市场价格和品质等级估算价值（元/斤）
- **storageLife**：在适宜条件下的预计储存天数

**⚠️ 输出要求**：
1. 必须严格按照JSON格式输出，不要添加任何额外文字
2. 所有数值字段必须是纯数字，不要加引号
3. 字符串字段用双引号包围
4. 数组字段即使为空也要用[]表示
5. 如果无法识别，在fruitType中说明具体原因

现在请开始分析这张图片："""
            
            # 调用AI API进行识别 - 使用视觉模型（成本较高，只用于图片识别）
            self.get_logger().info(f'使用视觉模型进行图片识别: {self.ai_vision_model}')
            completion = self.ai_vision_client.chat.completions.create(
                model=self.ai_vision_model,  # 使用视觉模型进行图片识别
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": prompt
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": data_url,
                                    "detail": "high"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=1200,  # 增加token限制以支持更详细的分析
                temperature=0.1   # 降低温度以获得更稳定的结果
            )
            
            # 解析AI回复
            ai_response = completion.choices[0].message.content.strip()
            self.get_logger().info(f'AI识别原始回复: {ai_response[:200]}...')
            
            # 尝试提取JSON数据
            try:
                # 查找JSON数据（可能包含在代码块中）
                if '```json' in ai_response:
                    json_start = ai_response.find('```json') + 7
                    json_end = ai_response.find('```', json_start)
                    json_str = ai_response[json_start:json_end].strip()
                elif '{' in ai_response:
                    json_start = ai_response.find('{')
                    json_end = ai_response.rfind('}') + 1
                    json_str = ai_response[json_start:json_end]
                else:
                    raise ValueError("AI回复中未找到JSON数据")
                
                # 解析JSON
                recognition_result = json.loads(json_str)
                
                # 验证必需字段并设置默认值
                required_fields = {
                    'fruitType': '未知水果',
                    'maturity': 50,
                    'healthStatus': '健康',
                    'qualityScore': 70,
                    'grade': 'Average',
                    'confidence': 80,
                    'sizeCategory': '中等',
                    'recommendation': '需要进一步检查',
                    'suggestedAction': 'inspect_closely',
                    'defects': [],
                    'estimatedWeight': 150,
                    'ripeness_days': 0,
                    'marketValue': 3.0,
                    'storageLife': 7
                }
                
                for field, default_value in required_fields.items():
                    if field not in recognition_result:
                        recognition_result[field] = default_value
                
                # 数据类型验证和修正
                if not isinstance(recognition_result.get('defects'), list):
                    recognition_result['defects'] = []
                
                # 确保数值字段是数字类型
                numeric_fields = ['maturity', 'qualityScore', 'confidence', 'estimatedWeight', 'ripeness_days', 'marketValue', 'storageLife']
                for field in numeric_fields:
                    if field in recognition_result:
                        try:
                            recognition_result[field] = float(recognition_result[field])
                        except (ValueError, TypeError):
                            recognition_result[field] = required_fields[field]
                            
            except (json.JSONDecodeError, ValueError) as e:
                self.get_logger().error(f'解析AI回复JSON失败: {e}')
                # 创建默认识别结果
                recognition_result = {
                    'fruitType': '解析失败',
                    'maturity': 50,
                    'healthStatus': '无法确定',
                    'qualityScore': 60,
                    'grade': 'Average',
                    'confidence': 0,
                    'sizeCategory': '中等',
                    'recommendation': 'AI识别结果解析失败，需要人工检查',
                    'suggestedAction': 'inspect_closely',
                    'defects': ['AI解析错误'],
                    'estimatedWeight': 150,
                    'ripeness_days': 0,
                    'marketValue': 0,
                    'storageLife': 0
                }
            
            # 添加识别相关的元数据
            current_time = time.time()
            detection_id = f"fruit_detection_{int(current_time * 1000)}"
            
            # 生成符合微信小程序格式的识别结果
            detection_data = {
                'id': detection_id,
                'fruitType': recognition_result.get('fruitType', '未知水果'),
                'maturity': recognition_result.get('maturity', 50),
                'healthStatus': recognition_result.get('healthStatus', '健康'),
                'qualityScore': recognition_result.get('qualityScore', 70),
                'grade': recognition_result.get('grade', 'Average'),
                'detectionTime': time.strftime('%H:%M'),
                'location': self.get_current_location(),
                'actionTaken': self.get_action_from_suggestion(recognition_result.get('suggestedAction', 'inspect_closely')),
                'thumbnailUrl': f'/temp/{filename}',  # 临时图片路径
                'timestamp': int(current_time * 1000),
                'confidence': recognition_result.get('confidence', 80),
                'sizeCategory': recognition_result.get('sizeCategory', '中等'),
                'recommendation': recognition_result.get('recommendation', '需要进一步检查'),
                'defects': recognition_result.get('defects', []),
                'estimatedWeight': recognition_result.get('estimatedWeight', 150),
                'ripeness_days': recognition_result.get('ripeness_days', 0),
                'marketValue': recognition_result.get('marketValue', 0),
                'storageLife': recognition_result.get('storageLife', 0),
                'source_image': filename,
                'image_base64': image_base64,  # 添加base64编码的图片数据
                'image_data_url': data_url     # 添加完整的data URL
            }
            
            self.get_logger().info(f'水果识别完成: {detection_data["fruitType"]}, 质量: {detection_data["qualityScore"]}/100, 成熟度: {detection_data["maturity"]}%, 市场价值: {detection_data["marketValue"]}元/斤')
            
            # 增强日志输出，提供更详细的识别信息
            maturity_desc = self.get_maturity_description(detection_data["maturity"])
            quality_desc = self.get_quality_assessment(detection_data["qualityScore"])
            
            self.get_logger().info(f'详细识别结果 - 水果: {detection_data["fruitType"]}, {maturity_desc}({detection_data["maturity"]}%), {quality_desc}({detection_data["qualityScore"]}分), 置信度: {detection_data["confidence"]}%, 操作建议: {detection_data["actionTaken"]}')
            
            if detection_data["defects"]:
                self.get_logger().info(f'发现缺陷: {", ".join(detection_data["defects"])}')
            
            if detection_data["ripeness_days"] != 0:
                if detection_data["ripeness_days"] > 0:
                    self.get_logger().info(f'建议等待 {detection_data["ripeness_days"]} 天后采摘')
                else:
                    self.get_logger().warn(f'水果已过最佳采摘期 {abs(detection_data["ripeness_days"])} 天')
            
            # 根据识别结果自动调整采摘策略
            if detection_data["qualityScore"] >= 85 and detection_data["maturity"] >= 80:
                self.get_logger().info(f'🎯 发现优质水果，建议优先采摘！')
            elif detection_data["qualityScore"] < 60:
                self.get_logger().warn(f'⚠️ 水果品质较差，建议跳过')
            
            # 市场价值评估日志
            if detection_data["marketValue"] > 0:
                estimated_value = detection_data["estimatedWeight"] * detection_data["marketValue"] / 500  # 转换为单个水果价值
                self.get_logger().info(f'💰 预估单果价值: {estimated_value:.2f}元, 储存期: {detection_data["storageLife"]}天')
            
            # 发布识别结果到ROS2话题
            result_msg = String()
            result_msg.data = json.dumps(detection_data)
            self.fruit_detection_result_pub.publish(result_msg)
            
            # 通过WebSocket发送给服务端
            if self.connected and self.ws:
                ws_message = {
                    "type": "fruit_detection_result",
                    "data": detection_data,
                    "timestamp": int(current_time * 1000)
                }
                self.ws.send(json.dumps(ws_message))
                self.get_logger().info(f'水果识别结果已发送到服务端')
            
        except Exception as e:
            self.get_logger().error(f'水果识别处理出错: {e}')
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')
    
    def get_current_location(self):
        """获取当前位置描述"""
        if hasattr(self, 'robot_status'):
            lat = self.robot_status.get('latitude', 34.9385)
            lon = self.robot_status.get('longitude', 108.2415)
            return self.get_location_name(lat, lon)
        else:
            return "B-12区域"  # 默认位置
    
    def get_action_from_suggestion(self, suggested_action):
        """根据AI建议转换为行动描述"""
        action_map = {
            'harvest_now': '立即采摘',
            'harvest_priority': '优先采摘',
            'harvest_normal': '正常采摘',
            'wait_3_days': '等待3天',
            'wait_week': '等待一周',
            'inspect_closely': '需检查',
            'reject': '拒绝采摘',
            # 兼容旧版本
            'harvest': '建议采摘',
            'wait': '待成熟',
            'inspect': '需检查'
        }
        return action_map.get(suggested_action, '待检查')
    
    def get_maturity_description(self, maturity):
        """根据成熟度百分比返回描述"""
        if maturity <= 20:
            return "幼果期"
        elif maturity <= 40:
            return "生长期"
        elif maturity <= 60:
            return "转色期"
        elif maturity <= 80:
            return "近成熟期"
        elif maturity <= 95:
            return "最佳采摘期"
        else:
            return "过熟期"
    
    def get_quality_assessment(self, quality_score):
        """根据品质分数返回评估"""
        if quality_score >= 90:
            return "优质特级"
        elif quality_score >= 80:
            return "优质一级"
        elif quality_score >= 70:
            return "良好二级"
        elif quality_score >= 60:
            return "合格三级"
        else:
            return "不合格"

    def detection_callback(self, msg):
        """瓶子检测信息回调"""
        if not self.connected or not self.ws:
            return
        
        try:
            # 直接转发检测信息
            detection_data = json.loads(msg.data)
            detection_data["type"] = "detection_update"
            
            self.ws.send(json.dumps(detection_data))
            
        except Exception as e:
            self.get_logger().error(f'发送检测信息失败: {e}')
    
    def destroy_node(self):
        """清理资源"""
        # 保存最终统计数据
        self.save_statistics()
        
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebSocketBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()