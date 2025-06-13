#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket桥接节点
负责连接WebSocket服务器，接收控制命令并转发到ROS2系统
同时将视频流和状态信息发送到服务器
现在集成了真实的AI对话功能
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
        self.declare_parameter('ai_api_key', 'sk-1b880a05df7249d3927443d4872e2839oklzor2ja52wf1eu')
        self.declare_parameter('ai_model', 'doubao-1.5-lite-32k')
        self.declare_parameter('ai_max_tokens', 300)
        
        # 获取参数
        self.server_url = self.get_parameter('server_url').value
        self.max_reconnect_attempts = self.get_parameter('reconnect_attempts').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.robot_id = self.get_parameter('robot_id').value
        
        # AI参数
        self.ai_enabled = self.get_parameter('ai_enabled').value
        self.ai_base_url = self.get_parameter('ai_base_url').value
        self.ai_api_key = self.get_parameter('ai_api_key').value
        self.ai_model = self.get_parameter('ai_model').value
        self.ai_max_tokens = self.get_parameter('ai_max_tokens').value
        
        # 初始化OpenAI客户端
        self.ai_client = None
        if self.ai_enabled:
            try:
                self.ai_client = OpenAI(
                    base_url=self.ai_base_url,
                    api_key=self.ai_api_key,
                )
                self.get_logger().info('AI客户端初始化成功')
            except Exception as e:
                self.get_logger().error(f'AI客户端初始化失败: {e}')
                self.ai_enabled = False
        
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
        
        # 消息队列
        self.image_queue = queue.Queue(maxsize=10)
        self.status_queue = queue.Queue(maxsize=10)
        
        # 状态变量
        self.current_mode = "manual"
        self.auto_harvest_active = False
        
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
        if not self.ai_enabled or not self.ai_client:
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
        """在单独线程中处理AI请求"""
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
            
            # 调用AI API
            completion = self.ai_client.chat.completions.create(
                model=self.ai_model,
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
            )
            
            # 提取AI回复
            ai_response = completion.choices[0].message.content
            
            self.get_logger().info(f'AI回复生成成功 - 客户端: {client_id}, 回复长度: {len(ai_response)}字符')
            
            # 发送成功响应
            response = {
                "type": "ai_chat_response",
                "success": True,
                "message": ai_response,
                "timestamp": timestamp,
                "client_id": client_id,
                "robot_id": robot_id
            }
            
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
                self.ws.send(json.dumps(response))
    
    def build_robot_context(self, robot_id):
        """构建包含机器人状态的上下文信息"""
        context = """你是AgriSage智能助手，专门为农业采摘机器人提供服务。你需要：
1. 基于当前机器人的实时状态数据回答用户问题
2. 提供专业、准确的农业采摘相关建议
3. 保持友好、专业的对话风格
4. 如果用户询问具体数据，优先使用实时状态信息

当前机器人状态信息：
"""
        
        # 添加实时机器人状态
        try:
            robot_data = self.statistics_data
            context += f"""
- 机器人ID: {robot_id}
- 工作模式: {self.current_mode}
- 自动采摘: {'启用' if self.auto_harvest_active else '禁用'}
- 今日采摘: {robot_data.get('today_harvested', 0)}个
- 总计采摘: {robot_data.get('total_harvested', 0)}个
- 作业面积: {robot_data.get('working_area', 0.0):.2f}亩
- 工作时长: {(time.time() - robot_data.get('today_start_time', time.time())) / 3600.0:.2f}小时
- 当前时间: {time.strftime('%Y-%m-%d %H:%M:%S')}
"""
            
            # 如果有位置信息
            if robot_data.get('last_position'):
                context += f"- 当前位置: 经度{robot_data['last_position'][0]:.6f}, 纬度{robot_data['last_position'][1]:.6f}\n"
            
            # 如果有采摘点历史
            if robot_data.get('harvest_points'):
                recent_harvest = len(robot_data['harvest_points'])
                context += f"- 最近采摘点数量: {recent_harvest}个\n"
            
        except Exception as e:
            self.get_logger().error(f'构建机器人上下文时出错: {e}')
        
        context += """
请基于以上信息回答用户问题，提供有用的建议和分析。
"""
        
        return context
    
    def ai_request_callback(self, msg):
        """AI请求回调函数 - 用于从其他ROS2节点接收AI请求"""
        try:
            data = json.loads(msg.data)
            user_message = data.get("message", "")
            robot_id = data.get("robot_id", self.robot_id)
            
            if not self.ai_enabled or not self.ai_client:
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
        """处理来自ROS2内部的AI请求"""
        try:
            # 构建上下文
            system_message = self.build_robot_context(robot_id)
            
            # 调用AI API
            completion = self.ai_client.chat.completions.create(
                model=self.ai_model,
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
            )
            
            # 提取AI回复
            ai_response = completion.choices[0].message.content
            
            # 发布AI回复到ROS2话题
            response_msg = String()
            response_data = {
                "message": ai_response,
                "robot_id": robot_id,
                "timestamp": int(time.time() * 1000),
                "success": True
            }
            response_msg.data = json.dumps(response_data)
            self.ai_response_pub.publish(response_msg)
            
            self.get_logger().info(f'AI回复已发布到ROS2话题: {ai_response[:50]}...')
            
        except Exception as e:
            self.get_logger().error(f'处理内部AI请求出错: {e}')
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