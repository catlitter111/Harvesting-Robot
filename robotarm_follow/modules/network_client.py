#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
网络通信客户端模块：负责与服务器建立WebSocket连接
"""

import cv2
import base64
import json
import websocket
import threading
import time
import numpy as np
import psutil
import logging
import queue
from typing import Dict, Any, Optional

# 质量预设配置
QUALITY_PRESETS = {
    "high": {
        "resolution": (640, 480),
        "fps": 15,
        "bitrate": 800,  # Kbps
        "quality": 80  # JPEG质量(1-100)
    },
    "medium": {
        "resolution": (480, 360),
        "fps": 10,
        "bitrate": 500,
        "quality": 70
    },
    "low": {
        "resolution": (320, 240),
        "fps": 8,
        "bitrate": 300,
        "quality": 60
    },
    "very_low": {
        "resolution": (240, 180),
        "fps": 5,
        "bitrate": 150,
        "quality": 50
    },
    "minimum": {
        "resolution": (160, 120),
        "fps": 3,
        "bitrate": 80,
        "quality": 40
    }
}

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04

logger = logging.getLogger("网络客户端")

class NetworkClient:
    """WebSocket网络客户端，负责与远程服务器通信"""
    
    def __init__(self, server_url: str, initial_preset: str = "medium", robot_id: str = None):
        """
        初始化网络客户端
        
        Args:
            server_url: WebSocket服务器URL基础地址
            initial_preset: 初始视频质量预设
            robot_id: 机器人ID，如果为None则使用默认ID
        """
        # WebSocket连接配置
        self.robot_id = robot_id or "robot_123"
        self.server_url = f"{server_url}/ws/robot/{self.robot_id}"
        self.ws = None
        self.connected = False
        self.reconnect_count = 0
        self.max_reconnect_attempts = 5
        self.reconnect_interval = 3  # 重连间隔(秒)
        self.running = True
        
        # 视频配置
        self.current_preset = initial_preset
        self.current_config = QUALITY_PRESETS[initial_preset]
        self.frame_queue = queue.Queue(maxsize=10)  # 帧缓冲队列
        
        # 机器人状态
        self.robot_status = {
            "battery_level": 85,
            "position": {"x": 0, "y": 0, "latitude": 0.0, "longitude": 0.0},
            "harvested_count": 0,
            "cpu_usage": 0,
            "signal_strength": 70,
            "upload_bandwidth": 1000,  # 初始估计值(Kbps)
            "frames_sent": 0,
            "bytes_sent": 0,
            "last_bandwidth_check": time.time(),
            "last_bytes_sent": 0,
            "current_speed": 50,  # 默认速度为50%
            "current_direction": DIR_STOP  # 默认方向为停止
        }
        
        # 回调函数
        self.command_callback = None  # 接收到命令时的回调
        
        # 线程列表
        self.threads = []
    
    def set_command_callback(self, callback):
        """设置接收到命令时的回调函数"""
        self.command_callback = callback
    
    def on_message(self, ws, message):
        """处理接收到的WebSocket消息"""
        try:
            data = json.loads(message)
            message_type = data.get("type")
            
            if message_type == "command":
                # 调用命令回调函数
                if self.command_callback:
                    self.command_callback(data)
            elif message_type == "quality_adjustment":
                self.handle_quality_adjustment(data)
            
        except json.JSONDecodeError:
            logger.error(f"收到无效JSON: {message}")
        except Exception as e:
            logger.error(f"处理消息错误: {e}")
    
    def on_error(self, ws, error):
        """处理WebSocket错误"""
        logger.error(f"WebSocket错误: {error}")
    
    def on_close(self, ws, close_status_code, close_msg):
        """处理WebSocket连接关闭"""
        self.connected = False
        logger.warning(f"WebSocket连接关闭: {close_status_code} {close_msg}")
        self.schedule_reconnect()
    
    def on_open(self, ws):
        """处理WebSocket连接建立"""
        self.connected = True
        self.reconnect_count = 0
        logger.info("WebSocket连接已建立")
        
        # 发送初始状态
        self.send_status_update()
    
    def schedule_reconnect(self):
        """计划重新连接"""
        if not self.running:
            return
        
        if self.reconnect_count < self.max_reconnect_attempts:
            self.reconnect_count += 1
            logger.info(f"计划第 {self.reconnect_count} 次重连，{self.reconnect_interval}秒后尝试...")
            
            time.sleep(self.reconnect_interval)
            
            if not self.connected and self.running:
                self.connect()
        else:
            logger.error(f"达到最大重连次数({self.max_reconnect_attempts})，停止重连")
    
    def connect(self):
        """连接到WebSocket服务器"""
        # 创建WebSocket连接
        self.ws = websocket.WebSocketApp(self.server_url,
                                    on_open=self.on_open,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)
        
        # 在新线程中运行WebSocket连接
        ws_thread = threading.Thread(target=self.ws.run_forever)
        ws_thread.daemon = True
        ws_thread.start()
        self.threads.append(ws_thread)
    
    def send_video_frame(self, frame):
        """发送视频帧到服务器"""
        if not self.ws or not self.connected:
            return False
        
        try:
            # 调整大小以匹配当前分辨率设置
            if frame.shape[1] != self.current_config["resolution"][0] or frame.shape[0] != self.current_config["resolution"][1]:
                frame = cv2.resize(frame, self.current_config["resolution"])
            
            # 压缩帧为JPEG格式
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.current_config["quality"]]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            
            # 转为Base64编码
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            # 构建消息
            message = {
                "type": "video_frame",
                "preset": self.current_preset,
                "resolution": f"{self.current_config['resolution'][0]}x{self.current_config['resolution'][1]}",
                "timestamp": int(time.time() * 1000),  # 毫秒时间戳
                "data": jpg_as_text
            }
            
            # 发送视频帧
            message_json = json.dumps(message)
            self.ws.send(message_json)
            
            # 更新统计信息
            self.robot_status["frames_sent"] += 1
            self.robot_status["bytes_sent"] += len(message_json)
            
            return True
        except Exception as e:
            logger.error(f"发送视频帧错误: {e}")
            return False
    
    def send_status_update(self):
        """发送状态更新到服务器"""
        if not self.ws or not self.connected:
            return False
        
        try:
            # 获取CPU使用率
            self.robot_status["cpu_usage"] = psutil.cpu_percent(interval=0.1)
            
            # 发送状态信息
            self.ws.send(json.dumps({
                "type": "status_update",
                "data": {
                    "battery_level": self.robot_status["battery_level"],
                    "cpu_usage": self.robot_status["cpu_usage"],
                    "signal_strength": self.robot_status["signal_strength"],
                    "upload_bandwidth": self.robot_status["upload_bandwidth"],
                    "frames_sent": self.robot_status["frames_sent"],
                    "bytes_sent": self.robot_status["bytes_sent"],
                    "current_preset": self.current_preset,
                    "current_speed": self.robot_status["current_speed"],
                    "current_direction": self.robot_status["current_direction"],
                    "position": self.robot_status["position"]
                }
            }))
            return True
        except Exception as e:
            logger.error(f"发送状态更新失败: {e}")
            return False
    
    def send_position_update(self, latitude, longitude):
        """发送位置更新到服务器"""
        if not self.ws or not self.connected:
            return False
        
        try:
            # 更新本地位置信息
            self.robot_status["position"]["latitude"] = latitude
            self.robot_status["position"]["longitude"] = longitude
            
            # 发送位置信息
            self.ws.send(json.dumps({
                "type": "position_update",
                "data": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "timestamp": int(time.time() * 1000)
                }
            }))
            return True
        except Exception as e:
            logger.error(f"发送位置更新失败: {e}")
            return False
    
    def handle_quality_adjustment(self, adjustment_data):
        """处理质量调整命令"""
        preset = adjustment_data.get("preset")
        
        if preset in QUALITY_PRESETS:
            logger.info(f"收到质量调整命令: {preset}")
            
            # 更新当前质量设置
            self.current_preset = preset
            self.current_config = QUALITY_PRESETS[preset]
            
            # 发送调整结果
            if self.ws and self.connected:
                try:
                    self.ws.send(json.dumps({
                        "type": "quality_adjustment_result",
                        "success": True,
                        "preset": preset,
                        "actual_resolution": f"{self.current_config['resolution'][0]}x{self.current_config['resolution'][1]}",
                        "actual_fps": self.current_config["fps"]
                    }))
                except Exception as e:
                    logger.error(f"发送质量调整结果失败: {e}")
            
            return True
        else:
            logger.error(f"未知的质量预设: {preset}")
            return False
    
    def video_sending_thread_func(self):
        """视频发送线程函数"""
        last_frame_time = time.time()
        frame_interval = 1.0 / self.current_config["fps"]
        
        # 用于计算实际FPS
        fps_counter = 0
        fps_timer = time.time()
        
        while self.running:
            try:
                # 控制发送频率
                current_time = time.time()
                elapsed = current_time - last_frame_time
                
                # 根据当前预设更新帧间隔
                frame_interval = 1.0 / self.current_config["fps"]
                
                if elapsed < frame_interval:
                    time.sleep(0.001)  # 短暂休眠，减少CPU使用
                    continue
                
                # 尝试从队列获取一帧
                try:
                    frame = self.frame_queue.get(block=False)
                except queue.Empty:
                    time.sleep(0.01)
                    continue
                
                # 发送帧
                if self.send_video_frame(frame):
                    # 更新最后发送时间
                    last_frame_time = time.time()
                    
                    # 计算实际FPS
                    fps_counter += 1
                    if current_time - fps_timer >= 1.0:  # 每秒计算一次
                        fps_counter = 0
                        fps_timer = current_time
                
            except Exception as e:
                logger.error(f"视频发送线程错误: {e}")
                time.sleep(0.1)
    
    def status_update_thread_func(self):
        """状态更新线程函数"""
        last_status_time = 0
        status_interval = 5  # 每5秒发送一次状态
        
        while self.running:
            try:
                current_time = time.time()
                
                # 计算上传带宽
                bytes_sent_diff = self.robot_status["bytes_sent"] - self.robot_status["last_bytes_sent"]
                time_diff = current_time - self.robot_status["last_bandwidth_check"]
                
                if time_diff > 0:
                    # 计算Kbps
                    upload_speed = (bytes_sent_diff * 8) / (time_diff * 1000)
                    # 平滑带宽估计
                    self.robot_status["upload_bandwidth"] = (self.robot_status["upload_bandwidth"] * 0.7) + (upload_speed * 0.3)
                    
                    # 更新检查点
                    self.robot_status["last_bandwidth_check"] = current_time
                    self.robot_status["last_bytes_sent"] = self.robot_status["bytes_sent"]
                
                # 定期发送状态更新
                if current_time - last_status_time >= status_interval:
                    self.send_status_update()
                    last_status_time = current_time
                
            except Exception as e:
                logger.error(f"状态更新错误: {e}")
            
            time.sleep(1)
    
    def add_frame_to_queue(self, frame):
        """添加帧到发送队列"""
        try:
            # 非阻塞方式，如果队列满了就丢弃帧
            if not self.frame_queue.full():
                self.frame_queue.put_nowait(frame)
                return True
            return False
        except Exception as e:
            logger.error(f"添加帧到队列失败: {e}")
            return False
    
    def start(self):
        """启动网络客户端"""
        # 连接到服务器
        self.connect()
        
        # 启动状态更新线程
        status_thread = threading.Thread(target=self.status_update_thread_func)
        status_thread.daemon = True
        status_thread.start()
        self.threads.append(status_thread)
        
        # 启动视频发送线程
        video_thread = threading.Thread(target=self.video_sending_thread_func)
        video_thread.daemon = True
        video_thread.start()
        self.threads.append(video_thread)
        
        logger.info("网络客户端已启动")
        return True
    
    def stop(self):
        """停止网络客户端"""
        self.running = False
        
        # 关闭WebSocket连接
        if self.ws:
            self.ws.close()
        
        logger.info("网络客户端已停止") 