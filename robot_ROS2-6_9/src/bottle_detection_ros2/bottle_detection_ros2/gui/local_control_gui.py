 #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地控制界面GUI
基于用户设计的界面，实现机器人本地控制和状态监控
"""

import tkinter as tk
from tkinter import ttk, font
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32, Bool
from bottle_detection_msgs.msg import BottleDetection, RobotCommand, RobotStatus, HarvestCommand
import cv2
import numpy as np
from PIL import Image, ImageTk
import threading
import time
import traceback
import subprocess
import os
import json
from datetime import datetime

class LocalControlGUI(Node):
    """本地控制界面节点"""
    
    def __init__(self):
        try:
            super().__init__('local_control_gui')
            
            # 初始化tkinter
            self.root = tk.Tk()
            self.setup_window()
            
            # 数据存储
            self.current_image = None
            self.detection_data = {
                'bottle_count': 0,
                'nearest_distance': 0.0,
                'confidence': 0.0
            }
            self.robot_status_data = {
                'battery_level': 0.0,
                'cpu_usage': 0.0,
                'temperature': 28.0,
                'position_x': 0.0,
                'position_y': 0.0,
                'latitude': 34.938500,
                'longitude': 108.241500,
                'location_name': '苹果园3号地块',
                'harvested_count': 0,
                'today_harvested': 0,
                'total_harvested': 2543,
                'working_hours': 5.5,
                'signal_strength': 75,
                'harvest_accuracy': 96.5,
                'running_days': 45,
                'health_status': '优秀',
                'current_speed': 0.0,
                'is_moving': False,
                'is_harvesting': False
            }
            self.current_fps = 0.0
            self.system_running = False
            self.work_mode = "manual"  # manual/auto
            
            # 线程锁
            self.data_lock = threading.Lock()
            
            # 创建ROS2订阅者和发布者
            self.setup_ros_interfaces()
            
            # 创建GUI组件
            self.create_widgets()
            
            # 启动数据更新定时器
            self.update_timer()
            
            self.get_logger().info('本地控制界面GUI已启动')
            
        except Exception as e:
            self.get_logger().error(f'初始化GUI失败: {e}')
            traceback.print_exc()
    
    def setup_window(self):
        """设置窗口属性"""
        try:
            self.root.title("智慧农业采摘系统")
            self.root.geometry("1200x800")
            self.root.configure(bg='#E8F5E8')
            
            # 设置图标（如果存在）
            try:
                # self.root.iconbitmap('icon.ico')  # 可选图标
                pass
            except:
                pass
                
            # 字体设置
            self.title_font = font.Font(family="Microsoft YaHei", size=16, weight="bold")
            self.header_font = font.Font(family="Microsoft YaHei", size=12, weight="bold")
            self.normal_font = font.Font(family="Microsoft YaHei", size=10)
            self.small_font = font.Font(family="Microsoft YaHei", size=8)
            
        except Exception as e:
            print(f"设置窗口失败: {e}")
            traceback.print_exc()
    
    def setup_ros_interfaces(self):
        """设置ROS2接口"""
        try:
            # 创建QoS配置
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # 订阅者
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/camera/processed_image/compressed',
                self.image_callback,
                qos
            )
            
            self.detection_sub = self.create_subscription(
                BottleDetection,
                '/bottle_detection',
                self.detection_callback,
                10
            )
            
            self.status_sub = self.create_subscription(
                RobotStatus,
                '/robot_status',
                self.status_callback,
                10
            )
            
            self.fps_sub = self.create_subscription(
                Float32,
                '/camera/fps',
                self.fps_callback,
                qos
            )
            
            # 发布者
            self.robot_cmd_pub = self.create_publisher(
                RobotCommand,
                '/robot_command',
                10
            )
            
            self.harvest_cmd_pub = self.create_publisher(
                HarvestCommand,
                '/robot/harvest_command',
                10
            )
            
            self.mode_pub = self.create_publisher(
                String,
                '/work_mode',
                10
            )
            
        except Exception as e:
            self.get_logger().error(f'设置ROS接口失败: {e}')
            traceback.print_exc()
    
    def create_widgets(self):
        """创建GUI组件"""
        try:
            # 主标题栏
            self.create_title_bar()
            
            # 主内容区域
            main_frame = tk.Frame(self.root, bg='#E8F5E8')
            main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
            
            # 上半部分：控制中心 + 实时监控 + 系统状态
            top_frame = tk.Frame(main_frame, bg='#E8F5E8')
            top_frame.pack(fill=tk.BOTH, expand=True, pady=5)
            
            self.create_control_center(top_frame)
            self.create_monitoring_center(top_frame)
            self.create_system_status(top_frame)
            
            # 下半部分：农场数据总览
            self.create_farm_overview(main_frame)
            
        except Exception as e:
            self.get_logger().error(f'创建GUI组件失败: {e}')
            traceback.print_exc()
    
    def create_title_bar(self):
        """创建标题栏"""
        try:
            title_frame = tk.Frame(self.root, bg='#2E7D32', height=60)
            title_frame.pack(fill=tk.X)
            title_frame.pack_propagate(False)
            
            # 左侧图标和标题
            left_frame = tk.Frame(title_frame, bg='#2E7D32')
            left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=10)
            
            # 系统图标
            icon_label = tk.Label(left_frame, text="🌱", font=("Arial", 20), 
                                bg='#2E7D32', fg='white')
            icon_label.pack(side=tk.LEFT, padx=(0, 10))
            
            # 标题
            title_label = tk.Label(left_frame, text="智慧农业采摘系统", 
                                 font=self.title_font, bg='#2E7D32', fg='white')
            title_label.pack(side=tk.LEFT)
            
            subtitle_label = tk.Label(left_frame, text="Smart Agricultural Harvesting Robot", 
                                    font=self.small_font, bg='#2E7D32', fg='#C8E6C9')
            subtitle_label.pack(side=tk.LEFT, padx=(10, 0))
            
            # 右侧状态和时间
            right_frame = tk.Frame(title_frame, bg='#2E7D32')
            right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=15, pady=10)
            
            # 运行状态
            self.status_indicator = tk.Label(right_frame, text="● 运行中", 
                                           font=self.normal_font, bg='#2E7D32', fg='#4CAF50')
            self.status_indicator.pack(side=tk.RIGHT)
            
            # 时间显示
            self.time_label = tk.Label(right_frame, text="12:34", 
                                     font=self.header_font, bg='#2E7D32', fg='white')
            self.time_label.pack(side=tk.RIGHT, padx=(0, 20))
            
        except Exception as e:
            print(f"创建标题栏失败: {e}")
            traceback.print_exc()
    
    def create_control_center(self, parent):
        """创建控制中心"""
        try:
            # 控制中心框架
            control_frame = tk.LabelFrame(parent, text="🎮 控制中心", font=self.header_font,
                                        bg='#E8F5E8', fg='#2E7D32', bd=2, relief=tk.RAISED)
            control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
            control_frame.configure(width=200)
            control_frame.pack_propagate(False)
            
            # 系统运行控制
            system_frame = tk.Frame(control_frame, bg='#E8F5E8')
            system_frame.pack(fill=tk.X, padx=10, pady=10)
            
            # 系统运行开关
            switch_frame = tk.Frame(system_frame, bg='#E8F5E8')
            switch_frame.pack(fill=tk.X, pady=5)
            
            tk.Label(switch_frame, text="系统运行", font=self.normal_font,
                    bg='#E8F5E8').pack(side=tk.LEFT)
            
            self.system_switch = tk.Button(switch_frame, text="启动系统", 
                                         command=self.toggle_system,
                                         bg='#4CAF50', fg='white', font=self.normal_font,
                                         relief=tk.RAISED, bd=2)
            self.system_switch.pack(side=tk.RIGHT)
            
            # 工作模式
            mode_frame = tk.Frame(system_frame, bg='#E8F5E8')
            mode_frame.pack(fill=tk.X, pady=10)
            
            tk.Label(mode_frame, text="工作模式", font=self.normal_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            mode_buttons_frame = tk.Frame(mode_frame, bg='#E8F5E8')
            mode_buttons_frame.pack(fill=tk.X, pady=5)
            
            self.manual_btn = tk.Button(mode_buttons_frame, text="手动",
                                      command=lambda: self.set_work_mode("manual"),
                                      bg='#81C784', fg='white', font=self.normal_font,
                                      relief=tk.RAISED, bd=2)
            self.manual_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
            
            self.auto_btn = tk.Button(mode_buttons_frame, text="自动",
                                    command=lambda: self.set_work_mode("auto"),
                                    bg='#A5D6A7', fg='white', font=self.normal_font,
                                    relief=tk.RAISED, bd=2)
            self.auto_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))
            
            # 采摘控制
            harvest_frame = tk.Frame(system_frame, bg='#E8F5E8')
            harvest_frame.pack(fill=tk.X, pady=10)
            
            tk.Label(harvest_frame, text="采摘控制", font=self.normal_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            self.harvest_btn = tk.Button(harvest_frame, text="🍎 开始采摘",
                                       command=self.toggle_harvest,
                                       bg='#4CAF50', fg='white', font=self.normal_font,
                                       relief=tk.RAISED, bd=2)
            self.harvest_btn.pack(fill=tk.X, pady=5)
            
            # 紧急停止
            emergency_btn = tk.Button(harvest_frame, text="⭕ 紧急停止",
                                    command=self.emergency_stop,
                                    bg='#F44336', fg='white', font=self.normal_font,
                                    relief=tk.RAISED, bd=2)
            emergency_btn.pack(fill=tk.X, pady=5)
            
            # 今日成果
            result_frame = tk.LabelFrame(system_frame, text="📊 今日成果", font=self.normal_font,
                                       bg='#E8F5E8', fg='#2E7D32')
            result_frame.pack(fill=tk.X, pady=10)
            
            self.today_count_label = tk.Label(result_frame, text="127", 
                                            font=("Arial", 24, "bold"),
                                            bg='#E8F5E8', fg='#2E7D32')
            self.today_count_label.pack(pady=10)
            
        except Exception as e:
            self.get_logger().error(f'创建控制中心失败: {e}')
            traceback.print_exc()
    
    def create_monitoring_center(self, parent):
        """创建实时监控"""
        try:
            # 监控中心框架
            monitor_frame = tk.LabelFrame(parent, text="📹 实时监控", font=self.header_font,
                                        bg='#E8F5E8', fg='#2E7D32', bd=2, relief=tk.RAISED)
            monitor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            # FPS显示
            fps_frame = tk.Frame(monitor_frame, bg='#E8F5E8')
            fps_frame.pack(fill=tk.X, padx=10, pady=5)
            
            self.fps_label = tk.Label(fps_frame, text="FPS: 29", font=self.normal_font,
                                    bg='#E8F5E8', fg='#2E7D32')
            self.fps_label.pack(side=tk.RIGHT)
            
            # 视频显示区域
            self.video_frame = tk.Frame(monitor_frame, bg='black', relief=tk.SUNKEN, bd=2)
            self.video_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
            
            self.video_label = tk.Label(self.video_frame, text="等待摄像头数据...",
                                      bg='black', fg='white', font=self.normal_font)
            self.video_label.pack(expand=True)
            
            # 检测信息显示
            info_frame = tk.Frame(monitor_frame, bg='#E8F5E8')
            info_frame.pack(fill=tk.X, padx=10, pady=5)
            
            self.detection_info_label = tk.Label(info_frame, 
                                                text="检测到 4 个目标 | 最近距离: 0.65m | 准确率: 96.5%",
                                                font=self.normal_font, bg='#E8F5E8', fg='#2E7D32')
            self.detection_info_label.pack()
            
        except Exception as e:
            self.get_logger().error(f'创建监控中心失败: {e}')
            traceback.print_exc()
    
    def create_system_status(self, parent):
        """创建系统状态"""
        try:
            # 系统状态框架
            status_frame = tk.LabelFrame(parent, text="🔧 系统状态", font=self.header_font,
                                       bg='#E8F5E8', fg='#2E7D32', bd=2, relief=tk.RAISED)
            status_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
            status_frame.configure(width=220)
            status_frame.pack_propagate(False)
            
            # 电池电量
            battery_frame = tk.Frame(status_frame, bg='#E8F5E8')
            battery_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(battery_frame, text="电池电量", font=self.normal_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            battery_progress_frame = tk.Frame(battery_frame, bg='#E8F5E8')
            battery_progress_frame.pack(fill=tk.X, pady=2)
            
            self.battery_progress = ttk.Progressbar(battery_progress_frame, length=150,
                                                  mode='determinate', style='green.Horizontal.TProgressbar')
            self.battery_progress.pack(side=tk.LEFT)
            self.battery_progress['value'] = 75
            
            self.battery_label = tk.Label(battery_progress_frame, text="75%", 
                                        font=self.small_font, bg='#E8F5E8', fg='#2E7D32')
            self.battery_label.pack(side=tk.RIGHT, padx=(5, 0))
            
            # CPU使用率
            cpu_frame = tk.Frame(status_frame, bg='#E8F5E8')
            cpu_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(cpu_frame, text="CPU使用率", font=self.normal_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            cpu_progress_frame = tk.Frame(cpu_frame, bg='#E8F5E8')
            cpu_progress_frame.pack(fill=tk.X, pady=2)
            
            self.cpu_progress = ttk.Progressbar(cpu_progress_frame, length=150,
                                              mode='determinate', style='orange.Horizontal.TProgressbar')
            self.cpu_progress.pack(side=tk.LEFT)
            self.cpu_progress['value'] = 52
            
            self.cpu_label = tk.Label(cpu_progress_frame, text="52%", 
                                    font=self.small_font, bg='#E8F5E8', fg='#FF9800')
            self.cpu_label.pack(side=tk.RIGHT, padx=(5, 0))
            
            # 系统温度
            temp_frame = tk.Frame(status_frame, bg='#E8F5E8')
            temp_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(temp_frame, text="系统温度", font=self.normal_font,
                    bg='#E8F5E8').pack(side=tk.LEFT)
            
            self.temp_label = tk.Label(temp_frame, text="28°C", font=self.normal_font,
                                     bg='#E8F5E8', fg='#4CAF50')
            self.temp_label.pack(side=tk.RIGHT)
            
            tk.Label(temp_frame, text="正常工作温度", font=self.small_font,
                    bg='#E8F5E8', fg='#666').pack(anchor=tk.W)
            
            # 位置信息
            pos_frame = tk.LabelFrame(status_frame, text="📍 位置信息", font=self.normal_font,
                                    bg='#E8F5E8', fg='#2E7D32')
            pos_frame.pack(fill=tk.X, padx=10, pady=10)
            
            tk.Label(pos_frame, text="当前位置", font=self.small_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            self.position_label = tk.Label(pos_frame, 
                                         text="纬度: 34.938500°\n经度: 108.241500°\n区域: 苹果园3号地块",
                                         font=self.small_font, bg='#E8F5E8', fg='#333',
                                         justify=tk.LEFT)
            self.position_label.pack(anchor=tk.W, pady=2)
            
            # 工作状态
            work_frame = tk.Frame(status_frame, bg='#E8F5E8')
            work_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(work_frame, text="工作状态", font=self.normal_font,
                    bg='#E8F5E8').pack(anchor=tk.W)
            
            self.work_status_label = tk.Label(work_frame, 
                                            text="正在采摘作业\n工作时长: 5小时23分\n移动速度: 0.3 m/s",
                                            font=self.small_font, bg='#E8F5E8', fg='#333',
                                            justify=tk.LEFT)
            self.work_status_label.pack(anchor=tk.W, pady=2)
            
        except Exception as e:
            self.get_logger().error(f'创建系统状态失败: {e}')
            traceback.print_exc()
    
    def create_farm_overview(self, parent):
        """创建农场数据总览"""
        try:
            # 农场数据框架
            farm_frame = tk.LabelFrame(parent, text="🌾 农场数据总览", font=self.header_font,
                                     bg='#E8F5E8', fg='#2E7D32', bd=2, relief=tk.RAISED)
            farm_frame.pack(fill=tk.X, padx=5, pady=5)
            
            # 数据项框架
            data_frame = tk.Frame(farm_frame, bg='#E8F5E8')
            data_frame.pack(fill=tk.X, padx=10, pady=10)
            
            # 创建数据项
            data_items = [
                ("累计采摘", "2,543 个", "#4CAF50"),
                ("作业精度", "96.5%", "#2196F3"),
                ("信号强度", "75%", "#FF9800"),
                ("运行天数", "45 天", "#9C27B0"),
                ("健康状态", "优秀", "#4CAF50")
            ]
            
            for i, (label, value, color) in enumerate(data_items):
                item_frame = tk.Frame(data_frame, bg='white', relief=tk.RAISED, bd=1)
                item_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
                
                # 标题
                tk.Label(item_frame, text=label, font=self.small_font,
                        bg='white', fg='#666').pack(pady=(5, 0))
                
                # 数值
                value_label = tk.Label(item_frame, text=value, font=self.header_font,
                                     bg='white', fg=color)
                value_label.pack(pady=(0, 5))
                
                # 存储标签引用以便更新
                if label == "累计采摘":
                    self.total_harvest_label = value_label
                elif label == "作业精度":
                    self.accuracy_label = value_label
                elif label == "信号强度":
                    self.signal_label = value_label
                elif label == "运行天数":
                    self.running_days_label = value_label
                elif label == "健康状态":
                    self.health_label = value_label
            
        except Exception as e:
            self.get_logger().error(f'创建农场数据总览失败: {e}')
            traceback.print_exc()
    
    def toggle_system(self):
        """切换系统运行状态"""
        try:
            if not self.system_running:
                self.start_system()
            else:
                self.stop_system()
        except Exception as e:
            self.get_logger().error(f'切换系统状态失败: {e}')
            traceback.print_exc()
    
    def start_system(self):
        """启动系统"""
        try:
            # 启动集成系统launch文件
            command = [
                'ros2', 'launch', 'bottle_detection_ros2', 'integrated_system.launch.py'
            ]
            
            # 在后台启动
            self.system_process = subprocess.Popen(command, 
                                                 stdout=subprocess.PIPE, 
                                                 stderr=subprocess.PIPE)
            
            self.system_running = True
            self.system_switch.configure(text="停止系统", bg='#F44336')
            self.status_indicator.configure(text="● 运行中", fg='#4CAF50')
            
            self.get_logger().info('系统已启动')
            
        except Exception as e:
            self.get_logger().error(f'启动系统失败: {e}')
            traceback.print_exc()
    
    def stop_system(self):
        """停止系统"""
        try:
            if hasattr(self, 'system_process'):
                self.system_process.terminate()
                
            self.system_running = False
            self.system_switch.configure(text="启动系统", bg='#4CAF50')
            self.status_indicator.configure(text="● 已停止", fg='#F44336')
            
            self.get_logger().info('系统已停止')
            
        except Exception as e:
            self.get_logger().error(f'停止系统失败: {e}')
            traceback.print_exc()
    
    def set_work_mode(self, mode):
        """设置工作模式"""
        try:
            self.work_mode = mode
            
            # 更新按钮状态
            if mode == "manual":
                self.manual_btn.configure(bg='#4CAF50')
                self.auto_btn.configure(bg='#A5D6A7')
            else:
                self.manual_btn.configure(bg='#A5D6A7')
                self.auto_btn.configure(bg='#4CAF50')
            
            # 发布模式切换消息
            mode_msg = String()
            mode_msg.data = mode
            self.mode_pub.publish(mode_msg)
            
            self.get_logger().info(f'工作模式切换到: {mode}')
            
        except Exception as e:
            self.get_logger().error(f'设置工作模式失败: {e}')
            traceback.print_exc()
    
    def toggle_harvest(self):
        """切换采摘状态"""
        try:
            with self.data_lock:
                is_harvesting = self.robot_status_data.get('is_harvesting', False)
            
            # 发布采摘命令
            harvest_msg = HarvestCommand()
            harvest_msg.header.stamp = self.get_clock().now().to_msg()
            
            if not is_harvesting:
                harvest_msg.start_harvest = True
                harvest_msg.stop_harvest = False
                self.harvest_btn.configure(text="🛑 停止采摘", bg='#F44336')
                self.get_logger().info('开始采摘')
            else:
                harvest_msg.start_harvest = False
                harvest_msg.stop_harvest = True
                self.harvest_btn.configure(text="🍎 开始采摘", bg='#4CAF50')
                self.get_logger().info('停止采摘')
            
            self.harvest_cmd_pub.publish(harvest_msg)
            
        except Exception as e:
            self.get_logger().error(f'切换采摘状态失败: {e}')
            traceback.print_exc()
    
    def emergency_stop(self):
        """紧急停止"""
        try:
            # 发布紧急停止命令
            cmd_msg = RobotCommand()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.command = "emergencyStop"
            cmd_msg.emergency_stop = True
            cmd_msg.speed = 0.0
            
            self.robot_cmd_pub.publish(cmd_msg)
            
            # 同时停止采摘
            harvest_msg = HarvestCommand()
            harvest_msg.header.stamp = self.get_clock().now().to_msg()
            harvest_msg.stop_harvest = True
            self.harvest_cmd_pub.publish(harvest_msg)
            
            self.get_logger().warning('紧急停止已激活')
            
        except Exception as e:
            self.get_logger().error(f'紧急停止失败: {e}')
            traceback.print_exc()
    
    def image_callback(self, msg):
        """图像回调"""
        try:
            # 解压缩图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                with self.data_lock:
                    self.current_image = cv_image
                    
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')
            traceback.print_exc()
    
    def detection_callback(self, msg):
        """检测结果回调"""
        try:
            with self.data_lock:
                self.detection_data = {
                    'bottle_count': msg.bottle_count,
                    'nearest_distance': msg.distance,
                    'confidence': msg.confidence * 100
                }
        except Exception as e:
            self.get_logger().error(f'处理检测数据失败: {e}')
            traceback.print_exc()
    
    def status_callback(self, msg):
        """机器人状态回调"""
        try:
            with self.data_lock:
                self.robot_status_data.update({
                    'battery_level': msg.battery_level,
                    'cpu_usage': msg.cpu_usage,
                    'temperature': msg.temperature if hasattr(msg, 'temperature') else 28.0,
                    'position_x': msg.position_x,
                    'position_y': msg.position_y,
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'harvested_count': msg.harvested_count,
                    'today_harvested': msg.today_harvested,
                    'total_harvested': msg.total_harvested,
                    'working_hours': msg.working_hours,
                    'current_speed': msg.current_speed,
                    'is_moving': msg.is_moving,
                    'is_harvesting': msg.is_harvesting,
                    'signal_strength': msg.signal_strength if hasattr(msg, 'signal_strength') else 75,
                    'harvest_accuracy': msg.harvest_accuracy if hasattr(msg, 'harvest_accuracy') else 96.5
                })
        except Exception as e:
            self.get_logger().error(f'处理状态数据失败: {e}')
            traceback.print_exc()
    
    def fps_callback(self, msg):
        """FPS回调"""
        try:
            with self.data_lock:
                self.current_fps = msg.data
        except Exception as e:
            self.get_logger().error(f'处理FPS数据失败: {e}')
            traceback.print_exc()
    
    def update_timer(self):
        """更新定时器"""
        try:
            self.update_display()
            self.root.after(100, self.update_timer)  # 100ms更新一次
        except Exception as e:
            self.get_logger().error(f'更新定时器失败: {e}')
            traceback.print_exc()
    
    def update_display(self):
        """更新显示"""
        try:
            with self.data_lock:
                # 更新时间
                current_time = datetime.now().strftime("%H:%M")
                self.time_label.configure(text=current_time)
                
                # 更新FPS
                self.fps_label.configure(text=f"FPS: {self.current_fps:.0f}")
                
                # 更新检测信息
                detection_text = (f"检测到 {self.detection_data['bottle_count']} 个目标 | "
                               f"最近距离: {self.detection_data['nearest_distance']:.2f}m | "
                               f"准确率: {self.detection_data['confidence']:.1f}%")
                self.detection_info_label.configure(text=detection_text)
                
                # 更新电池电量
                battery_level = self.robot_status_data['battery_level']
                self.battery_progress['value'] = battery_level
                self.battery_label.configure(text=f"{battery_level:.0f}%")
                
                # 更新CPU使用率
                cpu_usage = self.robot_status_data['cpu_usage']
                self.cpu_progress['value'] = cpu_usage
                self.cpu_label.configure(text=f"{cpu_usage:.0f}%")
                
                # 更新温度
                temp = self.robot_status_data['temperature']
                self.temp_label.configure(text=f"{temp:.0f}°C")
                
                # 更新位置信息
                lat = self.robot_status_data['latitude']
                lon = self.robot_status_data['longitude']
                location = self.robot_status_data.get('location_name', '苹果园3号地块')
                position_text = f"纬度: {lat:.6f}°\n经度: {lon:.6f}°\n区域: {location}"
                self.position_label.configure(text=position_text)
                
                # 更新工作状态
                working_hours = self.robot_status_data['working_hours']
                speed = self.robot_status_data['current_speed']
                is_harvesting = self.robot_status_data['is_harvesting']
                
                work_status = "正在采摘作业" if is_harvesting else "待机状态"
                hours = int(working_hours)
                minutes = int((working_hours - hours) * 60)
                work_status_text = f"{work_status}\n工作时长: {hours}小时{minutes}分\n移动速度: {speed:.1f} m/s"
                self.work_status_label.configure(text=work_status_text)
                
                # 更新今日成果
                today_count = self.robot_status_data['today_harvested']
                self.today_count_label.configure(text=str(today_count))
                
                # 更新农场数据
                total_harvest = self.robot_status_data['total_harvested']
                self.total_harvest_label.configure(text=f"{total_harvest:,} 个")
                
                accuracy = self.robot_status_data['harvest_accuracy']
                self.accuracy_label.configure(text=f"{accuracy:.1f}%")
                
                signal = self.robot_status_data['signal_strength']
                self.signal_label.configure(text=f"{signal}%")
                
                # 更新视频显示
                if self.current_image is not None:
                    self.update_video_display(self.current_image)
                
        except Exception as e:
            self.get_logger().error(f'更新显示失败: {e}')
            traceback.print_exc()
    
    def update_video_display(self, cv_image):
        """更新视频显示"""
        try:
            # 调整图像大小适应显示区域
            height, width = cv_image.shape[:2]
            display_width = 600
            display_height = int(height * display_width / width)
            
            resized = cv2.resize(cv_image, (display_width, display_height))
            
            # 转换为RGB
            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # 转换为PIL图像
            pil_image = Image.fromarray(rgb_image)
            
            # 转换为Tkinter图像
            tk_image = ImageTk.PhotoImage(pil_image)
            
            # 更新显示
            self.video_label.configure(image=tk_image, text="")
            self.video_label.image = tk_image  # 保持引用
            
        except Exception as e:
            self.get_logger().error(f'更新视频显示失败: {e}')
            traceback.print_exc()
    
    def run(self):
        """运行GUI"""
        try:
            # 设置进度条样式
            style = ttk.Style()
            style.theme_use('clam')
            style.configure('green.Horizontal.TProgressbar', 
                          background='#4CAF50', troughcolor='#E8F5E8')
            style.configure('orange.Horizontal.TProgressbar', 
                          background='#FF9800', troughcolor='#E8F5E8')
            
            # 启动ROS2 spin线程
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            # 启动GUI主循环
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
            
        except Exception as e:
            self.get_logger().error(f'运行GUI失败: {e}')
            traceback.print_exc()
    
    def ros_spin(self):
        """ROS2 spin线程"""
        try:
            rclpy.spin(self)
        except Exception as e:
            self.get_logger().error(f'ROS spin失败: {e}')
            traceback.print_exc()
    
    def on_closing(self):
        """关闭窗口时的处理"""
        try:
            # 停止系统
            if self.system_running:
                self.stop_system()
            
            # 销毁节点
            self.destroy_node()
            
            # 关闭窗口
            self.root.destroy()
            
        except Exception as e:
            print(f'关闭GUI失败: {e}')
            traceback.print_exc()


def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        
        gui_node = LocalControlGUI()
        gui_node.run()
        
    except Exception as e:
        print(f'启动本地控制GUI失败: {e}')
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()