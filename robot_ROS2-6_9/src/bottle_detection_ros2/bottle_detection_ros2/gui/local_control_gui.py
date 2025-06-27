#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人本地控制界面
基于tkinter实现的农业采摘机器人本地监控和控制系统
"""

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
import threading
import time
import json
import traceback
import subprocess
import signal
import os
from datetime import datetime
from PIL import Image, ImageTk
import cv2
import numpy as np

# ROS2消息类型
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from bottle_detection_msgs.msg import (
    BottleDetection, RobotStatus, RobotCommand, 
    HarvestCommand, ServoCommand, ServoStatus
)

class LocalControlGUI:
    """本地控制界面类"""
    
    def __init__(self):
        """初始化界面"""
        try:
            # 创建主窗口
            self.root = tk.Tk()
            self.root.title("智慧农业采摘系统")
            self.root.geometry("1200x800")
            self.root.configure(bg='#E8F5E8')
            
            # 系统状态
            self.system_running = False
            self.launch_process = None
            self.work_mode = "manual"  # manual 或 auto
            self.harvesting = False
            
            # 数据存储
            self.data = {
                'fps': 0,
                'detection_count': 0,
                'nearest_distance': 0.0,
                'accuracy': 0.0,
                'battery_level': 0,
                'cpu_usage': 0,
                'temperature': 0,
                'latitude': 0.0,
                'longitude': 0.0,
                'location_name': '',
                'work_hours': 0,
                'move_speed': 0.0,
                'total_harvested': 0,
                'today_harvested': 0,
                'work_accuracy': 0.0,
                'signal_strength': 0,
                'running_days': 0,
                'health_status': '优秀'
            }
            
            # 当前图像
            self.current_image = None
            
            # 创建界面
            self.create_interface()
            
            # 初始化ROS2
            self.ros_node = None
            self.ros_thread = None
            
        except Exception as e:
            self.handle_error("初始化界面", e)
    
    def handle_error(self, function_name, error):
        """统一错误处理"""
        error_msg = f"错误在 {function_name}: {str(error)}"
        print(error_msg)
        print(traceback.format_exc())
    
    def create_interface(self):
        """创建主界面"""
        try:
            # 顶部标题栏
            self.create_header()
            
            # 主体区域
            main_frame = tk.Frame(self.root, bg='#E8F5E8')
            main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
            
            # 左侧控制面板
            self.create_control_panel(main_frame)
            
            # 中间监控区域
            self.create_monitor_panel(main_frame)
            
            # 右侧状态面板
            self.create_status_panel(main_frame)
            
            # 底部农场数据
            self.create_farm_data_panel()
            
        except Exception as e:
            self.handle_error("创建界面", e)
    
    def create_header(self):
        """创建顶部标题栏"""
        try:
            header_frame = tk.Frame(self.root, bg='#4CAF50', height=60)
            header_frame.pack(fill=tk.X)
            header_frame.pack_propagate(False)
            
            # 标题
            title_label = tk.Label(header_frame, text="智慧农业采摘系统", 
                                 font=("Microsoft YaHei", 20, "bold"),
                                 fg='white', bg='#4CAF50')
            title_label.pack(side=tk.LEFT, padx=20, pady=15)
            
            # 英文副标题
            subtitle_label = tk.Label(header_frame, text="Smart Agricultural Harvesting Robot", 
                                    font=("Arial", 12),
                                    fg='white', bg='#4CAF50')
            subtitle_label.pack(side=tk.LEFT, padx=(0, 20), pady=15)
            
            # 右侧状态和时间
            right_frame = tk.Frame(header_frame, bg='#4CAF50')
            right_frame.pack(side=tk.RIGHT, padx=20, pady=10)
            
            # 运行状态指示器
            self.status_indicator = tk.Label(right_frame, text="● 待机中", 
                                           font=("Microsoft YaHei", 12),
                                           fg='#FFC107', bg='#4CAF50')
            self.status_indicator.pack(side=tk.TOP)
            
            # 时间显示
            self.time_label = tk.Label(right_frame, text="", 
                                     font=("Arial", 14, "bold"),
                                     fg='white', bg='#4CAF50')
            self.time_label.pack(side=tk.TOP)
            
            # 启动时间更新
            self.update_time()
            
        except Exception as e:
            self.handle_error("创建标题栏", e)
    
    def create_control_panel(self, parent):
        """创建左侧控制面板"""
        try:
            control_frame = tk.Frame(parent, bg='white', relief=tk.RAISED, bd=2)
            control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
            control_frame.configure(width=300)
            control_frame.pack_propagate(False)
            
            # 控制中心标题
            tk.Label(control_frame, text="🎮 控制中心", 
                    font=("Microsoft YaHei", 16, "bold"),
                    bg='white', fg='#4CAF50').pack(pady=10)
            
            # 系统运行开关
            self.create_system_switch(control_frame)
            
            # 工作模式选择
            self.create_work_mode_selector(control_frame)
            
            # 采摘控制
            self.create_harvest_control(control_frame)
            
            # 紧急停止按钮
            self.create_emergency_stop(control_frame)
            
            # 今日成果
            self.create_daily_results(control_frame)
            
        except Exception as e:
            self.handle_error("创建控制面板", e)
    
    def create_system_switch(self, parent):
        """创建系统运行开关"""
        try:
            switch_frame = tk.Frame(parent, bg='white')
            switch_frame.pack(fill=tk.X, padx=20, pady=10)
            
            tk.Label(switch_frame, text="系统运行", 
                    font=("Microsoft YaHei", 12),
                    bg='white').pack(anchor=tk.W)
            
            # 开关按钮
            self.system_switch_frame = tk.Frame(switch_frame, bg='#E0E0E0', 
                                              height=30, relief=tk.RAISED, bd=2)
            self.system_switch_frame.pack(fill=tk.X, pady=5)
            self.system_switch_frame.pack_propagate(False)
            
            self.system_switch_button = tk.Button(self.system_switch_frame, 
                                                text="启动系统",
                                                font=("Microsoft YaHei", 10),
                                                command=self.toggle_system,
                                                bg='#4CAF50', fg='white',
                                                relief=tk.FLAT)
            self.system_switch_button.pack(fill=tk.BOTH, expand=True)
            
        except Exception as e:
            self.handle_error("创建系统开关", e)
    
    def create_work_mode_selector(self, parent):
        """创建工作模式选择器"""
        try:
            mode_frame = tk.Frame(parent, bg='white')
            mode_frame.pack(fill=tk.X, padx=20, pady=10)
            
            tk.Label(mode_frame, text="工作模式", 
                    font=("Microsoft YaHei", 12),
                    bg='white').pack(anchor=tk.W)
            
            mode_buttons_frame = tk.Frame(mode_frame, bg='white')
            mode_buttons_frame.pack(fill=tk.X, pady=5)
            
            # 手动模式按钮
            self.manual_btn = tk.Button(mode_buttons_frame, text="手动", 
                                      font=("Microsoft YaHei", 10),
                                      command=lambda: self.set_work_mode("manual"),
                                      bg='#4CAF50', fg='white', relief=tk.FLAT)
            self.manual_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
            
            # 自动模式按钮
            self.auto_btn = tk.Button(mode_buttons_frame, text="自动", 
                                    font=("Microsoft YaHei", 10),
                                    command=lambda: self.set_work_mode("auto"),
                                    bg='#E0E0E0', fg='black', relief=tk.FLAT)
            self.auto_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
        except Exception as e:
            self.handle_error("创建工作模式选择器", e)
    
    def create_harvest_control(self, parent):
        """创建采摘控制"""
        try:
            harvest_frame = tk.Frame(parent, bg='white')
            harvest_frame.pack(fill=tk.X, padx=20, pady=10)
            
            tk.Label(harvest_frame, text="采摘控制", 
                    font=("Microsoft YaHei", 12),
                    bg='white').pack(anchor=tk.W)
            
            self.harvest_btn = tk.Button(harvest_frame, text="🍎 开始采摘", 
                                       font=("Microsoft YaHei", 12),
                                       command=self.toggle_harvest,
                                       bg='#4CAF50', fg='white', 
                                       height=2, relief=tk.FLAT)
            self.harvest_btn.pack(fill=tk.X, pady=5)
            
        except Exception as e:
            self.handle_error("创建采摘控制", e)
    
    def create_emergency_stop(self, parent):
        """创建紧急停止按钮"""
        try:
            emergency_frame = tk.Frame(parent, bg='white')
            emergency_frame.pack(fill=tk.X, padx=20, pady=10)
            
            self.emergency_btn = tk.Button(emergency_frame, text="⭕ 紧急停止", 
                                         font=("Microsoft YaHei", 12, "bold"),
                                         command=self.emergency_stop,
                                         bg='#F44336', fg='white', 
                                         height=2, relief=tk.FLAT)
            self.emergency_btn.pack(fill=tk.X, pady=5)
            
        except Exception as e:
            self.handle_error("创建紧急停止按钮", e)
    
    def create_daily_results(self, parent):
        """创建今日成果显示"""
        try:
            results_frame = tk.Frame(parent, bg='#E8F5E8', relief=tk.RAISED, bd=2)
            results_frame.pack(fill=tk.X, padx=20, pady=10)
            
            tk.Label(results_frame, text="📊 今日成果", 
                    font=("Microsoft YaHei", 12, "bold"),
                    bg='#E8F5E8', fg='#4CAF50').pack(pady=5)
            
            self.daily_count_label = tk.Label(results_frame, text="127", 
                                            font=("Microsoft YaHei", 24, "bold"),
                                            bg='#E8F5E8', fg='#4CAF50')
            self.daily_count_label.pack()
            
        except Exception as e:
            self.handle_error("创建今日成果", e)
    
    def create_monitor_panel(self, parent):
        """创建中间监控面板"""
        try:
            monitor_frame = tk.Frame(parent, bg='white', relief=tk.RAISED, bd=2)
            monitor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
            
            # 监控标题
            title_frame = tk.Frame(monitor_frame, bg='white')
            title_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(title_frame, text="📹 实时监控", 
                    font=("Microsoft YaHei", 16, "bold"),
                    bg='white', fg='#4CAF50').pack(side=tk.LEFT)
            
            self.fps_label = tk.Label(title_frame, text="FPS: 29", 
                                    font=("Microsoft YaHei", 12),
                                    bg='white', fg='#666')
            self.fps_label.pack(side=tk.RIGHT)
            
            # 视频显示区域
            self.video_frame = tk.Frame(monitor_frame, bg='black', relief=tk.SUNKEN, bd=2)
            self.video_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
            
            self.video_label = tk.Label(self.video_frame, text="等待视频流...", 
                                      font=("Microsoft YaHei", 16),
                                      bg='black', fg='white')
            self.video_label.pack(expand=True)
            
            # 检测信息
            info_frame = tk.Frame(monitor_frame, bg='white')
            info_frame.pack(fill=tk.X, padx=10, pady=5)
            
            self.detection_info_label = tk.Label(info_frame, 
                                                text="检测到 4 个目标 | 最近距离: 0.65m | 准确率: 96.5%", 
                                                font=("Microsoft YaHei", 12),
                                                bg='white', fg='#333')
            self.detection_info_label.pack()
            
        except Exception as e:
            self.handle_error("创建监控面板", e)
    
    def create_status_panel(self, parent):
        """创建右侧状态面板"""
        try:
            status_frame = tk.Frame(parent, bg='white', relief=tk.RAISED, bd=2)
            status_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
            status_frame.configure(width=280)
            status_frame.pack_propagate(False)
            
            # 系统状态标题
            tk.Label(status_frame, text="🔧 系统状态", 
                    font=("Microsoft YaHei", 16, "bold"),
                    bg='white', fg='#4CAF50').pack(pady=10)
            
            # 电池电量
            self.create_battery_status(status_frame)
            
            # CPU使用率
            self.create_cpu_status(status_frame)
            
            # 系统温度
            self.create_temperature_status(status_frame)
            
            # 位置信息
            self.create_location_info(status_frame)
            
            # 工作状态
            self.create_work_status(status_frame)
            
        except Exception as e:
            self.handle_error("创建状态面板", e)
    
    def create_battery_status(self, parent):
        """创建电池状态显示"""
        try:
            battery_frame = tk.Frame(parent, bg='white')
            battery_frame.pack(fill=tk.X, padx=15, pady=5)
            
            tk.Label(battery_frame, text="电池电量", 
                    font=("Microsoft YaHei", 10),
                    bg='white', fg='#666').pack(anchor=tk.W)
            
            # 电池进度条背景
            battery_bg = tk.Frame(battery_frame, bg='#E0E0E0', height=20)
            battery_bg.pack(fill=tk.X, pady=2)
            battery_bg.pack_propagate(False)
            
            # 电池进度条
            self.battery_bar = tk.Frame(battery_bg, bg='#4CAF50', height=18)
            self.battery_bar.place(x=1, y=1, relwidth=0.75, height=18)
            
            self.battery_label = tk.Label(battery_frame, text="75%", 
                                        font=("Microsoft YaHei", 12, "bold"),
                                        bg='white', fg='#4CAF50')
            self.battery_label.pack(anchor=tk.E)
            
        except Exception as e:
            self.handle_error("创建电池状态", e)
    
    def create_cpu_status(self, parent):
        """创建CPU状态显示"""
        try:
            cpu_frame = tk.Frame(parent, bg='white')
            cpu_frame.pack(fill=tk.X, padx=15, pady=5)
            
            tk.Label(cpu_frame, text="CPU使用率", 
                    font=("Microsoft YaHei", 10),
                    bg='white', fg='#666').pack(anchor=tk.W)
            
            # CPU进度条背景
            cpu_bg = tk.Frame(cpu_frame, bg='#E0E0E0', height=20)
            cpu_bg.pack(fill=tk.X, pady=2)
            cpu_bg.pack_propagate(False)
            
            # CPU进度条
            self.cpu_bar = tk.Frame(cpu_bg, bg='#FF9800', height=18)
            self.cpu_bar.place(x=1, y=1, relwidth=0.52, height=18)
            
            self.cpu_label = tk.Label(cpu_frame, text="52%", 
                                    font=("Microsoft YaHei", 12, "bold"),
                                    bg='white', fg='#FF9800')
            self.cpu_label.pack(anchor=tk.E)
            
        except Exception as e:
            self.handle_error("创建CPU状态", e)
    
    def create_temperature_status(self, parent):
        """创建温度状态显示"""
        try:
            temp_frame = tk.Frame(parent, bg='white')
            temp_frame.pack(fill=tk.X, padx=15, pady=5)
            
            tk.Label(temp_frame, text="系统温度", 
                    font=("Microsoft YaHei", 10),
                    bg='white', fg='#666').pack(anchor=tk.W)
            
            self.temp_label = tk.Label(temp_frame, text="28°C", 
                                     font=("Microsoft YaHei", 20, "bold"),
                                     bg='white', fg='#4CAF50')
            self.temp_label.pack(anchor=tk.W)
            
            tk.Label(temp_frame, text="正常工作温度", 
                    font=("Microsoft YaHei", 8),
                    bg='white', fg='#999').pack(anchor=tk.W)
            
        except Exception as e:
            self.handle_error("创建温度状态", e)
    
    def create_location_info(self, parent):
        """创建位置信息显示"""
        try:
            location_frame = tk.Frame(parent, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            location_frame.pack(fill=tk.X, padx=15, pady=10)
            
            tk.Label(location_frame, text="📍 位置信息", 
                    font=("Microsoft YaHei", 12, "bold"),
                    bg='#E8F5E8', fg='#4CAF50').pack(pady=5)
            
            tk.Label(location_frame, text="当前位置", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack(anchor=tk.W, padx=5)
            
            self.location_label = tk.Label(location_frame, 
                                         text="纬度: 34.938500°\n经度: 108.241500°\n区域: 苹果园3号地块", 
                                         font=("Microsoft YaHei", 9),
                                         bg='#E8F5E8', fg='#333',
                                         justify=tk.LEFT)
            self.location_label.pack(anchor=tk.W, padx=5, pady=2)
            
        except Exception as e:
            self.handle_error("创建位置信息", e)
    
    def create_work_status(self, parent):
        """创建工作状态显示"""
        try:
            work_frame = tk.Frame(parent, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            work_frame.pack(fill=tk.X, padx=15, pady=5)
            
            tk.Label(work_frame, text="工作状态", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack(anchor=tk.W, padx=5)
            
            self.work_status_label = tk.Label(work_frame, 
                                            text="正在采摘作业\n工作时长: 5小时23分\n移动速度: 0.3 m/s", 
                                            font=("Microsoft YaHei", 9),
                                            bg='#E8F5E8', fg='#333',
                                            justify=tk.LEFT)
            self.work_status_label.pack(anchor=tk.W, padx=5, pady=5)
            
        except Exception as e:
            self.handle_error("创建工作状态", e)
    
    def create_farm_data_panel(self):
        """创建底部农场数据总览"""
        try:
            farm_frame = tk.Frame(self.root, bg='white', relief=tk.RAISED, bd=2, height=100)
            farm_frame.pack(fill=tk.X, padx=10, pady=5)
            farm_frame.pack_propagate(False)
            
            # 标题
            tk.Label(farm_frame, text="🌾 农场数据总览", 
                    font=("Microsoft YaHei", 14, "bold"),
                    bg='white', fg='#4CAF50').pack(pady=5)
            
            # 数据指标
            data_frame = tk.Frame(farm_frame, bg='white')
            data_frame.pack(fill=tk.X, padx=20)
            
            # 累计采摘
            harvested_frame = tk.Frame(data_frame, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            harvested_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            
            tk.Label(harvested_frame, text="累计采摘", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack()
            self.total_harvested_label = tk.Label(harvested_frame, text="2,543 个", 
                                                font=("Microsoft YaHei", 12, "bold"),
                                                bg='#E8F5E8', fg='#4CAF50')
            self.total_harvested_label.pack()
            
            # 作业精度
            accuracy_frame = tk.Frame(data_frame, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            accuracy_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            
            tk.Label(accuracy_frame, text="作业精度", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack()
            self.work_accuracy_label = tk.Label(accuracy_frame, text="96.5%", 
                                              font=("Microsoft YaHei", 12, "bold"),
                                              bg='#E8F5E8', fg='#4CAF50')
            self.work_accuracy_label.pack()
            
            # 信号强度
            signal_frame = tk.Frame(data_frame, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            signal_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            
            tk.Label(signal_frame, text="信号强度", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack()
            self.signal_strength_label = tk.Label(signal_frame, text="75%", 
                                                 font=("Microsoft YaHei", 12, "bold"),
                                                 bg='#E8F5E8', fg='#4CAF50')
            self.signal_strength_label.pack()
            
            # 运行天数
            days_frame = tk.Frame(data_frame, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            days_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            
            tk.Label(days_frame, text="运行天数", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack()
            self.running_days_label = tk.Label(days_frame, text="45 天", 
                                             font=("Microsoft YaHei", 12, "bold"),
                                             bg='#E8F5E8', fg='#4CAF50')
            self.running_days_label.pack()
            
            # 健康状态
            health_frame = tk.Frame(data_frame, bg='#E8F5E8', relief=tk.RAISED, bd=1)
            health_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            
            tk.Label(health_frame, text="健康状态", 
                    font=("Microsoft YaHei", 10),
                    bg='#E8F5E8', fg='#666').pack()
            self.health_status_label = tk.Label(health_frame, text="优秀", 
                                              font=("Microsoft YaHei", 12, "bold"),
                                              bg='#E8F5E8', fg='#4CAF50')
            self.health_status_label.pack()
            
        except Exception as e:
            self.handle_error("创建农场数据面板", e)
    
    # ===== 控制功能方法 =====
    
    def toggle_system(self):
        """切换系统运行状态"""
        try:
            if not self.system_running:
                self.start_system()
            else:
                self.stop_system()
        except Exception as e:
            self.handle_error("切换系统状态", e)
    
    def start_system(self):
        """启动系统"""
        try:
            # 启动integrated_system.launch.py
            launch_file = "integrated_system.launch.py"
            pkg_name = "bottle_detection_ros2"
            
            cmd = f"ros2 launch {pkg_name} {launch_file}"
            self.launch_process = subprocess.Popen(
                cmd, shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            self.system_running = True
            self.system_switch_button.config(text="停止系统", bg='#F44336')
            self.status_indicator.config(text="● 运行中", fg='#4CAF50')
            
            # 启动ROS2节点
            self.init_ros_node()
            
            print(f"系统已启动，进程ID: {self.launch_process.pid}")
            
        except Exception as e:
            self.handle_error("启动系统", e)
    
    def stop_system(self):
        """停止系统"""
        try:
            if self.launch_process:
                # 终止整个进程组
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process = None
            
            # 停止ROS2节点
            self.shutdown_ros_node()
            
            self.system_running = False
            self.system_switch_button.config(text="启动系统", bg='#4CAF50')
            self.status_indicator.config(text="● 待机中", fg='#FFC107')
            
            # 清空视频显示
            self.video_label.config(image='', text="等待视频流...")
            
            print("系统已停止")
            
        except Exception as e:
            self.handle_error("停止系统", e)
    
    def set_work_mode(self, mode):
        """设置工作模式"""
        try:
            self.work_mode = mode
            
            if mode == "manual":
                self.manual_btn.config(bg='#4CAF50', fg='white')
                self.auto_btn.config(bg='#E0E0E0', fg='black')
            else:
                self.manual_btn.config(bg='#E0E0E0', fg='black')
                self.auto_btn.config(bg='#4CAF50', fg='white')
            
            # 发布模式切换消息
            if self.ros_node:
                self.ros_node.publish_mode_change(mode)
            
            print(f"工作模式切换为: {mode}")
            
        except Exception as e:
            self.handle_error("设置工作模式", e)
    
    def toggle_harvest(self):
        """切换采摘状态"""
        try:
            self.harvesting = not self.harvesting
            
            if self.harvesting:
                self.harvest_btn.config(text="🍎 停止采摘", bg='#F44336')
                if self.ros_node:
                    self.ros_node.publish_harvest_command(True)
            else:
                self.harvest_btn.config(text="🍎 开始采摘", bg='#4CAF50')
                if self.ros_node:
                    self.ros_node.publish_harvest_command(False)
            
            print(f"采摘状态: {'开始' if self.harvesting else '停止'}")
            
        except Exception as e:
            self.handle_error("切换采摘状态", e)
    
    def emergency_stop(self):
        """紧急停止"""
        try:
            # 发布紧急停止命令
            if self.ros_node:
                self.ros_node.publish_emergency_stop()
            
            # 重置状态
            self.harvesting = False
            self.harvest_btn.config(text="🍎 开始采摘", bg='#4CAF50')
            
            print("紧急停止已触发")
            
        except Exception as e:
            self.handle_error("紧急停止", e)
    
    def update_time(self):
        """更新时间显示"""
        try:
            current_time = datetime.now().strftime("%H:%M")
            self.time_label.config(text=current_time)
            self.root.after(1000, self.update_time)
        except Exception as e:
            self.handle_error("更新时间", e)
    
    # ===== ROS2节点管理 =====
    
    def init_ros_node(self):
        """初始化ROS2节点"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = ROSControlNode(self)
            
            # 在单独线程中运行ROS2
            self.ros_thread = threading.Thread(target=self.run_ros_node, daemon=True)
            self.ros_thread.start()
            
            print("ROS2节点已启动")
            
        except Exception as e:
            self.handle_error("初始化ROS2节点", e)
    
    def run_ros_node(self):
        """运行ROS2节点"""
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            self.handle_error("运行ROS2节点", e)
    
    def shutdown_ros_node(self):
        """关闭ROS2节点"""
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
                self.ros_node = None
            
            if rclpy.ok():
                rclpy.shutdown()
            
            print("ROS2节点已关闭")
            
        except Exception as e:
            self.handle_error("关闭ROS2节点", e)
    
    # ===== 数据更新方法 =====
    
    def update_video_frame(self, cv_image):
        """更新视频帧"""
        try:
            # 调整图像大小以适应显示区域
            height, width = cv_image.shape[:2]
            display_width = 640
            display_height = int(display_width * height / width)
            
            resized_image = cv2.resize(cv_image, (display_width, display_height))
            
            # 转换为PIL图像
            rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
            self.current_image = ImageTk.PhotoImage(pil_image)
            
            # 更新显示
            self.video_label.config(image=self.current_image, text="")
            
        except Exception as e:
            self.handle_error("更新视频帧", e)
    
    def update_detection_data(self, detection_msg):
        """更新检测数据"""
        try:
            self.data['detection_count'] = detection_msg.detection_count
            self.data['nearest_distance'] = detection_msg.nearest_distance
            self.data['accuracy'] = detection_msg.confidence
            
            # 更新检测信息显示
            info_text = f"检测到 {self.data['detection_count']} 个目标 | 最近距离: {self.data['nearest_distance']:.2f}m | 准确率: {self.data['accuracy']:.1f}%"
            self.detection_info_label.config(text=info_text)
            
        except Exception as e:
            self.handle_error("更新检测数据", e)
    
    def update_robot_status(self, status_msg):
        """更新机器人状态"""
        try:
            # 更新数据
            self.data['battery_level'] = status_msg.battery_level
            self.data['cpu_usage'] = status_msg.cpu_usage
            self.data['temperature'] = status_msg.temperature
            self.data['latitude'] = status_msg.latitude
            self.data['longitude'] = status_msg.longitude
            self.data['location_name'] = status_msg.location_description
            self.data['work_hours'] = status_msg.work_hours
            self.data['move_speed'] = status_msg.move_speed
            self.data['today_harvested'] = status_msg.today_harvested_count
            self.data['total_harvested'] = status_msg.total_harvested_count
            self.data['work_accuracy'] = status_msg.harvest_accuracy
            self.data['signal_strength'] = status_msg.network_strength
            
            # 更新界面显示
            self.update_status_displays()
            
        except Exception as e:
            self.handle_error("更新机器人状态", e)
    
    def update_status_displays(self):
        """更新状态显示"""
        try:
            # 更新电池显示
            battery_percent = self.data['battery_level'] / 100.0
            self.battery_bar.place(relwidth=battery_percent)
            self.battery_label.config(text=f"{self.data['battery_level']}%")
            
            # 根据电池电量改变颜色
            if self.data['battery_level'] > 50:
                self.battery_bar.config(bg='#4CAF50')
                self.battery_label.config(fg='#4CAF50')
            elif self.data['battery_level'] > 20:
                self.battery_bar.config(bg='#FF9800')
                self.battery_label.config(fg='#FF9800')
            else:
                self.battery_bar.config(bg='#F44336')
                self.battery_label.config(fg='#F44336')
            
            # 更新CPU显示
            cpu_percent = self.data['cpu_usage'] / 100.0
            self.cpu_bar.place(relwidth=cpu_percent)
            self.cpu_label.config(text=f"{self.data['cpu_usage']}%")
            
            # 更新温度显示
            self.temp_label.config(text=f"{self.data['temperature']}°C")
            
            # 更新位置信息
            location_text = f"纬度: {self.data['latitude']:.6f}°\n经度: {self.data['longitude']:.6f}°\n区域: {self.data['location_name']}"
            self.location_label.config(text=location_text)
            
            # 更新工作状态
            work_hours = int(self.data['work_hours'])
            work_minutes = int((self.data['work_hours'] - work_hours) * 60)
            work_status_text = f"正在采摘作业\n工作时长: {work_hours}小时{work_minutes}分\n移动速度: {self.data['move_speed']:.1f} m/s"
            self.work_status_label.config(text=work_status_text)
            
            # 更新今日成果
            self.daily_count_label.config(text=str(self.data['today_harvested']))
            
            # 更新农场数据
            self.total_harvested_label.config(text=f"{self.data['total_harvested']:,} 个")
            self.work_accuracy_label.config(text=f"{self.data['work_accuracy']:.1f}%")
            self.signal_strength_label.config(text=f"{self.data['signal_strength']}%")
            
        except Exception as e:
            self.handle_error("更新状态显示", e)
    
    def update_fps(self, fps):
        """更新FPS显示"""
        try:
            self.data['fps'] = fps
            self.fps_label.config(text=f"FPS: {fps}")
        except Exception as e:
            self.handle_error("更新FPS", e)
    
    def run(self):
        """运行界面"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except Exception as e:
            self.handle_error("运行界面", e)
    
    def on_closing(self):
        """关闭窗口时的处理"""
        try:
            self.stop_system()
            self.root.destroy()
        except Exception as e:
            self.handle_error("关闭窗口", e)


class ROSControlNode(Node):
    """ROS2控制节点"""
    
    def __init__(self, gui):
        """初始化节点"""
        try:
            super().__init__('local_control_gui_node')
            self.gui = gui
            
            # 创建订阅者
            self.create_subscriptions()
            
            # 创建发布者
            self.create_publishers()
            
            self.get_logger().info('本地控制界面节点已启动')
            
        except Exception as e:
            self.gui.handle_error("初始化ROS节点", e)
    
    def create_subscriptions(self):
        """创建订阅者"""
        try:
            # 订阅视频流
            self.image_subscription = self.create_subscription(
                CompressedImage,
                '/camera/processed_image/compressed',
                self.image_callback,
                10
            )
            
            # 订阅检测结果
            self.detection_subscription = self.create_subscription(
                BottleDetection,
                '/bottle_detection',
                self.detection_callback,
                10
            )
            
            # 订阅机器人状态
            self.status_subscription = self.create_subscription(
                RobotStatus,
                '/robot_status',
                self.status_callback,
                10
            )
            
            # 订阅FPS信息
            self.fps_subscription = self.create_subscription(
                Float32,
                '/camera/fps',
                self.fps_callback,
                10
            )
            
        except Exception as e:
            self.gui.handle_error("创建ROS订阅者", e)
    
    def create_publishers(self):
        """创建发布者"""
        try:
            # 机器人控制命令发布者
            self.robot_command_publisher = self.create_publisher(
                RobotCommand,
                '/robot_command',
                10
            )
            
            # 采摘命令发布者
            self.harvest_command_publisher = self.create_publisher(
                HarvestCommand,
                '/harvest_command',
                10
            )
            
            # 模式切换发布者
            self.mode_publisher = self.create_publisher(
                String,
                '/work_mode',
                10
            )
            
        except Exception as e:
            self.gui.handle_error("创建ROS发布者", e)
    
    def image_callback(self, msg):
        """图像消息回调"""
        try:
            # 解压缩图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 在主线程中更新界面
            self.gui.root.after(0, lambda: self.gui.update_video_frame(cv_image))
            
        except Exception as e:
            self.gui.handle_error("处理图像消息", e)
    
    def detection_callback(self, msg):
        """检测结果回调"""
        try:
            self.gui.root.after(0, lambda: self.gui.update_detection_data(msg))
        except Exception as e:
            self.gui.handle_error("处理检测消息", e)
    
    def status_callback(self, msg):
        """状态消息回调"""
        try:
            self.gui.root.after(0, lambda: self.gui.update_robot_status(msg))
        except Exception as e:
            self.gui.handle_error("处理状态消息", e)
    
    def fps_callback(self, msg):
        """FPS消息回调"""
        try:
            self.gui.root.after(0, lambda: self.gui.update_fps(int(msg.data)))
        except Exception as e:
            self.gui.handle_error("处理FPS消息", e)
    
    def publish_robot_command(self, command_type, speed=0.5):
        """发布机器人控制命令"""
        try:
            msg = RobotCommand()
            msg.command = command_type
            msg.speed = speed
            msg.timestamp = self.get_clock().now().to_msg()
            
            self.robot_command_publisher.publish(msg)
            
        except Exception as e:
            self.gui.handle_error("发布机器人命令", e)
    
    def publish_harvest_command(self, start):
        """发布采摘命令"""
        try:
            msg = HarvestCommand()
            msg.action = "start" if start else "stop"
            msg.timestamp = self.get_clock().now().to_msg()
            
            self.harvest_command_publisher.publish(msg)
            
        except Exception as e:
            self.gui.handle_error("发布采摘命令", e)
    
    def publish_emergency_stop(self):
        """发布紧急停止命令"""
        try:
            msg = RobotCommand()
            msg.command = "emergency_stop"
            msg.speed = 0.0
            msg.timestamp = self.get_clock().now().to_msg()
            
            self.robot_command_publisher.publish(msg)
            
        except Exception as e:
            self.gui.handle_error("发布紧急停止命令", e)
    
    def publish_mode_change(self, mode):
        """发布模式切换命令"""
        try:
            msg = String()
            msg.data = mode
            
            self.mode_publisher.publish(msg)
            
        except Exception as e:
            self.gui.handle_error("发布模式切换命令", e)


def main():
    """主函数"""
    try:
        gui = LocalControlGUI()
        gui.run()
    except Exception as e:
        print(f"主函数错误: {str(e)}")
        print(traceback.format_exc())


if __name__ == '__main__':
    main() 