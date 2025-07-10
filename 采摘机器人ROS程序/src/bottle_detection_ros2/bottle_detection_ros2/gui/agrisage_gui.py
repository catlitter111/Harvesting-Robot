#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
农业采摘机器人GUI控制界面
基于tkinter实现，农业主题设计
仅包含界面布局，预留ROS2通信接口
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from datetime import datetime
from PIL import Image, ImageTk
import cv2
import numpy as np

class AgriculturalRobotGUI:
    """农业机器人GUI主类"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.setup_window()
        self.setup_style()
        
        # 系统状态 - 修改默认模式为自动
        self.system_running = False
        self.current_mode = "auto"  # manual/auto - 修改默认为自动
        self.auto_harvest_active = True  # 修改默认为启用自动采摘
        
        # 模拟数据（实际使用时从ROS2获取）
        self.robot_data = {
            'battery_level': 75,
            'cpu_usage': 52,
            'temperature': 28,
            'harvested_today': 127,
            'total_harvested': 2543,
            'working_area': 3.2,
            'working_hours': 5.4,
            'accuracy': 96.5,
            'signal_strength': 75,
            'nearest_distance': 0.65,
            'detection_count': 2,
            'fps': 29.8,
            'latitude': 34.938500,
            'longitude': 108.241500,
            'status_text': '待机中',
            'current_speed': 0.0
        }
        
        # 创建界面
        self.create_widgets()
        self.setup_layout()
        
        # 启动数据更新线程
        self.start_update_thread()
    
    def setup_window(self):
        """设置窗口属性"""
        self.root.title("🌾 智慧农业采摘系统")
        self.root.geometry("1024x600")
        self.root.configure(bg='#87ceeb')
        self.root.resizable(False, False)
    
    def setup_style(self):
        """设置样式"""
        self.style = ttk.Style()
        self.style.theme_use('clam')
        
        # 定义颜色
        self.colors = {
            'primary': '#4caf50',      # 主绿色
            'primary_dark': '#2e7d32', # 深绿色
            'secondary': '#388e3c',     # 次要绿色
            'light_green': '#e8f5e8',  # 浅绿色
            'white': '#ffffff',
            'light_gray': '#f5f5f5',
            'gray': '#666666',
            'red': '#f44336',
            'orange': '#ff9800',
            'background': '#87ceeb'
        }
    
    def create_widgets(self):
        """创建界面组件"""
        # 主框架
        self.main_frame = tk.Frame(self.root, bg=self.colors['background'])
        
        # 标题栏
        self.create_title_bar()
        
        # 左侧控制面板
        self.create_control_panel()
        
        # 中央摄像头区域
        self.create_camera_area()
        
        # 右侧状态面板
        self.create_status_panel()
        
        # 底部数据面板
        self.create_data_panel()
    
    def create_title_bar(self):
        """创建标题栏"""
        title_frame = tk.Frame(self.main_frame, bg=self.colors['primary_dark'], height=70)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        # 标题
        tk.Label(title_frame, text="🌾 智慧农业采摘系统", 
                font=('Arial', 28, 'bold'), fg='white', 
                bg=self.colors['primary_dark']).pack(side='left', padx=30, pady=15)
        
        tk.Label(title_frame, text="Smart Agricultural Harvesting Robot",
                font=('Arial', 14), fg='#c8e6c9',
                bg=self.colors['primary_dark']).pack(side='left', padx=(30, 0), pady=(40, 0))
        
        # 状态指示器
        status_frame = tk.Frame(title_frame, bg=self.colors['primary_dark'])
        status_frame.pack(side='right', padx=30, pady=15)
        
        self.status_indicator = tk.Canvas(status_frame, width=20, height=20, 
                                        bg=self.colors['primary_dark'], highlightthickness=0)
        self.status_indicator.pack(side='left', padx=(0, 10))
        self.status_circle = self.status_indicator.create_oval(2, 2, 18, 18, fill='red', outline='')
        
        self.status_label = tk.Label(status_frame, text="已停止", font=('Arial', 14, 'bold'),
                                   fg='white', bg=self.colors['primary_dark'])
        self.status_label.pack(side='left')
        
        self.time_label = tk.Label(status_frame, text="12:34", font=('Arial', 14),
                                 fg='white', bg=self.colors['primary_dark'])
        self.time_label.pack(side='right', padx=(20, 0))
    
    def create_control_panel(self):
        """创建左侧控制面板"""
        self.control_frame = tk.Frame(self.main_frame, bg=self.colors['white'], 
                                    width=240, relief='raised', bd=2)
        self.control_frame.pack(side='left', fill='y', padx=(20, 10), pady=20)
        self.control_frame.pack_propagate(False)
        
        # 控制标题
        title_frame = tk.Frame(self.control_frame, bg=self.colors['primary'], height=50)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        tk.Label(title_frame, text="🎛️ 控制中心", font=('Arial', 18, 'bold'),
                fg='white', bg=self.colors['primary']).pack(pady=12)
        
        # 系统开关
        switch_frame = tk.Frame(self.control_frame, bg=self.colors['white'])
        switch_frame.pack(fill='x', padx=20, pady=20)
        
        self.system_button = tk.Button(switch_frame, text="启动系统", 
                                     font=('Arial', 16, 'bold'), fg='white',
                                     bg=self.colors['primary'], height=2,
                                     command=self.toggle_system)
        self.system_button.pack(fill='x')
        
        # 工作模式选择
        mode_frame = tk.Frame(self.control_frame, bg=self.colors['white'])
        mode_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(mode_frame, text="工作模式", font=('Arial', 14, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w')
        
        mode_buttons_frame = tk.Frame(mode_frame, bg=self.colors['white'])
        mode_buttons_frame.pack(fill='x', pady=5)
        
        self.manual_button = tk.Button(mode_buttons_frame, text="手动", 
                                     font=('Arial', 14), height=1,
                                     bg=self.colors['light_green'], fg=self.colors['primary'],
                                     command=lambda: self.set_mode("manual"))
        self.manual_button.pack(side='left', fill='x', expand=True, padx=(0, 5))
        
        self.auto_button = tk.Button(mode_buttons_frame, text="自动",
                                   font=('Arial', 14, 'bold'), height=1,
                                   bg=self.colors['primary'], fg='white',
                                   command=lambda: self.set_mode("auto"))
        self.auto_button.pack(side='left', fill='x', expand=True, padx=(5, 0))
        
        # 采摘控制
        harvest_frame = tk.Frame(self.control_frame, bg=self.colors['white'])
        harvest_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(harvest_frame, text="采摘控制", font=('Arial', 14, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w')
        
        self.harvest_button = tk.Button(harvest_frame, text="🍎 开始采摘",
                                      font=('Arial', 14, 'bold'), fg='white',
                                      bg=self.colors['secondary'], height=1,
                                      command=self.toggle_harvest)
        self.harvest_button.pack(fill='x', pady=5)
        
        # 紧急停止
        emergency_frame = tk.Frame(self.control_frame, bg=self.colors['white'])
        emergency_frame.pack(fill='x', padx=20, pady=20)
        
        self.emergency_button = tk.Button(emergency_frame, text="🛑 紧急停止",
                                        font=('Arial', 16, 'bold'), fg='white',
                                        bg=self.colors['red'], height=2,
                                        command=self.emergency_stop)
        self.emergency_button.pack(fill='x')
        
        # 今日统计
        stats_frame = tk.Frame(self.control_frame, bg=self.colors['light_green'], 
                             relief='solid', bd=1)
        stats_frame.pack(fill='x', padx=20, pady=20)
        
        tk.Label(stats_frame, text="📊 今日成果", font=('Arial', 16, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['light_green']).pack(pady=10)
        
        self.harvest_count_label = tk.Label(stats_frame, text="127", 
                                          font=('Arial', 32, 'bold'),
                                          fg=self.colors['primary'], 
                                          bg=self.colors['light_green'])
        self.harvest_count_label.pack()
        
        tk.Label(stats_frame, text="个苹果已采摘", font=('Arial', 14),
                fg=self.colors['primary_dark'], bg=self.colors['light_green']).pack()
        
        stats_detail_frame = tk.Frame(stats_frame, bg=self.colors['light_green'])
        stats_detail_frame.pack(fill='x', padx=10, pady=10)
        
        self.efficiency_label = tk.Label(stats_detail_frame, text="效率: 25.4/小时",
                                        font=('Arial', 12), fg=self.colors['gray'],
                                        bg=self.colors['light_green'])
        self.efficiency_label.pack(anchor='w')
        
        self.area_label = tk.Label(stats_detail_frame, text="面积: 3.2亩",
                                 font=('Arial', 12), fg=self.colors['gray'],
                                 bg=self.colors['light_green'])
        self.area_label.pack(anchor='w')
    
    def create_camera_area(self):
        """创建摄像头区域"""
        camera_frame = tk.Frame(self.main_frame, bg=self.colors['white'], 
                              width=520, height=370, relief='raised', bd=2)
        camera_frame.pack(side='left', padx=10, pady=20)
        camera_frame.pack_propagate(False)
        
        # 摄像头标题
        title_frame = tk.Frame(camera_frame, bg=self.colors['primary'], height=40)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        tk.Label(title_frame, text="📹 实时监控", font=('Arial', 16, 'bold'),
                fg='white', bg=self.colors['primary']).pack(side='left', padx=20, pady=8)
        
        self.fps_label = tk.Label(title_frame, text="FPS: 29.8 | 1280×480",
                                font=('Arial', 14), fg='#c8e6c9',
                                bg=self.colors['primary'])
        self.fps_label.pack(side='right', padx=20, pady=8)
        
        # 摄像头画面
        self.camera_canvas = tk.Canvas(camera_frame, width=480, height=280, bg='black')
        self.camera_canvas.pack(padx=20, pady=(20, 10))
        
        # 默认显示文字
        self.camera_canvas.create_text(240, 140, text="摄像头画面", 
                                     fill='gray', font=('Arial', 18))
        
        # 检测信息
        info_frame = tk.Frame(camera_frame, bg=self.colors['white'])
        info_frame.pack(fill='x', padx=20, pady=10)
        
        self.detection_info_label = tk.Label(info_frame, 
                                           text="检测到 2 个目标 | 最近距离: 0.65m | 准确率: 96.5%",
                                           font=('Arial', 14), fg=self.colors['primary_dark'],
                                           bg=self.colors['white'])
        self.detection_info_label.pack()
    
    def create_status_panel(self):
        """创建右侧状态面板"""
        status_panel_frame = tk.Frame(self.main_frame, bg=self.colors['background'])
        status_panel_frame.pack(side='left', fill='y', padx=10, pady=20)
        
        # 系统状态卡片
        self.create_system_status_card(status_panel_frame)
        
        # 位置信息卡片
        self.create_position_card(status_panel_frame)
    
    def create_system_status_card(self, parent):
        """创建系统状态卡片"""
        card_frame = tk.Frame(parent, bg=self.colors['white'], width=184, height=180,
                            relief='raised', bd=2)
        card_frame.pack(pady=(0, 20))
        card_frame.pack_propagate(False)
        
        # 标题
        title_frame = tk.Frame(card_frame, bg=self.colors['primary'], height=35)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        tk.Label(title_frame, text="🔋 系统状态", font=('Arial', 14, 'bold'),
                fg='white', bg=self.colors['primary']).pack(pady=8)
        
        content_frame = tk.Frame(card_frame, bg=self.colors['white'])
        content_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # 电池状态
        tk.Label(content_frame, text="电池电量", font=('Arial', 12, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w')
        
        battery_frame = tk.Frame(content_frame, bg=self.colors['white'])
        battery_frame.pack(fill='x', pady=2)
        
        self.battery_canvas = tk.Canvas(battery_frame, width=144, height=12, 
                                      bg='#e0e0e0', highlightthickness=0)
        self.battery_canvas.pack(side='left')
        self.battery_bar = self.battery_canvas.create_rectangle(2, 2, 108, 10, 
                                                              fill=self.colors['primary'], outline='')
        
        self.battery_label = tk.Label(battery_frame, text="75%", font=('Arial', 12, 'bold'),
                                    fg=self.colors['primary'], bg=self.colors['white'])
        self.battery_label.pack(side='right')
        
        # CPU使用率
        tk.Label(content_frame, text="CPU使用率", font=('Arial', 12, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w', pady=(10, 0))
        
        cpu_frame = tk.Frame(content_frame, bg=self.colors['white'])
        cpu_frame.pack(fill='x', pady=2)
        
        self.cpu_canvas = tk.Canvas(cpu_frame, width=144, height=12,
                                  bg='#e0e0e0', highlightthickness=0)
        self.cpu_canvas.pack(side='left')
        self.cpu_bar = self.cpu_canvas.create_rectangle(2, 2, 72, 10,
                                                      fill=self.colors['orange'], outline='')
        
        self.cpu_label = tk.Label(cpu_frame, text="52%", font=('Arial', 12, 'bold'),
                                fg=self.colors['orange'], bg=self.colors['white'])
        self.cpu_label.pack(side='right')
        
        # 温度
        tk.Label(content_frame, text="系统温度", font=('Arial', 12, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w', pady=(10, 0))
        
        temp_frame = tk.Frame(content_frame, bg=self.colors['white'])
        temp_frame.pack(fill='x')
        
        self.temp_label = tk.Label(temp_frame, text="28°C", font=('Arial', 20, 'bold'),
                                 fg=self.colors['primary'], bg=self.colors['white'])
        self.temp_label.pack(anchor='w')
        
        tk.Label(temp_frame, text="正常工作温度", font=('Arial', 10),
                fg=self.colors['gray'], bg=self.colors['white']).pack(anchor='w')
    
    def create_position_card(self, parent):
        """创建位置信息卡片"""
        card_frame = tk.Frame(parent, bg=self.colors['white'], width=184, height=170,
                            relief='raised', bd=2)
        card_frame.pack()
        card_frame.pack_propagate(False)
        
        # 标题
        title_frame = tk.Frame(card_frame, bg=self.colors['primary'], height=35)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        tk.Label(title_frame, text="📍 位置信息", font=('Arial', 14, 'bold'),
                fg='white', bg=self.colors['primary']).pack(pady=8)
        
        content_frame = tk.Frame(card_frame, bg=self.colors['white'])
        content_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # 当前位置
        tk.Label(content_frame, text="当前位置", font=('Arial', 12, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w')
        
        self.lat_label = tk.Label(content_frame, text="纬度: 34.938500°", font=('Arial', 11),
                                fg=self.colors['gray'], bg=self.colors['white'])
        self.lat_label.pack(anchor='w')
        
        self.lon_label = tk.Label(content_frame, text="经度: 108.241500°", font=('Arial', 11),
                                fg=self.colors['gray'], bg=self.colors['white'])
        self.lon_label.pack(anchor='w')
        
        self.area_name_label = tk.Label(content_frame, text="区域: 苹果园3号地块", font=('Arial', 11),
                                      fg=self.colors['gray'], bg=self.colors['white'])
        self.area_name_label.pack(anchor='w')
        
        # 工作状态
        tk.Label(content_frame, text="工作状态", font=('Arial', 12, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w', pady=(10, 0))
        
        self.work_status_label = tk.Label(content_frame, text="待机中", font=('Arial', 11, 'bold'),
                                        fg=self.colors['primary'], bg=self.colors['white'])
        self.work_status_label.pack(anchor='w')
        
        self.work_time_label = tk.Label(content_frame, text="工作时长: 5小时23分", font=('Arial', 11),
                                      fg=self.colors['gray'], bg=self.colors['white'])
        self.work_time_label.pack(anchor='w')
        
        self.speed_label = tk.Label(content_frame, text="移动速度: 0.0 m/s", font=('Arial', 11),
                                  fg=self.colors['gray'], bg=self.colors['white'])
        self.speed_label.pack(anchor='w')
    
    def create_data_panel(self):
        """创建底部数据面板"""
        data_frame = tk.Frame(self.main_frame, bg=self.colors['white'], height=90,
                            relief='raised', bd=2)
        data_frame.pack(fill='x', padx=20, pady=(0, 20))
        data_frame.pack_propagate(False)
        
        tk.Label(data_frame, text="🌱 农场数据总览", font=('Arial', 16, 'bold'),
                fg=self.colors['primary_dark'], bg=self.colors['white']).pack(anchor='w', padx=40, pady=(10, 0))
        
        # 数据卡片容器
        cards_frame = tk.Frame(data_frame, bg=self.colors['white'])
        cards_frame.pack(fill='x', padx=40, pady=10)
        
        # 创建数据卡片
        self.data_cards = []
        
        card_data = [
            ("累计采摘", "2,543 个", "total_harvested"),
            ("作业精度", "96.5%", "accuracy"),
            ("信号强度", "75%", "signal_strength"),
            ("运行天数", "45 天", "uptime_days"),
            ("健康状态", "优秀", "health_status")
        ]
        
        for i, (title, value, key) in enumerate(card_data):
            card = tk.Frame(cards_frame, bg=self.colors['light_green'], width=140, height=40,
                          relief='solid', bd=1)
            card.pack(side='left', padx=5)
            card.pack_propagate(False)
            
            tk.Label(card, text=title, font=('Arial', 10),
                    fg=self.colors['primary_dark'], bg=self.colors['light_green']).pack(anchor='w', padx=10, pady=(5, 0))
            
            value_label = tk.Label(card, text=value, font=('Arial', 16, 'bold'),
                                 fg=self.colors['primary'], bg=self.colors['light_green'])
            value_label.pack(anchor='w', padx=10)
            
            self.data_cards.append((key, value_label))
    
    def setup_layout(self):
        """设置布局"""
        self.main_frame.pack(fill='both', expand=True)
    
    # ===================== 预留的ROS2接口方法 =====================
    
    def init_ros2_interface(self):
        """
        初始化ROS2接口
        TODO: 在这里添加ROS2节点初始化代码
        """
        print("TODO: 初始化ROS2接口")
        # 示例：
        # rclpy.init()
        # self.ros_node = RobotGUINode()
        # self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        # self.ros_thread.start()
    
    def publish_mode_command(self, mode, auto_harvest=True):
        """
        发布模式切换命令到ROS2
        TODO: 实现向'robot/mode'话题发布消息
        """
        print(f"TODO: 发布模式命令 - mode: {mode}, auto_harvest: {auto_harvest}")
        # 示例：
        # mode_data = {"mode": mode, "auto_harvest": auto_harvest}
        # msg = String()
        # msg.data = json.dumps(mode_data)
        # self.ros_node.mode_pub.publish(msg)
    
    def publish_harvest_command(self, start=True):
        """
        发布采摘控制命令到ROS2
        TODO: 实现向'robot/harvest_command'话题发布消息
        """
        print(f"TODO: 发布采摘命令 - start: {start}")
        # 示例：
        # harvest_cmd = HarvestCommand()
        # harvest_cmd.start_harvest = start
        # self.ros_node.harvest_cmd_pub.publish(harvest_cmd)
    
    def publish_emergency_stop(self):
        """
        发布紧急停止命令到ROS2
        TODO: 实现紧急停止功能
        """
        print("TODO: 发布紧急停止命令")
        # 示例：
        # robot_cmd = RobotCommand()
        # robot_cmd.emergency_stop = True
        # self.ros_node.robot_cmd_pub.publish(robot_cmd)
    
    def start_system_launch(self):
        """
        启动ROS2 launch文件
        TODO: 实现启动integrated_system.launch.py
        """
        print("TODO: 启动ROS2系统")
        # 示例：
        # import subprocess
        # self.launch_process = subprocess.Popen([
        #     'ros2', 'launch', 'integrated_system.launch.py'
        # ])
    
    def stop_system_launch(self):
        """
        停止ROS2系统
        TODO: 实现停止launch进程
        """
        print("TODO: 停止ROS2系统")
        # 示例：
        # if self.launch_process:
        #     self.launch_process.terminate()
        #     self.launch_process = None
    
    def subscribe_robot_status(self, status_data):
        """
        ROS2机器人状态消息回调
        TODO: 订阅'robot/status'话题
        """
        print("TODO: 处理机器人状态更新")
        # 示例：
        # self.robot_data.update({
        #     'battery_level': status_data.battery_level,
        #     'cpu_usage': status_data.cpu_usage,
        #     'harvested_today': status_data.today_harvested,
        #     ...
        # })
    
    def subscribe_detection_info(self, detection_data):
        """
        ROS2检测信息消息回调
        TODO: 订阅'bottle_detection/info'话题
        """
        print("TODO: 处理检测信息更新")
        # 示例：
        # data = json.loads(detection_data)
        # self.robot_data['fps'] = data.get('fps', 0)
        # self.robot_data['detection_count'] = data.get('total_count', 0)
    
    def subscribe_camera_image(self, image_data):
        """
        ROS2摄像头图像消息回调
        TODO: 订阅'bottle_detection/compressed_image'话题
        """
        print("TODO: 处理摄像头图像更新")
        # 示例：
        # np_arr = np.frombuffer(image_data.data, np.uint8)
        # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # self.update_camera_display(cv_image)
    
    def update_camera_display(self, cv_image):
        """
        更新摄像头显示
        TODO: 将OpenCV图像显示到Canvas上
        """
        print("TODO: 更新摄像头显示")
        # 示例：
        # resized = cv2.resize(cv_image, (480, 280))
        # rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # pil_image = Image.fromarray(rgb_image)
        # photo = ImageTk.PhotoImage(pil_image)
        # self.camera_canvas.create_image(0, 0, anchor='nw', image=photo)
        # self.camera_canvas.image = photo
    
    # ===================== 界面事件处理方法 =====================
    
    def toggle_system(self):
        """切换系统状态"""
        if not self.system_running:
            print("启动系统...")
            self.start_system()
        else:
            print("停止系统...")
            self.stop_system()
    
    def start_system(self):
        """启动系统"""
        # TODO: 调用start_system_launch()
        self.start_system_launch()
        
        # 更新界面状态
        self.system_running = True
        self.system_button.config(text="停止系统", bg=self.colors['red'])
        self.status_indicator.itemconfig(self.status_circle, fill='green')
        self.status_label.config(text="运行中")
        
        # TODO: 发布初始模式
        self.publish_mode_command(self.current_mode, self.auto_harvest_active)
    
    def stop_system(self):
        """停止系统"""
        # TODO: 调用stop_system_launch()
        self.stop_system_launch()
        
        # 更新界面状态
        self.system_running = False
        self.system_button.config(text="启动系统", bg=self.colors['primary'])
        self.status_indicator.itemconfig(self.status_circle, fill='red')
        self.status_label.config(text="已停止")
        
        # 清除摄像头画面
        self.camera_canvas.delete("all")
        self.camera_canvas.create_text(240, 140, text="摄像头画面", 
                                     fill='gray', font=('Arial', 18))
    
    def set_mode(self, mode):
        """设置工作模式"""
        self.current_mode = mode
        
        if mode == "manual":
            self.manual_button.config(bg=self.colors['primary'], fg='white', font=('Arial', 14, 'bold'))
            self.auto_button.config(bg=self.colors['light_green'], fg=self.colors['primary'], font=('Arial', 14))
            self.auto_harvest_active = False
        else:
            self.auto_button.config(bg=self.colors['primary'], fg='white', font=('Arial', 14, 'bold'))
            self.manual_button.config(bg=self.colors['light_green'], fg=self.colors['primary'], font=('Arial', 14))
            self.auto_harvest_active = True
        
        # TODO: 发布模式消息
        self.publish_mode_command(mode, self.auto_harvest_active)
        
        print(f"切换到{mode}模式")
    
    def toggle_harvest(self):
        """切换采摘状态"""
        if not self.auto_harvest_active:
            self.auto_harvest_active = True
            self.harvest_button.config(text="🍎 停止采摘", bg=self.colors['orange'])
            
            # TODO: 发布开始采摘命令
            self.publish_harvest_command(True)
            
            print("开始采摘")
        else:
            self.auto_harvest_active = False
            self.harvest_button.config(text="🍎 开始采摘", bg=self.colors['secondary'])
            
            # TODO: 发布停止采摘命令
            self.publish_harvest_command(False)
            
            print("停止采摘")
    
    def emergency_stop(self):
        """紧急停止"""
        if messagebox.askquestion("确认", "确定要执行紧急停止吗？") == 'yes':
            # TODO: 发布紧急停止命令
            self.publish_emergency_stop()
            
            # 重置界面状态
            self.auto_harvest_active = False
            self.harvest_button.config(text="🍎 开始采摘", bg=self.colors['secondary'])
            
            print("执行紧急停止")
            messagebox.showinfo("完成", "紧急停止命令已发送")
    
    # ===================== 数据更新相关方法 =====================
    
    def start_update_thread(self):
        """启动数据更新线程"""
        update_thread = threading.Thread(target=self.update_loop, daemon=True)
        update_thread.start()
    
    def update_loop(self):
        """数据更新循环"""
        while True:
            try:
                # 在主线程中更新GUI
                self.root.after(0, self.update_gui)
                time.sleep(0.1)  # 100ms更新一次
            except Exception as e:
                print(f"更新循环错误: {e}")
                time.sleep(1)
    
    def update_gui(self):
        """更新GUI显示"""
        try:
            # 更新时间
            current_time = datetime.now().strftime("%H:%M")
            self.time_label.config(text=current_time)
            
            # TODO: 这里的数据应该从ROS2消息中获取
            data = self.robot_data
            
            # 更新今日采摘数量
            self.harvest_count_label.config(text=str(data['harvested_today']))
            
            # 更新效率和面积
            if data['working_hours'] > 0:
                efficiency = data['harvested_today'] / data['working_hours']
                self.efficiency_label.config(text=f"效率: {efficiency:.1f}/小时")
            
            self.area_label.config(text=f"面积: {data['working_area']:.1f}亩")
            
            # 更新电池状态
            battery_width = int(144 * data['battery_level'] / 100)
            self.battery_canvas.coords(self.battery_bar, 2, 2, battery_width, 10)
            self.battery_label.config(text=f"{data['battery_level']:.0f}%")
            
            # 更新CPU状态
            cpu_width = int(144 * data['cpu_usage'] / 100)
            self.cpu_canvas.coords(self.cpu_bar, 2, 2, cpu_width, 10)
            self.cpu_label.config(text=f"{data['cpu_usage']:.0f}%")
            
            # 更新温度
            self.temp_label.config(text=f"{data['temperature']:.0f}°C")
            
            # 更新位置信息
            self.lat_label.config(text=f"纬度: {data['latitude']:.6f}°")
            self.lon_label.config(text=f"经度: {data['longitude']:.6f}°")
            
            # 更新工作状态
            status_text = data.get('status_text', '待机中')
            if self.system_running:
                if self.auto_harvest_active:
                    status_text = "正在采摘"
                elif self.current_mode == "auto":
                    status_text = "自动模式"
                else:
                    status_text = "手动模式"
            else:
                status_text = "待机中"
            
            self.work_status_label.config(text=status_text)
            
            # 更新工作时长
            hours = int(data['working_hours'])
            minutes = int((data['working_hours'] - hours) * 60)
            self.work_time_label.config(text=f"工作时长: {hours}小时{minutes}分")
            
            # 更新速度
            self.speed_label.config(text=f"移动速度: {data['current_speed']:.1f} m/s")
            
            # 更新检测信息
            info_text = f"检测到 {data['detection_count']} 个目标 | 最近距离: {data['nearest_distance']:.2f}m | 准确率: {data['accuracy']:.1f}%"
            self.detection_info_label.config(text=info_text)
            
            # 更新FPS
            self.fps_label.config(text=f"FPS: {data['fps']:.1f} | 1280×480")
            
            # 更新底部数据卡片
            for key, label in self.data_cards:
                if key == 'total_harvested':
                    label.config(text=f"{data['total_harvested']:,} 个")
                elif key == 'accuracy':
                    label.config(text=f"{data['accuracy']:.1f}%")
                elif key == 'signal_strength':
                    label.config(text=f"{data['signal_strength']:.0f}%")
                elif key == 'uptime_days':
                    uptime_days = max(1, int(data['working_hours'] / 8))
                    label.config(text=f"{uptime_days} 天")
                elif key == 'health_status':
                    if data['temperature'] < 35 and data['cpu_usage'] < 80 and data['battery_level'] > 20:
                        label.config(text="优秀")
                    elif data['temperature'] < 40 and data['cpu_usage'] < 90 and data['battery_level'] > 10:
                        label.config(text="良好")
                    else:
                        label.config(text="注意")
            
        except Exception as e:
            print(f"更新GUI错误: {e}")
    
    def run(self):
        """运行主循环"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            # TODO: 初始化ROS2接口
            # self.init_ros2_interface()
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_closing()
    
    def on_closing(self):
        """关闭程序"""
        try:
            # TODO: 停止系统和清理ROS2资源
            if self.system_running:
                self.stop_system()
            
            # TODO: 关闭ROS2节点
            # if hasattr(self, 'ros_node'):
            #     self.ros_node.destroy_node()
            #     rclpy.shutdown()
            
            self.root.destroy()
            
        except Exception as e:
            print(f"关闭程序时出错: {e}")


def main():
    """主函数"""
    try:
        app = AgriculturalRobotGUI()
        app.run()
    except Exception as e:
        print(f"程序启动失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()