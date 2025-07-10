#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2瓶子检测系统调试可视化程序 - Material Design版
修复画面闪烁，美化界面，优化布局
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tkinter as tk
from tkinter import ttk, scrolledtext, font
import threading
import time
import json
import numpy as np
from datetime import datetime
from collections import deque
import cv2
from PIL import Image, ImageTk, ImageDraw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as mpatches

# ROS2消息类型
from sensor_msgs.msg import CompressedImage, Image as RosImage
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import String, Float32, Int32, Bool
from bottle_detection_msgs.msg import (
    BottleDetection, RobotStatus, RobotCommand, 
    HarvestCommand, ServoCommand, ServoStatus
)


# Material Design 配色方案
class MaterialColors:
    # 主色调
    PRIMARY = '#1976D2'  # Blue 700
    PRIMARY_DARK = '#1565C0'  # Blue 800
    PRIMARY_LIGHT = '#42A5F5'  # Blue 400
    
    # 强调色
    ACCENT = '#00BCD4'  # Cyan 500
    ACCENT_DARK = '#00ACC1'  # Cyan 600
    
    # 背景色
    BG_PRIMARY = '#FFFFFF'
    BG_SECONDARY = '#F5F5F5'  # Grey 100
    BG_TERTIARY = '#E0E0E0'  # Grey 300
    
    # 文字颜色
    TEXT_PRIMARY = '#212121'  # Grey 900
    TEXT_SECONDARY = '#757575'  # Grey 600
    TEXT_DISABLED = '#BDBDBD'  # Grey 400
    
    # 状态颜色
    SUCCESS = '#4CAF50'  # Green 500
    WARNING = '#FF9800'  # Orange 500
    ERROR = '#F44336'  # Red 500
    INFO = '#2196F3'  # Blue 500
    
    # 阴影
    SHADOW = '#00000014'


class ModernLabel(tk.Frame):
    """现代化标签组件"""
    def __init__(self, parent, title, value="--", **kwargs):
        super().__init__(parent, bg=MaterialColors.BG_PRIMARY, **kwargs)
        
        # 标题
        self.title_label = tk.Label(self, text=title, 
                                   bg=MaterialColors.BG_PRIMARY,
                                   fg=MaterialColors.TEXT_SECONDARY,
                                   font=('Roboto', 9))
        self.title_label.pack(anchor='w')
        
        # 值
        self.value_label = tk.Label(self, text=value,
                                   bg=MaterialColors.BG_PRIMARY,
                                   fg=MaterialColors.TEXT_PRIMARY,
                                   font=('Roboto', 12, 'bold'))
        self.value_label.pack(anchor='w')
    
    def set_value(self, value, color=None):
        self.value_label.config(text=value)
        if color:
            self.value_label.config(fg=color)


class MaterialCard(tk.Frame):
    """Material Design 卡片组件"""
    def __init__(self, parent, title="", **kwargs):
        super().__init__(parent, bg=MaterialColors.BG_PRIMARY, 
                        relief=tk.FLAT, **kwargs)
        
        # 添加内边距
        self.container = tk.Frame(self, bg=MaterialColors.BG_PRIMARY)
        self.container.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)
        
        if title:
            self.title_label = tk.Label(self.container, text=title,
                                       bg=MaterialColors.BG_PRIMARY,
                                       fg=MaterialColors.TEXT_PRIMARY,
                                       font=('Roboto', 14, 'bold'))
            self.title_label.pack(anchor='w', pady=(0, 12))
        
        # 模拟阴影效果
        self.configure(highlightbackground=MaterialColors.BG_TERTIARY,
                      highlightthickness=1)


class DebugVisualizerNode(Node):
    """调试可视化节点"""
    
    def __init__(self, gui):
        super().__init__('debug_visualizer_node')
        self.gui = gui
        
        # QoS配置
        self.qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 数据存储
        self.data = {
            'fps': 0.0,
            'detection_count': 0,
            'nearest_distance': -1.0,
            'robot_mode': 'unknown',
            'robot_speed': 0.0,
            'battery_level': 0.0,
            'cpu_usage': 0.0,
            'servo_positions': [],
            'harvest_state': 0,
            'last_update': {},
            'motor_commands': [],
            'servo_commands': [],
        }
        
        # 历史数据（用于绘图）
        self.history_length = 100
        self.distance_history = deque(maxlen=self.history_length)
        self.fps_history = deque(maxlen=self.history_length)
        self.detection_history = deque(maxlen=self.history_length)
        self.time_history = deque(maxlen=self.history_length)
        
        # 命令历史
        self.command_history = deque(maxlen=50)
        
        # 创建订阅者
        self._create_subscribers()
        
        # 更新定时器 - 降低频率减少闪烁
        self.create_timer(0.2, self.update_gui)  # 从0.1改为0.2秒
        
        self.get_logger().info('调试可视化节点已启动')
    
    def _create_subscribers(self):
        """创建所有订阅者"""
        # 图像订阅
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            'bottle_detection/compressed_image',
            self.compressed_image_callback,
            self.qos_best_effort
        )
        
        # 检测结果订阅
        self.detection_sub = self.create_subscription(
            BottleDetection,
            'bottle_detection/detection',
            self.detection_callback,
            10
        )
        
        self.detection_info_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_info_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        self.count_sub = self.create_subscription(
            Int32,
            'bottle_detection/count',
            self.count_callback,
            10
        )
        
        # 机器人状态订阅
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            'robot/status',
            self.robot_status_callback,
            10
        )
        
        self.robot_mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.robot_mode_callback,
            10
        )
        
        # 控制命令订阅
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.robot_cmd_sub = self.create_subscription(
            RobotCommand,
            'robot/command',
            self.robot_command_callback,
            10
        )
        
        # 舵机相关订阅
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            'servo/status',
            self.servo_status_callback,
            10
        )
        
        self.servo_command_sub = self.create_subscription(
            ServoCommand,
            'servo/command',
            self.servo_command_callback,
            10
        )
        
        self.tracking_target_sub = self.create_subscription(
            Point,
            'servo/tracking_target',
            self.tracking_target_callback,
            10
        )
        
        # 采摘状态订阅
        self.harvest_status_sub = self.create_subscription(
            String,
            'harvest/status',
            self.harvest_status_callback,
            10
        )
        
        self.harvest_command_sub = self.create_subscription(
            HarvestCommand,
            'robot/harvest_command',
            self.harvest_command_callback,
            10
        )
    
    def compressed_image_callback(self, msg):
        """压缩图像回调"""
        try:
            # 解码JPEG图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # 转换为RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                # 调整大小以适应GUI
                height, width = cv_image.shape[:2]
                if width > 800:  # 增大显示尺寸
                    scale = 800 / width
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    cv_image = cv2.resize(cv_image, (new_width, new_height))
                
                # 更新GUI
                self.gui.update_image(cv_image)
                
            self.data['last_update']['image'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')
    
    def detection_callback(self, msg):
        """检测结果回调"""
        self.data['bottle_detected'] = msg.bottle_detected
        self.data['bottle_count'] = msg.bottle_count
        
        if msg.bottle_detected:
            self.data['nearest_bottle'] = {
                'x': msg.nearest_bottle_x,
                'y': msg.nearest_bottle_y,
                'confidence': msg.confidence,
                'distance': msg.distance,
                'bbox': [msg.bbox_left, msg.bbox_top, msg.bbox_right, msg.bbox_bottom],
                'position_3d': [msg.position_x, msg.position_y, msg.position_z],
                'status': msg.status
            }
        else:
            self.data['nearest_bottle'] = None
        
        # 更新历史
        current_time = time.time()
        self.time_history.append(current_time)
        self.detection_history.append(msg.bottle_count)
        if msg.distance > 0:
            self.distance_history.append(msg.distance)
        else:
            self.distance_history.append(0)
        
        self.data['last_update']['detection'] = current_time
    
    def detection_info_callback(self, msg):
        """检测信息回调"""
        try:
            info = json.loads(msg.data)
            self.data['fps'] = info.get('fps', 0.0)
            self.fps_history.append(self.data['fps'])
            self.data['last_update']['info'] = time.time()
        except:
            pass
    
    def distance_callback(self, msg):
        """距离回调"""
        self.data['nearest_distance'] = msg.data
        self.data['last_update']['distance'] = time.time()
    
    def count_callback(self, msg):
        """计数回调"""
        self.data['detection_count'] = msg.data
        self.data['last_update']['count'] = time.time()
    
    def robot_status_callback(self, msg):
        """机器人状态回调"""
        self.data['battery_level'] = msg.battery_level
        self.data['cpu_usage'] = msg.cpu_usage
        self.data['current_speed'] = msg.current_speed
        self.data['current_direction'] = msg.current_direction
        self.data['position'] = {
            'x': msg.position_x,
            'y': msg.position_y,
            'lat': msg.latitude,
            'lon': msg.longitude
        }
        self.data['harvested_count'] = msg.harvested_count
        self.data['is_moving'] = msg.is_moving
        self.data['emergency_stop'] = msg.emergency_stop
        self.data['last_update']['robot_status'] = time.time()
    
    def robot_mode_callback(self, msg):
        """机器人模式回调"""
        try:
            mode_data = json.loads(msg.data)
            self.data['robot_mode'] = mode_data.get('mode', 'unknown')
            self.data['auto_harvest'] = mode_data.get('auto_harvest', False)
            self.data['last_update']['mode'] = time.time()
        except:
            pass
    
    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        self.data['cmd_vel'] = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }
        self.data['last_update']['cmd_vel'] = time.time()
        
        # 解析为运动命令
        motion_cmd = self._parse_motion_command(msg)
        if motion_cmd:
            self.command_history.append({
                'time': datetime.now(),
                'type': 'motor',
                'command': motion_cmd,
                'icon': '🚗'
            })
    
    def robot_command_callback(self, msg):
        """机器人命令回调"""
        self.data['robot_command'] = {
            'command': msg.command,
            'speed': msg.speed,
            'emergency_stop': msg.emergency_stop
        }
        self.data['last_update']['robot_command'] = time.time()
        
        # 添加到命令历史
        cmd_text = f"{msg.command} 速度:{msg.speed:.1f}"
        if msg.emergency_stop:
            cmd_text += " [紧急停止]"
        
        self.command_history.append({
            'time': datetime.now(),
            'type': 'robot',
            'command': cmd_text,
            'icon': '🤖'
        })
    
    def servo_command_callback(self, msg):
        """舵机命令回调"""
        if msg.stop:
            cmd_text = f"舵机{msg.servo_id} 停止"
        else:
            cmd_text = f"舵机{msg.servo_id} 位置:{msg.position} 时间:{msg.time_ms}ms"
        
        self.command_history.append({
            'time': datetime.now(),
            'type': 'servo',
            'command': cmd_text,
            'icon': '⚙️'
        })
        
        self.data['last_update']['servo_command'] = time.time()
    
    def tracking_target_callback(self, msg):
        """舵机跟踪目标回调"""
        cmd_text = f"跟踪目标: ({int(msg.x)}, {int(msg.y)})"
        
        self.command_history.append({
            'time': datetime.now(),
            'type': 'tracking',
            'command': cmd_text,
            'icon': '🎯'
        })
        
        self.data['last_update']['tracking'] = time.time()
    
    def servo_status_callback(self, msg):
        """舵机状态回调"""
        self.data['servo_positions'] = list(msg.servo_positions)
        self.data['harvest_state'] = msg.harvest_state
        self.data['tracking_active'] = msg.tracking_active
        self.data['last_update']['servo'] = time.time()
    
    def harvest_command_callback(self, msg):
        """采摘命令回调"""
        if msg.start_harvest:
            cmd_text = "开始采摘动作"
        elif msg.stop_harvest:
            cmd_text = "停止采摘"
        else:
            cmd_text = "未知采摘命令"
        
        self.command_history.append({
            'time': datetime.now(),
            'type': 'harvest',
            'command': cmd_text,
            'icon': '🌿'
        })
        
        self.data['last_update']['harvest_command'] = time.time()
    
    def harvest_status_callback(self, msg):
        """采摘状态回调"""
        try:
            status = json.loads(msg.data)
            self.data['harvest_status'] = status
            self.data['last_update']['harvest'] = time.time()
        except:
            pass
    
    def _parse_motion_command(self, twist_msg):
        """解析Twist消息为运动命令"""
        linear_x = twist_msg.linear.x
        angular_z = twist_msg.angular.z
        
        # 判断运动类型
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            return "停止"
        elif abs(linear_x) > abs(angular_z):
            if linear_x > 0:
                return f"前进 {linear_x:.2f}m/s"
            else:
                return f"后退 {abs(linear_x):.2f}m/s"
        else:
            if angular_z > 0:
                return f"左转 {angular_z:.2f}rad/s"
            else:
                return f"右转 {abs(angular_z):.2f}rad/s"
    
    def update_gui(self):
        """更新GUI显示"""
        self.gui.update_data(self.data, self.distance_history, 
                           self.fps_history, self.detection_history,
                           self.command_history, self.time_history)


class DebugVisualizerGUI:
    """调试可视化GUI - Material Design风格"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS2 瓶子检测系统调试器")
        self.root.geometry("1600x1000")
        self.root.configure(bg=MaterialColors.BG_SECONDARY)
        
        # 设置字体
        self.setup_fonts()
        
        # 设置样式
        self.setup_styles()
        
        # 创建主容器
        self.main_container = tk.Frame(self.root, bg=MaterialColors.BG_SECONDARY)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # 创建布局
        self.create_layout()
        
        # 数据显示变量
        self.image_photo = None
        self.last_image_update = 0
        
        # 图表相关
        self.figure = None
        self.canvas = None
        self.setup_charts()
        
    def setup_fonts(self):
        """设置字体"""
        # 尝试使用系统字体
        available_fonts = font.families()
        
        # 优先使用的字体列表
        preferred_fonts = ['Roboto', 'Helvetica Neue', 'Arial', 'Sans']
        
        self.main_font = 'Arial'
        for f in preferred_fonts:
            if f in available_fonts:
                self.main_font = f
                break
    
    def setup_styles(self):
        """设置ttk样式"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # 配置样式
        style.configure('Card.TFrame', 
                       background=MaterialColors.BG_PRIMARY,
                       relief='flat',
                       borderwidth=1)
        
        style.configure('Primary.TButton',
                       background=MaterialColors.PRIMARY,
                       foreground='white',
                       borderwidth=0,
                       focuscolor='none')
        
        style.map('Primary.TButton',
                 background=[('active', MaterialColors.PRIMARY_DARK)])
    
    def create_layout(self):
        """创建布局"""
        # 顶部标题栏
        self.create_header()
        
        # 主要内容区域
        content_frame = tk.Frame(self.main_container, bg=MaterialColors.BG_SECONDARY)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=(20, 0))
        
        # 左右分栏
        # 左侧 - 视觉检测
        left_frame = tk.Frame(content_frame, bg=MaterialColors.BG_SECONDARY)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # 右侧 - 系统信息
        right_frame = tk.Frame(content_frame, bg=MaterialColors.BG_SECONDARY)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # 创建各个组件
        self.create_vision_panel(left_frame)
        self.create_status_panel(right_frame)
        self.create_command_panel(right_frame)
        self.create_log_panel(right_frame)
    
    def create_header(self):
        """创建顶部标题栏"""
        header = tk.Frame(self.main_container, bg=MaterialColors.PRIMARY, height=60)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        
        # 标题
        title = tk.Label(header, text="🤖 ROS2 瓶子检测系统调试器",
                        bg=MaterialColors.PRIMARY, fg='white',
                        font=(self.main_font, 20, 'bold'))
        title.pack(side=tk.LEFT, padx=20, pady=15)
        
        # 状态指示器
        self.status_frame = tk.Frame(header, bg=MaterialColors.PRIMARY)
        self.status_frame.pack(side=tk.RIGHT, padx=20, pady=15)
        
        self.connection_status = tk.Label(self.status_frame, text="● 已连接",
                                        bg=MaterialColors.PRIMARY, 
                                        fg=MaterialColors.SUCCESS,
                                        font=(self.main_font, 12))
        self.connection_status.pack(side=tk.RIGHT)
    
    def create_vision_panel(self, parent):
        """创建视觉检测面板"""
        # 图像显示卡片
        image_card = MaterialCard(parent, title="📷 实时图像")
        image_card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 图像显示
        self.image_label = tk.Label(image_card.container, bg=MaterialColors.BG_SECONDARY)
        self.image_label.pack(fill=tk.BOTH, expand=True)
        
        # 创建初始图像
        initial_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.update_image(initial_image)
        
        # 检测信息卡片
        info_card = MaterialCard(parent, title="📊 检测信息")
        info_card.pack(fill=tk.X, pady=(0, 10))
        
        # 创建信息网格
        info_grid = tk.Frame(info_card.container, bg=MaterialColors.BG_PRIMARY)
        info_grid.pack(fill=tk.X)
        
        # 信息项
        self.detection_labels = {}
        info_items = [
            ('fps', 'FPS', 0, 0),
            ('detection_count', '检测数量', 0, 1),
            ('nearest_distance', '最近距离', 1, 0),
            ('confidence', '置信度', 1, 1),
            ('status', '状态', 2, 0),
            ('position', '3D位置', 2, 1)
        ]
        
        for key, title, row, col in info_items:
            label = ModernLabel(info_grid, title)
            label.grid(row=row, column=col, padx=10, pady=5, sticky='w')
            self.detection_labels[key] = label
        
        # 实时数据图表卡片
        chart_card = MaterialCard(parent, title="📈 实时数据")
        chart_card.pack(fill=tk.BOTH, expand=True)
        
        # 图表将在这里创建
        self.chart_frame = chart_card.container
    
    def create_status_panel(self, parent):
        """创建系统状态面板"""
        status_card = MaterialCard(parent, title="🔧 系统状态")
        status_card.pack(fill=tk.X, pady=(0, 10))
        
        # 状态网格
        status_grid = tk.Frame(status_card.container, bg=MaterialColors.BG_PRIMARY)
        status_grid.pack(fill=tk.X)
        
        self.status_labels = {}
        status_items = [
            ('robot_mode', '运行模式', 0, 0),
            ('battery_level', '电池电量', 0, 1),
            ('cpu_usage', 'CPU使用率', 1, 0),
            ('current_speed', '当前速度', 1, 1),
            ('harvested_count', '采摘数量', 2, 0),
            ('position_xy', '位置坐标', 2, 1)
        ]
        
        for key, title, row, col in status_items:
            label = ModernLabel(status_grid, title)
            label.grid(row=row, column=col, padx=10, pady=5, sticky='w')
            self.status_labels[key] = label
    
    def create_command_panel(self, parent):
        """创建控制命令面板"""
        cmd_card = MaterialCard(parent, title="🎮 控制命令历史")
        cmd_card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 创建滚动区域
        canvas = tk.Canvas(cmd_card.container, bg=MaterialColors.BG_PRIMARY, 
                          height=250, highlightthickness=0)
        scrollbar = ttk.Scrollbar(cmd_card.container, orient="vertical", 
                                 command=canvas.yview)
        self.cmd_frame = tk.Frame(canvas, bg=MaterialColors.BG_PRIMARY)
        
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas_frame = canvas.create_window((0, 0), window=self.cmd_frame, anchor="nw")
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 绑定事件
        self.cmd_frame.bind("<Configure>", 
                           lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        
        self.cmd_canvas = canvas
        self.cmd_entries = []
    
    def create_log_panel(self, parent):
        """创建日志面板"""
        log_card = MaterialCard(parent, title="📝 系统日志")
        log_card.pack(fill=tk.BOTH, expand=True)
        
        # 日志文本框
        self.log_text = scrolledtext.ScrolledText(
            log_card.container, 
            height=12,
            bg=MaterialColors.BG_SECONDARY,
            fg=MaterialColors.TEXT_PRIMARY,
            font=(self.main_font, 10),
            wrap=tk.WORD,
            relief=tk.FLAT
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 配置标签样式
        self.log_text.tag_configure('info', foreground=MaterialColors.INFO)
        self.log_text.tag_configure('warning', foreground=MaterialColors.WARNING)
        self.log_text.tag_configure('error', foreground=MaterialColors.ERROR)
        self.log_text.tag_configure('success', foreground=MaterialColors.SUCCESS)
    
    def setup_charts(self):
        """设置图表"""
        # 创建matplotlib图形
        plt.style.use('seaborn-v0_8-whitegrid')
        self.figure = Figure(figsize=(8, 3), dpi=80, facecolor=MaterialColors.BG_PRIMARY)
        
        # 创建子图
        self.ax_distance = self.figure.add_subplot(131)
        self.ax_fps = self.figure.add_subplot(132)
        self.ax_detection = self.figure.add_subplot(133)
        
        # 设置子图样式
        for ax in [self.ax_distance, self.ax_fps, self.ax_detection]:
            ax.set_facecolor(MaterialColors.BG_PRIMARY)
            ax.grid(True, alpha=0.3)
            ax.tick_params(colors=MaterialColors.TEXT_SECONDARY)
        
        # 设置标题
        self.ax_distance.set_title('距离 (m)', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        self.ax_fps.set_title('FPS', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        self.ax_detection.set_title('检测数量', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        
        # 调整布局
        self.figure.tight_layout()
        
        # 创建canvas
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.chart_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def update_image(self, cv_image):
        """更新图像显示"""
        try:
            # 添加圆角效果
            image = Image.fromarray(cv_image)
            
            # 创建圆角蒙版
            mask = Image.new('L', image.size, 0)
            draw = ImageDraw.Draw(mask)
            draw.rounded_rectangle([(0, 0), image.size], radius=10, fill=255)
            
            # 应用蒙版
            output = Image.new('RGBA', image.size, (0, 0, 0, 0))
            output.paste(image, (0, 0))
            output.putalpha(mask)
            
            # 转换为PhotoImage
            self.image_photo = ImageTk.PhotoImage(output)
            self.image_label.configure(image=self.image_photo)
            
            self.last_image_update = time.time()
        except Exception as e:
            print(f"更新图像失败: {e}")
    
    def add_command_entry(self, cmd_info):
        """添加命令条目"""
        # 创建命令条目框架
        entry_frame = tk.Frame(self.cmd_frame, bg=MaterialColors.BG_SECONDARY, 
                              height=40, relief=tk.FLAT)
        entry_frame.pack(fill=tk.X, padx=5, pady=2)
        entry_frame.pack_propagate(False)
        
        # 图标
        icon_label = tk.Label(entry_frame, text=cmd_info['icon'],
                            bg=MaterialColors.BG_SECONDARY,
                            font=(self.main_font, 16))
        icon_label.pack(side=tk.LEFT, padx=(10, 5))
        
        # 内容容器
        content_frame = tk.Frame(entry_frame, bg=MaterialColors.BG_SECONDARY)
        content_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 命令文本
        cmd_label = tk.Label(content_frame, text=cmd_info['command'],
                           bg=MaterialColors.BG_SECONDARY,
                           fg=MaterialColors.TEXT_PRIMARY,
                           font=(self.main_font, 10),
                           anchor='w')
        cmd_label.pack(fill=tk.X)
        
        # 时间戳
        time_label = tk.Label(content_frame, 
                            text=cmd_info['time'].strftime("%H:%M:%S.%f")[:-3],
                            bg=MaterialColors.BG_SECONDARY,
                            fg=MaterialColors.TEXT_SECONDARY,
                            font=(self.main_font, 8),
                            anchor='w')
        time_label.pack(fill=tk.X)
        
        # 保存引用
        self.cmd_entries.append(entry_frame)
        
        # 限制条目数量
        if len(self.cmd_entries) > 20:
            self.cmd_entries[0].destroy()
            self.cmd_entries.pop(0)
        
        # 滚动到底部
        self.cmd_canvas.update_idletasks()
        self.cmd_canvas.yview_moveto(1.0)
    
    def update_charts(self, distance_history, fps_history, detection_history, time_history):
        """更新图表 - 使用双缓冲减少闪烁"""
        if not time_history or len(time_history) < 2:
            return
        
        # 清除旧图
        self.ax_distance.clear()
        self.ax_fps.clear()
        self.ax_detection.clear()
        
        # 准备时间轴
        times = list(time_history)
        if times:
            times = [(t - times[0]) for t in times]  # 相对时间
        
        # 绘制距离历史
        if distance_history:
            self.ax_distance.plot(times[-len(distance_history):], 
                                list(distance_history), 
                                color=MaterialColors.PRIMARY, linewidth=2)
            self.ax_distance.fill_between(times[-len(distance_history):], 
                                        list(distance_history), 
                                        alpha=0.3, color=MaterialColors.PRIMARY_LIGHT)
        
        # 绘制FPS历史
        if fps_history:
            self.ax_fps.plot(times[-len(fps_history):], 
                           list(fps_history), 
                           color=MaterialColors.SUCCESS, linewidth=2)
            self.ax_fps.fill_between(times[-len(fps_history):], 
                                   list(fps_history), 
                                   alpha=0.3, color=MaterialColors.SUCCESS)
        
        # 绘制检测数量历史
        if detection_history:
            self.ax_detection.plot(times[-len(detection_history):], 
                                 list(detection_history), 
                                 color=MaterialColors.ACCENT, linewidth=2)
            self.ax_detection.fill_between(times[-len(detection_history):], 
                                         list(detection_history), 
                                         alpha=0.3, color=MaterialColors.ACCENT)
        
        # 设置样式
        for ax in [self.ax_distance, self.ax_fps, self.ax_detection]:
            ax.set_xlabel('时间 (s)', color=MaterialColors.TEXT_SECONDARY, fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.set_facecolor(MaterialColors.BG_PRIMARY)
        
        # 重新设置标题
        self.ax_distance.set_title('距离 (m)', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        self.ax_fps.set_title('FPS', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        self.ax_detection.set_title('检测数量', color=MaterialColors.TEXT_PRIMARY, fontsize=10)
        
        # 调整布局并刷新
        self.figure.tight_layout()
        self.canvas.draw_idle()  # 使用draw_idle代替draw减少闪烁
    
    def update_data(self, data, distance_history, fps_history, detection_history, 
                   command_history, time_history):
        """更新所有数据显示"""
        # 更新检测信息
        self.detection_labels['fps'].set_value(f"{data.get('fps', 0):.1f}")
        self.detection_labels['detection_count'].set_value(str(data.get('detection_count', 0)))
        
        distance = data.get('nearest_distance', -1)
        if distance > 0:
            self.detection_labels['nearest_distance'].set_value(f"{distance:.2f} m")
        else:
            self.detection_labels['nearest_distance'].set_value("--")
        
        # 更新最近瓶子信息
        if data.get('nearest_bottle'):
            bottle = data['nearest_bottle']
            self.detection_labels['confidence'].set_value(
                f"{bottle.get('confidence', 0):.2%}",
                MaterialColors.SUCCESS if bottle.get('confidence', 0) > 0.8 else MaterialColors.WARNING
            )
            self.detection_labels['status'].set_value(bottle.get('status', '--'))
            pos = bottle.get('position_3d', [0, 0, 0])
            self.detection_labels['position'].set_value(
                f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
            )
        else:
            self.detection_labels['confidence'].set_value("--")
            self.detection_labels['status'].set_value("未检测到", MaterialColors.TEXT_SECONDARY)
            self.detection_labels['position'].set_value("--")
        
        # 更新机器人状态
        mode = data.get('robot_mode', 'unknown')
        if data.get('auto_harvest'):
            mode += " 🌿"
        self.status_labels['robot_mode'].set_value(mode)
        
        battery = data.get('battery_level', 0)
        battery_color = MaterialColors.SUCCESS if battery > 50 else (
            MaterialColors.WARNING if battery > 20 else MaterialColors.ERROR
        )
        self.status_labels['battery_level'].set_value(f"{battery:.1f}%", battery_color)
        
        cpu = data.get('cpu_usage', 0)
        cpu_color = MaterialColors.SUCCESS if cpu < 70 else (
            MaterialColors.WARNING if cpu < 90 else MaterialColors.ERROR
        )
        self.status_labels['cpu_usage'].set_value(f"{cpu:.1f}%", cpu_color)
        
        self.status_labels['current_speed'].set_value(f"{data.get('current_speed', 0):.2f} m/s")
        self.status_labels['harvested_count'].set_value(str(data.get('harvested_count', 0)))
        
        pos = data.get('position', {})
        self.status_labels['position_xy'].set_value(
            f"({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})"
        )
        
        # 更新命令历史
        while len(command_history) > len(self.cmd_entries):
            cmd_info = command_history[len(self.cmd_entries)]
            self.add_command_entry(cmd_info)
        
        # 检查超时
        current_time = time.time()
        for topic, last_time in data.get('last_update', {}).items():
            if current_time - last_time > 2.0:
                self.add_log(f"警告: {topic} 超过2秒未更新", 'warning')
        
        # 更新图表
        self.update_charts(distance_history, fps_history, detection_history, time_history)
    
    def add_log(self, message, level='info'):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry, level)
        self.log_text.see(tk.END)
        
        # 限制日志长度
        if int(self.log_text.index('end-1c').split('.')[0]) > 200:
            self.log_text.delete('1.0', '2.0')
    
    def run(self):
        """运行GUI"""
        self.root.mainloop()


def main(args=None):
    """主函数"""
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建GUI
    gui = DebugVisualizerGUI()
    
    # 创建ROS2节点
    node = DebugVisualizerNode(gui)
    
    # 在单独线程运行ROS2
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # 运行GUI主循环
        gui.run()
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()