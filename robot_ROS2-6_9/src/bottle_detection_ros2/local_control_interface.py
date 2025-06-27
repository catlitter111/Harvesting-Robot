#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智慧农业采摘系统本地控制界面 - 优化版本
基于PySide6实现的图形用户界面，集成ROS2实时图像显示
"""

import sys
import traceback
import threading
import time
from datetime import datetime

# 安全导入OpenCV和NumPy
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    print("OpenCV未安装，图像处理功能将被禁用")
    CV2_AVAILABLE = False
    # 创建占位符
    class np:
        ndarray = object

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QFrame, QProgressBar, QButtonGroup,
    QSizePolicy, QSpacerItem, QScrollArea
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QPixmap, QPainter, QColor, QIcon, QKeySequence, QShortcut, QImage

# ROS2相关导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    print("ROS2未安装，将在模拟模式下运行")
    ROS2_AVAILABLE = False


class ROS2ImageSubscriber(QThread):
    """ROS2图像订阅线程"""
    imageReceived = Signal(np.ndarray)  # 发送numpy数组图像信号
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = False
        
    def run(self):
        """在独立线程中运行ROS2节点"""
        if not ROS2_AVAILABLE:
            print("ROS2不可用，跳过图像订阅")
            return
            
        if not CV2_AVAILABLE:
            print("OpenCV不可用，跳过图像订阅")
            return
            
        try:
            rclpy.init()
            self.node = ImageSubscriberNode(self.imageReceived)
            self.running = True
            
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
        except Exception as e:
            print(f"ROS2节点运行错误: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    
    def stop(self):
        """停止ROS2节点"""
        self.running = False


class ImageSubscriberNode(Node):
    """ROS2图像订阅节点"""
    
    def __init__(self, image_signal):
        super().__init__('agriculture_ui_image_subscriber')
        self.image_signal = image_signal
        
        # 创建QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅压缩图像
        self.image_sub = self.create_subscription(
            CompressedImage,
            'bottle_detection/compressed_image',
            self.image_callback,
            qos
        )
        
        self.get_logger().info('图像订阅节点已启动，订阅话题: bottle_detection/compressed_image')
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            if not CV2_AVAILABLE:
                self.get_logger().error('OpenCV不可用，无法处理图像')
                return
                
            # 将压缩图像数据转换为OpenCV格式
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # 发送图像信号
                self.image_signal.emit(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')


class SmartAgricultureInterface(QMainWindow):
    """智慧农业采摘系统主界面"""
    
    def __init__(self):
        super().__init__()
        
        try:
            self.setup_window()
            self.setup_ui()
            self.setup_timers()
            self.setup_data()
            self.setup_ros2()  # 添加ROS2设置
        except Exception as e:
            print(f"初始化界面失败: {e}")
            traceback.print_exc()
    
    def setup_ros2(self):
        """设置ROS2图像订阅"""
        try:
            if ROS2_AVAILABLE and CV2_AVAILABLE:
                # 启动ROS2图像订阅线程
                self.ros2_thread = ROS2ImageSubscriber()
                self.ros2_thread.imageReceived.connect(self.update_camera_display)
                self.ros2_thread.start()
                print("ROS2图像订阅已启动")
            else:
                if not ROS2_AVAILABLE:
                    print("ROS2不可用，使用模拟模式")
                if not CV2_AVAILABLE:
                    print("OpenCV不可用，使用简单模拟模式")
                # 启动模拟图像更新
                self.setup_simulation_mode()
                
        except Exception as e:
            print(f"设置ROS2失败: {e}")
            self.setup_simulation_mode()
    
    def setup_simulation_mode(self):
        """设置模拟模式（当ROS2不可用时）"""
        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation_display)
        self.simulation_timer.start(100)  # 每100ms更新一次模拟画面
    
    def setup_window(self):
        """设置窗口属性"""
        try:
            self.setWindowTitle("智慧农业采摘系统")
            
            # 设置全屏模式
            self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
            
            # 获取屏幕尺寸并设置为全屏
            screen = QApplication.primaryScreen().geometry()
            self.setGeometry(screen)
            
            # 设置窗口样式
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #E8F5E8;
                }
            """)
            
            # 添加退出全屏的快捷键（Esc键或F11键）
            self.escape_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
            self.escape_shortcut.activated.connect(self.toggle_fullscreen)
            
            self.f11_shortcut = QShortcut(QKeySequence(Qt.Key.Key_F11), self)
            self.f11_shortcut.activated.connect(self.toggle_fullscreen)
            
            # 全屏标志
            self.is_fullscreen = True
            
        except Exception as e:
            print(f"设置窗口属性失败: {e}")
            traceback.print_exc()
    
    def setup_ui(self):
        """设置用户界面"""
        try:
            # 创建中央部件
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # 创建主布局 - 为全屏显示优化边距
            main_layout = QVBoxLayout(central_widget)
            main_layout.setContentsMargins(15, 10, 15, 10)  # 全屏时增加边距
            main_layout.setSpacing(10)
            
            # 创建顶部标题栏
            self.create_header(main_layout)
            
            # 创建主要内容区域
            content_layout = QHBoxLayout()
            content_layout.setSpacing(12)  # 全屏时适当增加间距
            main_layout.addLayout(content_layout)
            
            # 创建左侧控制面板
            self.create_control_panel(content_layout)
            
            # 创建中间监控区域
            self.create_monitor_panel(content_layout)
            
            # 创建右侧状态面板
            self.create_status_panel(content_layout)
            
            # 创建底部数据总览
            self.create_data_overview(main_layout)
            
        except Exception as e:
            print(f"设置用户界面失败: {e}")
            traceback.print_exc()
    
    def create_header(self, parent_layout):
        """创建顶部标题栏"""
        try:
            header_frame = QFrame()
            header_frame.setFixedHeight(75)  # 减少高度
            header_frame.setStyleSheet("""
                QFrame {
                    background-color: #4CAF50;
                    border-radius: 10px;
                    margin: 3px;
                }
                QLabel {
                    color: white;
                    font-weight: bold;
                }
            """)
            
            header_layout = QHBoxLayout(header_frame)
            header_layout.setContentsMargins(15, 8, 15, 8)  # 减少内边距
            
            # 系统图标和标题
            icon_label = QLabel("🌾")
            icon_label.setFont(QFont("Arial", 20))  # 稍微减小图标
            header_layout.addWidget(icon_label)
            
            # 标题垂直布局
            title_layout = QVBoxLayout()
            title_layout.setSpacing(1)
            
            title_label = QLabel("智慧农业采摘系统")
            title_label.setFont(QFont("SimHei", 18, QFont.Weight.Bold))  # 稍微减小字体
            title_label.setStyleSheet("color: white;")
            title_layout.addWidget(title_label)
            
            subtitle_label = QLabel("Smart Agricultural Harvesting Robot")
            subtitle_label.setFont(QFont("Arial", 10))
            subtitle_label.setStyleSheet("color: #E8F5E8;")
            title_layout.addWidget(subtitle_label)
            
            header_layout.addLayout(title_layout)
            
            # 弹性空间
            header_layout.addStretch()
            
            # 全屏提示
            fullscreen_hint = QLabel("🖵 全屏模式 (Esc/F11退出)")
            fullscreen_hint.setFont(QFont("SimHei", 9))
            fullscreen_hint.setStyleSheet("color: #E8F5E8; margin-right: 15px;")
            header_layout.addWidget(fullscreen_hint)
            
            # 状态指示器
            self.status_indicator = QLabel("● 运行中")
            self.status_indicator.setFont(QFont("SimHei", 11))
            self.status_indicator.setStyleSheet("color: #90EE90;")
            header_layout.addWidget(self.status_indicator)
            
            # 时间显示
            self.time_label = QLabel("12:34")
            self.time_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
            header_layout.addWidget(self.time_label)
            
            parent_layout.addWidget(header_frame)
            
        except Exception as e:
            print(f"创建标题栏失败: {e}")
            traceback.print_exc()
    
    def create_control_panel(self, parent_layout):
        """创建左侧控制面板"""
        try:
            control_frame = QFrame()
            control_frame.setFixedWidth(290)  # 稍微减少宽度
            control_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 3px;
                }
            """)
            
            control_layout = QVBoxLayout(control_frame)
            control_layout.setContentsMargins(15, 15, 15, 15)  # 减少内边距
            control_layout.setSpacing(12)  # 减少间距
            
            # 控制中心标题
            control_title = QLabel("🎛️ 控制中心")
            control_title.setFont(QFont("SimHei", 13, QFont.Weight.Bold))
            control_title.setStyleSheet("color: #2E7D32; margin-bottom: 8px;")
            control_layout.addWidget(control_title)
            
            # 系统运行开关
            system_frame = QFrame()
            system_frame.setStyleSheet("""
                QFrame {
                    background-color: #4CAF50;
                    border-radius: 25px;
                    padding: 8px;
                }
            """)
            system_layout = QHBoxLayout(system_frame)
            
            system_label = QLabel("系统运行中")
            system_label.setFont(QFont("SimHei", 11, QFont.Weight.Bold))
            system_label.setStyleSheet("color: white;")
            system_layout.addWidget(system_label)
            
            system_layout.addStretch()
            
            # 开关按钮（用圆形表示）
            self.system_switch = QPushButton("●")
            self.system_switch.setFixedSize(28, 28)
            self.system_switch.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    border-radius: 14px;
                    font-size: 14px;
                    color: #4CAF50;
                }
                QPushButton:pressed {
                    background-color: #E0E0E0;
                }
            """)
            self.system_switch.clicked.connect(self.toggle_system)
            system_layout.addWidget(self.system_switch)
            
            control_layout.addWidget(system_frame)
            
            # 工作模式选择
            mode_label = QLabel("工作模式")
            mode_label.setFont(QFont("SimHei", 9))
            mode_label.setStyleSheet("color: #666; margin-top: 8px;")
            control_layout.addWidget(mode_label)
            
            mode_layout = QHBoxLayout()
            
            self.manual_btn = QPushButton("手动")
            self.auto_btn = QPushButton("自动")
            
            # 按钮样式
            button_style = """
                QPushButton {
                    padding: 6px 14px;
                    border-radius: 18px;
                    font-size: 11px;
                    font-weight: bold;
                    border: 2px solid #4CAF50;
                }
                QPushButton:checked {
                    background-color: #4CAF50;
                    color: white;
                }
                QPushButton:!checked {
                    background-color: white;
                    color: #4CAF50;
                }
            """
            
            self.manual_btn.setStyleSheet(button_style)
            self.auto_btn.setStyleSheet(button_style)
            self.manual_btn.setCheckable(True)
            self.auto_btn.setCheckable(True)
            self.manual_btn.setChecked(True)
            
            # 创建按钮组确保单选
            self.mode_group = QButtonGroup()
            self.mode_group.addButton(self.manual_btn)
            self.mode_group.addButton(self.auto_btn)
            
            self.manual_btn.clicked.connect(self.set_manual_mode)
            self.auto_btn.clicked.connect(self.set_auto_mode)
            
            mode_layout.addWidget(self.manual_btn)
            mode_layout.addWidget(self.auto_btn)
            control_layout.addLayout(mode_layout)
            
            # 采摘控制
            harvest_label = QLabel("采摘控制")
            harvest_label.setFont(QFont("SimHei", 9))
            harvest_label.setStyleSheet("color: #666; margin-top: 10px;")
            control_layout.addWidget(harvest_label)
            
            self.harvest_btn = QPushButton("🍎 自动采摘中")
            self.harvest_btn.setFixedHeight(36)
            self.harvest_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border-radius: 18px;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 8px;
                }
                QPushButton:pressed {
                    background-color: #388E3C;
                }
            """)
            self.harvest_btn.clicked.connect(self.toggle_harvest)
            control_layout.addWidget(self.harvest_btn)
            
            # 紧急停止按钮
            self.emergency_btn = QPushButton("⭕ 紧急停止")
            self.emergency_btn.setFixedHeight(36)
            self.emergency_btn.setStyleSheet("""
                QPushButton {
                    background-color: #F44336;
                    color: white;
                    border-radius: 18px;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 8px;
                    margin-top: 8px;
                }
                QPushButton:pressed {
                    background-color: #D32F2F;
                }
            """)
            self.emergency_btn.clicked.connect(self.emergency_stop)
            control_layout.addWidget(self.emergency_btn)
            
            # 今日成果
            result_frame = QFrame()
            result_frame.setMinimumHeight(100)  # 减少高度
            result_frame.setStyleSheet("""
                QFrame {
                    background-color: #E8F5E8;
                    border-radius: 15px;
                    margin-top: 15px;
                }
            """)
            result_layout = QVBoxLayout(result_frame)
            result_layout.setContentsMargins(12, 12, 12, 12)
            result_layout.setSpacing(6)
            
            result_title = QLabel("📊 今日成果")
            result_title.setFont(QFont("SimHei", 11, QFont.Weight.Bold))
            result_title.setStyleSheet("color: #2E7D32;")
            result_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            result_layout.addWidget(result_title)
            
            self.result_number = QLabel("127")
            self.result_number.setFont(QFont("Arial", 24, QFont.Weight.Bold))
            self.result_number.setStyleSheet("color: #4CAF50; margin-top: 3px;")
            self.result_number.setAlignment(Qt.AlignmentFlag.AlignCenter)
            result_layout.addWidget(self.result_number)
            
            control_layout.addWidget(result_frame)
            
            # 弹性空间
            control_layout.addStretch()
            
            parent_layout.addWidget(control_frame)
            
        except Exception as e:
            print(f"创建控制面板失败: {e}")
            traceback.print_exc()
    
    def create_monitor_panel(self, parent_layout):
        """创建中间监控面板"""
        try:
            monitor_frame = QFrame()
            monitor_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 3px;
                }
            """)
            
            monitor_layout = QVBoxLayout(monitor_frame)
            monitor_layout.setContentsMargins(15, 15, 15, 15)
            monitor_layout.setSpacing(12)
            
            # 监控标题
            monitor_header = QHBoxLayout()
            
            monitor_title = QLabel("📺 实时监控")
            monitor_title.setFont(QFont("SimHei", 13, QFont.Weight.Bold))
            monitor_title.setStyleSheet("color: #2E7D32;")
            monitor_header.addWidget(monitor_title)
            
            monitor_header.addStretch()
            
            # FPS显示
            self.fps_label = QLabel("FPS: 29")
            self.fps_label.setFont(QFont("Arial", 11))
            self.fps_label.setStyleSheet("color: #666;")
            monitor_header.addWidget(self.fps_label)
            
            monitor_layout.addLayout(monitor_header)
            
            # 摄像头画面区域
            self.camera_display = QLabel()
            self.camera_display.setMinimumSize(800, 600)  # 全屏时增大摄像头画面
            self.camera_display.setStyleSheet("""
                QLabel {
                    background-color: #1a1a1a;
                    border-radius: 10px;
                    border: 2px solid #4CAF50;
                    color: #666;
                    font-size: 16px;
                }
            """)
            self.camera_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.camera_display.setText("正在连接摄像头...\n等待ROS2图像数据")
            self.camera_display.setScaledContents(True)  # 允许图像缩放
            
            monitor_layout.addWidget(self.camera_display)
            
            # 检测信息
            self.detection_info = QLabel("检测到 4 个目标 | 最近距离: 0.65m | 准确率: 96.5%")
            self.detection_info.setFont(QFont("SimHei", 10))
            self.detection_info.setStyleSheet("color: #2E7D32; margin-top: 8px;")
            self.detection_info.setAlignment(Qt.AlignmentFlag.AlignCenter)
            monitor_layout.addWidget(self.detection_info)
            
            parent_layout.addWidget(monitor_frame)
            
        except Exception as e:
            print(f"创建监控面板失败: {e}")
            traceback.print_exc()
    
    def create_status_panel(self, parent_layout):
        """创建右侧状态面板"""
        try:
            status_frame = QFrame()
            status_frame.setFixedWidth(270)  # 稍微减少宽度
            status_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 3px;
                }
            """)
            
            # 使用滚动区域确保内容都能显示
            scroll_area = QScrollArea(status_frame)
            scroll_area.setWidgetResizable(True)
            scroll_area.setStyleSheet("""
                QScrollArea {
                    border: none;
                    background-color: transparent;
                }
                QScrollBar:vertical {
                    background-color: #F0F0F0;
                    width: 8px;
                    border-radius: 4px;
                }
                QScrollBar::handle:vertical {
                    background-color: #C0C0C0;
                    border-radius: 4px;
                }
            """)
            
            scroll_widget = QWidget()
            scroll_area.setWidget(scroll_widget)
            
            status_layout = QVBoxLayout(scroll_widget)
            status_layout.setContentsMargins(15, 15, 15, 15)
            status_layout.setSpacing(15)  # 减少间距
            
            # 为状态面板创建布局
            main_status_layout = QVBoxLayout(status_frame)
            main_status_layout.setContentsMargins(0, 0, 0, 0)
            main_status_layout.addWidget(scroll_area)
            
            # 系统状态标题
            status_title = QLabel("📊 系统状态")
            status_title.setFont(QFont("SimHei", 13, QFont.Weight.Bold))
            status_title.setStyleSheet("color: #2E7D32; margin-bottom: 8px;")
            status_layout.addWidget(status_title)
            
            # 电池电量
            battery_label = QLabel("电池电量")
            battery_label.setFont(QFont("SimHei", 9))
            battery_label.setStyleSheet("color: #666;")
            status_layout.addWidget(battery_label)
            
            battery_layout = QHBoxLayout()
            self.battery_bar = QProgressBar()
            self.battery_bar.setValue(75)
            self.battery_bar.setFixedHeight(20)  # 设置固定高度
            self.battery_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #E0E0E0;
                    border-radius: 10px;
                    text-align: center;
                    font-weight: bold;
                    background-color: #F5F5F5;
                    font-size: 9px;
                }
                QProgressBar::chunk {
                    background-color: #4CAF50;
                    border-radius: 8px;
                }
            """)
            battery_layout.addWidget(self.battery_bar)
            
            self.battery_percent = QLabel("75%")
            self.battery_percent.setFont(QFont("Arial", 9, QFont.Weight.Bold))
            self.battery_percent.setStyleSheet("color: #4CAF50;")
            battery_layout.addWidget(self.battery_percent)
            
            status_layout.addLayout(battery_layout)
            
            # CPU使用率
            cpu_label = QLabel("CPU使用率")
            cpu_label.setFont(QFont("SimHei", 9))
            cpu_label.setStyleSheet("color: #666;")
            status_layout.addWidget(cpu_label)
            
            cpu_layout = QHBoxLayout()
            self.cpu_bar = QProgressBar()
            self.cpu_bar.setValue(52)
            self.cpu_bar.setFixedHeight(20)
            self.cpu_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #E0E0E0;
                    border-radius: 10px;
                    text-align: center;
                    font-weight: bold;
                    background-color: #F5F5F5;
                    font-size: 9px;
                }
                QProgressBar::chunk {
                    background-color: #FF9800;
                    border-radius: 8px;
                }
            """)
            cpu_layout.addWidget(self.cpu_bar)
            
            self.cpu_percent = QLabel("52%")
            self.cpu_percent.setFont(QFont("Arial", 9, QFont.Weight.Bold))
            self.cpu_percent.setStyleSheet("color: #FF9800;")
            cpu_layout.addWidget(self.cpu_percent)
            
            status_layout.addLayout(cpu_layout)
            
            # 系统温度
            temp_label = QLabel("系统温度")
            temp_label.setFont(QFont("SimHei", 9))
            temp_label.setStyleSheet("color: #666;")
            status_layout.addWidget(temp_label)
            
            self.temp_value = QLabel("28°C")
            self.temp_value.setFont(QFont("Arial", 16, QFont.Weight.Bold))
            self.temp_value.setStyleSheet("color: #4CAF50;")
            status_layout.addWidget(self.temp_value)
            
            temp_status = QLabel("正常工作温度")
            temp_status.setFont(QFont("SimHei", 8))
            temp_status.setStyleSheet("color: #999;")
            status_layout.addWidget(temp_status)
            
            # 位置信息 - 优化布局
            position_frame = QFrame()
            position_frame.setStyleSheet("""
                QFrame {
                    background-color: #E8F5E8;
                    border-radius: 10px;
                    padding: 12px;
                    margin-top: 8px;
                }
            """)
            position_layout = QVBoxLayout(position_frame)
            position_layout.setSpacing(6)  # 减少间距
            
            position_title = QLabel("📍 位置信息")
            position_title.setFont(QFont("SimHei", 11, QFont.Weight.Bold))
            position_title.setStyleSheet("color: #2E7D32;")
            position_layout.addWidget(position_title)
            
            self.position_text = QLabel("当前位置\n纬度: 34.938500°\n经度: 108.241500°\n区域: 苹果园3号地块")
            self.position_text.setFont(QFont("SimHei", 8))
            self.position_text.setStyleSheet("color: #666; line-height: 1.2;")
            position_layout.addWidget(self.position_text)
            
            work_status_title = QLabel("工作状态")
            work_status_title.setFont(QFont("SimHei", 9, QFont.Weight.Bold))
            work_status_title.setStyleSheet("color: #2E7D32; margin-top: 8px;")
            position_layout.addWidget(work_status_title)
            
            self.work_status_text = QLabel("正在采摘作业\n工作时长: 5小时23分\n移动速度: 0.3 m/s")
            self.work_status_text.setFont(QFont("SimHei", 8))
            self.work_status_text.setStyleSheet("color: #666; line-height: 1.2;")
            position_layout.addWidget(self.work_status_text)
            
            status_layout.addWidget(position_frame)
            
            # 弹性空间
            status_layout.addStretch()
            
            parent_layout.addWidget(status_frame)
            
        except Exception as e:
            print(f"创建状态面板失败: {e}")
            traceback.print_exc()
    
    def create_data_overview(self, parent_layout):
        """创建底部数据总览"""
        try:
            overview_frame = QFrame()
            overview_frame.setFixedHeight(160)  # 全屏时恢复较大高度
            overview_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 3px;
                }
            """)
            
            overview_layout = QVBoxLayout(overview_frame)
            overview_layout.setContentsMargins(15, 15, 15, 15)
            overview_layout.setSpacing(10)  # 减少间距
            
            # 标题
            overview_title = QLabel("🌾 农场数据总览")
            overview_title.setFont(QFont("SimHei", 13, QFont.Weight.Bold))
            overview_title.setStyleSheet("color: #2E7D32; margin-bottom: 3px;")
            overview_layout.addWidget(overview_title)
            
            # 数据指标
            metrics_layout = QHBoxLayout()
            metrics_layout.setSpacing(12)  # 减少间距
            
            # 创建指标卡片
            metrics_data = [
                ("累计采摘", "2,543 个", "#4CAF50"),
                ("作业精度", "96.5%", "#2196F3"),
                ("信号强度", "75%", "#FF9800"),
                ("运行天数", "45 天", "#9C27B0"),
                ("健康状态", "优秀", "#4CAF50")
            ]
            
            self.metric_values = {}
            
            for title, value, color in metrics_data:
                metric_frame = QFrame()
                metric_frame.setMinimumWidth(180)  # 减少宽度
                metric_frame.setFixedHeight(70)  # 减少高度
                metric_frame.setStyleSheet(f"""
                    QFrame {{
                        background-color: white;
                        border-radius: 10px;
                        border: 1px solid #E0E0E0;
                        margin: 1px;
                    }}
                    QFrame:hover {{
                        border: 2px solid {color};
                        background-color: #FAFAFA;
                    }}
                """)
                
                metric_layout = QVBoxLayout(metric_frame)
                metric_layout.setContentsMargins(12, 8, 12, 8)  # 减少边距
                metric_layout.setSpacing(4)  # 减少间距
                metric_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
                
                metric_title = QLabel(title)
                metric_title.setFont(QFont("SimHei", 9))
                metric_title.setStyleSheet("color: #666;")
                metric_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
                metric_layout.addWidget(metric_title)
                
                metric_value = QLabel(value)
                metric_value.setFont(QFont("Arial", 16, QFont.Weight.Bold))  # 减少字体大小
                metric_value.setStyleSheet(f"color: {color};")
                metric_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
                metric_layout.addWidget(metric_value)
                
                # 保存引用以便后续更新
                self.metric_values[title] = metric_value
                
                metrics_layout.addWidget(metric_frame)
            
            overview_layout.addLayout(metrics_layout)
            
            parent_layout.addWidget(overview_frame)
            
        except Exception as e:
            print(f"创建数据总览失败: {e}")
            traceback.print_exc()
    
    def setup_timers(self):
        """设置定时器"""
        try:
            # 时间更新定时器
            self.time_timer = QTimer()
            self.time_timer.timeout.connect(self.update_time)
            self.time_timer.start(1000)  # 每秒更新一次
            
            # 数据更新定时器
            self.data_timer = QTimer()
            self.data_timer.timeout.connect(self.update_data)
            self.data_timer.start(2000)  # 每2秒更新一次
            
        except Exception as e:
            print(f"设置定时器失败: {e}")
            traceback.print_exc()
    
    def setup_data(self):
        """初始化数据"""
        try:
            self.system_running = True
            self.harvest_active = True
            self.manual_mode = True
            
            # 模拟数据
            self.battery_level = 75
            self.cpu_usage = 52
            self.temperature = 28
            self.fps = 29
            
            # 摄像头状态
            self.camera_status_shown = False
            
        except Exception as e:
            print(f"初始化数据失败: {e}")
            traceback.print_exc()
    
    def update_time(self):
        """更新时间显示"""
        try:
            current_time = datetime.now().strftime("%H:%M")
            self.time_label.setText(current_time)
        except Exception as e:
            print(f"更新时间失败: {e}")
            traceback.print_exc()
    
    def update_data(self):
        """更新数据显示"""
        try:
            # 模拟数据变化
            import random
            
            # 更新FPS
            self.fps = random.randint(25, 30)
            self.fps_label.setText(f"FPS: {self.fps}")
            
            # 更新CPU使用率
            self.cpu_usage = random.randint(45, 60)
            self.cpu_bar.setValue(self.cpu_usage)
            self.cpu_percent.setText(f"{self.cpu_usage}%")
            
            # 更新温度
            self.temperature = random.randint(26, 32)
            self.temp_value.setText(f"{self.temperature}°C")
            
            # 更新检测信息
            targets = random.randint(2, 6)
            distance = random.uniform(0.5, 1.5)
            accuracy = random.uniform(94, 98)
            self.detection_info.setText(f"检测到 {targets} 个目标 | 最近距离: {distance:.2f}m | 准确率: {accuracy:.1f}%")
            
        except Exception as e:
            print(f"更新数据失败: {e}")
            traceback.print_exc()
    
    def update_camera_display(self, cv_image):
        """更新摄像头显示（接收OpenCV图像）"""
        try:
            if not CV2_AVAILABLE:
                self.camera_display.setText("OpenCV不可用\n无法显示图像")
                return
                
            # 转换颜色格式 BGR -> RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 获取图像尺寸
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            
            # 转换为Qt图像格式
            qt_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
            
            # 缩放图像以适应显示区域
            display_size = self.camera_display.size()
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                display_size.width() - 4,  # 减去边框宽度
                display_size.height() - 4,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            # 显示图像
            self.camera_display.setPixmap(scaled_pixmap)
            
            # 更新状态
            if hasattr(self, 'camera_status_shown') and not self.camera_status_shown:
                print("ROS2图像数据接收成功，摄像头画面已连接")
                self.camera_status_shown = True
                
        except Exception as e:
            print(f"更新摄像头显示失败: {e}")
    
    def update_simulation_display(self):
        """更新模拟显示（当ROS2不可用时）"""
        try:
            if not CV2_AVAILABLE:
                # 简单文本模拟
                timestamp = time.strftime("%H:%M:%S")
                simulation_text = f"""模拟摄像头画面
时间: {timestamp}

检测状态: 运行中
检测目标: 4个苹果
检测精度: 96.5%

OpenCV不可用
显示简化模拟信息"""
                self.camera_display.setText(simulation_text)
                return
                
            # 创建一个模拟的图像（彩色渐变）
            import random
            
            # 创建640x480的模拟图像
            width, height = 640, 480
            image = np.zeros((height, width, 3), dtype=np.uint8)
            
            # 添加渐变背景
            for y in range(height):
                for x in range(width):
                    image[y, x] = [
                        int(128 + 64 * np.sin(x * 0.01 + time.time())),
                        int(128 + 64 * np.sin(y * 0.01 + time.time() * 1.1)),
                        int(128 + 64 * np.sin((x + y) * 0.005 + time.time() * 0.8))
                    ]
            
            # 添加一些模拟的检测框
            cv2.rectangle(image, (100, 100), (200, 200), (0, 255, 0), 2)
            cv2.putText(image, "Apple - 85%", (105, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.rectangle(image, (300, 150), (400, 250), (255, 255, 0), 2)
            cv2.putText(image, "Apple - 78%", (305, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 添加时间戳
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(image, f"SIMULATION MODE - {timestamp}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示模拟图像
            self.update_camera_display(image)
            
        except Exception as e:
            print(f"更新模拟显示失败: {e}")
            # 降级到文本显示
            self.camera_display.setText(f"模拟模式\n时间: {time.strftime('%H:%M:%S')}\n图像处理出错")
    
    def toggle_fullscreen(self):
        """切换全屏模式"""
        try:
            if self.is_fullscreen:
                # 退出全屏，显示为普通窗口
                self.setWindowFlags(Qt.WindowType.Window)
                self.setGeometry(100, 100, 1440, 960)
                self.show()
                self.is_fullscreen = False
                print("退出全屏模式 (按F11或Esc可重新进入全屏)")
            else:
                # 进入全屏
                self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
                screen = QApplication.primaryScreen().geometry()
                self.setGeometry(screen)
                self.show()
                self.is_fullscreen = True
                print("进入全屏模式")
        except Exception as e:
            print(f"切换全屏模式失败: {e}")
            traceback.print_exc()

    def closeEvent(self, event):
        """窗口关闭事件"""
        try:
            # 停止ROS2线程
            if hasattr(self, 'ros2_thread'):
                self.ros2_thread.stop()
                self.ros2_thread.wait(3000)  # 等待最多3秒
            
            # 停止定时器
            if hasattr(self, 'simulation_timer'):
                self.simulation_timer.stop()
                
            print("应用程序已安全关闭")
            event.accept()
            
        except Exception as e:
            print(f"关闭应用程序时出错: {e}")
            event.accept()
        """切换全屏模式"""
        try:
            if self.is_fullscreen:
                # 退出全屏，显示为普通窗口
                self.setWindowFlags(Qt.WindowType.Window)
                self.setGeometry(100, 100, 1440, 960)
                self.show()
                self.is_fullscreen = False
                print("退出全屏模式 (按F11或Esc可重新进入全屏)")
            else:
                # 进入全屏
                self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
                screen = QApplication.primaryScreen().geometry()
                self.setGeometry(screen)
                self.show()
                self.is_fullscreen = True
                print("进入全屏模式")
        except Exception as e:
            print(f"切换全屏模式失败: {e}")
            traceback.print_exc()

    # 控制按钮回调函数
    def toggle_system(self):
        """切换系统状态"""
        try:
            self.system_running = not self.system_running
            if self.system_running:
                self.status_indicator.setText("● 运行中")
                self.status_indicator.setStyleSheet("color: #90EE90;")
            else:
                self.status_indicator.setText("● 已停止")
                self.status_indicator.setStyleSheet("color: #FFB6C1;")
            print(f"系统状态切换: {'运行中' if self.system_running else '已停止'}")
        except Exception as e:
            print(f"切换系统状态失败: {e}")
            traceback.print_exc()
    
    def set_manual_mode(self):
        """设置手动模式"""
        try:
            self.manual_mode = True
            print("切换到手动模式")
        except Exception as e:
            print(f"设置手动模式失败: {e}")
            traceback.print_exc()
    
    def set_auto_mode(self):
        """设置自动模式"""
        try:
            self.manual_mode = False
            print("切换到自动模式")
        except Exception as e:
            print(f"设置自动模式失败: {e}")
            traceback.print_exc()
    
    def toggle_harvest(self):
        """切换采摘状态"""
        try:
            self.harvest_active = not self.harvest_active
            if self.harvest_active:
                self.harvest_btn.setText("🍎 自动采摘中")
                self.harvest_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #4CAF50;
                        color: white;
                        border-radius: 18px;
                        font-size: 11px;
                        font-weight: bold;
                        padding: 8px;
                    }
                """)
            else:
                self.harvest_btn.setText("⏸️ 采摘暂停")
                self.harvest_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #FF9800;
                        color: white;
                        border-radius: 18px;
                        font-size: 11px;
                        font-weight: bold;
                        padding: 8px;
                    }
                """)
            print(f"采摘状态切换: {'活动中' if self.harvest_active else '暂停'}")
        except Exception as e:
            print(f"切换采摘状态失败: {e}")
            traceback.print_exc()
    
    def emergency_stop(self):
        """紧急停止"""
        try:
            print("执行紧急停止！")
            self.system_running = False
            self.harvest_active = False
            self.status_indicator.setText("● 紧急停止")
            self.status_indicator.setStyleSheet("color: #FF6B6B;")
            
            # 更新采摘按钮状态
            self.harvest_btn.setText("⏸️ 采摘暂停")
            self.harvest_btn.setStyleSheet("""
                QPushButton {
                    background-color: #FF9800;
                    color: white;
                    border-radius: 18px;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 8px;
                }
            """)
            
            # 这里后续会添加发布ROS消息的代码
            
        except Exception as e:
            print(f"紧急停止失败: {e}")
            traceback.print_exc()


def main():
    """主函数"""
    try:
        # 初始化Qt应用
        app = QApplication(sys.argv)
        
        # 设置应用样式
        app.setStyle('Fusion')
        
        # 创建主窗口
        window = SmartAgricultureInterface()
        
        # 直接显示为全屏
        window.showFullScreen()
        
        print("智慧农业采摘系统已启动 (全屏模式)")
        print("按 Esc 或 F11 键可以切换全屏模式")
        
        # 显示系统状态
        if ROS2_AVAILABLE and CV2_AVAILABLE:
            print("✅ 完整模式：ROS2功能已启用，正在订阅图像话题: bottle_detection/compressed_image")
        elif ROS2_AVAILABLE and not CV2_AVAILABLE:
            print("⚠️  部分功能模式：ROS2可用但OpenCV不可用")
        elif not ROS2_AVAILABLE and CV2_AVAILABLE:
            print("🔄 模拟模式：ROS2不可用，使用OpenCV模拟图像")
        else:
            print("📝 简化模式：ROS2和OpenCV都不可用，使用文本模拟")
        
        # 运行应用
        result = app.exec()
        
        # 确保ROS2线程正确关闭
        if hasattr(window, 'ros2_thread'):
            window.ros2_thread.stop()
            window.ros2_thread.wait(3000)
        
        sys.exit(result)
        
    except Exception as e:
        print(f"启动应用失败: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()