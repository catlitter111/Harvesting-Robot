#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智慧农业采摘系统本地控制界面
基于PySide6实现的图形用户界面
"""

import sys
import traceback
import threading
import time
from datetime import datetime
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QFrame, QProgressBar, QButtonGroup,
    QSizePolicy, QSpacerItem
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QPixmap, QPainter, QColor, QIcon


class SmartAgricultureInterface(QMainWindow):
    """智慧农业采摘系统主界面"""
    
    def __init__(self):
        super().__init__()
        
        try:
            self.setup_window()
            self.setup_ui()
            self.setup_timers()
            self.setup_data()
        except Exception as e:
            print(f"初始化界面失败: {e}")
            traceback.print_exc()
    
    def setup_window(self):
        """设置窗口属性"""
        try:
            self.setWindowTitle("智慧农业采摘系统")
            self.setGeometry(100, 100, 1400, 900)
            self.setMinimumSize(1200, 800)
            
            # 设置窗口样式
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #E8F5E8;
                }
            """)
            
        except Exception as e:
            print(f"设置窗口属性失败: {e}")
            traceback.print_exc()
    
    def setup_ui(self):
        """设置用户界面"""
        try:
            # 创建中央部件
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # 创建主布局
            main_layout = QVBoxLayout(central_widget)
            main_layout.setContentsMargins(10, 10, 10, 10)
            main_layout.setSpacing(10)
            
            # 创建顶部标题栏
            self.create_header(main_layout)
            
            # 创建主要内容区域
            content_layout = QHBoxLayout()
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
            header_frame.setFixedHeight(80)
            header_frame.setStyleSheet("""
                QFrame {
                    background-color: #4CAF50;
                    border-radius: 10px;
                    margin: 5px;
                }
                QLabel {
                    color: white;
                    font-weight: bold;
                }
            """)
            
            header_layout = QHBoxLayout(header_frame)
            header_layout.setContentsMargins(20, 10, 20, 10)
            
            # 系统图标和标题
            icon_label = QLabel("🌾")
            icon_label.setFont(QFont("Arial", 24))
            header_layout.addWidget(icon_label)
            
            title_label = QLabel("智慧农业采摘系统")
            title_label.setFont(QFont("SimHei", 18, QFont.Weight.Bold))
            header_layout.addWidget(title_label)
            
            subtitle_label = QLabel("Smart Agricultural Harvesting Robot")
            subtitle_label.setFont(QFont("Arial", 10))
            subtitle_label.setStyleSheet("color: #E8F5E8;")
            
            title_layout = QVBoxLayout()
            title_layout.addWidget(title_label)
            title_layout.addWidget(subtitle_label)
            header_layout.addLayout(title_layout)
            
            # 弹性空间
            header_layout.addStretch()
            
            # 状态指示器
            self.status_indicator = QLabel("● 运行中")
            self.status_indicator.setFont(QFont("SimHei", 12))
            self.status_indicator.setStyleSheet("color: #90EE90;")
            header_layout.addWidget(self.status_indicator)
            
            # 时间显示
            self.time_label = QLabel("12:34")
            self.time_label.setFont(QFont("Arial", 16, QFont.Weight.Bold))
            header_layout.addWidget(self.time_label)
            
            parent_layout.addWidget(header_frame)
            
        except Exception as e:
            print(f"创建标题栏失败: {e}")
            traceback.print_exc()
    
    def create_control_panel(self, parent_layout):
        """创建左侧控制面板"""
        try:
            control_frame = QFrame()
            control_frame.setFixedWidth(300)
            control_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 5px;
                }
            """)
            
            control_layout = QVBoxLayout(control_frame)
            control_layout.setContentsMargins(20, 20, 20, 20)
            control_layout.setSpacing(15)
            
            # 控制中心标题
            control_title = QLabel("🎛️ 控制中心")
            control_title.setFont(QFont("SimHei", 14, QFont.Weight.Bold))
            control_title.setStyleSheet("color: #2E7D32; margin-bottom: 10px;")
            control_layout.addWidget(control_title)
            
            # 系统运行开关
            system_frame = QFrame()
            system_frame.setStyleSheet("""
                QFrame {
                    background-color: #4CAF50;
                    border-radius: 25px;
                    padding: 10px;
                }
            """)
            system_layout = QHBoxLayout(system_frame)
            
            system_label = QLabel("系统运行中")
            system_label.setFont(QFont("SimHei", 12, QFont.Weight.Bold))
            system_label.setStyleSheet("color: white;")
            system_layout.addWidget(system_label)
            
            system_layout.addStretch()
            
            # 开关按钮（用圆形表示）
            self.system_switch = QPushButton("●")
            self.system_switch.setFixedSize(30, 30)
            self.system_switch.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    border-radius: 15px;
                    font-size: 16px;
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
            mode_label.setFont(QFont("SimHei", 10))
            mode_label.setStyleSheet("color: #666; margin-top: 10px;")
            control_layout.addWidget(mode_label)
            
            mode_layout = QHBoxLayout()
            
            self.manual_btn = QPushButton("手动")
            self.auto_btn = QPushButton("自动")
            
            # 按钮样式
            button_style = """
                QPushButton {
                    padding: 8px 16px;
                    border-radius: 20px;
                    font-size: 12px;
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
            harvest_label.setFont(QFont("SimHei", 10))
            harvest_label.setStyleSheet("color: #666; margin-top: 15px;")
            control_layout.addWidget(harvest_label)
            
            self.harvest_btn = QPushButton("🍎 自动采摘中")
            self.harvest_btn.setFixedHeight(40)
            self.harvest_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border-radius: 20px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 10px;
                }
                QPushButton:pressed {
                    background-color: #388E3C;
                }
            """)
            self.harvest_btn.clicked.connect(self.toggle_harvest)
            control_layout.addWidget(self.harvest_btn)
            
            # 紧急停止按钮
            self.emergency_btn = QPushButton("⭕ 紧急停止")
            self.emergency_btn.setFixedHeight(40)
            self.emergency_btn.setStyleSheet("""
                QPushButton {
                    background-color: #F44336;
                    color: white;
                    border-radius: 20px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 10px;
                    margin-top: 10px;
                }
                QPushButton:pressed {
                    background-color: #D32F2F;
                }
            """)
            self.emergency_btn.clicked.connect(self.emergency_stop)
            control_layout.addWidget(self.emergency_btn)
            
            # 今日成果
            result_frame = QFrame()
            result_frame.setStyleSheet("""
                QFrame {
                    background-color: #E8F5E8;
                    border-radius: 15px;
                    padding: 15px;
                    margin-top: 20px;
                }
            """)
            result_layout = QVBoxLayout(result_frame)
            
            result_title = QLabel("📊 今日成果")
            result_title.setFont(QFont("SimHei", 12, QFont.Weight.Bold))
            result_title.setStyleSheet("color: #2E7D32;")
            result_layout.addWidget(result_title)
            
            self.result_number = QLabel("127")
            self.result_number.setFont(QFont("Arial", 32, QFont.Weight.Bold))
            self.result_number.setStyleSheet("color: #4CAF50;")
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
                    margin: 5px;
                }
            """)
            
            monitor_layout = QVBoxLayout(monitor_frame)
            monitor_layout.setContentsMargins(20, 20, 20, 20)
            monitor_layout.setSpacing(15)
            
            # 监控标题
            monitor_header = QHBoxLayout()
            
            monitor_title = QLabel("📺 实时监控")
            monitor_title.setFont(QFont("SimHei", 14, QFont.Weight.Bold))
            monitor_title.setStyleSheet("color: #2E7D32;")
            monitor_header.addWidget(monitor_title)
            
            monitor_header.addStretch()
            
            # FPS显示
            self.fps_label = QLabel("FPS: 29")
            self.fps_label.setFont(QFont("Arial", 12))
            self.fps_label.setStyleSheet("color: #666;")
            monitor_header.addWidget(self.fps_label)
            
            monitor_layout.addLayout(monitor_header)
            
            # 摄像头画面区域
            self.camera_frame = QFrame()
            self.camera_frame.setMinimumSize(640, 480)
            self.camera_frame.setStyleSheet("""
                QFrame {
                    background-color: #1a1a1a;
                    border-radius: 10px;
                    border: 2px solid #4CAF50;
                }
            """)
            
            # 在摄像头画面上添加一些模拟检测框
            camera_layout = QVBoxLayout(self.camera_frame)
            camera_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # 模拟画面标签
            camera_placeholder = QLabel("摄像头画面区域\n（检测到的水果将在此显示）")
            camera_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            camera_placeholder.setStyleSheet("""
                QLabel {
                    color: #666;
                    font-size: 14px;
                    background-color: transparent;
                    border: none;
                }
            """)
            camera_layout.addWidget(camera_placeholder)
            
            monitor_layout.addWidget(self.camera_frame)
            
            # 检测信息
            self.detection_info = QLabel("检测到 4 个目标 | 最近距离: 0.65m | 准确率: 96.5%")
            self.detection_info.setFont(QFont("SimHei", 11))
            self.detection_info.setStyleSheet("color: #2E7D32; margin-top: 10px;")
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
            status_frame.setFixedWidth(280)
            status_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 5px;
                }
            """)
            
            status_layout = QVBoxLayout(status_frame)
            status_layout.setContentsMargins(20, 20, 20, 20)
            status_layout.setSpacing(20)
            
            # 系统状态标题
            status_title = QLabel("📊 系统状态")
            status_title.setFont(QFont("SimHei", 14, QFont.Weight.Bold))
            status_title.setStyleSheet("color: #2E7D32; margin-bottom: 10px;")
            status_layout.addWidget(status_title)
            
            # 电池电量
            battery_label = QLabel("电池电量")
            battery_label.setFont(QFont("SimHei", 10))
            battery_label.setStyleSheet("color: #666;")
            status_layout.addWidget(battery_label)
            
            battery_layout = QHBoxLayout()
            self.battery_bar = QProgressBar()
            self.battery_bar.setValue(75)
            self.battery_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #E0E0E0;
                    border-radius: 10px;
                    text-align: center;
                    font-weight: bold;
                    background-color: #F5F5F5;
                }
                QProgressBar::chunk {
                    background-color: #4CAF50;
                    border-radius: 8px;
                }
            """)
            battery_layout.addWidget(self.battery_bar)
            
            self.battery_percent = QLabel("75%")
            self.battery_percent.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.battery_percent.setStyleSheet("color: #4CAF50;")
            battery_layout.addWidget(self.battery_percent)
            
            status_layout.addLayout(battery_layout)
            
            # CPU使用率
            cpu_label = QLabel("CPU使用率")
            cpu_label.setFont(QFont("SimHei", 10))
            cpu_label.setStyleSheet("color: #666;")
            status_layout.addWidget(cpu_label)
            
            cpu_layout = QHBoxLayout()
            self.cpu_bar = QProgressBar()
            self.cpu_bar.setValue(52)
            self.cpu_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #E0E0E0;
                    border-radius: 10px;
                    text-align: center;
                    font-weight: bold;
                    background-color: #F5F5F5;
                }
                QProgressBar::chunk {
                    background-color: #FF9800;
                    border-radius: 8px;
                }
            """)
            cpu_layout.addWidget(self.cpu_bar)
            
            self.cpu_percent = QLabel("52%")
            self.cpu_percent.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.cpu_percent.setStyleSheet("color: #FF9800;")
            cpu_layout.addWidget(self.cpu_percent)
            
            status_layout.addLayout(cpu_layout)
            
            # 系统温度
            temp_label = QLabel("系统温度")
            temp_label.setFont(QFont("SimHei", 10))
            temp_label.setStyleSheet("color: #666;")
            status_layout.addWidget(temp_label)
            
            self.temp_value = QLabel("28°C")
            self.temp_value.setFont(QFont("Arial", 18, QFont.Weight.Bold))
            self.temp_value.setStyleSheet("color: #4CAF50;")
            status_layout.addWidget(self.temp_value)
            
            temp_status = QLabel("正常工作温度")
            temp_status.setFont(QFont("SimHei", 9))
            temp_status.setStyleSheet("color: #999;")
            status_layout.addWidget(temp_status)
            
            # 位置信息
            position_frame = QFrame()
            position_frame.setStyleSheet("""
                QFrame {
                    background-color: #E8F5E8;
                    border-radius: 10px;
                    padding: 15px;
                    margin-top: 10px;
                }
            """)
            position_layout = QVBoxLayout(position_frame)
            
            position_title = QLabel("📍 位置信息")
            position_title.setFont(QFont("SimHei", 12, QFont.Weight.Bold))
            position_title.setStyleSheet("color: #2E7D32;")
            position_layout.addWidget(position_title)
            
            self.position_text = QLabel("当前位置\n纬度: 34.938500°\n经度: 108.241500°\n区域: 苹果园3号地块")
            self.position_text.setFont(QFont("SimHei", 9))
            self.position_text.setStyleSheet("color: #666; line-height: 1.4;")
            position_layout.addWidget(self.position_text)
            
            work_status_title = QLabel("工作状态")
            work_status_title.setFont(QFont("SimHei", 10, QFont.Weight.Bold))
            work_status_title.setStyleSheet("color: #2E7D32; margin-top: 10px;")
            position_layout.addWidget(work_status_title)
            
            self.work_status_text = QLabel("正在采摘作业\n工作时长: 5小时23分\n移动速度: 0.3 m/s")
            self.work_status_text.setFont(QFont("SimHei", 9))
            self.work_status_text.setStyleSheet("color: #666; line-height: 1.4;")
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
            overview_frame.setFixedHeight(120)
            overview_frame.setStyleSheet("""
                QFrame {
                    background-color: white;
                    border-radius: 15px;
                    margin: 5px;
                }
            """)
            
            overview_layout = QVBoxLayout(overview_frame)
            overview_layout.setContentsMargins(20, 15, 20, 15)
            overview_layout.setSpacing(10)
            
            # 标题
            overview_title = QLabel("🌾 农场数据总览")
            overview_title.setFont(QFont("SimHei", 14, QFont.Weight.Bold))
            overview_title.setStyleSheet("color: #2E7D32;")
            overview_layout.addWidget(overview_title)
            
            # 数据指标
            metrics_layout = QHBoxLayout()
            
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
                metric_frame.setStyleSheet(f"""
                    QFrame {{
                        background-color: #F8F9FA;
                        border-radius: 10px;
                        border-left: 4px solid {color};
                        padding: 10px;
                    }}
                """)
                
                metric_layout = QVBoxLayout(metric_frame)
                metric_layout.setContentsMargins(10, 5, 10, 5)
                
                metric_title = QLabel(title)
                metric_title.setFont(QFont("SimHei", 9))
                metric_title.setStyleSheet("color: #666;")
                metric_layout.addWidget(metric_title)
                
                metric_value = QLabel(value)
                metric_value.setFont(QFont("Arial", 14, QFont.Weight.Bold))
                metric_value.setStyleSheet(f"color: {color};")
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
                        border-radius: 20px;
                        font-size: 12px;
                        font-weight: bold;
                        padding: 10px;
                    }
                """)
            else:
                self.harvest_btn.setText("⏸️ 采摘暂停")
                self.harvest_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #FF9800;
                        color: white;
                        border-radius: 20px;
                        font-size: 12px;
                        font-weight: bold;
                        padding: 10px;
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
                    border-radius: 20px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 10px;
                }
            """)
            
            # 这里后续会添加发布ROS消息的代码
            
        except Exception as e:
            print(f"紧急停止失败: {e}")
            traceback.print_exc()


def main():
    """主函数"""
    try:
        app = QApplication(sys.argv)
        
        # 设置应用样式
        app.setStyle('Fusion')
        
        # 创建主窗口
        window = SmartAgricultureInterface()
        window.show()
        
        # 运行应用
        sys.exit(app.exec())
        
    except Exception as e:
        print(f"启动应用失败: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main() 