#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智慧农业采摘系统本地控制界面 - 简化版
Smart Agricultural Harvesting Robot Local Control GUI - Simple Version
完全按照设计图实现的tkinter界面
"""

import tkinter as tk
from tkinter import ttk
import traceback
from datetime import datetime
import threading
import time

class SmartHarvestingGUI:
    """智慧农业采摘系统主界面类"""
    
    def __init__(self):
        try:
            self.root = tk.Tk()
            self.setup_window()
            self.setup_colors()
            self.setup_fonts()
            self.setup_variables()
            self.create_main_layout()
            self.start_update_timer()
        except Exception as e:
            print(f"初始化GUI失败: {e}")
            traceback.print_exc()
    
    def setup_window(self):
        """设置主窗口"""
        try:
            self.root.title("智慧农业采摘系统")
            self.root.geometry("1400x900")
            self.root.configure(bg='#E8F5E8')
            self.root.resizable(True, True)
            
            # 设置窗口居中
            self.center_window()
        except Exception as e:
            print(f"设置窗口失败: {e}")
            traceback.print_exc()
    
    def center_window(self):
        """窗口居中显示"""
        try:
            self.root.update_idletasks()
            width = self.root.winfo_width()
            height = self.root.winfo_height()
            x = (self.root.winfo_screenwidth() // 2) - (width // 2)
            y = (self.root.winfo_screenheight() // 2) - (height // 2)
            self.root.geometry(f'{width}x{height}+{x}+{y}')
        except Exception as e:
            print(f"窗口居中失败: {e}")
            traceback.print_exc()
    
    def setup_colors(self):
        """设置颜色主题"""
        try:
            self.colors = {
                'primary_green': '#4CAF50',
                'dark_green': '#2E7D32',
                'light_green': '#81C784',
                'bg_green': '#E8F5E8',
                'white': '#FFFFFF',
                'light_gray': '#F5F5F5',
                'dark_gray': '#424242',
                'red': '#F44336',
                'orange': '#FF9800',
                'blue': '#2196F3',
                'text_dark': '#212121',
                'text_secondary': '#757575'
            }
        except Exception as e:
            print(f"设置颜色主题失败: {e}")
            traceback.print_exc()
    
    def setup_fonts(self):
        """设置字体"""
        try:
            self.fonts = {
                'title_large': ('Arial', 20, 'bold'),
                'title_medium': ('Arial', 16, 'bold'),
                'title_small': ('Arial', 14, 'bold'),
                'text_large': ('Arial', 14),
                'text_medium': ('Arial', 12),
                'text_small': ('Arial', 10),
                'number_large': ('Arial', 24, 'bold'),
                'number_medium': ('Arial', 18, 'bold')
            }
        except Exception as e:
            print(f"设置字体失败: {e}")
            traceback.print_exc()
    
    def setup_variables(self):
        """设置tkinter变量"""
        try:
            # 系统状态变量
            self.system_running = tk.BooleanVar(value=True)
            self.work_mode = tk.StringVar(value="手动")
            self.harvest_active = tk.BooleanVar(value=False)
            
            # 数据变量
            self.current_time = tk.StringVar(value="12:34")
            self.fps_value = tk.StringVar(value="29")
            self.battery_level = tk.IntVar(value=75)
            self.cpu_usage = tk.IntVar(value=52)
            self.system_temp = tk.StringVar(value="28°C")
            self.detection_count = tk.StringVar(value="4")
            self.nearest_distance = tk.StringVar(value="0.65m")
            self.accuracy = tk.StringVar(value="96.5%")
            self.today_harvest = tk.StringVar(value="197")
            
            # 位置信息
            self.latitude = tk.StringVar(value="34.938500°")
            self.longitude = tk.StringVar(value="108.241500°")
            self.location_area = tk.StringVar(value="苹果园3号地块")
            self.work_status = tk.StringVar(value="正在采摘作业")
            self.work_time = tk.StringVar(value="5小时23分")
            self.move_speed = tk.StringVar(value="0.3 m/s")
            
            # 农场数据
            self.total_harvest = tk.StringVar(value="2,543")
            self.work_accuracy = tk.StringVar(value="96.5%")
            self.signal_strength = tk.StringVar(value="75%")
            self.work_days = tk.StringVar(value="45")
            self.health_status = tk.StringVar(value="优秀")
            
        except Exception as e:
            print(f"设置变量失败: {e}")
            traceback.print_exc()
    
    def create_main_layout(self):
        """创建主要布局"""
        try:
            # 创建顶部标题栏
            self.create_header()
            
            # 创建主要内容区域
            self.main_frame = tk.Frame(self.root, bg=self.colors['bg_green'])
            self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
            
            # 创建左中右三列布局
            self.create_left_panel()
            self.create_center_panel()
            self.create_right_panel()
            
            # 创建底部农场数据面板
            self.create_bottom_panel()
            
        except Exception as e:
            print(f"创建主布局失败: {e}")
            traceback.print_exc()
    
    def create_header(self):
        """创建顶部标题栏"""
        try:
            header_frame = tk.Frame(self.root, bg=self.colors['primary_green'], height=80)
            header_frame.pack(fill=tk.X)
            header_frame.pack_propagate(False)
            
            # 左侧标题
            left_frame = tk.Frame(header_frame, bg=self.colors['primary_green'])
            left_frame.pack(side=tk.LEFT, padx=20, pady=15)
            
            title_frame = tk.Frame(left_frame, bg=self.colors['primary_green'])
            title_frame.pack(side=tk.LEFT)
            
            tk.Label(title_frame, text="智慧农业采摘系统", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['title_large']).pack()
            tk.Label(title_frame, text="Smart Agricultural Harvesting Robot", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=('Arial', 10)).pack()
            
            # 右侧状态和时间
            right_frame = tk.Frame(header_frame, bg=self.colors['primary_green'])
            right_frame.pack(side=tk.RIGHT, padx=20, pady=15)
            
            # 运行状态
            status_frame = tk.Frame(right_frame, bg=self.colors['primary_green'])
            status_frame.pack(side=tk.LEFT, padx=(0, 20))
            
            # 运行状态指示器（用Label替代Frame）
            tk.Label(status_frame, text="●", 
                    bg=self.colors['primary_green'], fg=self.colors['light_green'],
                    font=('Arial', 12)).pack(side=tk.LEFT, padx=(0, 5))
            
            tk.Label(status_frame, text="运行中", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['text_medium']).pack(side=tk.LEFT)
            
            # 时间显示
            self.time_label = tk.Label(right_frame, textvariable=self.current_time,
                                     bg=self.colors['primary_green'], fg=self.colors['white'],
                                     font=self.fonts['title_medium'])
            self.time_label.pack(side=tk.LEFT)
            
        except Exception as e:
            print(f"创建标题栏失败: {e}")
            traceback.print_exc()
    
    def create_left_panel(self):
        """创建左侧控制面板"""
        try:
            self.left_frame = tk.Frame(self.main_frame, bg=self.colors['white'], 
                                     width=300, relief=tk.RAISED, bd=1)
            self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
            self.left_frame.pack_propagate(False)
            
            # 控制中心标题
            title_frame = tk.Frame(self.left_frame, bg=self.colors['primary_green'], height=50)
            title_frame.pack(fill=tk.X)
            title_frame.pack_propagate(False)
            
            tk.Label(title_frame, text="🎛️ 控制中心", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['title_medium']).pack(pady=15)
            
            # 控制内容
            control_frame = tk.Frame(self.left_frame, bg=self.colors['white'])
            control_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
            
            # 系统运行开关
            self.create_system_switch(control_frame)
            
            # 工作模式选择
            self.create_work_mode_selection(control_frame)
            
            # 采摘控制
            self.create_harvest_control(control_frame)
            
            # 紧急停止按钮
            self.create_emergency_stop(control_frame)
            
            # 今日成果显示
            self.create_today_result(control_frame)
            
        except Exception as e:
            print(f"创建左侧面板失败: {e}")
            traceback.print_exc()
    
    def create_system_switch(self, parent):
        """创建系统运行开关"""
        try:
            switch_frame = tk.Frame(parent, bg=self.colors['white'])
            switch_frame.pack(fill=tk.X, pady=(0, 20))
            
            # 系统运行开关
            switch_bg = tk.Frame(switch_frame, bg=self.colors['primary_green'], 
                               height=60, relief=tk.RAISED, bd=1)
            switch_bg.pack(fill=tk.X)
            
            inner_frame = tk.Frame(switch_bg, bg=self.colors['primary_green'])
            inner_frame.pack(expand=True, fill=tk.BOTH, padx=20, pady=10)
            
            tk.Label(inner_frame, text="系统运行中", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['text_large']).pack(side=tk.LEFT)
            
            # 开关指示器（用Label替代Frame）
            tk.Label(inner_frame, text="●", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=('Arial', 20)).pack(side=tk.RIGHT)
            
        except Exception as e:
            print(f"创建系统开关失败: {e}")
            traceback.print_exc()
    
    def create_work_mode_selection(self, parent):
        """创建工作模式选择"""
        try:
            mode_frame = tk.Frame(parent, bg=self.colors['white'])
            mode_frame.pack(fill=tk.X, pady=(0, 20))
            
            tk.Label(mode_frame, text="工作模式", 
                    bg=self.colors['white'], fg=self.colors['text_dark'],
                    font=self.fonts['text_large']).pack(anchor=tk.W, pady=(0, 10))
            
            # 模式按钮框架
            mode_buttons_frame = tk.Frame(mode_frame, bg=self.colors['white'])
            mode_buttons_frame.pack(fill=tk.X)
            
            # 手动按钮
            self.manual_btn = tk.Button(mode_buttons_frame, text="手动",
                                      command=lambda: self.set_work_mode("手动"),
                                      bg=self.colors['light_gray'], fg=self.colors['text_dark'],
                                      font=self.fonts['text_medium'], relief=tk.RAISED,
                                      width=8, height=2)
            self.manual_btn.pack(side=tk.LEFT, padx=(0, 10))
            
            # 自动按钮
            self.auto_btn = tk.Button(mode_buttons_frame, text="自动",
                                    command=lambda: self.set_work_mode("自动"),
                                    bg=self.colors['primary_green'], fg=self.colors['white'],
                                    font=self.fonts['text_medium'], relief=tk.RAISED,
                                    width=8, height=2)
            self.auto_btn.pack(side=tk.LEFT)
            
        except Exception as e:
            print(f"创建工作模式选择失败: {e}")
            traceback.print_exc()
    
    def create_harvest_control(self, parent):
        """创建采摘控制"""
        try:
            harvest_frame = tk.Frame(parent, bg=self.colors['white'])
            harvest_frame.pack(fill=tk.X, pady=(0, 20))
            
            tk.Label(harvest_frame, text="采摘控制", 
                    bg=self.colors['white'], fg=self.colors['text_dark'],
                    font=self.fonts['text_large']).pack(anchor=tk.W, pady=(0, 10))
            
            # 自动采摘按钮
            self.harvest_btn = tk.Button(harvest_frame, text="🔴 自动采摘中",
                                       command=self.toggle_harvest,
                                       bg=self.colors['primary_green'], fg=self.colors['white'],
                                       font=self.fonts['text_medium'], relief=tk.RAISED,
                                       height=2)
            self.harvest_btn.pack(fill=tk.X)
            
        except Exception as e:
            print(f"创建采摘控制失败: {e}")
            traceback.print_exc()
    
    def create_emergency_stop(self, parent):
        """创建紧急停止按钮"""
        try:
            emergency_frame = tk.Frame(parent, bg=self.colors['white'])
            emergency_frame.pack(fill=tk.X, pady=(0, 20))
            
            emergency_btn = tk.Button(emergency_frame, text="⭕ 紧急停止",
                                    command=self.emergency_stop,
                                    bg=self.colors['red'], fg=self.colors['white'],
                                    font=self.fonts['text_large'], relief=tk.RAISED,
                                    height=2)
            emergency_btn.pack(fill=tk.X)
            
        except Exception as e:
            print(f"创建紧急停止按钮失败: {e}")
            traceback.print_exc()
    
    def create_today_result(self, parent):
        """创建今日成果显示"""
        try:
            result_frame = tk.Frame(parent, bg=self.colors['light_green'], 
                                  relief=tk.RAISED, bd=1)
            result_frame.pack(fill=tk.X, pady=(20, 0))
            
            # 标题
            title_frame = tk.Frame(result_frame, bg=self.colors['light_green'])
            title_frame.pack(fill=tk.X, padx=15, pady=(15, 5))
            
            tk.Label(title_frame, text="📊 今日成果", 
                    bg=self.colors['light_green'], fg=self.colors['text_dark'],
                    font=self.fonts['text_medium']).pack(side=tk.LEFT)
            
            # 数字显示
            number_frame = tk.Frame(result_frame, bg=self.colors['light_green'])
            number_frame.pack(fill=tk.X, padx=15, pady=(0, 15))
            
            self.today_result_label = tk.Label(number_frame, textvariable=self.today_harvest,
                                             bg=self.colors['light_green'], fg=self.colors['primary_green'],
                                             font=self.fonts['number_large'])
            self.today_result_label.pack()
            
        except Exception as e:
            print(f"创建今日成果显示失败: {e}")
            traceback.print_exc()
    
    def create_center_panel(self):
        """创建中间实时监控面板"""
        try:
            self.center_frame = tk.Frame(self.main_frame, bg=self.colors['white'], 
                                       relief=tk.RAISED, bd=1)
            self.center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
            
            # 标题栏
            title_frame = tk.Frame(self.center_frame, bg=self.colors['primary_green'], height=50)
            title_frame.pack(fill=tk.X)
            title_frame.pack_propagate(False)
            
            # 标题
            tk.Label(title_frame, text="📹 实时监控", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['title_medium']).pack(side=tk.LEFT, padx=20, pady=15)
            
            # FPS显示
            self.fps_label = tk.Label(title_frame, text="FPS: 29",
                                    bg=self.colors['primary_green'], fg=self.colors['white'],
                                    font=self.fonts['text_medium'])
            self.fps_label.pack(side=tk.RIGHT, padx=20, pady=15)
            
            # 视频显示区域
            self.create_video_display()
            
            # 检测信息显示
            self.create_detection_info()
            
        except Exception as e:
            print(f"创建中间面板失败: {e}")
            traceback.print_exc()
    
    def create_video_display(self):
        """创建视频显示区域"""
        try:
            video_frame = tk.Frame(self.center_frame, bg=self.colors['white'])
            video_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(10, 0))
            
            # 视频画布
            self.video_canvas = tk.Canvas(video_frame, bg='black', height=400)
            self.video_canvas.pack(fill=tk.BOTH, expand=True)
            
            # 创建模拟的检测画面
            self.create_mock_detection_display()
            
        except Exception as e:
            print(f"创建视频显示区域失败: {e}")
            traceback.print_exc()
    
    def create_mock_detection_display(self):
        """创建模拟检测显示"""
        try:
            # 清除画布
            self.video_canvas.delete("all")
            
            # 绘制几个模拟的苹果检测框
            # 苹果1
            self.video_canvas.create_rectangle(100, 80, 150, 130, outline='#4CAF50', width=2)
            self.video_canvas.create_oval(110, 90, 140, 120, fill='#FF6B6B', outline='')
            self.video_canvas.create_text(125, 140, text="苹果 92%\n0.65m", 
                                        fill='#4CAF50', font=self.fonts['text_small'])
            
            # 苹果2
            self.video_canvas.create_rectangle(350, 60, 400, 110, outline='#4CAF50', width=2)
            self.video_canvas.create_oval(360, 70, 390, 100, fill='#FF8A80', outline='')
            self.video_canvas.create_text(375, 120, text="苹果 87%\n1.23m", 
                                        fill='#4CAF50', font=self.fonts['text_small'])
            
            # 苹果3
            self.video_canvas.create_oval(280, 180, 310, 210, fill='#FFAB91', outline='')
            
            # 苹果4
            self.video_canvas.create_oval(450, 200, 480, 230, fill='#FFCC80', outline='')
            
        except Exception as e:
            print(f"创建模拟检测显示失败: {e}")
            traceback.print_exc()
    
    def create_detection_info(self):
        """创建检测信息显示"""
        try:
            info_frame = tk.Frame(self.center_frame, bg=self.colors['white'])
            info_frame.pack(fill=tk.X, padx=20, pady=20)
            
            info_text = f"检测到 {self.detection_count.get()} 个目标 | " \
                       f"最近距离: {self.nearest_distance.get()} | " \
                       f"准确率: {self.accuracy.get()}"
            
            self.detection_info_label = tk.Label(info_frame, text=info_text,
                                               bg=self.colors['white'], fg=self.colors['text_dark'],
                                               font=self.fonts['text_medium'])
            self.detection_info_label.pack()
            
        except Exception as e:
            print(f"创建检测信息显示失败: {e}")
            traceback.print_exc()
    
    def create_right_panel(self):
        """创建右侧系统状态面板"""
        try:
            self.right_frame = tk.Frame(self.main_frame, bg=self.colors['white'], 
                                      width=280, relief=tk.RAISED, bd=1)
            self.right_frame.pack(side=tk.LEFT, fill=tk.Y)
            self.right_frame.pack_propagate(False)
            
            # 系统状态标题
            title_frame = tk.Frame(self.right_frame, bg=self.colors['primary_green'], height=50)
            title_frame.pack(fill=tk.X)
            title_frame.pack_propagate(False)
            
            tk.Label(title_frame, text="🔋 系统状态", 
                    bg=self.colors['primary_green'], fg=self.colors['white'],
                    font=self.fonts['title_medium']).pack(pady=15)
            
            # 状态内容
            status_content = tk.Frame(self.right_frame, bg=self.colors['white'])
            status_content.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
            
            # 电池电量
            self.create_battery_status(status_content)
            
            # CPU使用率
            self.create_cpu_status(status_content)
            
            # 系统温度
            self.create_temperature_status(status_content)
            
            # 位置信息面板
            self.create_position_panel(status_content)
            
        except Exception as e:
            print(f"创建右侧面板失败: {e}")
            traceback.print_exc()
    
    def create_battery_status(self, parent):
        """创建电池状态显示"""
        try:
            battery_frame = tk.Frame(parent, bg=self.colors['white'])
            battery_frame.pack(fill=tk.X, pady=(0, 15))
            
            tk.Label(battery_frame, text="电池电量", 
                    bg=self.colors['white'], fg=self.colors['text_dark'],
                    font=self.fonts['text_medium']).pack(anchor=tk.W)
            
            # 进度条框架
            progress_frame = tk.Frame(battery_frame, bg=self.colors['white'])
            progress_frame.pack(fill=tk.X, pady=(5, 0))
            
            # 电池进度条
            self.battery_progress = ttk.Progressbar(progress_frame, length=200, mode='determinate')
            self.battery_progress.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.battery_progress['value'] = 75
            
            # 百分比标签
            self.battery_percent_label = tk.Label(progress_frame, text="75%",
                                                bg=self.colors['white'], fg=self.colors['primary_green'],
                                                font=self.fonts['text_medium'])
            self.battery_percent_label.pack(side=tk.RIGHT, padx=(10, 0))
            
        except Exception as e:
            print(f"创建电池状态失败: {e}")
            traceback.print_exc()
    
    def create_cpu_status(self, parent):
        """创建CPU状态显示"""
        try:
            cpu_frame = tk.Frame(parent, bg=self.colors['white'])
            cpu_frame.pack(fill=tk.X, pady=(0, 15))
            
            tk.Label(cpu_frame, text="CPU使用率", 
                    bg=self.colors['white'], fg=self.colors['text_dark'],
                    font=self.fonts['text_medium']).pack(anchor=tk.W)
            
            # 进度条框架
            progress_frame = tk.Frame(cpu_frame, bg=self.colors['white'])
            progress_frame.pack(fill=tk.X, pady=(5, 0))
            
            # CPU进度条
            self.cpu_progress = ttk.Progressbar(progress_frame, length=200, mode='determinate')
            self.cpu_progress.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.cpu_progress['value'] = 52
            
            # 百分比标签
            self.cpu_percent_label = tk.Label(progress_frame, text="52%",
                                            bg=self.colors['white'], fg=self.colors['orange'],
                                            font=self.fonts['text_medium'])
            self.cpu_percent_label.pack(side=tk.RIGHT, padx=(10, 0))
            
        except Exception as e:
            print(f"创建CPU状态失败: {e}")
            traceback.print_exc()
    
    def create_temperature_status(self, parent):
        """创建温度状态显示"""
        try:
            temp_frame = tk.Frame(parent, bg=self.colors['white'])
            temp_frame.pack(fill=tk.X, pady=(0, 15))
            
            tk.Label(temp_frame, text="系统温度", 
                    bg=self.colors['white'], fg=self.colors['text_dark'],
                    font=self.fonts['text_medium']).pack(anchor=tk.W)
            
            self.temp_label = tk.Label(temp_frame, textvariable=self.system_temp,
                                     bg=self.colors['white'], fg=self.colors['primary_green'],
                                     font=self.fonts['number_medium'])
            self.temp_label.pack(anchor=tk.W, pady=(5, 0))
            
            tk.Label(temp_frame, text="正常工作温度", 
                    bg=self.colors['white'], fg=self.colors['text_secondary'],
                    font=self.fonts['text_small']).pack(anchor=tk.W)
            
        except Exception as e:
            print(f"创建温度状态失败: {e}")
            traceback.print_exc()
    
    def create_position_panel(self, parent):
        """创建位置信息面板"""
        try:
            pos_frame = tk.Frame(parent, bg=self.colors['light_green'], 
                               relief=tk.RAISED, bd=1)
            pos_frame.pack(fill=tk.X, pady=(20, 0))
            
            # 标题
            title_frame = tk.Frame(pos_frame, bg=self.colors['light_green'])
            title_frame.pack(fill=tk.X, padx=15, pady=(15, 10))
            
            tk.Label(title_frame, text="📍 位置信息", 
                    bg=self.colors['light_green'], fg=self.colors['text_dark'],
                    font=self.fonts['text_medium']).pack(anchor=tk.W)
            
            # 位置详情
            pos_content = tk.Frame(pos_frame, bg=self.colors['light_green'])
            pos_content.pack(fill=tk.X, padx=15, pady=(0, 15))
            
            # 当前位置
            tk.Label(pos_content, text="当前位置", 
                    bg=self.colors['light_green'], fg=self.colors['text_dark'],
                    font=self.fonts['text_small']).pack(anchor=tk.W)
            
            location_text = f"纬度: {self.latitude.get()}\n" \
                           f"经度: {self.longitude.get()}\n" \
                           f"区域: {self.location_area.get()}"
            
            tk.Label(pos_content, text=location_text,
                    bg=self.colors['light_green'], fg=self.colors['text_secondary'],
                    font=self.fonts['text_small'], justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 10))
            
            # 工作状态
            tk.Label(pos_content, text="工作状态", 
                    bg=self.colors['light_green'], fg=self.colors['text_dark'],
                    font=self.fonts['text_small']).pack(anchor=tk.W)
            
            work_text = f"{self.work_status.get()}\n" \
                       f"工作时长: {self.work_time.get()}\n" \
                       f"移动速度: {self.move_speed.get()}"
            
            tk.Label(pos_content, text=work_text,
                    bg=self.colors['light_green'], fg=self.colors['text_secondary'],
                    font=self.fonts['text_small'], justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 0))
            
        except Exception as e:
            print(f"创建位置信息面板失败: {e}")
            traceback.print_exc()
    
    def create_bottom_panel(self):
        """创建底部农场数据总览面板"""
        try:
            bottom_frame = tk.Frame(self.root, bg=self.colors['bg_green'], height=120)
            bottom_frame.pack(fill=tk.X, padx=20, pady=(0, 20))
            bottom_frame.pack_propagate(False)
            
            # 标题
            title_frame = tk.Frame(bottom_frame, bg=self.colors['bg_green'])
            title_frame.pack(fill=tk.X, pady=(10, 0))
            
            tk.Label(title_frame, text="🌾 农场数据总览", 
                    bg=self.colors['bg_green'], fg=self.colors['text_dark'],
                    font=self.fonts['title_medium']).pack(anchor=tk.W)
            
            # 数据卡片容器
            cards_frame = tk.Frame(bottom_frame, bg=self.colors['bg_green'])
            cards_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
            
            # 创建5个数据卡片
            self.create_data_card(cards_frame, "累计采摘", self.total_harvest.get() + " 个", 0)
            self.create_data_card(cards_frame, "工作精度", self.work_accuracy.get(), 1)
            self.create_data_card(cards_frame, "信号强度", self.signal_strength.get(), 2)
            self.create_data_card(cards_frame, "运行天数", self.work_days.get() + " 天", 3)
            self.create_data_card(cards_frame, "健康状态", self.health_status.get(), 4)
            
        except Exception as e:
            print(f"创建底部面板失败: {e}")
            traceback.print_exc()
    
    def create_data_card(self, parent, title, value, index):
        """创建数据卡片"""
        try:
            card = tk.Frame(parent, bg=self.colors['white'], relief=tk.RAISED, bd=1)
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10) if index < 4 else (0, 0))
            
            # 标题
            tk.Label(card, text=title, 
                    bg=self.colors['white'], fg=self.colors['text_secondary'],
                    font=self.fonts['text_small']).pack(pady=(15, 5))
            
            # 值
            tk.Label(card, text=value, 
                    bg=self.colors['white'], fg=self.colors['primary_green'],
                    font=self.fonts['number_medium']).pack(pady=(0, 15))
            
        except Exception as e:
            print(f"创建数据卡片失败: {e}")
            traceback.print_exc()
    
    def set_work_mode(self, mode):
        """设置工作模式"""
        try:
            self.work_mode.set(mode)
            if mode == "手动":
                self.manual_btn.config(bg=self.colors['primary_green'], fg=self.colors['white'])
                self.auto_btn.config(bg=self.colors['light_gray'], fg=self.colors['text_dark'])
            else:
                self.auto_btn.config(bg=self.colors['primary_green'], fg=self.colors['white'])
                self.manual_btn.config(bg=self.colors['light_gray'], fg=self.colors['text_dark'])
            print(f"工作模式切换为: {mode}")
        except Exception as e:
            print(f"设置工作模式失败: {e}")
            traceback.print_exc()
    
    def toggle_harvest(self):
        """切换采摘状态"""
        try:
            current = self.harvest_active.get()
            self.harvest_active.set(not current)
            
            if self.harvest_active.get():
                self.harvest_btn.config(text="🔴 自动采摘中", bg=self.colors['primary_green'])
                print("开始自动采摘")
            else:
                self.harvest_btn.config(text="⚪ 开始采摘", bg=self.colors['light_gray'])
                print("停止自动采摘")
        except Exception as e:
            print(f"切换采摘状态失败: {e}")
            traceback.print_exc()
    
    def emergency_stop(self):
        """紧急停止"""
        try:
            print("执行紧急停止！")
            self.harvest_active.set(False)
            self.harvest_btn.config(text="⚪ 开始采摘", bg=self.colors['light_gray'])
            # 这里后续添加实际的紧急停止逻辑
        except Exception as e:
            print(f"紧急停止失败: {e}")
            traceback.print_exc()
    
    def update_time(self):
        """更新时间显示"""
        try:
            current_time = datetime.now().strftime("%H:%M")
            self.current_time.set(current_time)
        except Exception as e:
            print(f"更新时间失败: {e}")
            traceback.print_exc()
    
    def update_fps_display(self):
        """更新FPS显示"""
        try:
            fps_text = f"FPS: {self.fps_value.get()}"
            self.fps_label.config(text=fps_text)
        except Exception as e:
            print(f"更新FPS显示失败: {e}")
            traceback.print_exc()
    
    def update_detection_info_display(self):
        """更新检测信息显示"""
        try:
            info_text = f"检测到 {self.detection_count.get()} 个目标 | " \
                       f"最近距离: {self.nearest_distance.get()} | " \
                       f"准确率: {self.accuracy.get()}"
            self.detection_info_label.config(text=info_text)
        except Exception as e:
            print(f"更新检测信息显示失败: {e}")
            traceback.print_exc()
    
    def start_update_timer(self):
        """启动更新定时器"""
        try:
            def update_loop():
                while True:
                    try:
                        self.root.after(0, self.update_time)
                        self.root.after(0, self.update_fps_display)
                        self.root.after(0, self.update_detection_info_display)
                        time.sleep(1)  # 每秒更新一次
                    except Exception as e:
                        print(f"更新循环错误: {e}")
                        traceback.print_exc()
            
            update_thread = threading.Thread(target=update_loop, daemon=True)
            update_thread.start()
        except Exception as e:
            print(f"启动更新定时器失败: {e}")
            traceback.print_exc()
    
    def run(self):
        """运行主循环"""
        try:
            print("智慧农业采摘系统本地控制界面启动成功")
            self.root.mainloop()
        except Exception as e:
            print(f"运行主循环失败: {e}")
            traceback.print_exc()


def main():
    """主函数"""
    try:
        print("正在启动智慧农业采摘系统本地控制界面...")
        app = SmartHarvestingGUI()
        app.run()
    except Exception as e:
        print(f"程序启动失败: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
