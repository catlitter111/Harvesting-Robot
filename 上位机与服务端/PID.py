import tkinter as tk
from tkinter import ttk, messagebox, font
import serial
import serial.tools.list_ports
import threading
import struct
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import numpy as np
import platform
import warnings
import math
import os
import pwd
import grp
import queue

# 忽略字体警告
warnings.filterwarnings('ignore', category=UserWarning)

# 设置matplotlib支持中文显示
import matplotlib
if platform.system() == 'Windows':
    matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial']
elif platform.system() == 'Darwin':
    matplotlib.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'Heiti TC']
else:
    matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class MaterialCard(tk.Frame):
    """Material Design风格的卡片组件"""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self.configure(
            bg='white',
            relief=tk.FLAT,
            highlightthickness=0
        )
        
        # 添加阴影效果（通过边框模拟）
        self.configure(highlightbackground='#e0e0e0', highlightthickness=1)
        
        # 添加内边距
        self.grid_propagate(False)
        
    def add_shadow(self):
        """添加阴影效果"""
        # 在实际应用中，可以使用Canvas绘制真实阴影
        pass


class MaterialButton(tk.Button):
    """Material Design风格的按钮"""
    def __init__(self, parent, text="", command=None, style="primary", **kwargs):
        # Material Design颜色方案
        colors = {
            "primary": {"bg": "#1976D2", "fg": "white", "hover": "#1565C0", "active": "#0D47A1"},
            "secondary": {"bg": "#424242", "fg": "white", "hover": "#303030", "active": "#212121"},
            "success": {"bg": "#388E3C", "fg": "white", "hover": "#2E7D32", "active": "#1B5E20"},
            "danger": {"bg": "#D32F2F", "fg": "white", "hover": "#C62828", "active": "#B71C1C"},
            "warning": {"bg": "#F57C00", "fg": "white", "hover": "#E65100", "active": "#BF360C"},
            "fab": {"bg": "#FF4081", "fg": "white", "hover": "#F50057", "active": "#C51162"}
        }
        
        self.colors = colors.get(style, colors["primary"])
        
        super().__init__(
            parent,
            text=text,
            command=command,
            bg=self.colors["bg"],
            fg=self.colors["fg"],
            relief=tk.FLAT,
            padx=16,
            pady=8,
            font=("Roboto", 10, "bold"),
            cursor="hand2",
            **kwargs
        )
        
        # 绑定悬停效果
        self.bind("<Enter>", lambda e: self.config(bg=self.colors["hover"]))
        self.bind("<Leave>", lambda e: self.config(bg=self.colors["bg"]))
        self.bind("<Button-1>", lambda e: self.config(bg=self.colors["active"]))
        self.bind("<ButtonRelease-1>", lambda e: self.config(bg=self.colors["hover"]))


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Robot Control System")
        self.root.geometry("1400x900")
        
        # 硬编码的串口设置
        self.SERIAL_PORT = "/dev/ttyS3"  # Ubuntu系统常用的USB串口
        self.BAUDRATE = 115200
        
        # Material Design配色
        self.colors = {
            'primary': '#1976D2',
            'primary_dark': '#1565C0',
            'primary_light': '#42A5F5',
            'secondary': '#FF4081',
            'background': '#FAFAFA',
            'surface': '#FFFFFF',
            'error': '#D32F2F',
            'success': '#388E3C',
            'warning': '#F57C00',
            'text_primary': '#212121',
            'text_secondary': '#757575',
            'divider': '#BDBDBD'
        }
        
        self.root.configure(bg=self.colors['background'])
        
        # 串口相关变量
        self.serial_port = None
        self.running = False
        
        # 线程相关
        self.serial_thread = None
        self.send_thread = None
        self.connection_check_thread = None
        
        # 发送队列和控制
        self.send_queue = queue.Queue()
        self.last_send_time = 0
        self.min_send_interval = 0.05  # 最小发送间隔50ms
        
        # 线程锁
        self.serial_lock = threading.RLock()  # 使用可重入锁
        
        # 数据存储
        self.plot_data = {
            'time': deque(maxlen=200),
            'fr_actual': deque(maxlen=200),
            'fr_target': deque(maxlen=200),
            'rl_actual': deque(maxlen=200),
            'rl_target': deque(maxlen=200)
        }
        
        self.plot_start_time = None
        self.last_update_time = 0
        
        # PID参数
        self.pid_params = {
            'front_right': {'kp': 1.5, 'ki': 0.2, 'kd': 0.1},
            'rear_left': {'kp': 1.5, 'ki': 0.2, 'kd': 0.1}
        }
        
        # 当前状态
        self.current_speed_fr = 0.0
        self.current_speed_rl = 0.0
        self.target_speed_fr = 0.0
        self.target_speed_rl = 0.0
        self.pid_output_fr = 0.0
        self.pid_output_rl = 0.0
        
        # PID分析数据
        self.analysis_data = {
            'front_right': {
                'response_samples': [],
                'analysis_start_time': None,
                'target_changed': False
            },
            'rear_left': {
                'response_samples': [],
                'analysis_start_time': None,
                'target_changed': False
            }
        }
        
        # 创建主界面
        self.create_ui()
        self.check_serial_permissions()  # 检查串口权限
        
    def check_serial_permissions(self):
        """检查Linux下的串口权限"""
        if platform.system() == 'Linux':
            try:
                username = pwd.getpwuid(os.getuid()).pw_name
                dialout_group = grp.getgrnam('dialout')
                if username not in dialout_group.gr_mem:
                    messagebox.showwarning(
                        "Permission Warning",
                        f"You may need to add your user to the 'dialout' group to access serial ports.\n\n"
                        f"Run: sudo usermod -a -G dialout $USER\n"
                        f"Then logout and login again.\n\n"
                        f"Current serial port: {self.SERIAL_PORT}"
                    )
            except:
                pass
        
    def create_ui(self):
        """创建Material Design风格的UI"""
        # 顶部工具栏
        self.create_toolbar()
        
        # 主内容区域
        main_container = tk.Frame(self.root, bg=self.colors['background'])
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # 创建左右分割的PanedWindow
        paned = tk.PanedWindow(main_container, orient=tk.HORIZONTAL, bg=self.colors['background'], 
                              sashrelief=tk.FLAT, sashwidth=8, sashpad=2)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # 左侧控制面板 - 使用Canvas和Scrollbar支持滚动
        left_container = tk.Frame(paned, bg=self.colors['background'])
        
        # 创建Canvas和Scrollbar
        canvas = tk.Canvas(left_container, bg=self.colors['background'], highlightthickness=0)
        scrollbar = tk.Scrollbar(left_container, orient="vertical", command=canvas.yview)
        
        # 创建可滚动的Frame
        left_panel = tk.Frame(canvas, bg=self.colors['background'])
        
        # 配置Canvas
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas_frame = canvas.create_window((0, 0), window=left_panel, anchor="nw")
        
        # 更新Canvas滚动区域
        def configure_canvas(event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            # 设置Canvas宽度跟随容器
            canvas.itemconfig(canvas_frame, width=canvas.winfo_width())
        
        left_panel.bind("<Configure>", configure_canvas)
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(canvas_frame, width=e.width))
        
        # 布局Canvas和Scrollbar
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 添加鼠标滚轮支持
        def _on_mousewheel(event):
            if platform.system() == 'Linux':
                # Linux下的滚轮事件处理
                if event.num == 4:
                    canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    canvas.yview_scroll(1, "units")
            else:
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
                
        if platform.system() == 'Linux':
            canvas.bind_all("<Button-4>", _on_mousewheel)
            canvas.bind_all("<Button-5>", _on_mousewheel)
        else:
            canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # 串口连接卡片
        self.create_connection_card(left_panel)
        
        # 运动控制卡片
        self.create_motion_card(left_panel)
        
        # PID控制卡片
        self.create_pid_card(left_panel)
        
        # 将左侧容器添加到PanedWindow
        paned.add(left_container, width=450, minsize=400)
        
        # 右侧内容
        right_panel = tk.Frame(paned, bg=self.colors['background'])
        paned.add(right_panel, minsize=600)
        
        # 实时图表卡片
        self.create_chart_card(right_panel)
        
        # 状态显示卡片
        self.create_status_card(right_panel)
        
    def create_toolbar(self):
        """创建顶部工具栏"""
        toolbar = tk.Frame(self.root, bg=self.colors['primary'], height=60)
        toolbar.pack(fill=tk.X)
        toolbar.pack_propagate(False)
        
        # 应用标题
        title_label = tk.Label(
            toolbar,
            text="STM32 Robot Control System",
            bg=self.colors['primary'],
            fg="white",
            font=("Roboto", 18, "bold")
        )
        title_label.pack(side=tk.LEFT, padx=20, pady=15)
        
        # 串口信息显示
        port_info_label = tk.Label(
            toolbar,
            text=f"Port: {self.SERIAL_PORT}",
            bg=self.colors['primary'],
            fg="white",
            font=("Roboto", 12)
        )
        port_info_label.pack(side=tk.LEFT, padx=20)
        
        # 右侧状态指示器
        self.connection_indicator = tk.Label(
            toolbar,
            text="● Disconnected",
            bg=self.colors['primary'],
            fg="#FF5252",
            font=("Roboto", 12)
        )
        self.connection_indicator.pack(side=tk.RIGHT, padx=20)
        
    def create_connection_card(self, parent):
        """创建串口连接卡片"""
        card = MaterialCard(parent)
        card.pack(fill=tk.X, pady=(0, 15))
        
        # 卡片内容
        content = tk.Frame(card, bg="white", padx=20, pady=20)
        content.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        tk.Label(
            content,
            text="Serial Connection",
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 14, "bold")
        ).pack(anchor=tk.W, pady=(0, 15))
        
        # 串口信息显示
        info_frame = tk.Frame(content, bg="white")
        info_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(
            info_frame,
            text="Port:",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 10)
        ).pack(side=tk.LEFT, padx=(0, 10))
        
        tk.Label(
            info_frame,
            text=self.SERIAL_PORT,
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 10, "bold")
        ).pack(side=tk.LEFT)
        
        # 波特率显示
        baud_frame = tk.Frame(content, bg="white")
        baud_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(
            baud_frame,
            text="Baudrate:",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 10)
        ).pack(side=tk.LEFT, padx=(0, 10))
        
        tk.Label(
            baud_frame,
            text=str(self.BAUDRATE),
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 10, "bold")
        ).pack(side=tk.LEFT)
        
        # 连接状态和按钮
        button_frame = tk.Frame(content, bg="white")
        button_frame.pack(fill=tk.X, pady=(15, 0))
        
        # 连接按钮
        self.connect_btn = MaterialButton(
            button_frame,
            text="CONNECT",
            command=self.toggle_connection,
            style="primary"
        )
        self.connect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        # 检查串口按钮
        MaterialButton(
            button_frame,
            text="CHECK PORT",
            command=self.check_serial_port,
            style="secondary"
        ).pack(side=tk.LEFT)
        
    def check_serial_port(self):
        """检查串口是否存在"""
        if os.path.exists(self.SERIAL_PORT):
            try:
                # 尝试获取串口信息
                test_port = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
                test_port.close()
                messagebox.showinfo("Port Check", f"Serial port {self.SERIAL_PORT} is available!")
            except serial.SerialException as e:
                if "Permission denied" in str(e):
                    messagebox.showerror("Permission Error", 
                        f"Serial port {self.SERIAL_PORT} exists but access is denied.\n\n"
                        f"Try: sudo chmod 666 {self.SERIAL_PORT}\n"
                        f"Or add user to dialout group:\n"
                        f"sudo usermod -a -G dialout $USER")
                else:
                    messagebox.showerror("Port Error", f"Cannot access {self.SERIAL_PORT}:\n{e}")
            except Exception as e:
                messagebox.showerror("Error", f"Error checking port: {e}")
        else:
            messagebox.showerror("Port Not Found", 
                f"Serial port {self.SERIAL_PORT} does not exist!\n\n"
                f"Common alternatives:\n"
                f"• /dev/ttyUSB0, /dev/ttyUSB1\n"
                f"• /dev/ttyACM0, /dev/ttyACM1\n"
                f"• /dev/ttyS0, /dev/ttyS1")
        
    def create_motion_card(self, parent):
        """创建运动控制卡片"""
        card = MaterialCard(parent)
        card.pack(fill=tk.X, pady=(0, 15))
        
        content = tk.Frame(card, bg="white", padx=20, pady=20)
        content.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        tk.Label(
            content,
            text="Motion Control",
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 14, "bold")
        ).pack(anchor=tk.W, pady=(0, 15))
        
        # 方向控制按钮
        control_frame = tk.Frame(content, bg="white")
        control_frame.pack()
        
        # 创建圆形按钮样式的方向键
        btn_size = 50
        
        # 前进
        forward_btn = tk.Button(
            control_frame,
            text="▲",
            command=lambda: self.send_direction_cmd(0x00),
            bg=self.colors['primary'],
            fg="white",
            font=("Arial", 16),
            width=btn_size//10,
            height=btn_size//20,
            relief=tk.FLAT,
            cursor="hand2"
        )
        forward_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # 左转
        left_btn = tk.Button(
            control_frame,
            text="◄",
            command=lambda: self.send_direction_cmd(0x02),
            bg=self.colors['primary'],
            fg="white",
            font=("Arial", 16),
            width=btn_size//10,
            height=btn_size//20,
            relief=tk.FLAT,
            cursor="hand2"
        )
        left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        # 停止（使用不同颜色）
        stop_btn = tk.Button(
            control_frame,
            text="■",
            command=lambda: self.send_direction_cmd(0x04),
            bg=self.colors['error'],
            fg="white",
            font=("Arial", 16),
            width=btn_size//10,
            height=btn_size//20,
            relief=tk.FLAT,
            cursor="hand2"
        )
        stop_btn.grid(row=1, column=1, padx=5, pady=5)
        
        # 右转
        right_btn = tk.Button(
            control_frame,
            text="►",
            command=lambda: self.send_direction_cmd(0x03),
            bg=self.colors['primary'],
            fg="white",
            font=("Arial", 16),
            width=btn_size//10,
            height=btn_size//20,
            relief=tk.FLAT,
            cursor="hand2"
        )
        right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # 后退
        backward_btn = tk.Button(
            control_frame,
            text="▼",
            command=lambda: self.send_direction_cmd(0x01),
            bg=self.colors['primary'],
            fg="white",
            font=("Arial", 16),
            width=btn_size//10,
            height=btn_size//20,
            relief=tk.FLAT,
            cursor="hand2"
        )
        backward_btn.grid(row=2, column=1, padx=5, pady=5)
        
        # 为所有方向按钮添加悬停效果
        for btn in [forward_btn, left_btn, right_btn, backward_btn]:
            btn.bind("<Enter>", lambda e, b=btn: b.config(bg=self.colors['primary_dark']))
            btn.bind("<Leave>", lambda e, b=btn: b.config(bg=self.colors['primary']))
            
        stop_btn.bind("<Enter>", lambda e: stop_btn.config(bg='#C62828'))
        stop_btn.bind("<Leave>", lambda e: stop_btn.config(bg=self.colors['error']))
        
        # 速度控制
        tk.Label(
            content,
            text="Speed Control",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 12)
        ).pack(anchor=tk.W, pady=(20, 5))
        
        speed_frame = tk.Frame(content, bg="white")
        speed_frame.pack(fill=tk.X)
        
        self.speed_var = tk.IntVar(value=50)
        self.speed_scale = tk.Scale(
            speed_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            bg="white",
            fg=self.colors['primary'],
            highlightthickness=0,
            troughcolor='#E0E0E0',
            activebackground=self.colors['primary_light'],
            command=self.on_speed_change
        )
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.speed_label = tk.Label(
            speed_frame,
            text="50%",
            bg="white",
            fg=self.colors['primary'],
            font=("Roboto", 12, "bold"),
            width=5
        )
        self.speed_label.pack(side=tk.LEFT, padx=(10, 0))
        
    def create_pid_card(self, parent):
        """创建PID控制卡片"""
        card = MaterialCard(parent)
        card.pack(fill=tk.X, pady=(0, 15))
        
        content = tk.Frame(card, bg="white", padx=20, pady=20)
        content.pack(fill=tk.BOTH, expand=True)
        
        # 标题和开关
        header_frame = tk.Frame(content, bg="white")
        header_frame.pack(fill=tk.X, pady=(0, 15))
        
        tk.Label(
            header_frame,
            text="PID Control",
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 14, "bold")
        ).pack(side=tk.LEFT)
        
        # PID开关
        self.pid_enabled_var = tk.BooleanVar(value=True)
        switch_frame = tk.Frame(header_frame, bg="white")
        switch_frame.pack(side=tk.RIGHT)
        
        self.pid_switch = tk.Checkbutton(
            switch_frame,
            text="Enable",
            variable=self.pid_enabled_var,
            command=self.toggle_pid,
            bg="white",
            fg=self.colors['text_secondary'],
            selectcolor="white",
            font=("Roboto", 10)
        )
        self.pid_switch.pack()
        
        # 前右电机PID
        fr_frame = tk.LabelFrame(
            content,
            text="Front Right Motor (TIM2)",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 10),
            relief=tk.GROOVE,
            borderwidth=1
        )
        fr_frame.pack(fill=tk.X, pady=5)
        
        self.create_pid_inputs(fr_frame, 'front_right')
        
        # 后左电机PID
        rl_frame = tk.LabelFrame(
            content,
            text="Rear Left Motor (TIM4)",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 10),
            relief=tk.GROOVE,
            borderwidth=1
        )
        rl_frame.pack(fill=tk.X, pady=5)
        
        self.create_pid_inputs(rl_frame, 'rear_left')
        
        # 操作按钮 - 使用两行布局
        btn_frame1 = tk.Frame(content, bg="white")
        btn_frame1.pack(fill=tk.X, pady=(10, 5))
        
        MaterialButton(
            btn_frame1,
            text="APPLY PARAMETERS",
            command=self.apply_pid_params,
            style="primary"
        ).pack(fill=tk.X)
        
        btn_frame2 = tk.Frame(content, bg="white")
        btn_frame2.pack(fill=tk.X)
        
        MaterialButton(
            btn_frame2,
            text="RESET",
            command=self.reset_pid,
            style="secondary"
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        MaterialButton(
            btn_frame2,
            text="TEST",
            command=self.test_communication,
            style="secondary"
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))
        
        # 智能分析按钮
        btn_frame3 = tk.Frame(content, bg="white")
        btn_frame3.pack(fill=tk.X, pady=(5, 0))
        
        MaterialButton(
            btn_frame3,
            text="SMART ANALYSIS",
            command=self.analyze_pid_performance,
            style="success"
        ).pack(fill=tk.X)
        
        # PID建议显示区域
        self.suggestion_frame = tk.LabelFrame(
            content,
            text="PID Tuning Suggestions",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 10),
            relief=tk.GROOVE,
            borderwidth=1
        )
        self.suggestion_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        
        # 建议文本显示
        self.suggestion_text = tk.Text(
            self.suggestion_frame,
            height=8,
            wrap=tk.WORD,
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 9),
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground="#BDBDBD",
            highlightcolor=self.colors['primary']
        )
        self.suggestion_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 初始化提示文本
        self.suggestion_text.insert(tk.END, "Click 'SMART ANALYSIS' to analyze PID performance.\n\n"
                                           "The system will monitor motor response and provide tuning suggestions.")
        self.suggestion_text.config(state=tk.DISABLED)
        
    def create_pid_inputs(self, parent, motor_name):
        """创建PID参数输入框"""
        frame = tk.Frame(parent, bg="white", padx=10, pady=10)
        frame.pack(fill=tk.X)
        
        # 使用网格布局，让输入框更紧凑
        params = ['Kp', 'Ki', 'Kd']
        
        for i, param in enumerate(params):
            # 参数标签和输入框放在同一行
            row_frame = tk.Frame(frame, bg="white")
            row_frame.pack(fill=tk.X, pady=3)
            
            tk.Label(
                row_frame,
                text=f"{param}:",
                bg="white",
                fg=self.colors['text_secondary'],
                font=("Roboto", 10),
                width=4
            ).pack(side=tk.LEFT, padx=(0, 5))
            
            var = tk.DoubleVar(value=self.pid_params[motor_name][param.lower()])
            entry = tk.Entry(
                row_frame,
                width=10,
                bg="white",
                fg=self.colors['text_primary'],
                insertbackground=self.colors['primary'],
                relief=tk.FLAT,
                font=("Roboto", 10),
                highlightbackground="#BDBDBD",
                highlightcolor=self.colors['primary'],
                highlightthickness=1
            )
            entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
            entry.insert(0, str(var.get()))
            
            # 保存引用
            if motor_name == 'front_right':
                if param == 'Kp':
                    self.fr_kp_var = var
                    self.fr_kp_entry = entry
                elif param == 'Ki':
                    self.fr_ki_var = var
                    self.fr_ki_entry = entry
                else:
                    self.fr_kd_var = var
                    self.fr_kd_entry = entry
            else:
                if param == 'Kp':
                    self.rl_kp_var = var
                    self.rl_kp_entry = entry
                elif param == 'Ki':
                    self.rl_ki_var = var
                    self.rl_ki_entry = entry
                else:
                    self.rl_kd_var = var
                    self.rl_kd_entry = entry
            
    def create_chart_card(self, parent):
        """创建实时图表卡片"""
        card = MaterialCard(parent)
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 15))
        
        content = tk.Frame(card, bg="white", padx=20, pady=20)
        content.pack(fill=tk.BOTH, expand=True)
        
        # 标题栏
        header_frame = tk.Frame(content, bg="white")
        header_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(
            header_frame,
            text="Real-time Speed Chart",
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 14, "bold")
        ).pack(side=tk.LEFT)
        
        MaterialButton(
            header_frame,
            text="CLEAR",
            command=self.clear_chart,
            style="secondary"
        ).pack(side=tk.RIGHT)
        
        # 创建图表
        self.fig = Figure(figsize=(10, 5), dpi=100, facecolor='white')
        self.ax = self.fig.add_subplot(111)
        
        # 设置Material Design风格
        self.ax.set_facecolor('#FAFAFA')
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['left'].set_color('#E0E0E0')
        self.ax.spines['bottom'].set_color('#E0E0E0')
        
        self.ax.set_xlabel('Time (s)', color=self.colors['text_secondary'])
        self.ax.set_ylabel('Speed', color=self.colors['text_secondary'])
        self.ax.tick_params(colors=self.colors['text_secondary'])
        self.ax.grid(True, alpha=0.2, linestyle='-', color='#E0E0E0')
        
        # 初始化曲线
        self.line_fr_actual, = self.ax.plot([], [], color=self.colors['primary'], 
                                           label='FR Actual', linewidth=2)
        self.line_fr_target, = self.ax.plot([], [], color=self.colors['primary'], 
                                           label='FR Target', linewidth=1, linestyle='--', alpha=0.7)
        self.line_rl_actual, = self.ax.plot([], [], color=self.colors['secondary'], 
                                           label='RL Actual', linewidth=2)
        self.line_rl_target, = self.ax.plot([], [], color=self.colors['secondary'], 
                                           label='RL Target', linewidth=1, linestyle='--', alpha=0.7)
        
        self.ax.legend(loc='upper right', frameon=False)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlim(0, 20)
        
        # 嵌入图表
        self.canvas = FigureCanvasTkAgg(self.fig, master=content)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 启动动画
        self.animate_plot()
        
    def create_status_card(self, parent):
        """创建状态显示卡片"""
        card = MaterialCard(parent)
        card.pack(fill=tk.X)
        
        content = tk.Frame(card, bg="white", padx=20, pady=20)
        content.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        tk.Label(
            content,
            text="System Status",
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 14, "bold")
        ).pack(anchor=tk.W, pady=(0, 15))
        
        # 状态网格
        status_grid = tk.Frame(content, bg="white")
        status_grid.pack(fill=tk.BOTH, expand=True)
        
        # 配置网格权重
        status_grid.grid_columnconfigure(0, weight=1)
        status_grid.grid_columnconfigure(1, weight=1)
        status_grid.grid_columnconfigure(2, weight=1)
        
        # 前右电机状态
        fr_frame = self.create_status_group(status_grid, "Front Right Motor", 0, 0)
        self.fr_speed_label = self.create_status_item(fr_frame, "Speed", "0.0")
        self.fr_target_label = self.create_status_item(fr_frame, "Target", "0.0")
        self.fr_output_label = self.create_status_item(fr_frame, "PID Out", "0.0")
        
        # 后左电机状态
        rl_frame = self.create_status_group(status_grid, "Rear Left Motor", 0, 1)
        self.rl_speed_label = self.create_status_item(rl_frame, "Speed", "0.0")
        self.rl_target_label = self.create_status_item(rl_frame, "Target", "0.0")
        self.rl_output_label = self.create_status_item(rl_frame, "PID Out", "0.0")
        
        # 通信状态
        comm_frame = self.create_status_group(status_grid, "Communication", 0, 2)
        self.comm_status_label = self.create_status_item(comm_frame, "Status", "Waiting...")
        self.packet_count_label = self.create_status_item(comm_frame, "Packets", "0")
        
    def create_status_group(self, parent, title, row, col):
        """创建状态组"""
        frame = tk.Frame(parent, bg="white")
        frame.grid(row=row, column=col, padx=10, pady=5, sticky=tk.NSEW)
        
        tk.Label(
            frame,
            text=title,
            bg="white",
            fg=self.colors['primary'],
            font=("Roboto", 11, "bold")
        ).pack(anchor=tk.W)
        
        return frame
        
    def create_status_item(self, parent, label, value):
        """创建状态项"""
        frame = tk.Frame(parent, bg="white")
        frame.pack(fill=tk.X, pady=2)
        
        tk.Label(
            frame,
            text=f"{label}:",
            bg="white",
            fg=self.colors['text_secondary'],
            font=("Roboto", 9)
        ).pack(side=tk.LEFT)
        
        value_label = tk.Label(
            frame,
            text=value,
            bg="white",
            fg=self.colors['text_primary'],
            font=("Roboto", 9, "bold")
        )
        value_label.pack(side=tk.LEFT, padx=(5, 0))
        
        return value_label
        
    # ========== 串口通信相关方法 ==========
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """连接串口"""
        try:
            # 添加超时设置，防止阻塞
            self.serial_port = serial.Serial(
                port=self.SERIAL_PORT, 
                baudrate=self.BAUDRATE, 
                timeout=0.1,
                write_timeout=0.5
            )
            
            # 清除输入缓冲区
            self.serial_port.reset_input_buffer()
            
            self.running = True
            
            # 启动发送线程
            self.send_thread = threading.Thread(target=self.send_thread_func, daemon=True)
            self.send_thread.start()
            
            # 启动接收线程
            self.serial_thread = threading.Thread(target=self.serial_receive_thread, daemon=True)
            self.serial_thread.start()
            
            # 启动连接检查线程
            self.connection_check_thread = threading.Thread(target=self.connection_check_thread_func, daemon=True)
            self.connection_check_thread.start()
            
            # 更新UI
            self.connect_btn.config(text="DISCONNECT")
            self.connect_btn.colors = {
                "bg": self.colors['error'],
                "fg": "white",
                "hover": "#C62828",
                "active": "#B71C1C"
            }
            self.connect_btn.config(bg=self.colors['error'])
            
            self.connection_indicator.config(text="● Connected", fg="#4CAF50")
            
            # 清除图表
            self.clear_chart()
            
            # 启动定时器
            self.start_pid_status_timer()
            
            messagebox.showinfo("Success", f"Connected to {self.SERIAL_PORT}")
            
        except serial.SerialException as e:
            error_msg = str(e)
            if "Permission denied" in error_msg:
                messagebox.showerror("Permission Denied", 
                    f"Cannot access the serial port {self.SERIAL_PORT}.\n\n"
                    f"Try:\n"
                    f"1. sudo chmod 666 {self.SERIAL_PORT}\n"
                    f"2. Or add your user to dialout group:\n"
                    f"   sudo usermod -a -G dialout $USER\n"
                    f"   Then logout and login again.")
            elif "No such file or directory" in error_msg:
                messagebox.showerror("Port Not Found", 
                    f"Serial port {self.SERIAL_PORT} does not exist!\n\n"
                    f"Common alternatives:\n"
                    f"• /dev/ttyUSB0, /dev/ttyUSB1\n"
                    f"• /dev/ttyACM0, /dev/ttyACM1\n"
                    f"• /dev/ttyS0, /dev/ttyS1\n\n"
                    f"Please check your device connection.")
            else:
                messagebox.showerror("Connection Failed", error_msg)
        except Exception as e:
            messagebox.showerror("Connection Failed", str(e))
            
    def disconnect(self):
        """断开串口连接"""
        self.running = False
        
        # 清空发送队列
        while not self.send_queue.empty():
            try:
                self.send_queue.get_nowait()
            except queue.Empty:
                break
        
        # 等待线程结束
        threads_to_join = [
            (self.send_thread, "Send thread"),
            (self.serial_thread, "Serial thread"),
            (self.connection_check_thread, "Connection check thread")
        ]
        
        for thread, name in threads_to_join:
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
                if thread.is_alive():
                    print(f"Warning: {name} did not terminate gracefully")
        
        with self.serial_lock:
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
                self.serial_port = None
            
        self.connect_btn.config(text="CONNECT")
        self.connect_btn.colors = {
            "bg": self.colors['primary'],
            "fg": "white",
            "hover": self.colors['primary_dark'],
            "active": "#0D47A1"
        }
        self.connect_btn.config(bg=self.colors['primary'])
        
        self.connection_indicator.config(text="● Disconnected", fg="#FF5252")
        
    def send_thread_func(self):
        """专用发送线程"""
        while self.running:
            try:
                # 等待发送队列中的数据，设置超时避免阻塞
                packet_data = self.send_queue.get(timeout=0.1)
                
                # 再次检查running状态
                if not self.running:
                    break
                
                # 检查发送间隔
                current_time = time.time()
                time_since_last_send = current_time - self.last_send_time
                if time_since_last_send < self.min_send_interval:
                    time.sleep(self.min_send_interval - time_since_last_send)
                
                # 发送数据
                with self.serial_lock:
                    if self.serial_port and self.serial_port.is_open and self.running:
                        try:
                            self.serial_port.write(packet_data)
                            self.serial_port.flush()
                            self.last_send_time = time.time()
                        except serial.SerialTimeoutException:
                            print("Serial write timeout")
                            if self.running:
                                self.root.after(0, self.handle_disconnection)
                            break
                        except Exception as e:
                            print(f"Serial write error: {e}")
                            if self.running:
                                self.root.after(0, self.handle_disconnection)
                            break
                
                # 标记任务完成
                self.send_queue.task_done()
                
            except queue.Empty:
                # 队列为空，继续等待
                continue
            except Exception as e:
                print(f"Send thread error: {e}")
                break
        
        print("Send thread exiting")
                
    def connection_check_thread_func(self):
        """检查串口连接状态的线程"""
        while self.running:
            try:
                with self.serial_lock:
                    if not self.running:
                        break
                    if self.serial_port and not self.serial_port.is_open:
                        # 串口已断开
                        if self.running:
                            self.root.after(0, self.handle_disconnection)
                        break
            except Exception as e:
                print(f"Connection check error: {e}")
                if self.running:
                    self.root.after(0, self.handle_disconnection)
                break
                
            time.sleep(0.5)
        
        print("Connection check thread exiting")
            
    def handle_disconnection(self):
        """处理串口断开"""
        if not self.running:
            return
            
        self.running = False
        
        # 更新UI状态
        self.connect_btn.config(text="CONNECT")
        self.connect_btn.colors = {
            "bg": self.colors['primary'],
            "fg": "white",
            "hover": self.colors['primary_dark'],
            "active": "#0D47A1"
        }
        self.connect_btn.config(bg=self.colors['primary'])
        
        self.connection_indicator.config(text="● Disconnected", fg="#FF5252")
        self.comm_status_label.config(text="Disconnected")
        
        messagebox.showwarning("Connection Lost", "Serial port connection has been lost!")
        
    def serial_receive_thread(self):
        """串口接收线程"""
        packet_state = 'WAIT_HEADER1'
        packet_data = []
        packet_cmd = 0
        packet_len = 0
        packet_checksum = 0
        packet_count = 0
        consecutive_errors = 0
        
        while self.running:
            try:
                with self.serial_lock:
                    if not self.serial_port or not self.serial_port.is_open or not self.running:
                        break
                        
                    if self.serial_port.in_waiting:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        consecutive_errors = 0  # 重置错误计数
                        
                        for byte in data:
                            if not self.running:  # 在处理数据时也检查running状态
                                break
                                
                            if packet_state == 'WAIT_HEADER1':
                                if byte == 0xAA:
                                    packet_state = 'WAIT_HEADER2'
                                    
                            elif packet_state == 'WAIT_HEADER2':
                                if byte == 0x55:
                                    packet_state = 'WAIT_CMD'
                                else:
                                    packet_state = 'WAIT_HEADER1'
                                    
                            elif packet_state == 'WAIT_CMD':
                                packet_cmd = byte
                                packet_state = 'WAIT_LEN'
                                
                            elif packet_state == 'WAIT_LEN':
                                packet_len = byte
                                packet_data = []
                                packet_checksum = packet_cmd + packet_len
                                if packet_len > 0:
                                    packet_state = 'WAIT_DATA'
                                else:
                                    packet_state = 'WAIT_CHECK'
                                    
                            elif packet_state == 'WAIT_DATA':
                                packet_data.append(byte)
                                packet_checksum += byte
                                if len(packet_data) >= packet_len:
                                    packet_state = 'WAIT_CHECK'
                                    
                            elif packet_state == 'WAIT_CHECK':
                                if byte == (packet_checksum & 0xFF):
                                    packet_count += 1
                                    self.process_packet(packet_cmd, packet_data)
                                    if self.running:
                                        self.root.after(0, self.update_packet_count, packet_count)
                                packet_state = 'WAIT_HEADER1'
                                
            except serial.SerialException as e:
                consecutive_errors += 1
                print(f"Serial exception: {e}")
                if consecutive_errors > 3:
                    # 多次错误后认为连接已断开
                    if self.running:
                        self.root.after(0, self.handle_disconnection)
                    break
            except Exception as e:
                print(f"Receive error: {e}")
                break
                
            time.sleep(0.001)
        
        print("Receive thread exiting")
            
    def process_packet(self, cmd, data):
        """处理接收到的数据包"""
        if cmd == 0x08:  # CMD_PID_GET_STATUS
            if len(data) >= 7:
                motor_id = data[0]
                
                def convert_signed_16bit(high_byte, low_byte):
                    value = (high_byte << 8) | low_byte
                    if value & 0x8000:
                        value = value - 0x10000
                    return value / 10.0
                
                current_speed = convert_signed_16bit(data[1], data[2])
                target_speed = convert_signed_16bit(data[3], data[4])
                pid_output = convert_signed_16bit(data[5], data[6])
                
                current_time = time.time()
                if self.plot_start_time is None:
                    self.plot_start_time = current_time
                    
                elapsed_time = current_time - self.plot_start_time
                
                if motor_id == 0:  # 前右电机
                    # 检测目标值变化
                    if abs(target_speed - self.target_speed_fr) > 1.0:
                        self.analysis_data['front_right']['target_changed'] = True
                        self.analysis_data['front_right']['analysis_start_time'] = current_time
                        self.analysis_data['front_right']['response_samples'] = []
                        
                    self.current_speed_fr = current_speed
                    self.target_speed_fr = target_speed
                    self.pid_output_fr = pid_output
                    
                    # 收集分析数据
                    if self.analysis_data['front_right']['target_changed']:
                        self.analysis_data['front_right']['response_samples'].append({
                            'time': current_time - self.analysis_data['front_right']['analysis_start_time'],
                            'current': current_speed,
                            'target': target_speed,
                            'output': pid_output
                        })
                        
                    self.root.after(0, self.update_fr_display)
                    
                elif motor_id == 1:  # 后左电机
                    # 检测目标值变化
                    if abs(target_speed - self.target_speed_rl) > 1.0:
                        self.analysis_data['rear_left']['target_changed'] = True
                        self.analysis_data['rear_left']['analysis_start_time'] = current_time
                        self.analysis_data['rear_left']['response_samples'] = []
                        
                    self.current_speed_rl = current_speed
                    self.target_speed_rl = target_speed
                    self.pid_output_rl = pid_output
                    
                    # 收集分析数据
                    if self.analysis_data['rear_left']['target_changed']:
                        self.analysis_data['rear_left']['response_samples'].append({
                            'time': current_time - self.analysis_data['rear_left']['analysis_start_time'],
                            'current': current_speed,
                            'target': target_speed,
                            'output': pid_output
                        })
                        
                    self.root.after(0, self.update_rl_display)
                    
                self.update_plot_data(elapsed_time)
                self.root.after(0, self.update_comm_status, "Active")
                
    def send_packet(self, cmd, data):
        """发送数据包到下位机（非阻塞）"""
        if not self.running or not self.serial_port or not self.serial_port.is_open:
            print("Warning: Serial port not connected")
            return False
            
        try:
            packet = bytearray()
            packet.append(0xAA)  # Header 1
            packet.append(0x55)  # Header 2
            packet.append(cmd)   # Command
            packet.append(len(data))  # Length
            
            checksum = cmd + len(data)
            for byte in data:
                packet.append(byte)
                checksum += byte
                
            packet.append(checksum & 0xFF)  # Checksum
            
            # 将数据包放入发送队列
            self.send_queue.put(packet, block=False)
            return True
            
        except queue.Full:
            print("Warning: Send queue is full, dropping packet")
            return False
        except Exception as e:
            print(f"Error creating packet: {e}")
            return False
            
    # ========== 控制命令相关方法 ==========
    
    def send_direction_cmd(self, direction):
        """发送方向控制命令"""
        speed = self.speed_var.get()
        data = [direction, speed]
        self.send_packet(0x01, data)
        
    def on_speed_change(self, value):
        """速度滑块变化时的回调"""
        speed = int(float(value))
        self.speed_label.config(text=f"{speed}%")
        
        # 只有在连接状态下才发送速度命令
        if self.running and self.serial_port and self.serial_port.is_open:
            self.send_packet(0x02, [speed])
            
    def toggle_pid(self):
        """切换PID控制状态"""
        enable = 1 if self.pid_enabled_var.get() else 0
        self.send_packet(0x06, [enable])
        
    def apply_pid_params(self):
        """应用PID参数"""
        try:
            # 获取前右电机参数
            self.fr_kp_var.set(float(self.fr_kp_entry.get()))
            self.fr_ki_var.set(float(self.fr_ki_entry.get()))
            self.fr_kd_var.set(float(self.fr_kd_entry.get()))
            
            # 获取后左电机参数
            self.rl_kp_var.set(float(self.rl_kp_entry.get()))
            self.rl_ki_var.set(float(self.rl_ki_entry.get()))
            self.rl_kd_var.set(float(self.rl_kd_entry.get()))
            
            # 发送前右电机PID参数
            kp_fr = int(self.fr_kp_var.get() * 100)
            ki_fr = int(self.fr_ki_var.get() * 100)
            kd_fr = int(self.fr_kd_var.get() * 100)
            
            data_fr = [0,  # Motor ID
                      (kp_fr >> 8) & 0xFF, kp_fr & 0xFF,
                      (ki_fr >> 8) & 0xFF, ki_fr & 0xFF,
                      (kd_fr >> 8) & 0xFF, kd_fr & 0xFF]
                      
            self.send_packet(0x07, data_fr)
            
            # 延时后发送后左电机参数
            self.root.after(100, self.send_rl_pid_params)
            
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values")
            
    def send_rl_pid_params(self):
        """发送后左电机PID参数"""
        kp_rl = int(self.rl_kp_var.get() * 100)
        ki_rl = int(self.rl_ki_var.get() * 100)
        kd_rl = int(self.rl_kd_var.get() * 100)
        
        data_rl = [1,  # Motor ID
                  (kp_rl >> 8) & 0xFF, kp_rl & 0xFF,
                  (ki_rl >> 8) & 0xFF, ki_rl & 0xFF,
                  (kd_rl >> 8) & 0xFF, kd_rl & 0xFF]
                  
        self.send_packet(0x07, data_rl)
        
        messagebox.showinfo("Success", "PID parameters updated")
        
    def reset_pid(self):
        """重置PID参数为默认值"""
        # 重置变量
        self.fr_kp_var.set(1.5)
        self.fr_ki_var.set(0.2)
        self.fr_kd_var.set(0.1)
        self.rl_kp_var.set(1.5)
        self.rl_ki_var.set(0.2)
        self.rl_kd_var.set(0.1)
        
        # 更新输入框
        self.fr_kp_entry.delete(0, tk.END)
        self.fr_kp_entry.insert(0, "1.5")
        self.fr_ki_entry.delete(0, tk.END)
        self.fr_ki_entry.insert(0, "0.2")
        self.fr_kd_entry.delete(0, tk.END)
        self.fr_kd_entry.insert(0, "0.1")
        
        self.rl_kp_entry.delete(0, tk.END)
        self.rl_kp_entry.insert(0, "1.5")
        self.rl_ki_entry.delete(0, tk.END)
        self.rl_ki_entry.insert(0, "0.2")
        self.rl_kd_entry.delete(0, tk.END)
        self.rl_kd_entry.insert(0, "0.1")
        
        # 应用到STM32
        self.apply_pid_params()
        
    def test_communication(self):
        """测试与STM32的通信"""
        if not self.running or not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Warning", "Please connect serial port first")
            return
            
        self.send_packet(0x06, [1])  # 启用PID
        self.root.after(100, lambda: self.send_packet(0x08, [0]))  # 请求前右电机状态
        self.root.after(200, lambda: self.send_packet(0x08, [1]))  # 请求后左电机状态
        
        messagebox.showinfo("Test", "Communication test started")
        
    def start_pid_status_timer(self):
        """启动定时器获取PID状态"""
        self.request_pid_status()
        
    def request_pid_status(self):
        """请求PID状态"""
        if not self.running:
            return
            
        if self.running and self.serial_port and self.serial_port.is_open:
            self.send_packet(0x08, [0])  # 请求前右电机状态
            if self.running:
                self.root.after(50, lambda: self.send_packet(0x08, [1]) if self.running else None)  # 请求后左电机状态
            if self.running:
                self.root.after(200, lambda: self.request_pid_status() if self.running else None)  # 更改间隔为200ms
                
    # ========== 显示更新相关方法 ==========
    
    def update_plot_data(self, elapsed_time):
        """更新图表数据"""
        if elapsed_time - self.last_update_time < 0.05:
            return
            
        self.last_update_time = elapsed_time
        
        self.plot_data['time'].append(elapsed_time)
        self.plot_data['fr_actual'].append(self.current_speed_fr)
        self.plot_data['fr_target'].append(self.target_speed_fr)
        self.plot_data['rl_actual'].append(self.current_speed_rl)
        self.plot_data['rl_target'].append(self.target_speed_rl)
        
    def animate_plot(self):
        """更新实时曲线图"""
        if not self.running:
            return
            
        if len(self.plot_data['time']) > 1:
            time_data = list(self.plot_data['time'])
            
            self.line_fr_actual.set_data(time_data, list(self.plot_data['fr_actual']))
            self.line_fr_target.set_data(time_data, list(self.plot_data['fr_target']))
            self.line_rl_actual.set_data(time_data, list(self.plot_data['rl_actual']))
            self.line_rl_target.set_data(time_data, list(self.plot_data['rl_target']))
            
            if time_data:
                current_time = time_data[-1]
                if current_time <= 20:
                    self.ax.set_xlim(0, 20)
                else:
                    self.ax.set_xlim(current_time - 20, current_time)
                    
            all_speeds = (list(self.plot_data['fr_actual']) + 
                         list(self.plot_data['fr_target']) + 
                         list(self.plot_data['rl_actual']) + 
                         list(self.plot_data['rl_target']))
                         
            if all_speeds:
                non_zero_speeds = [s for s in all_speeds if abs(s) > 0.1]
                if non_zero_speeds:
                    y_min = min(non_zero_speeds) - 10
                    y_max = max(non_zero_speeds) + 10
                    self.ax.set_ylim(max(y_min, -150), min(y_max, 150))
                else:
                    self.ax.set_ylim(-50, 50)
                    
            try:
                self.canvas.draw()
            except:
                return  # 如果绘图失败，直接返回
            
        if self.running:
            self.root.after(100, self.animate_plot)
        
    def clear_chart(self):
        """清除图表数据"""
        for key in self.plot_data:
            self.plot_data[key].clear()
            
        self.plot_start_time = None
        self.last_update_time = 0
        
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(-100, 100)
        
        self.line_fr_actual.set_data([], [])
        self.line_fr_target.set_data([], [])
        self.line_rl_actual.set_data([], [])
        self.line_rl_target.set_data([], [])
        
        self.canvas.draw()
        
    def update_fr_display(self):
        """更新前右电机显示"""
        self.fr_speed_label.config(text=f"{self.current_speed_fr:.1f}")
        self.fr_target_label.config(text=f"{self.target_speed_fr:.1f}")
        self.fr_output_label.config(text=f"{self.pid_output_fr:.1f}")
        
    def update_rl_display(self):
        """更新后左电机显示"""
        self.rl_speed_label.config(text=f"{self.current_speed_rl:.1f}")
        self.rl_target_label.config(text=f"{self.target_speed_rl:.1f}")
        self.rl_output_label.config(text=f"{self.pid_output_rl:.1f}")
        
    def update_comm_status(self, status):
        """更新通信状态"""
        self.comm_status_label.config(text=status)
        
    def update_packet_count(self, count):
        """更新接收包数"""
        self.packet_count_label.config(text=str(count))
        
    def on_closing(self):
        """关闭窗口时的处理"""
        print("Closing application...")
        
        # 立即停止所有操作
        self.running = False
        
        # 停止定时器
        try:
            self.root.after_cancel(self.request_pid_status)
        except:
            pass
            
        try:
            self.root.after_cancel(self.animate_plot)
        except:
            pass
        
        # 清空发送队列
        queue_cleared = 0
        while not self.send_queue.empty():
            try:
                self.send_queue.get_nowait()
                queue_cleared += 1
            except queue.Empty:
                break
        print(f"Cleared {queue_cleared} items from send queue")
        
        # 关闭串口
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    print("Closing serial port...")
                    self.serial_port.close()
                except Exception as e:
                    print(f"Error closing serial port: {e}")
                self.serial_port = None
        
        # 强制终止线程
        threads_to_join = [
            (self.send_thread, "Send thread"),
            (self.serial_thread, "Serial thread"),
            (self.connection_check_thread, "Connection check thread")
        ]
        
        for thread, name in threads_to_join:
            if thread and thread.is_alive():
                print(f"Waiting for {name} to exit...")
                thread.join(timeout=0.5)
                if thread.is_alive():
                    print(f"Warning: {name} did not terminate gracefully")
                else:
                    print(f"{name} exited successfully")
        
        # 销毁窗口
        try:
            print("Destroying main window...")
            self.root.quit()  # 退出主循环
            self.root.destroy()  # 销毁窗口
        except Exception as e:
            print(f"Error destroying window: {e}")
        
        # 强制退出程序
        print("Application closed successfully")
        import sys
        import os
        try:
            os._exit(0)  # 强制退出，不执行清理代码
        except:
            sys.exit(0)  # 备用退出方式
    
    # ========== PID智能分析相关方法 ==========
    
    def analyze_pid_performance(self):
        """分析PID性能并给出调节建议"""
        if not self.running or not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Warning", "Please connect serial port first")
            return
            
        # 清空建议文本
        self.suggestion_text.config(state=tk.NORMAL)
        self.suggestion_text.delete(1.0, tk.END)
        self.suggestion_text.insert(tk.END, "Analyzing PID performance...\n\n")
        
        # 发送测试命令
        self.suggestion_text.insert(tk.END, "Sending test commands...\n")
        self.suggestion_text.update()
        
        # 重置分析数据
        for motor in ['front_right', 'rear_left']:
            self.analysis_data[motor]['target_changed'] = False
            self.analysis_data[motor]['response_samples'] = []
            
        # 发送测试速度命令
        test_speed = 50
        self.send_packet(0x02, [test_speed])
        self.send_direction_cmd(0x00)  # 前进
        
        # 等待数据收集
        self.root.after(3000, self.perform_pid_analysis)
        
    def perform_pid_analysis(self):
        """执行PID分析"""
        suggestions_fr = self.analyze_motor_response('front_right', 'Front Right Motor')
        suggestions_rl = self.analyze_motor_response('rear_left', 'Rear Left Motor')
        
        # 停止电机
        self.send_direction_cmd(0x04)
        
        # 显示分析结果
        self.suggestion_text.delete(1.0, tk.END)
        self.suggestion_text.insert(tk.END, "=== PID Performance Analysis ===\n\n")
        
        # 显示前右电机分析
        self.suggestion_text.insert(tk.END, "FRONT RIGHT MOTOR (TIM2):\n", "motor_title")
        self.suggestion_text.tag_config("motor_title", foreground=self.colors['primary'], font=("Roboto", 10, "bold"))
        
        for suggestion in suggestions_fr:
            self.suggestion_text.insert(tk.END, f"• {suggestion}\n")
            
        self.suggestion_text.insert(tk.END, "\n")
        
        # 显示后左电机分析
        self.suggestion_text.insert(tk.END, "REAR LEFT MOTOR (TIM4):\n", "motor_title")
        
        for suggestion in suggestions_rl:
            self.suggestion_text.insert(tk.END, f"• {suggestion}\n")
            
        # 添加一般性建议
        self.suggestion_text.insert(tk.END, "\n=== General Tips ===\n", "tips_title")
        self.suggestion_text.tag_config("tips_title", foreground=self.colors['secondary'], font=("Roboto", 10, "bold"))
        self.suggestion_text.insert(tk.END, "• Start with small Kp and gradually increase\n")
        self.suggestion_text.insert(tk.END, "• Add Ki only after Kp is tuned\n")
        self.suggestion_text.insert(tk.END, "• Use Kd to reduce overshoot\n")
        self.suggestion_text.insert(tk.END, "• Test with different speeds and loads\n")
        
        self.suggestion_text.config(state=tk.DISABLED)
        
    def analyze_motor_response(self, motor_name, display_name):
        """分析单个电机的响应特性"""
        suggestions = []
        samples = self.analysis_data[motor_name]['response_samples']
        
        if len(samples) < 10:
            suggestions.append(f"Insufficient data for analysis (only {len(samples)} samples)")
            return suggestions
            
        # 提取数据
        times = [s['time'] for s in samples]
        currents = [s['current'] for s in samples]
        targets = [s['target'] for s in samples[-10:]]  # 取最后的目标值
        outputs = [s['output'] for s in samples]
        
        if not targets or all(t == 0 for t in targets):
            suggestions.append("No valid target speed detected")
            return suggestions
            
        target = np.mean(targets)
        
        # 计算性能指标
        # 1. 超调量
        if target != 0:
            overshoot = max([(c - target) / abs(target) * 100 for c in currents if abs(c) > abs(target)])
            overshoot = overshoot if overshoot > 0 else 0
        else:
            overshoot = 0
            
        # 2. 稳态误差
        if len(currents) > 20:
            steady_state_values = currents[-10:]  # 最后10个值
            steady_state_error = abs(np.mean(steady_state_values) - target)
            steady_state_percent = (steady_state_error / abs(target) * 100) if target != 0 else 0
        else:
            steady_state_error = abs(currents[-1] - target) if currents else 0
            steady_state_percent = (steady_state_error / abs(target) * 100) if target != 0 else 0
            
        # 3. 响应时间（达到目标值90%的时间）
        rise_time = None
        for i, (t, c) in enumerate(zip(times, currents)):
            if abs(c) >= abs(target) * 0.9:
                rise_time = t
                break
                
        # 4. 震荡检测
        oscillations = 0
        if len(currents) > 3:
            for i in range(2, len(currents) - 1):
                if (currents[i] - currents[i-1]) * (currents[i+1] - currents[i]) < 0:
                    oscillations += 1
                    
        oscillation_rate = oscillations / len(currents) if currents else 0
        
        # 5. PID输出饱和检测
        max_output = max(abs(o) for o in outputs) if outputs else 0
        output_saturation = max_output > 90  # 假设输出范围是-100到100
        
        # 生成建议
        current_params = self.pid_params[motor_name]
        
        # 基于超调量的建议
        if overshoot > 20:
            suggestions.append(f"High overshoot ({overshoot:.1f}%) detected:")
            suggestions.append(f"  → Reduce Kp from {current_params['kp']:.2f} to {current_params['kp']*0.8:.2f}")
            suggestions.append(f"  → Or increase Kd from {current_params['kd']:.2f} to {current_params['kd']*1.5:.2f}")
        elif overshoot > 10:
            suggestions.append(f"Moderate overshoot ({overshoot:.1f}%) detected:")
            suggestions.append(f"  → Slightly reduce Kp to {current_params['kp']*0.9:.2f}")
            
        # 基于稳态误差的建议
        if steady_state_percent > 10:
            suggestions.append(f"Large steady-state error ({steady_state_percent:.1f}%) detected:")
            suggestions.append(f"  → Increase Ki from {current_params['ki']:.2f} to {current_params['ki']*1.5:.2f}")
        elif steady_state_percent > 5:
            suggestions.append(f"Steady-state error ({steady_state_percent:.1f}%) detected:")
            suggestions.append(f"  → Slightly increase Ki to {current_params['ki']*1.2:.2f}")
            
        # 基于响应时间的建议
        if rise_time is not None:
            if rise_time > 2.0:
                suggestions.append(f"Slow response time ({rise_time:.2f}s):")
                suggestions.append(f"  → Increase Kp from {current_params['kp']:.2f} to {current_params['kp']*1.3:.2f}")
            elif rise_time < 0.3:
                suggestions.append(f"Very fast response ({rise_time:.2f}s) - may cause instability")
                
        # 基于震荡的建议
        if oscillation_rate > 0.3:
            suggestions.append(f"High oscillation detected ({oscillations} oscillations):")
            suggestions.append(f"  → Reduce Kp to {current_params['kp']*0.7:.2f}")
            suggestions.append(f"  → Increase Kd to {current_params['kd']*2:.2f}")
        elif oscillation_rate > 0.15:
            suggestions.append("Moderate oscillation detected:")
            suggestions.append(f"  → Increase Kd to {current_params['kd']*1.3:.2f}")
            
        # 基于输出饱和的建议
        if output_saturation:
            suggestions.append("PID output saturation detected:")
            suggestions.append("  → Consider reducing all gains proportionally")
            suggestions.append("  → Or increase motor power/reduce load")
            
        # 如果没有明显问题
        if not suggestions:
            suggestions.append("PID performance is good!")
            suggestions.append("Fine-tune parameters based on specific requirements")
            
        return suggestions
        
    def send_test_sequence(self):
        """发送测试序列用于PID分析"""
        # 这个方法可以扩展为发送不同的测试模式
        # 例如：阶跃响应、斜坡响应、正弦响应等
        pass


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = RobotControlGUI(root)
        
        # 绑定关闭事件
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        
        # 设置窗口图标等
        try:
            root.iconname("Robot Control")
        except:
            pass
        
        print("Starting Robot Control GUI...")
        root.mainloop()
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        print("Program exiting...")
        import sys
        import os
        try:
            os._exit(0)
        except:
            sys.exit(0)