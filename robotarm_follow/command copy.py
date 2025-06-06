#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车命令生成器
用于生成和发送小车控制命令的Python程序
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import binascii

# 命令类型常量
CMD_SET_DIRECTION = 0x01
CMD_SET_SPEED = 0x02
CMD_SET_MOTOR = 0x03
CMD_REQUEST_STATUS = 0x04

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04

# 电机ID常量
MOTOR_FRONT_LEFT = 0
MOTOR_FRONT_RIGHT = 1
MOTOR_REAR_LEFT = 2
MOTOR_REAR_RIGHT = 3


class CarCommandGenerator:
    """小车命令生成器类"""

    def __init__(self):
        """初始化命令生成器"""
        self.serial_port = None
        self.is_connected = False
        self.receive_thread = None
        self.is_receiving = False

    def generate_packet(self, cmd, data=None):
        """
        生成数据包
        :param cmd: 命令类型
        :param data: 数据列表
        :return: 完整的命令字节序列
        """
        if data is None:
            data = []

        # 计算校验和
        checksum = cmd + len(data)
        for byte in data:
            checksum += byte
        checksum &= 0xFF  # 取低8位

        # 构建数据包
        packet = [0xAA, 0x55, cmd, len(data)] + data + [checksum]
        return bytes(packet)

    def generate_direction_command(self, direction, speed):
        """
        生成设置方向和速度的命令
        :param direction: 方向
        :param speed: 速度 (0-100)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return self.generate_packet(CMD_SET_DIRECTION, [direction, speed])

    def generate_speed_command(self, speed):
        """
        生成设置速度的命令
        :param speed: 速度 (0-100)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return self.generate_packet(CMD_SET_SPEED, [speed])

    def generate_motor_command(self, motor_id, speed, direction):
        """
        生成控制单个电机的命令
        :param motor_id: 电机ID (0-3)
        :param speed: 速度 (0-100)
        :param direction: 方向 (0:正向, 1:反向)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return self.generate_packet(CMD_SET_MOTOR, [motor_id, speed, direction])

    def generate_status_request(self):
        """
        生成请求状态的命令
        :return: 命令字节序列
        """
        return self.generate_packet(CMD_REQUEST_STATUS)

    def connect(self, port, baudrate=115200):
        """
        连接到串口
        :param port: 串口名
        :param baudrate: 波特率
        :return: 是否成功连接
        """
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.is_receiving = False
            if self.receive_thread and self.receive_thread.is_alive():
                self.receive_thread.join(1)
            self.serial_port.close()
            self.is_connected = False

    def send_command(self, command):
        """
        发送命令到小车
        :param command: 命令字节序列
        :return: 是否成功发送
        """
        if not self.is_connected or not self.serial_port:
            return False

        try:
            self.serial_port.write(command)
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def start_receiving(self, callback=None):
        """
        开始接收串口数据
        :param callback: 接收到数据时的回调函数
        """
        if not self.is_connected or not self.serial_port:
            return

        self.is_receiving = True
        self.receive_thread = threading.Thread(
            target=self._receive_loop,
            args=(callback,),
            daemon=True
        )
        self.receive_thread.start()

    def _receive_loop(self, callback):
        """
        接收数据的循环
        :param callback: 接收到数据时的回调函数
        """
        while self.is_receiving and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    if callback:
                        callback(data)
            except Exception as e:
                print(f"接收错误: {e}")
                break
            time.sleep(0.01)  # 避免占用过多CPU


class CarControlGUI:
    """小车控制GUI类"""

    def __init__(self, root):
        """
        初始化GUI
        :param root: tkinter根窗口
        """
        self.root = root
        self.root.title("小车命令生成器")
        self.root.geometry("900x700")
        self.root.resizable(True, True)

        self.generator = CarCommandGenerator()
        self.create_widgets()
        self.update_ports()

    def create_widgets(self):
        """创建GUI控件"""
        # 创建主框架
        mainframe = ttk.Frame(self.root, padding="10")
        mainframe.pack(fill=tk.BOTH, expand=True)

        # 串口设置框架
        port_frame = ttk.LabelFrame(mainframe, text="串口设置", padding="5")
        port_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(port_frame, text="串口:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_combobox = ttk.Combobox(port_frame)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        ttk.Label(port_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baudrate_combobox = ttk.Combobox(port_frame, values=["9600", "19200", "38400", "57600", "115200"])
        self.baudrate_combobox.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        self.baudrate_combobox.set("115200")

        self.refresh_button = ttk.Button(port_frame, text="刷新", command=self.update_ports)
        self.refresh_button.grid(row=0, column=4, padx=5, pady=5)

        self.connect_button = ttk.Button(port_frame, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=5, pady=5)

        # 命令生成框架
        command_frame = ttk.LabelFrame(mainframe, text="命令生成", padding="5")
        command_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 创建选项卡
        tabs = ttk.Notebook(command_frame)
        tabs.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 方向控制选项卡
        direction_tab = ttk.Frame(tabs, padding="10")
        tabs.add(direction_tab, text="方向控制")

        ttk.Label(direction_tab, text="方向:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.direction_combobox = ttk.Combobox(direction_tab,
                                               values=["前进", "后退", "左转", "右转", "停止"],
                                               state="readonly")
        self.direction_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.direction_combobox.current(0)

        ttk.Label(direction_tab, text="速度:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.direction_speed_var = tk.IntVar(value=50)
        direction_speed_scale = ttk.Scale(direction_tab, from_=0, to=100,
                                          variable=self.direction_speed_var,
                                          orient=tk.HORIZONTAL, length=200)
        direction_speed_scale.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        direction_speed_label = ttk.Label(direction_tab, textvariable=self.direction_speed_var)
        direction_speed_label.grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)

        direction_send_button = ttk.Button(direction_tab, text="发送",
                                           command=self.send_direction_command)
        direction_send_button.grid(row=2, column=1, padx=5, pady=10)

        # 速度控制选项卡
        speed_tab = ttk.Frame(tabs, padding="10")
        tabs.add(speed_tab, text="速度控制")

        ttk.Label(speed_tab, text="速度:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.speed_var = tk.IntVar(value=50)
        speed_scale = ttk.Scale(speed_tab, from_=0, to=100,
                                variable=self.speed_var,
                                orient=tk.HORIZONTAL, length=200)
        speed_scale.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        speed_label = ttk.Label(speed_tab, textvariable=self.speed_var)
        speed_label.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)

        speed_send_button = ttk.Button(speed_tab, text="发送",
                                       command=self.send_speed_command)
        speed_send_button.grid(row=1, column=1, padx=5, pady=10)

        # 电机控制选项卡
        motor_tab = ttk.Frame(tabs, padding="10")
        tabs.add(motor_tab, text="电机控制")

        ttk.Label(motor_tab, text="电机ID:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.motor_id_combobox = ttk.Combobox(motor_tab,
                                              values=["前左电机", "前右电机", "后左电机", "后右电机"],
                                              state="readonly")
        self.motor_id_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.motor_id_combobox.current(0)

        ttk.Label(motor_tab, text="速度:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.motor_speed_var = tk.IntVar(value=50)
        motor_speed_scale = ttk.Scale(motor_tab, from_=0, to=100,
                                      variable=self.motor_speed_var,
                                      orient=tk.HORIZONTAL, length=200)
        motor_speed_scale.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        motor_speed_label = ttk.Label(motor_tab, textvariable=self.motor_speed_var)
        motor_speed_label.grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)

        ttk.Label(motor_tab, text="方向:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        self.motor_direction_var = tk.StringVar(value="正向")
        motor_direction_radio1 = ttk.Radiobutton(motor_tab, text="正向",
                                                 variable=self.motor_direction_var, value="正向")
        motor_direction_radio1.grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)

        motor_direction_radio2 = ttk.Radiobutton(motor_tab, text="反向",
                                                 variable=self.motor_direction_var, value="反向")
        motor_direction_radio2.grid(row=2, column=2, padx=5, pady=5, sticky=tk.W)

        motor_send_button = ttk.Button(motor_tab, text="发送",
                                       command=self.send_motor_command)
        motor_send_button.grid(row=3, column=1, padx=5, pady=10)

        # 状态请求选项卡
        status_tab = ttk.Frame(tabs, padding="10")
        tabs.add(status_tab, text="状态请求")

        status_send_button = ttk.Button(status_tab, text="请求状态",
                                        command=self.send_status_request)
        status_send_button.pack(padx=5, pady=10)

        # 新增：命令生成器选项卡（只生成不发送）
        generator_tab = ttk.Frame(tabs, padding="10")
        tabs.add(generator_tab, text="命令生成器")

        # 创建子选项卡用于不同类型的命令生成
        gen_tabs = ttk.Notebook(generator_tab)
        gen_tabs.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 方向命令生成
        gen_direction_tab = ttk.Frame(gen_tabs, padding="10")
        gen_tabs.add(gen_direction_tab, text="方向命令")

        ttk.Label(gen_direction_tab, text="方向:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_direction_combobox = ttk.Combobox(gen_direction_tab,
                                                   values=["前进", "后退", "左转", "右转", "停止"],
                                                   state="readonly")
        self.gen_direction_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.gen_direction_combobox.current(0)

        ttk.Label(gen_direction_tab, text="速度:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_direction_speed_var = tk.IntVar(value=50)
        gen_direction_speed_scale = ttk.Scale(gen_direction_tab, from_=0, to=100,
                                              variable=self.gen_direction_speed_var,
                                              orient=tk.HORIZONTAL, length=200)
        gen_direction_speed_scale.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        gen_direction_speed_label = ttk.Label(gen_direction_tab, textvariable=self.gen_direction_speed_var)
        gen_direction_speed_label.grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)

        gen_direction_button = ttk.Button(gen_direction_tab, text="生成命令",
                                          command=self.generate_direction_command_only)
        gen_direction_button.grid(row=2, column=1, padx=5, pady=10)

        # 速度命令生成
        gen_speed_tab = ttk.Frame(gen_tabs, padding="10")
        gen_tabs.add(gen_speed_tab, text="速度命令")

        ttk.Label(gen_speed_tab, text="速度:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_speed_var = tk.IntVar(value=50)
        gen_speed_scale = ttk.Scale(gen_speed_tab, from_=0, to=100,
                                    variable=self.gen_speed_var,
                                    orient=tk.HORIZONTAL, length=200)
        gen_speed_scale.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        gen_speed_label = ttk.Label(gen_speed_tab, textvariable=self.gen_speed_var)
        gen_speed_label.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)

        gen_speed_button = ttk.Button(gen_speed_tab, text="生成命令",
                                      command=self.generate_speed_command_only)
        gen_speed_button.grid(row=1, column=1, padx=5, pady=10)

        # 电机命令生成
        gen_motor_tab = ttk.Frame(gen_tabs, padding="10")
        gen_tabs.add(gen_motor_tab, text="电机命令")

        ttk.Label(gen_motor_tab, text="电机ID:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_motor_id_combobox = ttk.Combobox(gen_motor_tab,
                                                  values=["前左电机", "前右电机", "后左电机", "后右电机"],
                                                  state="readonly")
        self.gen_motor_id_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.gen_motor_id_combobox.current(0)

        ttk.Label(gen_motor_tab, text="速度:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_motor_speed_var = tk.IntVar(value=50)
        gen_motor_speed_scale = ttk.Scale(gen_motor_tab, from_=0, to=100,
                                          variable=self.gen_motor_speed_var,
                                          orient=tk.HORIZONTAL, length=200)
        gen_motor_speed_scale.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        gen_motor_speed_label = ttk.Label(gen_motor_tab, textvariable=self.gen_motor_speed_var)
        gen_motor_speed_label.grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)

        ttk.Label(gen_motor_tab, text="方向:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        self.gen_motor_direction_var = tk.StringVar(value="正向")
        gen_motor_direction_radio1 = ttk.Radiobutton(gen_motor_tab, text="正向",
                                                     variable=self.gen_motor_direction_var, value="正向")
        gen_motor_direction_radio1.grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)

        gen_motor_direction_radio2 = ttk.Radiobutton(gen_motor_tab, text="反向",
                                                     variable=self.gen_motor_direction_var, value="反向")
        gen_motor_direction_radio2.grid(row=2, column=2, padx=5, pady=5, sticky=tk.W)

        gen_motor_button = ttk.Button(gen_motor_tab, text="生成命令",
                                      command=self.generate_motor_command_only)
        gen_motor_button.grid(row=3, column=1, padx=5, pady=10)

        # 状态请求命令生成
        gen_status_tab = ttk.Frame(gen_tabs, padding="10")
        gen_tabs.add(gen_status_tab, text="状态命令")

        gen_status_button = ttk.Button(gen_status_tab, text="生成命令",
                                       command=self.generate_status_command_only)
        gen_status_button.pack(padx=5, pady=10)

        # 生成的命令显示区域
        command_display_frame = ttk.LabelFrame(generator_tab, text="生成的命令", padding="5")
        command_display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.command_display_text = scrolledtext.ScrolledText(command_display_frame, height=6, font=("Consolas", 10))
        self.command_display_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 命令显示区域的按钮
        command_buttons_frame = ttk.Frame(command_display_frame)
        command_buttons_frame.pack(fill=tk.X, padx=5, pady=5)

        copy_command_button = ttk.Button(command_buttons_frame, text="复制命令",
                                         command=self.copy_generated_command)
        copy_command_button.pack(side=tk.LEFT, padx=5)

        clear_command_button = ttk.Button(command_buttons_frame, text="清除",
                                          command=self.clear_generated_commands)
        clear_command_button.pack(side=tk.LEFT, padx=5)

        send_generated_button = ttk.Button(command_buttons_frame, text="发送最新命令",
                                           command=self.send_latest_generated_command)
        send_generated_button.pack(side=tk.LEFT, padx=5)

        # 快捷控制按钮（方向键控制）
        control_frame = ttk.LabelFrame(mainframe, text="快捷控制", padding="5")
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        button_frame = ttk.Frame(control_frame)
        button_frame.pack(padx=20, pady=10)

        # 第一行 - 前进按钮
        forward_button = ttk.Button(button_frame, text="前进", width=10,
                                    command=lambda: self.quick_control(DIR_FORWARD))
        forward_button.grid(row=0, column=1, padx=5, pady=5)

        # 第二行 - 左转、停止、右转按钮
        left_button = ttk.Button(button_frame, text="左转", width=10,
                                 command=lambda: self.quick_control(DIR_LEFT))
        left_button.grid(row=1, column=0, padx=5, pady=5)

        stop_button = ttk.Button(button_frame, text="停止", width=10,
                                 command=lambda: self.quick_control(DIR_STOP))
        stop_button.grid(row=1, column=1, padx=5, pady=5)

        right_button = ttk.Button(button_frame, text="右转", width=10,
                                  command=lambda: self.quick_control(DIR_RIGHT))
        right_button.grid(row=1, column=2, padx=5, pady=5)

        # 第三行 - 后退按钮
        backward_button = ttk.Button(button_frame, text="后退", width=10,
                                     command=lambda: self.quick_control(DIR_BACKWARD))
        backward_button.grid(row=2, column=1, padx=5, pady=5)

        # 消息日志框架
        log_frame = ttk.LabelFrame(mainframe, text="消息日志", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 清除日志按钮
        clear_button = ttk.Button(log_frame, text="清除日志", command=self.clear_log)
        clear_button.pack(pady=5)

        # 用于存储最新生成的命令
        self.latest_generated_command = None

    def update_ports(self):
        """更新可用的串口列表"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)
            self.log("串口列表已更新")
        else:
            self.log("没有找到可用串口")

    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.generator.is_connected:
            port = self.port_combobox.get()
            try:
                baudrate = int(self.baudrate_combobox.get())
            except ValueError:
                baudrate = 115200

            if self.generator.connect(port, baudrate):
                self.connect_button.config(text="断开")
                self.log(f"已连接到 {port}, 波特率: {baudrate}")
                self.generator.start_receiving(self.handle_receive)
            else:
                self.log(f"连接到 {port} 失败")
        else:
            self.generator.disconnect()
            self.connect_button.config(text="连接")
            self.log("已断开连接")

    def send_direction_command(self):
        """发送方向控制命令"""
        direction_text = self.direction_combobox.get()
        direction_map = {"前进": DIR_FORWARD, "后退": DIR_BACKWARD,
                         "左转": DIR_LEFT, "右转": DIR_RIGHT, "停止": DIR_STOP}
        direction = direction_map.get(direction_text, DIR_STOP)
        speed = self.direction_speed_var.get()

        command = self.generator.generate_direction_command(direction, speed)
        self.send_command(command, f"发送方向命令: {direction_text}, 速度: {speed}%")

    def send_speed_command(self):
        """发送速度控制命令"""
        speed = self.speed_var.get()
        command = self.generator.generate_speed_command(speed)
        self.send_command(command, f"发送速度命令: {speed}%")

    def send_motor_command(self):
        """发送电机控制命令"""
        motor_text = self.motor_id_combobox.get()
        motor_map = {"前左电机": MOTOR_FRONT_LEFT, "前右电机": MOTOR_FRONT_RIGHT,
                     "后左电机": MOTOR_REAR_LEFT, "后右电机": MOTOR_REAR_RIGHT}
        motor_id = motor_map.get(motor_text, 0)

        speed = self.motor_speed_var.get()
        direction = 0 if self.motor_direction_var.get() == "正向" else 1

        command = self.generator.generate_motor_command(motor_id, speed, direction)
        dir_text = "正向" if direction == 0 else "反向"
        self.send_command(command, f"发送电机命令: {motor_text}, 速度: {speed}%, 方向: {dir_text}")

    def send_status_request(self):
        """发送状态请求命令"""
        command = self.generator.generate_status_request()
        self.send_command(command, "发送状态请求")

    def quick_control(self, direction):
        """快捷控制"""
        speed = self.direction_speed_var.get()
        command = self.generator.generate_direction_command(direction, speed)

        direction_text = {
            DIR_FORWARD: "前进",
            DIR_BACKWARD: "后退",
            DIR_LEFT: "左转",
            DIR_RIGHT: "右转",
            DIR_STOP: "停止"
        }.get(direction, "未知")

        self.send_command(command, f"快捷控制: {direction_text}, 速度: {speed}%")

    def send_command(self, command, log_text):
        """发送命令并记录日志"""
        if not self.generator.is_connected:
            self.log("未连接到串口，请先连接")
            return

        hex_command = binascii.hexlify(command).decode('ascii').upper()
        hex_command = ' '.join(hex_command[i:i + 2] for i in range(0, len(hex_command), 2))

        if self.generator.send_command(command):
            self.log(f"{log_text}\n命令: {hex_command}")
        else:
            self.log(f"发送失败: {hex_command}")

    # 新增：只生成命令的方法
    def generate_direction_command_only(self):
        """只生成方向控制命令，不发送"""
        direction_text = self.gen_direction_combobox.get()
        direction_map = {"前进": DIR_FORWARD, "后退": DIR_BACKWARD,
                         "左转": DIR_LEFT, "右转": DIR_RIGHT, "停止": DIR_STOP}
        direction = direction_map.get(direction_text, DIR_STOP)
        speed = self.gen_direction_speed_var.get()

        command = self.generator.generate_direction_command(direction, speed)
        self.display_generated_command(command, f"方向命令: {direction_text}, 速度: {speed}%")

    def generate_speed_command_only(self):
        """只生成速度控制命令，不发送"""
        speed = self.gen_speed_var.get()
        command = self.generator.generate_speed_command(speed)
        self.display_generated_command(command, f"速度命令: {speed}%")

    def generate_motor_command_only(self):
        """只生成电机控制命令，不发送"""
        motor_text = self.gen_motor_id_combobox.get()
        motor_map = {"前左电机": MOTOR_FRONT_LEFT, "前右电机": MOTOR_FRONT_RIGHT,
                     "后左电机": MOTOR_REAR_LEFT, "后右电机": MOTOR_REAR_RIGHT}
        motor_id = motor_map.get(motor_text, 0)

        speed = self.gen_motor_speed_var.get()
        direction = 0 if self.gen_motor_direction_var.get() == "正向" else 1

        command = self.generator.generate_motor_command(motor_id, speed, direction)
        dir_text = "正向" if direction == 0 else "反向"
        self.display_generated_command(command, f"电机命令: {motor_text}, 速度: {speed}%, 方向: {dir_text}")

    def generate_status_command_only(self):
        """只生成状态请求命令，不发送"""
        command = self.generator.generate_status_request()
        self.display_generated_command(command, "状态请求命令")

    def display_generated_command(self, command, description):
        """显示生成的命令"""
        hex_command = binascii.hexlify(command).decode('ascii').upper()
        hex_command = ' '.join(hex_command[i:i + 2] for i in range(0, len(hex_command), 2))

        timestamp = time.strftime("%H:%M:%S", time.localtime())
        command_text = f"[{timestamp}] {description}\n十六进制: {hex_command}\n字节数组: {list(command)}\n{'-' * 50}\n"

        self.command_display_text.insert(tk.END, command_text)
        self.command_display_text.see(tk.END)

        # 保存最新生成的命令
        self.latest_generated_command = command

    def copy_generated_command(self):
        """复制生成的命令到剪贴板"""
        try:
            content = self.command_display_text.get(1.0, tk.END)
            if content.strip():
                self.root.clipboard_clear()
                self.root.clipboard_append(content)
                self.log("已复制生成的命令到剪贴板")
            else:
                self.log("没有生成的命令可以复制")
        except Exception as e:
            self.log(f"复制失败: {e}")

    def clear_generated_commands(self):
        """清除生成的命令显示"""
        self.command_display_text.delete(1.0, tk.END)
        self.latest_generated_command = None

    def send_latest_generated_command(self):
        """发送最新生成的命令"""
        if self.latest_generated_command is None:
            self.log("没有生成的命令可以发送")
            return

        if not self.generator.is_connected:
            self.log("未连接到串口，请先连接")
            return

        hex_command = binascii.hexlify(self.latest_generated_command).decode('ascii').upper()
        hex_command = ' '.join(hex_command[i:i + 2] for i in range(0, len(hex_command), 2))

        if self.generator.send_command(self.latest_generated_command):
            self.log(f"发送生成的命令成功\n命令: {hex_command}")
        else:
            self.log(f"发送生成的命令失败: {hex_command}")

    def handle_receive(self, data):
        """处理接收到的数据"""
        hex_data = binascii.hexlify(data).decode('ascii').upper()
        hex_data = ' '.join(hex_data[i:i + 2] for i in range(0, len(hex_data), 2))
        self.log(f"接收: {hex_data}")

        # 尝试解析响应
        try:
            self.parse_response(data)
        except Exception as e:
            self.log(f"解析响应错误: {e}")

    def parse_response(self, data):
        """解析响应数据"""
        # 查找帧头
        for i in range(len(data) - 1):
            if data[i] == 0xAA and data[i + 1] == 0x55:
                # 找到帧头，开始解析
                if i + 3 < len(data):
                    cmd = data[i + 2]
                    length = data[i + 3]

                    # 检查数据包是否完整
                    if i + 4 + length + 1 <= len(data):
                        response_data = data[i + 4:i + 4 + length]

                        # 根据命令类型解析
                        if cmd == CMD_SET_DIRECTION and length >= 2:
                            self.log_response_direction(response_data)
                        elif cmd == CMD_SET_SPEED and length >= 1:
                            self.log(f"速度设置响应: {response_data[0]}%")
                        elif cmd == CMD_SET_MOTOR and length >= 3:
                            self.log_response_motor(response_data)
                        elif cmd == CMD_REQUEST_STATUS and length >= 6:
                            self.log_response_status(response_data)

    def log_response_direction(self, data):
        """记录方向响应日志"""
        direction_text = {
            DIR_FORWARD: "前进",
            DIR_BACKWARD: "后退",
            DIR_LEFT: "左转",
            DIR_RIGHT: "右转",
            DIR_STOP: "停止"
        }.get(data[0], "未知")

        speed = data[1]
        self.log(f"方向设置响应: {direction_text}, 速度: {speed}%")

    def log_response_motor(self, data):
        """记录电机响应日志"""
        motor_text = {
            MOTOR_FRONT_LEFT: "前左电机",
            MOTOR_FRONT_RIGHT: "前右电机",
            MOTOR_REAR_LEFT: "后左电机",
            MOTOR_REAR_RIGHT: "后右电机"
        }.get(data[0], f"电机{data[0]}")

        speed = data[1]
        direction = "正向" if data[2] == 0 else "反向"

        self.log(f"电机设置响应: {motor_text}, 速度: {speed}%, 方向: {direction}")

    def log_response_status(self, data):
        """记录状态响应日志"""
        direction_text = {
            DIR_FORWARD: "前进",
            DIR_BACKWARD: "后退",
            DIR_LEFT: "左转",
            DIR_RIGHT: "右转",
            DIR_STOP: "停止"
        }.get(data[0], "未知")

        global_speed = data[1]

        motor_speeds = []
        for i in range(4):
            if i + 2 < len(data):
                motor_speeds.append(data[i + 2])

        motors_text = ", ".join([f"电机{i}: {speed}%" for i, speed in enumerate(motor_speeds)])

        self.log(f"状态响应: 方向={direction_text}, 速度={global_speed}%\n{motors_text}")

    def log(self, message):
        """添加日志消息"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def clear_log(self):
        """清除日志"""
        self.log_text.delete(1.0, tk.END)


if __name__ == "__main__":
    root = tk.Tk()
    app = CarControlGUI(root)
    root.mainloop()