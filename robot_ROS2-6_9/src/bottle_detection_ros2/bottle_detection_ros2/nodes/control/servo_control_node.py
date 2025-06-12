#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制节点
负责控制机械臂舵机，实现目标跟踪和采摘动作
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import RealtimeCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Int32, Bool, String
from geometry_msgs.msg import Point
from bottle_detection_msgs.msg import HarvestCommand, ServoCommand, ServoStatus
from serial import Serial
import threading
import time
import json
from bottle_detection_ros2.core.qos_profiles import HIGH_FREQUENCY_QOS, REALTIME_CONTROL_QOS, STATUS_UPDATE_QOS

# 舵机控制常量
DEFAULT_SERVO_ID = 0
CENTER_POSITION = 1500
MIN_POSITION = 500
MAX_POSITION = 2500
SERVO_MODE = 3  # 180度顺时针模式

# 机械臂采摘动作指令
ARM_COMMANDS = {
    "rt_catch1": "#002P1650T2000!#003P1300T2000!#005P1700T2000!",
    "rt_catch2": "#002P1650T2000!#003P1300T2000!#005P1950T2000!",
    "rt_catch3": "#000P2200T2000!#001P1600T2000!#002P1850T2000!#003P2300T2000!#005P1950T2000!",
    "rt_catch4": "#000P2200T2000!#001P1600T2000!#002P1850T2000!#003P2300T2000!#005P1700T0500!",
    "rt_catch5": "#000P1380T1500!#001P0650T1500!#002P2150T1500!#003P0750T1500!#004P1970T1500!#005P1700T1500!"
}

# 采摘状态
HARVEST_IDLE = 0
HARVEST_STARTED = 1
HARVEST_STEP1 = 2
HARVEST_STEP2 = 3
HARVEST_STEP3 = 4
HARVEST_STEP4 = 5
HARVEST_COMPLETE = 6


class PIDController:
    """PID控制器类"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
            
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.last_error) / dt
        derivative_term = self.kd * derivative
        
        # PID输出
        output = proportional + integral_term + derivative_term
        
        # 输出限制
        if self.output_limit:
            output = max(self.output_limit[0], min(self.output_limit[1], output))
            
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class ServoControlNode(Node):
    """舵机控制节点类"""
    
    def __init__(self):
        super().__init__('servo_control_node')
        
        # 创建实时回调组
        self.realtime_callback_group = RealtimeCallbackGroup()
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyS9')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('tracking_deadzone', 30)  # 像素
        self.declare_parameter('tracking_speed', 7.5)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracking_frequency', 50.0)  # 跟踪频率50Hz
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.tracking_deadzone = self.get_parameter('tracking_deadzone').value
        self.tracking_speed = self.get_parameter('tracking_speed').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.tracking_frequency = self.get_parameter('tracking_frequency').value
        
        # 初始化串口
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # 舵机PWM范围设置
        self.horizontal_servo_range = (500, 2500)  # 水平方向180度
        self.vertical_servo_range = (500, 1500)    # 垂直方向90度
        
        # 舵机中心位置
        self.horizontal_servo_center = (self.horizontal_servo_range[0] + self.horizontal_servo_range[1]) // 2  # 1500
        self.vertical_servo_center = 600     # 600
        
        # 像素到PWM的转换比例 (像素变化1.1，PWM变化1)
        self.pixel_to_pwm_ratio = 1.0 / 1.1  # 约0.909
        
        # 初始化PID控制器 - 调整输出限制范围
        max_horizontal_change = (self.horizontal_servo_range[1] - self.horizontal_servo_range[0]) // 4  # 最大变化量
        max_vertical_change = (self.vertical_servo_range[1] - self.vertical_servo_range[0]) // 4
        
        self.horizontal_pid = PIDController(
            kp=0.8, ki=0.02, kd=0.3, output_limit=(-max_horizontal_change, max_horizontal_change)
        )
        self.vertical_pid = PIDController(
            kp=0.8, ki=0.02, kd=0.3, output_limit=(-max_vertical_change, max_vertical_change)
        )
        
        # 当前舵机位置
        self.current_horizontal_pos = self.horizontal_servo_center
        self.current_vertical_pos = self.vertical_servo_center
        
        # 死区和平滑参数
        self.dead_zone_x = 30
        self.dead_zone_y = 30
        self.smooth_factor = 0.85
        
        # 运动阈值 - 调整为PWM单位
        self.horizontal_movement_threshold = 5  # PWM单位
        self.vertical_movement_threshold = 5    # PWM单位
        
        # 系统状态
        self.tracking_enabled = False
        self.show_debug = True
        
        # 创建订阅者（使用高性能QoS和实时回调组）
        self.servo_cmd_sub = self.create_subscription(
            ServoCommand,
            'servo/command',
            self.servo_command_callback,
            REALTIME_CONTROL_QOS,
            callback_group=self.realtime_callback_group
        )
        
        self.harvest_cmd_sub = self.create_subscription(
            HarvestCommand,
            'robot/harvest_command',
            self.harvest_command_callback,
            REALTIME_CONTROL_QOS,
            callback_group=self.realtime_callback_group
        )
        
        self.tracking_target_sub = self.create_subscription(
            Point,
            'servo/tracking_target',
            self.tracking_target_callback,
            HIGH_FREQUENCY_QOS,  # 高频率低延迟
            callback_group=self.realtime_callback_group
        )
        
        # 创建发布者（使用优化的QoS）
        self.servo_status_pub = self.create_publisher(
            ServoStatus,
            'servo/status',
            STATUS_UPDATE_QOS
        )
        
        self.harvest_status_pub = self.create_publisher(
            String,
            'harvest/status',
            STATUS_UPDATE_QOS
        )
        
        # 状态变量
        self.current_positions = [CENTER_POSITION] * 6  # 6个舵机
        self.harvest_state = HARVEST_IDLE
        self.harvest_step_time = 0
        self.harvested_count = 0
        
        # 跟踪相关变量
        self.tracking_active = False
        
        # 创建定时器处理采摘状态机（高频率处理）
        self.create_timer(0.02, self.harvest_state_machine, callback_group=self.realtime_callback_group)  # 50Hz
        
        # 创建状态发布定时器（降低频率减少负载）
        self.create_timer(0.1, self.publish_status)  # 10Hz状态发布
        
        # 初始化舵机（在连接串口后）
        if self.serial:
            self._initialize_servos()
            # 设置机械臂到初始位置
            self.set_initial_position()
            time.sleep(1)
            self.get_logger().info("舵机已初始化并设置到初始位置")
        
        self.get_logger().info(
            f'舵机控制节点已启动\n'
            f'串口: {self.serial_port}\n'
            f'波特率: {self.baudrate}'
        )
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial = Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'成功连接到舵机串口: {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'连接舵机串口失败: {e}')
            return False
    
    def send_command(self, command, wait_for_response=False):
        """发送命令到舵机"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.get_logger().error('串口未连接，无法发送命令')
                return None
            
            try:
                self.serial.write(command.encode())
                self.get_logger().debug(f'发送命令: {command}')
                
                if wait_for_response:
                    time.sleep(0.1)
                    if self.serial.in_waiting:
                        response = self.serial.read(self.serial.in_waiting).decode().strip()
                        self.get_logger().debug(f'舵机响应: {response}')
                        return response
                return True
            except Exception as e:
                self.get_logger().error(f'发送命令失败: {e}')
                return None
    
    def _initialize_servos(self):
        """初始化舵机到中心位置"""
        # 设置舵机模式 - 注释掉了
        # 移动到中心位置
        command = f"#{0:03d}P{self.horizontal_servo_center:04d}T{1000:04d}!#{1:03d}P{self.vertical_servo_center:04d}T{1000:04d}!"
        self.send_command(command)
        time.sleep(1.5)
        self.get_logger().info(f"舵机初始化完成 - 水平:{self.horizontal_servo_center}, 垂直:{self.vertical_servo_center}")
        
        # 更新当前位置记录
        self.current_positions[0] = self.horizontal_servo_center
        self.current_positions[1] = self.vertical_servo_center
    
    def set_initial_position(self):
        """设置机械臂到初始位置"""
        command = "#000P1380T1500!#001P0650T1500!#002P2150T1500!#003P0750T1500!#004P1970T1500!#005P1670T1500!"
        return self.send_command(command)
    
    def center_servo(self, servo_id):
        """将舵机移动到中心位置"""
        return self.move_servo(servo_id, CENTER_POSITION, 5000)
    
    def set_mode(self, servo_id, mode):
        """设置舵机工作模式"""
        mode = max(1, min(8, mode))
        command = f"#{servo_id:03d}PMOD{mode}!"
        response = self.send_command(command, wait_for_response=True)
        return response and response == "#OK!"
    
    def receive_catch(self, timeout=0.1):
        """非阻塞方式接收舵机数据"""
        try:
            with self.serial_lock:
                # 保存原始超时设置
                original_timeout = self.serial.timeout
                
                # 设置较短的超时时间
                self.serial.timeout = timeout
                
                # 尝试读取数据
                data = self.serial.read(256)
                
                # 恢复原始超时设置
                self.serial.timeout = original_timeout
                
                if data:
                    # 假设数据是字符串格式类似 "#000P1000!\r\n"
                    data_str = data.decode('utf-8').strip()
                    self.get_logger().debug(f"接收到数据: {data_str}")
                    
                    # 提取舵机编号和角度
                    if data_str.startswith("#") and data_str.endswith("!"):
                        data_content = data_str.strip("#!").strip().strip("\r\n")
                        parts = data_content.split('P')
                        if len(parts) >= 2:
                            servo_id = parts[0]
                            angle = int(parts[1].split('!')[0])  # 以防有其他字符
                            self.get_logger().info(f"舵机编号: {servo_id}, 角度: {angle}")
                            return int(angle)
                    else:
                        self.get_logger().debug("数据格式不正确")
                return None
        except Exception as e:
            self.get_logger().error(f"接收失败: {e}")
            return None
    
    def move_servo(self, servo_id, position, time_ms=1000):
        """控制舵机移动到指定位置"""
        # 确保参数在有效范围内
        servo_id = max(0, min(254, servo_id))
        position = max(MIN_POSITION, min(MAX_POSITION, position))
        time_ms = max(0, min(9999, time_ms))
        
        # 更新位置记录
        if servo_id < len(self.current_positions):
            self.current_positions[servo_id] = position
        
        # 构造舵机控制命令
        command = f"#{servo_id:03d}P{position:04d}T{time_ms:04d}!"
        return self.send_command(command)
    
    def track_object(self, frame_width, frame_height, center_x, center_y):
        """跟踪物体，控制舵机使其保持在画面中心"""
        if center_x is not None and center_y is not None:
            
            # 计算图像中心
            frame_center_x = frame_width // 2 + 80
            frame_center_y = frame_height // 2
            
            # 计算像素误差
            pixel_error_x = center_x - frame_center_x
            pixel_error_y = center_y - frame_center_y
            
            # 将像素误差转换为PWM误差（总是计算，用于调试显示）
            pwm_error_x = pixel_error_x * self.pixel_to_pwm_ratio
            pwm_error_y = pixel_error_y * self.pixel_to_pwm_ratio
            
            # 死区检测
            if abs(pixel_error_x) < self.dead_zone_x:
                pixel_error_x = 0
                pwm_error_x = 0
            if abs(pixel_error_y) < self.dead_zone_y:
                pixel_error_y = 0
                pwm_error_y = 0
            
            # PID控制 - 应用像素到PWM的转换比例
            if pixel_error_x != 0 or pixel_error_y != 0:
                horizontal_output = self.horizontal_pid.update(pwm_error_x)
                vertical_output = self.vertical_pid.update(pwm_error_y)  # 垂直方向反向
                
                # 水平舵机控制
                if abs(horizontal_output) > self.horizontal_movement_threshold:
                    new_h_pos = self.current_horizontal_pos - horizontal_output
                    
                    # 平滑滤波
                    new_h_pos = (self.smooth_factor * self.current_horizontal_pos + 
                                (1 - self.smooth_factor) * new_h_pos)
                    
                    # 限制在水平舵机范围内
                    new_h_pos = max(self.horizontal_servo_range[0],  
                                   min(self.horizontal_servo_range[1], int(new_h_pos)))
                    # 只有当位置变化足够大时才发送命令
                    if abs(new_h_pos - self.current_horizontal_pos) > 3:
                        try:
                            self.current_horizontal_pos = new_h_pos
                            self.current_positions[0] = new_h_pos
                        except Exception as e:
                            print(f"水平舵机控制错误: {e}")
                
                # 垂直舵机控制
                if abs(vertical_output) > self.vertical_movement_threshold:
                    new_v_pos = self.current_vertical_pos - vertical_output
                    
                    
                    # 平滑滤波
                    new_v_pos = (self.smooth_factor * self.current_vertical_pos + 
                                (1 - self.smooth_factor) * new_v_pos)
                    
                    # 限制在垂直舵机范围内
                    new_v_pos = max(self.vertical_servo_range[0], 
                                   min(self.vertical_servo_range[1], int(new_v_pos)))
                    # print(new_v_pos)
                    # 只有当位置变化足够大时才发送命令
                    if abs(new_v_pos - self.current_vertical_pos) > 3:
                        try:
                            self.current_vertical_pos = new_v_pos
                            self.current_positions[1] = new_v_pos
                        except Exception as e:
                            print(f"舵机控制错误: {e}")
                if  abs(horizontal_output) > self.horizontal_movement_threshold and abs(vertical_output) > self.vertical_movement_threshold:
                    command = f"#{0:03d}P{new_h_pos:04d}T{abs(new_h_pos - self.current_horizontal_pos):04d}!#{1:03d}P{new_v_pos:04d}T{abs(new_v_pos - self.current_vertical_pos):04d}!"
                    self.send_command(command)    
                else:
                    if abs(vertical_output) > self.vertical_movement_threshold:
                        command = f"#{1:03d}P{new_v_pos:04d}T{abs(new_v_pos - self.current_vertical_pos):04d}!"
                        self.send_command(command)
                    elif abs(horizontal_output) > self.horizontal_movement_threshold:
                        command = f"#{0:03d}P{new_h_pos:04d}T{abs(new_h_pos - self.current_horizontal_pos):04d}!"
                        self.send_command(command)
        else:
            # 没有检测到色块时，初始化误差变量为0（用于调试显示）
            pixel_error_x = pixel_error_y = 0
            pwm_error_x = pwm_error_y = 0



        # if center_x is None or center_y is None:
        #     return
        
        # # 计算图像中心
        # frame_center_x = frame_width // 2 + 80
        # frame_center_y = frame_height // 2
        
        # # 计算像素误差
        # pixel_error_x = center_x - frame_center_x
        # pixel_error_y = center_y - frame_center_y
        
        # # 将像素误差转换为PWM误差
        # pwm_error_x = pixel_error_x * self.pixel_to_pwm_ratio
        # pwm_error_y = pixel_error_y * self.pixel_to_pwm_ratio
        
        # # 死区检测
        # if abs(pixel_error_x) < self.dead_zone_x:
        #     pixel_error_x = 0
        #     pwm_error_x = 0
        # if abs(pixel_error_y) < self.dead_zone_y:
        #     pixel_error_y = 0
        #     pwm_error_y = 0
        
        # # PID控制
        # if pixel_error_x != 0 or pixel_error_y != 0:
        #     horizontal_output = self.horizontal_pid.update(pwm_error_x)
        #     vertical_output = self.vertical_pid.update(pwm_error_y)
            
        #     new_h_pos = None
        #     new_v_pos = None
            
        #     # 水平舵机控制
        #     if abs(horizontal_output) > self.horizontal_movement_threshold:
        #         new_h_pos = self.current_horizontal_pos - horizontal_output
                
        #         # 平滑滤波
        #         new_h_pos = (self.smooth_factor * self.current_horizontal_pos + 
        #                     (1 - self.smooth_factor) * new_h_pos)
                
        #         # 限制在水平舵机范围内
        #         new_h_pos = max(self.horizontal_servo_range[0], 
        #                        min(self.horizontal_servo_range[1], int(new_h_pos)))
            
        #     # 垂直舵机控制
        #     if abs(vertical_output) > self.vertical_movement_threshold:
        #         new_v_pos = self.current_vertical_pos - vertical_output
                
        #         # 平滑滤波
        #         new_v_pos = (self.smooth_factor * self.current_vertical_pos + 
        #                     (1 - self.smooth_factor) * new_v_pos)
                
        #         # 限制在垂直舵机范围内
        #         new_v_pos = max(self.vertical_servo_range[0], 
        #                        min(self.vertical_servo_range[1], int(new_v_pos)))
            
        #     # 发送命令
        #     if new_h_pos is not None and new_v_pos is not None:
        #         # 只有当两个位置都需要更新且变化足够大时才发送组合命令
        #         if abs(new_h_pos - self.current_horizontal_pos) > 3 and abs(new_v_pos - self.current_vertical_pos) > 3:
        #             command = f"#{0:03d}P{new_h_pos:04d}T{abs(new_h_pos - self.current_horizontal_pos):04d}!#{1:03d}P{new_v_pos:04d}T{abs(new_v_pos - self.current_vertical_pos):04d}!"
        #             self.send_command(command)
        #             self.current_horizontal_pos = new_h_pos
        #             self.current_vertical_pos = new_v_pos
        #             self.current_positions[0] = new_h_pos
        #             self.current_positions[1] = new_v_pos
        #     elif new_h_pos is not None:
        #         # 只更新水平位置
        #         if abs(new_h_pos - self.current_horizontal_pos) > 3:
        #             command = f"#{0:03d}P{new_h_pos:04d}T{abs(new_h_pos - self.current_horizontal_pos):04d}!"
        #             self.send_command(command)
        #             self.current_horizontal_pos = new_h_pos
        #             self.current_positions[0] = new_h_pos
        #     elif new_v_pos is not None:
        #         # 只更新垂直位置
        #         if abs(new_v_pos - self.current_vertical_pos) > 3:
        #             command = f"#{1:03d}P{new_v_pos:04d}T{abs(new_v_pos - self.current_vertical_pos):04d}!"
        #             self.send_command(command)
        #             self.current_vertical_pos = new_v_pos
        #             self.current_positions[1] = new_v_pos
    
    def servo_command_callback(self, msg):
        """舵机命令回调"""
        if msg.servo_id >= 0 and msg.position >= 0:
            self.move_servo(msg.servo_id, msg.position, msg.time_ms)
        
        if msg.stop:
            self.stop_servo(msg.servo_id)
        
        if msg.set_mode and msg.mode >= 0:
            self.set_mode(msg.servo_id, msg.mode)
    
    def harvest_command_callback(self, msg):
        """采摘命令回调"""
        if msg.start_harvest and self.harvest_state == HARVEST_IDLE:
            self.start_harvest()
        
        if msg.stop_harvest:
            self.stop_harvest()
    
    def tracking_target_callback(self, msg):
        """跟踪目标回调"""
        if not self.enable_tracking:
            self.get_logger().warn('舵机跟踪功能已禁用，跳过跟踪指令')
            return
        
        # 添加调试日志
        self.get_logger().info(
            f'收到跟踪目标: 坐标=({int(msg.x)}, {int(msg.y)}), '
            f'图像宽度={int(msg.z)}'
        )
        
        # 执行舵机跟踪
        self.track_object(
            int(640),  # frame_width (存储在z中)
            int(480),    # frame_height (假设固定)
            int(msg.x),  # object_cx
            int(msg.y)   # object_cy
        )
    
    def start_harvest(self):
        """开始采摘"""
        if self.harvest_state != HARVEST_IDLE:
            self.get_logger().warn('采摘任务已在进行中')
            return
        
        self.get_logger().info('开始采摘任务')
        self.harvest_state = HARVEST_STARTED
        self.harvest_step_time = time.time()
        
        # 发布采摘开始状态
        status_msg = String()
        status_msg.data = json.dumps({
            "state": "started",
            "timestamp": time.time()
        })
        self.harvest_status_pub.publish(status_msg)
    
    def stop_harvest(self):
        """停止采摘"""
        self.harvest_state = HARVEST_IDLE
        self.set_initial_position()
        self.get_logger().info('采摘任务已停止')
    
    def harvest_state_machine(self):
        """采摘状态机"""
        if self.harvest_state == HARVEST_IDLE:
            return
        
        current_time = time.time()
        
        if self.harvest_state == HARVEST_STARTED:
            # 发送初始指令
            self.send_command(ARM_COMMANDS["rt_catch1"])
            self.get_logger().info('采摘步骤1: 机械臂就位')
            self.harvest_state = HARVEST_STEP1
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP1 and current_time - self.harvest_step_time > 2.0:
            # 第一步完成
            self.send_command(ARM_COMMANDS["rt_catch2"])
            self.get_logger().info('采摘步骤2: 抓取')
            self.harvest_state = HARVEST_STEP2
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP2 and current_time - self.harvest_step_time > 2.0:
            # 第二步完成
            self.send_command(ARM_COMMANDS["rt_catch3"])
            self.get_logger().info('采摘步骤3: 松回目标')
            self.harvest_state = HARVEST_STEP3
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP3 and current_time - self.harvest_step_time > 2.0:
            # 第三步完成
            self.send_command(ARM_COMMANDS["rt_catch4"])
            self.get_logger().info('采摘步骤4: 放置目标')
            self.harvest_state = HARVEST_STEP4
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP4 and current_time - self.harvest_step_time > 2.0:
            # 第四步完成
            self.send_command(ARM_COMMANDS["rt_catch5"])
            self.get_logger().info('采摘步骤5: 返回初始位置')
            self.harvest_state = HARVEST_COMPLETE
            self.harvest_step_time = current_time
            
            # 增加采摘计数
            self.harvested_count += 1
            
        elif self.harvest_state == HARVEST_COMPLETE and current_time - self.harvest_step_time > 2.0:
            # 采摘完成
            self.get_logger().info(f'采摘完成，总计: {self.harvested_count}')
            
            # 发布采摘完成状态
            status_msg = String()
            status_msg.data = json.dumps({
                "state": "completed",
                "harvested_count": self.harvested_count,
                "harvest_completed": True,
                "timestamp": time.time()
            })
            self.harvest_status_pub.publish(status_msg)
            
            # 重置状态
            self.harvest_state = HARVEST_IDLE
    
    def stop_servo(self, servo_id):
        """立即停止舵机"""
        command = f"#{servo_id:03d}PDST!"
        return self.send_command(command)
    
    def read_position(self, servo_id):
        """读取舵机当前位置"""
        command = f"#{servo_id:03d}PRAD!"
        response = self.send_command(command, wait_for_response=True)
        
        if response and response.startswith(f"#{servo_id:03d}P") and response.endswith("!"):
            try:
                position_str = response[5:-1]
                position = int(position_str)
                return position
            except Exception as e:
                self.get_logger().error(f'解析舵机位置错误: {e}')
                return None
        return None
    
    def publish_status(self):
        """发布舵机状态"""
        status_msg = ServoStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 舵机位置
        status_msg.servo_positions = self.current_positions
        
        # 采摘状态
        status_msg.harvest_state = self.harvest_state
        status_msg.harvested_count = self.harvested_count
        
        # 跟踪状态
        status_msg.tracking_active = self.tracking_active
        
        self.servo_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """清理资源"""
        # 停止所有舵机
        for i in range(6):
            self.stop_servo(i)
        
        # 设置到初始位置（不调用center_servo）
        if self.serial and self.serial.is_open:
            self.set_initial_position()
        
        # 关闭串口
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 使用多线程执行器提升并发性能
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        node = ServoControlNode()
        executor.add_node(node)
        node.get_logger().info('舵机控制节点启动，使用多线程执行器')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭舵机控制节点...')
    except Exception as e:
        node.get_logger().error(f'舵机控制节点出现错误: {e}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()