#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制模块
处理舵机控制相关功能，用于控制机器臂移动
"""

import serial
import time
import logging
import threading

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("servo_controller")

# 舵机控制相关的常量
DEFAULT_SERVO_PORT = "/dev/ttyS9"  # 默认舵机串口
DEFAULT_SERVO_BAUDRATE = 115200  # 默认波特率
DEFAULT_SERVO_ID = 0  # 默认舵机ID
CENTER_POSITION = 1500  # 中心位置
MIN_POSITION = 500  # 最小位置
MAX_POSITION = 2500  # 最大位置
SERVO_MODE = 3  # 舵机模式：3表示180度顺时针

# 线程锁
servo_lock = threading.Lock()
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
class ServoController:
    """总线舵机控制器类"""
    
    def __init__(self, port=DEFAULT_SERVO_PORT, baudrate=DEFAULT_SERVO_BAUDRATE, timeout=1):
        """
        初始化舵机控制器
        
        参数:
        port -- 串口设备名，默认为"/dev/ttyS9"
        baudrate -- 波特率，默认为115200
        timeout -- 超时时间(秒)，默认为1秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        # 舵机PWM范围设置
        self.horizontal_servo_range = (500, 2500)  # 水平方向180度
        self.vertical_servo_range = (500, 1500)    # 垂直方向90度
        # 舵机中心位置
        self.horizontal_servo_center = (self.horizontal_servo_range[0] + self.horizontal_servo_range[1]) // 2  # 1500
        self.vertical_servo_center = 600     # 1000
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
        # 初始化舵机
        self._initialize_servos()

        self.connect()



    def _initialize_servos(self):
        """初始化舵机到中心位置"""
        # 设置舵机模式

        # 移动到中心位置
        command = f"#{0:03d}P{self.horizontal_servo_center:04d}T{1000:04d}!#{1:03d}P{self.vertical_servo_center:04d}T{1000:04d}!"
        self.send_command(command)
        time.sleep(1.5)
        print(f"舵机初始化完成 - 水平:{self.horizontal_servo_center}, 垂直:{self.vertical_servo_center}")   
    def connect(self):
        """连接到舵机"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"成功连接到舵机，串口：{self.port}，波特率：{self.baudrate}")
            return True
        except Exception as e:
            logger.error(f"连接舵机失败: {e}")
            return False
    
    def disconnect(self):
        """断开与舵机的连接"""
        if self.serial and self.serial.isOpen():
            self.serial.close()
            logger.info("已断开与舵机的连接")
    
    def send_command(self, command, wait_for_response=False):
        """
        发送命令到舵机
        
        参数:
        command -- 要发送的命令字符串
        wait_for_response -- 是否等待响应，默认为False
        
        返回:
        如果wait_for_response为True，返回舵机响应；否则返回None
        """
        if not self.serial or not self.serial.isOpen():
            logger.error("串口未连接，无法发送命令")
            return None
        
        try:
            self.serial.write(command.encode())
            logger.info(f"已发送命令: {command}")
            
            if wait_for_response:
                # 等待舵机响应
                #time.sleep(0.1)  # 给舵机一些响应时间
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting).decode().strip()
                    logger.debug(f"舵机响应: {response}")
                    return response
            return True
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            return None
    
    def move_servo(self, servo_id, position, time_ms=1000):
        """
        控制舵机移动到指定位置
        
        参数:
        servo_id -- 舵机ID，范围0-254
        position -- 位置值，范围500-2500
        time_ms -- 移动时间，单位毫秒，范围0-9999
        
        返回:
        成功返回True，失败返回False
        """
        # 确保参数在有效范围内
        servo_id = max(0, min(254, servo_id))
        position = max(MIN_POSITION, min(MAX_POSITION, position))
        time_ms = max(0, min(9999, time_ms))
        
        # 构造舵机控制命令: #000P1500T1000!
        command = f"#{servo_id:03d}P{position:04d}T{time_ms:04d}!"
        return self.send_command(command)
    
    def set_mode(self, servo_id, mode):
        """
        设置舵机工作模式
        
        参数:
        servo_id -- 舵机ID
        mode -- 工作模式(1-8):
                1: 舵机模式 270度顺时针
                2: 舵机模式 270度逆时针
                3: 舵机模式 180度顺时针
                4: 舵机模式 180度逆时针
                5: 马达模式 360度定圈顺时针
                6: 马达模式 360度定圈逆时针
                7: 马达模式 360度定时顺时针
                8: 马达模式 360度定时逆时针
        
        返回:
        成功返回True，失败返回False
        """
        # 确保模式在有效范围内
        mode = max(1, min(8, mode))
        
        # 构造设置模式命令: #000PMOD1!
        command = f"#{servo_id:03d}PMOD{mode}!"
        response = self.send_command(command, wait_for_response=True)
        
        # 验证响应是否正确，应为 #OK!
        return response and response == "#OK!"
    
    def center_servo(self, servo_id):
        """将舵机移动到中心位置(5000)"""
        return self.move_servo(servo_id, CENTER_POSITION, 5000)
    
    def set_initial_position(self):
        """设置所有舵机到初始位置"""
        command = "#000P1380T1500!#001P0650T1500!#002P2150T1500!#003P0750T1500!#004P1970T1500!#005P1670T1500!"
        return self.send_command(command)
    
    def receive_catch(self, timeout=0.1):
        """
        非阻塞方式接收舵机数据
        
        参数:
        timeout -- 最大等待时间(秒)
        
        返回:
        舵机位置(500-2500)，失败返回None
        """
        try:
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
                logger.debug(f"接收到数据: {data_str}")
                
                # 提取舵机编号和角度
                if data_str.startswith("#") and data_str.endswith("!"):
                    data_content = data_str.strip("#!").strip().strip("\r\n")
                    parts = data_content.split('P')
                    if len(parts) >= 2:
                        servo_id = parts[0]
                        angle = int(parts[1].split('!')[0])  # 以防有其他字符
                        logger.info(f"舵机编号: {servo_id}, 角度: {angle}")
                        return int(angle)
                else:
                    logger.debug("数据格式不正确")
            return None
        except Exception as e:
            logger.error(f"接收失败: {e}")
            return None
        
    def track_object(self, frame_width,frame_height, center_x, center_y, current_position=CENTER_POSITION):
        """
        跟踪物体，控制舵机使其保持在画面中心
        
        参数:
        frame_width -- 图像宽度
        object_cx -- 物体中心x坐标
        servo_id -- 舵机ID
        current_position -- 当前舵机位置
        
        返回:
        新的舵机位置
        """
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
  

        return 1



    def stop_servo(self, servo_id=DEFAULT_SERVO_ID):
        """
        立即停止舵机在当前位置
        
        参数:
        servo_id -- 舵机ID
        
        返回:
        成功返回True，失败返回False
        """
        command = f"#{servo_id:03d}PDST!"
        result = self.send_command(command)
        if result:
            logger.info(f"舵机 {servo_id} 已停止在当前位置")
        else:
            logger.error(f"舵机 {servo_id} 停止命令发送失败")
        return result
    

    def read_position(self, servo_id=DEFAULT_SERVO_ID):
        """
        读取舵机当前位置
        
        参数:
        servo_id -- 舵机ID
        
        返回:
        舵机位置(500-2500)，失败返回None
        """
        with servo_lock:
            # 构造读取位置命令: #000PRAD!
            command = f"#{servo_id:03d}PRAD!"
            response = self.send_command(command, wait_for_response=True)
            
            # 解析响应: #000P1500!
            if response and response.startswith(f"#{servo_id:03d}P") and response.endswith("!"):
                try:
                    # 提取位置值
                    position_str = response[5:-1]  # 去掉前缀和后缀
                    position = int(position_str)
                    logger.debug(f"舵机 {servo_id} 当前位置: {position}")
                    return position
                except Exception as e:
                    logger.error(f"解析舵机位置响应错误: {e}, 原始响应: {response}")
                    return None
            else:
                logger.error(f"读取舵机位置失败，无效响应: {response}")
                return None