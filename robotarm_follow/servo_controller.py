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
        self.connect()

        self.stop_flag_x=1 
        self.read_flag_x=1
        self.send_left=1
        self.send_right=1
        self.PID_STARTX=0

        self.stop_flag_y=1 
        self.read_flag_y=1
        self.send_up=1
        self.send_down=1
        self.PID_STARTY=0

    
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
        command = "#000P1250T1500!#001P0900T1500!#002P2000T1500!#003P0800T1500!#004P1500T1500!#005P1200T1500!"
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
        
    def track_object(self, frame_width,fram_hight, object_cx, object_cy, current_position=CENTER_POSITION):
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
        # 计算画面中心与物体中心的水平偏差
        frame_center_x = frame_width // 2+80
        offset_x = frame_center_x - object_cx
        SPEED = 7.5
        # 设置死区范围，避免微小偏差引起的频繁调整
        dead_zone = 30  # 较大的死区，减少频繁移动
        
        # 只有偏差超过死区才进行调整
        # if abs(offset_x) <= dead_zone:
        #     logger.debug(f"物体在中心区域内，偏差={offset_x}px,无需调整")
        #     return current_position
        # else:
            #print(cx - CENTERX)
        # print(offset_x)
        if abs(object_cx - frame_center_x) <= dead_zone:
            if self.stop_flag_x == 1:
                command1 = "#{:03d}PDST!".format(0)
                self.send_command(command1)
                self.stop_flag_x = 0
                self.read_flag_x = 1
                self.send_left=1
                self.send_right=1
        else:
            self.stop_flag_x = 1
            if self.read_flag_x == 1:
                command1 = "#{:03d}PRAD!".format(0)
                self.send_command(command1)
                self.PID_STARTX=self.receive_catch()
                self.read_flag_x=0
            else:
                #print(PID_STARTX)
                
                if frame_center_x - object_cx > dead_zone:
                    
                    if self.PID_STARTX > 2100:
                        command1 = "#{:03d}PDST!".format(0)
                    else:
                        temp=int((2167-self.PID_STARTX)*SPEED)
                        if temp<4000:
                            temp=4000
                        command1 = "#{:03d}P{:04d}T{:04d}!".format(0, 2167, temp)
                    if self.send_left==1:
                        self.send_command(command1)
                        self.send_left=0
                        self.send_right=1
                elif object_cx - frame_center_x > dead_zone:
                    
                    if self.PID_STARTX < 833:
                        command1 = "#{:03d}PDST!".format(0)
                    else:
                        temp=int((self.PID_STARTX-833)*SPEED)
                        if temp<3000:
                            temp=3000
                        command1 = "#{:03d}P{:04d}T{:04d}!".format(0, 833, temp)
                    if self.send_right==1:
                        self.send_command(command1)
                        self.send_right=0
                        self.send_left=1   
        frame_center_y = fram_hight // 2
        offset_y = frame_center_y - object_cy
        #print(cy - CENTERY)
        if abs(object_cy - frame_center_y) <= dead_zone:
            if self.stop_flag_y == 1:
                command1 = "#{:03d}PDST!".format(1)
                self.send_command(command1)
                self.stop_flag_y = 0
                self.read_flag_y = 1
                self.send_up=1
                self.send_down=1
        else:
            self.stop_flag_y = 1
            if self.read_flag_y == 1:
                command1 = "#{:03d}PRAD!".format(1)
                self.send_command(command1)
                self.PID_STARTY=self.receive_catch()
                self.read_flag_y=0
            else:
                if frame_center_y - object_cy > dead_zone:  
                    if self.PID_STARTY > 1480:
                        command1 = "#{:03d}PDST!".format(1)
                    else:
                        temp=int((1500 - self.PID_STARTY)*SPEED)
                        if temp<4000:
                            temp=4000
                        command1 = "#{:03d}P{:04d}T{:04d}!".format(1, 1500, temp)
                    if self.send_up==1:
                        self.send_command(command1)
                        self.send_up=0
                        self.send_down=1
                elif object_cy - frame_center_y > dead_zone:
                    
                    if self.PID_STARTY < 882:
                        command1 = "#{:03d}PDST!".format(1)
                    else:
                        temp=int((self.PID_STARTY - 882)*SPEED)
                        if temp <3000:
                            temp=3000
                        command1 = "#{:03d}P{:04d}T{:04d}!".format(1, 882,temp)
                    if self.send_down==1:
                        self.send_command(command1)
                        self.send_down=0
                        self.send_up=1
        # # 根据偏差计算舵机新位置
        # # 使用非线性映射，大偏差时移动更快，小偏差时移动更缓慢
        # # 偏移系数根据偏差大小动态调整
        # offset_coefficient = 1  # 基础系数
        # # if abs(offset_x) > 100:
        # #     offset_coefficient = 1.5  # 偏差较大时，加大调整幅度
        # # elif abs(offset_x) <= 30:
        # #     offset_coefficient = 0.5  # 偏差小时，减小调整幅度
        
        # # 计算新位置，加入平滑处理
        position_delta = int(offset_x * 10)
        # # 限制单次调整的最大幅度
        position_delta = max(-30, min(30, position_delta))
        
        new_position = current_position + position_delta
        
        # # 限制在有效范围内 (500-2500)
        # new_position = max(MIN_POSITION, min(MAX_POSITION, new_position))
        
        # # 只有当位置变化超过阈值时才调整，避免频繁小幅度调整
        # with servo_lock:
        #     if abs(new_position - current_position) > dead_zone:
        #         # 控制舵机，增加移动时间使移动更平滑
        #         time_ms = 5000  # 增加移动时间，使运动更平滑
        #         self.move_servo(servo_id, new_position, time_ms)
        #         logger.info(f"跟踪物体：位置偏差={offset_x}px, 舵机位置从{current_position}调整到{new_position}，时间={time_ms}ms")
        
        return new_position


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