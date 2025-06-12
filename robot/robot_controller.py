#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人控制模块
处理小车的移动控制，包括前进、后退、转向、停止等功能
"""

import serial
import logging
import time
import threading
from serial import Serial

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("robot_controller")

# 命令类型常量
CMD_SET_DIRECTION = 0x01
CMD_SET_SPEED = 0x02
CMD_SET_MOTOR = 0x03
CMD_REQUEST_STATUS = 0x04
CMD_SET_POSITION = 0x05

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04

# 创建线程锁
robot_serial_lock = threading.Lock()

class CommandGenerator:
    """命令生成器类，用于生成发送给机器人的命令"""
    
    @staticmethod
    def generate_packet(cmd, data=None):
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

    @staticmethod
    def generate_direction_command(direction, speed):
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

        return CommandGenerator.generate_packet(CMD_SET_DIRECTION, [direction, speed])

    @staticmethod
    def generate_speed_command(speed):
        """
        生成设置速度的命令
        :param speed: 速度 (0-100)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return CommandGenerator.generate_packet(CMD_SET_SPEED, [speed])

    @staticmethod
    def generate_position_command(latitude, longitude):
        """
        生成设置位置的命令
        :param latitude: 纬度
        :param longitude: 经度
        :return: 命令字节序列
        """
        # 将浮点数经纬度转换为整数（乘以10^6）
        lat_int = int(latitude * 1000000)
        lon_int = int(longitude * 1000000)

        # 将32位整数分解为4个字节
        position_data = [
            (lat_int >> 24) & 0xFF,
            (lat_int >> 16) & 0xFF,
            (lat_int >> 8) & 0xFF,
            lat_int & 0xFF,
            (lon_int >> 24) & 0xFF,
            (lon_int >> 16) & 0xFF,
            (lon_int >> 8) & 0xFF,
            lon_int & 0xFF
        ]

        return CommandGenerator.generate_packet(CMD_SET_POSITION, position_data)


class RobotController:
    """机器人控制器类，用于控制机器人的移动"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        """初始化机器人控制器"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connect()

    def connect(self):
        """连接到机器人控制串口"""
        try:
            self.serial = Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"成功连接到机器人控制串口：{self.port}，波特率：{self.baudrate}")
            return True
        except Exception as e:
            logger.error(f"连接机器人控制串口失败: {e}")
            # 尝试自动检测串口
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    # 使用第一个找到的串口
                    port = ports[0].device
                    logger.info(f"尝试自动选择串口: {port}")
                    self.port = port
                    self.serial = serial.Serial(
                        port=self.port,
                        baudrate=self.baudrate,
                        timeout=self.timeout
                    )
                    logger.info(f"成功连接到机器人控制串口：{self.port}")
                    return True
            except Exception as e2:
                logger.error(f"自动连接串口失败: {e2}")
            return False

    def disconnect(self):
        """断开与机器人控制串口的连接"""
        if self.serial and self.serial.isOpen():
            self.serial.close()
            logger.info("已断开与机器人控制串口的连接")

    def send_command(self, command):
        """
        发送命令到机器人
        :param command: 字节形式的命令
        :return: 成功返回True，失败返回False
        """
        with robot_serial_lock:
            if not self.serial or not self.serial.isOpen():
                logger.error("机器人控制串口未连接，无法发送命令")
                return False

            try:
                self.serial.write(command)
                # 将命令转为十六进制字符串便于日志记录
                hex_command = ' '.join([f"{b:02X}" for b in command])
                logger.info(f"发送机器人命令: {hex_command}")
                return True
            except Exception as e:
                logger.error(f"发送机器人命令失败: {e}")
                return False

    def move(self, direction, speed):
        """
        控制机器人移动
        :param direction: 方向常量(DIR_FORWARD, DIR_BACKWARD等)
        :param speed: 速度 (0-100)
        :return: 成功返回True，失败返回False
        """
        command = CommandGenerator.generate_direction_command(direction, speed)
        return self.send_command(command)

    def stop(self):
        """停止机器人"""
        command = CommandGenerator.generate_direction_command(DIR_STOP, 0)
        return self.send_command(command)

    def set_speed(self, speed):
        """设置机器人速度"""
        command = CommandGenerator.generate_speed_command(speed)
        return self.send_command(command)
        
    def set_position(self, latitude, longitude):
        """
        设置机器人位置
        :param latitude: 纬度
        :param longitude: 经度
        :return: 成功返回True，失败返回False
        """
        command = CommandGenerator.generate_position_command(latitude, longitude)
        return self.send_command(command)