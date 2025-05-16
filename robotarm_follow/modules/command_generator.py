#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
串口命令生成器：负责生成机器人底盘控制命令
"""

import logging

# 命令类型常量
CMD_SET_DIRECTION = 0x01
CMD_SET_SPEED = 0x02
CMD_SET_MOTOR = 0x03
CMD_REQUEST_STATUS = 0x04
CMD_SET_POSITION = 0x05  # 设置/更新位置信息

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

logger = logging.getLogger("命令生成器")

class CommandGenerator:
    """生成与机器人底盘通信的命令包"""
    
    @staticmethod
    def generate_packet(cmd, data=None):
        """
        生成数据包
        
        Args:
            cmd: 命令类型
            data: 数据列表
        
        Returns:
            bytes: 完整的命令字节序列
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
        
        Args:
            direction: 方向常量
            speed: 速度 (0-100)
        
        Returns:
            bytes: 命令字节序列
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
        
        Args:
            speed: 速度 (0-100)
        
        Returns:
            bytes: 命令字节序列
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
        
        Args:
            latitude: 纬度
            longitude: 经度
        
        Returns:
            bytes: 命令字节序列
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
    
    @staticmethod
    def generate_request_status_command():
        """
        生成请求状态的命令
        
        Returns:
            bytes: 命令字节序列
        """
        return CommandGenerator.generate_packet(CMD_REQUEST_STATUS)
        
    @staticmethod
    def parse_position_data(data):
        """
        解析位置数据
        
        Args:
            data: 位置数据字节列表(8字节)
        
        Returns:
            tuple: (纬度, 经度)
        """
        if len(data) < 8:
            return None, None
            
        # 从字节数组中解析出经纬度
        lat_int = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        lon_int = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]

        # 处理有符号整数
        if lat_int & 0x80000000:
            lat_int = lat_int - 0x100000000
        if lon_int & 0x80000000:
            lon_int = lon_int - 0x100000000

        # 转换回浮点数
        latitude = lat_int / 1000000.0
        longitude = lon_int / 1000000.0
        
        return latitude, longitude 