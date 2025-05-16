#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人控制器：处理网络命令和底盘控制
"""

import logging
import time
import threading
from typing import Dict, Any, Optional

from modules.command_generator import CommandGenerator, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, DIR_STOP
from modules.uart_controller import UARTController

logger = logging.getLogger("机器人控制器")

class RobotController:
    """机器人控制器类：处理网络命令和底盘控制"""
    
    def __init__(self, uart_controller: UARTController):
        """
        初始化机器人控制器
        
        Args:
            uart_controller: 串口控制器实例
        """
        self.uart_controller = uart_controller
        
        # 设置数据接收回调
        self.uart_controller.set_chassis_data_callback(self.handle_chassis_data)
        
        # 状态信息
        self.status = {
            "current_speed": 50,  # 默认速度为50%
            "current_direction": DIR_STOP,  # 默认方向为停止
            "position": {"latitude": 0.0, "longitude": 0.0}
        }
        
        # 启动底盘串口连接
        if not self.uart_controller.chassis_connected:
            self.uart_controller.connect_chassis()
    
    def handle_chassis_data(self, cmd: int, data: bytes):
        """
        处理从底盘接收到的数据
        
        Args:
            cmd: 命令类型
            data: 命令数据
        """
        from modules.command_generator import CMD_SET_DIRECTION, CMD_SET_SPEED, CMD_SET_POSITION, CMD_REQUEST_STATUS
        
        if cmd == CMD_SET_POSITION and len(data) >= 8:
            # 解析位置数据
            latitude, longitude = CommandGenerator.parse_position_data(data)
            if latitude is not None and longitude is not None:
                self.status["position"]["latitude"] = latitude
                self.status["position"]["longitude"] = longitude
                logger.info(f"位置已更新: 纬度={latitude}, 经度={longitude}")
        
        elif cmd == CMD_SET_DIRECTION and len(data) >= 2:
            # 更新方向和速度
            direction = data[0]
            speed = data[1]
            
            self.status["current_direction"] = direction
            self.status["current_speed"] = speed
            
            logger.info(f"方向已更新: 方向={direction}, 速度={speed}%")
        
        elif cmd == CMD_REQUEST_STATUS and len(data) >= 1:
            # 处理状态请求回应
            logger.debug(f"收到状态请求响应: {data}")
    
    def handle_command(self, command_data: Dict[str, Any]) -> bool:
        """
        处理收到的控制命令
        
        Args:
            command_data: 命令数据字典
        
        Returns:
            bool: 是否成功处理命令
        """
        cmd = command_data.get("command")
        params = command_data.get("params", {})
        speed = params.get("speed", self.status["current_speed"])
        
        # 确保速度在有效范围内
        speed = max(0, min(100, speed))
        
        logger.info(f"处理命令: {cmd}, 参数: {params}")
        
        # 更新当前速度
        self.status["current_speed"] = speed
        
        try:
            # 根据不同命令执行不同操作
            if cmd == "forward":
                self.status["current_direction"] = DIR_FORWARD
                command = CommandGenerator.generate_direction_command(DIR_FORWARD, speed)
                success = self.uart_controller.send_to_chassis(command)
                logger.info(f"机器人前进, 速度: {speed}%")
                return success
                
            elif cmd == "backward":
                self.status["current_direction"] = DIR_BACKWARD
                command = CommandGenerator.generate_direction_command(DIR_BACKWARD, speed)
                success = self.uart_controller.send_to_chassis(command)
                logger.info(f"机器人后退, 速度: {speed}%")
                return success
                
            elif cmd == "left":
                self.status["current_direction"] = DIR_LEFT
                command = CommandGenerator.generate_direction_command(DIR_LEFT, speed)
                success = self.uart_controller.send_to_chassis(command)
                logger.info(f"机器人左转, 速度: {speed}%")
                return success
                
            elif cmd == "right":
                self.status["current_direction"] = DIR_RIGHT
                command = CommandGenerator.generate_direction_command(DIR_RIGHT, speed)
                success = self.uart_controller.send_to_chassis(command)
                logger.info(f"机器人右转, 速度: {speed}%")
                return success
                
            elif cmd == "stop":
                self.status["current_direction"] = DIR_STOP
                command = CommandGenerator.generate_direction_command(DIR_STOP, 0)
                success = self.uart_controller.send_to_chassis(command)
                logger.info("机器人停止")
                return success
                
            elif cmd == "set_motor_speed":
                # 单独设置速度命令
                command = CommandGenerator.generate_speed_command(speed)
                success = self.uart_controller.send_to_chassis(command)
                logger.info(f"设置电机速度: {speed}%")
                
                # 如果当前有方向，更新该方向的速度
                if self.status["current_direction"] != DIR_STOP:
                    direction_cmd = CommandGenerator.generate_direction_command(
                        self.status["current_direction"], speed)
                    self.uart_controller.send_to_chassis(direction_cmd)
                
                return success
                
            elif cmd == "emergencyStop":
                # 紧急停止所有功能
                self.status["current_direction"] = DIR_STOP
                command = CommandGenerator.generate_direction_command(DIR_STOP, 0)
                success = self.uart_controller.send_to_chassis(command)
                logger.info("紧急停止所有操作")
                return success
                
            else:
                logger.warning(f"未知命令: {cmd}")
                return False
                
        except Exception as e:
            logger.error(f"处理命令失败: {e}")
            return False
    
    def set_position(self, latitude: float, longitude: float) -> bool:
        """
        设置位置并发送到底盘
        
        Args:
            latitude: 纬度
            longitude: 经度
            
        Returns:
            bool: 是否成功设置位置
        """
        try:
            # 更新本地状态
            self.status["position"]["latitude"] = latitude
            self.status["position"]["longitude"] = longitude
            
            # 生成并发送位置命令到底盘
            command = CommandGenerator.generate_position_command(latitude, longitude)
            success = self.uart_controller.send_to_chassis(command)
            
            if success:
                logger.info(f"位置已设置: 纬度={latitude}, 经度={longitude}")
            
            return success
        except Exception as e:
            logger.error(f"设置位置失败: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取当前状态
        
        Returns:
            Dict[str, Any]: 当前状态字典
        """
        return self.status.copy()
    
    def request_status(self) -> bool:
        """
        请求底盘状态
        
        Returns:
            bool: 是否成功发送请求
        """
        try:
            command = CommandGenerator.generate_request_status_command()
            return self.uart_controller.send_to_chassis(command)
        except Exception as e:
            logger.error(f"请求状态失败: {e}")
            return False 