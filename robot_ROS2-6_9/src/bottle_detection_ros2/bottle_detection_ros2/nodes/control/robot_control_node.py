#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人控制节点
负责控制机器人底盘的移动，包括前进、后退、转向等
通过串口与底层控制器通信
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from bottle_detection_msgs.msg import RobotCommand, RobotStatus
from serial import Serial
import threading
import json
import time
import struct
import random
import math
import datetime

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


class RobotControlNode(Node):
    """机器人控制节点类"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyS3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('status_publish_rate', 2.0)  # Hz
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.status_rate = self.get_parameter('status_publish_rate').value
        
        # 初始化串口
        self.serial = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # 创建订阅者
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos
        )
        
        self.robot_cmd_sub = self.create_subscription(
            RobotCommand,
            'robot/command',
            self.robot_command_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped,
            'robot/set_position',
            self.set_position_callback,
            10
        )
        
        # 创建发布者
        self.status_pub = self.create_publisher(
            RobotStatus,
            'robot/status',
            10
        )
        
        # 状态变量 - 确保所有数值都是浮点数，并设置有意义的初值
        self.current_speed = 0.0  # 浮点数
        self.current_direction = float(DIR_STOP)  # 转换为浮点数
        
        # 位置信息（默认在西安某苹果园）
        self.position = {
            'x': 0.0, 
            'y': 0.0, 
            'latitude': 34.938500 + random.uniform(-0.002, 0.002),  # 西安纬度 + 随机偏移
            'longitude': 108.241500 + random.uniform(-0.002, 0.002)  # 西安经度 + 随机偏移
        }
        
        # 设备状态
        self.battery_level = random.uniform(65, 90)  # 随机电池电量
        self.cpu_usage = 0.0
        
        # 采摘统计 - 设置有意义的初值
        self.harvested_count = 0
        self.today_harvested = 0
        self.total_harvested = 0
        self.working_hours = 0.0
        self.working_area = 0.0
        self.harvest_accuracy = 96.5
        self.temperature = 25.0
        self.signal_strength = 75
        self.upload_bandwidth = 50.0
        
        # 工作状态和模式
        self.work_mode = "idle"  # 工作模式：idle, moving, harvesting
        self.is_working = False  # 是否正在工作
        
        # 时间戳
        self.start_time = time.time()
        self.last_harvest_time = time.time()
        
        # 位置相关
        self.last_position = None
        self.position_noise = 0.0001  # 位置噪声（度）
        
        # 创建定时器发布状态
        self.status_timer = self.create_timer(2.0, self.publish_status)
        self.simulation_timer = self.create_timer(5.0, self.update_simulation_data)
        
        # 紧急停止标志
        self.emergency_stop = False
        
        self.get_logger().info(
            f'机器人控制节点已启动\n'
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
            self.get_logger().info(f'成功连接到串口: {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().error(f'连接串口失败: {e}')
            # 尝试自动检测串口
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    port = ports[0].device
                    self.get_logger().info(f'尝试自动选择串口: {port}')
                    self.serial_port = port
                    self.serial = Serial(
                        port=self.serial_port,
                        baudrate=self.baudrate,
                        timeout=self.timeout
                    )
                    self.get_logger().info(f'成功连接到串口: {self.serial_port}')
                    return True
            except Exception as e2:
                self.get_logger().error(f'自动连接串口失败: {e2}')
            return False
    
    def generate_packet(self, cmd, data=None):
        """生成通信数据包"""
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
    
    def send_command(self, command):
        """发送命令到机器人"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.get_logger().error('串口未连接，无法发送命令')
                return False
            
            try:
                self.serial.write(command)
                # 记录发送的命令（十六进制格式）
                hex_command = ' '.join([f"{b:02X}" for b in command])
                self.get_logger().debug(f'发送命令: {hex_command}')
                return True
            except Exception as e:
                self.get_logger().error(f'发送命令失败: {e}')
                return False
    
    def move(self, direction, speed):
        """控制机器人移动"""
        # 确保速度在0-100范围内
        speed_int = max(0, min(100, int(speed * 100)))
        
        # 生成方向和速度命令
        command = self.generate_packet(CMD_SET_DIRECTION, [direction, speed_int])
        
        # 更新状态
        self.current_direction = float(direction)  # 保持为浮点数
        self.current_speed = float(speed)  # 保持为浮点数
        
        return self.send_command(command)
    
    def stop(self):
        """停止机器人"""
        command = self.generate_packet(CMD_SET_DIRECTION, [DIR_STOP, 0])
        self.current_direction = float(DIR_STOP)  # 保持为浮点数
        self.current_speed = 0.0  # 保持为浮点数
        return self.send_command(command)
    
    def set_position(self, latitude, longitude):
        """设置机器人位置"""
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
        
        command = self.generate_packet(CMD_SET_POSITION, position_data)
        
        # 更新本地位置
        self.position['latitude'] = latitude
        self.position['longitude'] = longitude
        
        return self.send_command(command)
    
    def cmd_vel_callback(self, msg):
        """Twist命令回调"""
        if self.emergency_stop:
            self.get_logger().warn('紧急停止状态，忽略移动命令')
            return
        
        # 解析Twist消息
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 根据速度判断方向
        if abs(linear_x) > abs(angular_z):
            # 直线运动为主
            if linear_x > 0.01:
                direction = DIR_FORWARD
                speed = abs(linear_x) / self.max_linear_speed
            elif linear_x < -0.01:
                direction = DIR_BACKWARD
                speed = abs(linear_x) / self.max_linear_speed
            else:
                direction = DIR_STOP
                speed = 0
        else:
            # 旋转运动为主
            if angular_z > 0.01:
                direction = DIR_LEFT
                speed = abs(angular_z) / self.max_angular_speed
            elif angular_z < -0.01:
                direction = DIR_RIGHT
                speed = abs(angular_z) / self.max_angular_speed
            else:
                direction = DIR_STOP
                speed = 0
        
        # 发送移动命令
        self.move(direction, speed)
    
    def robot_command_callback(self, msg):
        """机器人命令回调"""
        if msg.emergency_stop:
            self.emergency_stop = True
            self.stop()
            self.get_logger().warn('收到紧急停止命令！')
            # 3秒后解除紧急停止
            self.create_timer(3.0, self.clear_emergency_stop)
            return
        
        if self.emergency_stop:
            self.get_logger().warn('紧急停止状态，忽略命令')
            return
        
        # 处理其他命令
        cmd = msg.command
        speed = msg.speed
        
        if cmd == "forward":
            self.move(DIR_FORWARD, speed)
        elif cmd == "backward":
            self.move(DIR_BACKWARD, speed)
        elif cmd == "left":
            self.move(DIR_LEFT, speed)
        elif cmd == "right":
            self.move(DIR_RIGHT, speed)
        elif cmd == "stop":
            self.stop()
    
    def set_position_callback(self, msg):
        """设置位置回调"""
        # 从PoseStamped消息提取经纬度
        longitude = msg.pose.position.x
        latitude = msg.pose.position.y
        
        self.set_position(latitude, longitude)
        
        self.get_logger().info(f'设置位置: 纬度={latitude}, 经度={longitude}')
    
    def clear_emergency_stop(self):
        """清除紧急停止状态"""
        self.emergency_stop = False
        self.get_logger().info('紧急停止已解除')
    
    def publish_status(self):
        """发布机器人状态"""
        # 模拟一些状态数据（实际应用中应从传感器读取）
        try:
            import psutil
            cpu_usage = psutil.cpu_percent(interval=0.1)
        except ImportError:
            cpu_usage = random.uniform(20, 80)  # 如果没有psutil，使用随机值
        
        status_msg = RobotStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = "base_link"
        
        # 基本状态 - 确保所有字段都是正确的类型
        status_msg.battery_level = float(self.battery_level)
        status_msg.cpu_usage = float(cpu_usage)
        status_msg.current_speed = float(self.current_speed)
        status_msg.current_direction = float(self.current_direction)
        
        # 位置信息
        status_msg.position_x = float(self.position['x'])
        status_msg.position_y = float(self.position['y'])
        status_msg.latitude = float(self.position['latitude'])
        status_msg.longitude = float(self.position['longitude'])
        
        # 采摘统计
        status_msg.harvested_count = int(self.harvested_count)
        status_msg.today_harvested = int(self.today_harvested)
        status_msg.total_harvested = int(self.total_harvested)
        
        # 工作统计
        status_msg.working_hours = float(self.working_hours)
        status_msg.working_area = float(self.working_area)
        
        # 其他状态
        status_msg.emergency_stop = bool(self.emergency_stop)
        status_msg.is_moving = bool(self.current_direction != DIR_STOP or self.work_mode == "moving")
        status_msg.is_harvesting = bool(self.work_mode == "harvesting")
        
        # 网络信息
        status_msg.signal_strength = int(self.signal_strength)
        status_msg.upload_bandwidth = float(self.upload_bandwidth)
        
        # 状态描述
        if self.emergency_stop:
            status_msg.status_text = "紧急停止"
        elif self.work_mode == "harvesting":
            status_msg.status_text = "正在采摘"
        elif self.work_mode == "moving" or self.current_direction != DIR_STOP:
            status_msg.status_text = "移动中"
        elif self.is_working:
            status_msg.status_text = "工作中"
        else:
            status_msg.status_text = "待机"
        
        # 额外字段
        status_msg.harvest_accuracy = float(self.harvest_accuracy)
        status_msg.temperature = float(self.temperature)
        status_msg.location_name = "苹果园区3号地块"
        
        self.status_pub.publish(status_msg)
        
        # 打印调试信息（每10次发布打印一次）
        if hasattr(self, '_status_count'):
            self._status_count += 1
        else:
            self._status_count = 1
            
        if self._status_count % 10 == 0:
            self.get_logger().info(
                f'状态发布 - 今日采摘: {self.today_harvested}, '
                f'作业面积: {self.working_area:.3f}亩, '
                f'工作时长: {self.working_hours:.2f}小时, '
                f'电池: {self.battery_level:.1f}%, '
                f'模式: {self.work_mode}'
            )
    
    def update_simulation_data(self):
        """更新模拟数据"""
        current_time = time.time()
        
        # 更新工作时长
        if self.is_working:
            self.working_hours = (current_time - self.start_time) / 3600.0
        
        # 模拟位置变化（如果正在移动）
        if self.current_direction != DIR_STOP or self.work_mode == "moving":
            # 添加随机位置变化模拟移动
            lat_change = random.uniform(-self.position_noise, self.position_noise)
            lon_change = random.uniform(-self.position_noise, self.position_noise)
            
            # 更新位置
            old_lat = self.position['latitude']
            old_lon = self.position['longitude']
            
            self.position['latitude'] += lat_change
            self.position['longitude'] += lon_change
            
            # 计算移动距离并累加作业面积
            if self.last_position is not None:
                distance = self.calculate_distance(
                    old_lat, old_lon, 
                    self.position['latitude'], self.position['longitude']
                )
                # 假设作业宽度为2米，将距离转换为面积（亩）
                area_increment = (distance * 2) / 666.67  # 1亩 = 666.67平方米
                self.working_area += area_increment
            
            self.last_position = (self.position['latitude'], self.position['longitude'])
        
        # 模拟采摘过程
        if self.work_mode == "harvesting" or (self.is_working and random.random() < 0.3):
            # 30%的概率进行采摘
            time_since_last_harvest = current_time - self.last_harvest_time
            
            if time_since_last_harvest > 10:  # 至少10秒间隔
                # 模拟一次采摘
                new_harvest = random.randint(1, 5)  # 一次采摘1-5个
                self.harvested_count += new_harvest
                self.today_harvested += new_harvest
                self.total_harvested += new_harvest
                self.last_harvest_time = current_time
                
                # 随机更新工作模式
                self.work_mode = random.choice(["harvesting", "moving", "idle"])
                
                self.get_logger().info(f'模拟采摘: +{new_harvest}, 今日总计: {self.today_harvested}')
        
        # 模拟电池消耗
        if self.is_working:
            battery_drain = random.uniform(0.1, 0.3)  # 每次消耗0.1-0.3%
            self.battery_level = max(10, self.battery_level - battery_drain)
        
        # 模拟设备温度变化
        if self.is_working:
            self.temperature += random.uniform(-0.5, 1.0)
        else:
            self.temperature += random.uniform(-0.3, 0.3)
        self.temperature = max(20, min(40, self.temperature))  # 限制在20-40度
        
        # 随机改变工作状态
        if random.random() < 0.1:  # 10%概率改变工作状态
            self.is_working = not self.is_working
            if self.is_working:
                self.get_logger().info('开始工作模拟')
            else:
                self.get_logger().info('停止工作模拟')
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间距离（米）"""
        # 使用Haversine公式
        R = 6371000  # 地球半径（米）
        
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2) * math.sin(dlat/2) + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
             math.sin(dlon/2) * math.sin(dlon/2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
    
    def destroy_node(self):
        """清理资源"""
        # 停止机器人
        self.stop()
        
        # 关闭串口
        if self.serial and self.serial.is_open:
            self.serial.close()
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()