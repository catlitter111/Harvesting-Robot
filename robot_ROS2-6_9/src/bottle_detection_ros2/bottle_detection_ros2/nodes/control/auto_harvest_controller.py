#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动采摘控制器
整合瓶子检测、机器人移动和舵机控制，实现自动采摘功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float32, Int32, Bool
from bottle_detection_msgs.msg import HarvestCommand, ServoCommand
import json
import time
import threading

# 距离阈值（米）
DISTANCE_VERY_FAR = 0.7   # 超远距离阈值，超过此距离只进行大角度调整
DISTANCE_FAR = 0.7       # 远距离阈值，超过此距离使用电机调整方向
DISTANCE_NEAR = 0.35       # 近距离阈值，低于此距离使用舵机调整方向
DISTANCE_HARVEST = 0.30   # 采摘距离阈值，低于此距离开始采摘


# 图像中心死区（像素）
CENTER_DEADZONE = 80

# 控制模式
MODE_MANUAL = "manual"
MODE_AUTO = "auto"

# 最大可能距离
MAX_POSSIBLE_DISTANCE = 15.0  # 米，与检测器保持一致

# 默认速度参数（优化后）
DEFAULT_APPROACH_SPEED = 60.0  # 提高默认接近速度
DEFAULT_TURN_SPEED = 50.0      # 提高默认转向速度
DEFAULT_FINE_APPROACH_SPEED =50.0  # 提高精细接近速度
DEFAULT_FINE_TURN_SPEED = 50.0      # 提高精细转向速度

# 全速前进参数
FULL_SPEED = 0.3  # m/s，根据测试数据：1秒60.5cm，2秒120cm，平均0.6m/s
SAFETY_DISTANCE = 0.1  # 安全距离，米


class AutoHarvestController(Node):
    """自动采摘控制器节点"""
    
    def __init__(self):
        super().__init__('auto_harvest_controller')
        
        # 声明参数 - 确保所有参数都是浮点数（优化默认值）
        self.declare_parameter('control_rate', 10.0)  # Hz
        self.declare_parameter('search_timeout', 5.0)  # 秒
        self.declare_parameter('approach_speed', DEFAULT_APPROACH_SPEED)  # 百分比 (0-100)
        self.declare_parameter('turn_speed', DEFAULT_TURN_SPEED)  # 百分比 (0-100)
        self.declare_parameter('fine_approach_speed', DEFAULT_FINE_APPROACH_SPEED)  # 百分比 (0-100)
        self.declare_parameter('fine_turn_speed', DEFAULT_FINE_TURN_SPEED)  # 百分比 (0-100)
        
        # 获取参数，提供默认值避免None
        self.control_rate = float(self.get_parameter('control_rate').value or 10.0)
        self.search_timeout = float(self.get_parameter('search_timeout').value or 5.0)
        self.approach_speed = float(self.get_parameter('approach_speed').value or DEFAULT_APPROACH_SPEED)
        self.turn_speed = float(self.get_parameter('turn_speed').value or DEFAULT_TURN_SPEED)
        self.fine_approach_speed = float(self.get_parameter('fine_approach_speed').value or DEFAULT_FINE_APPROACH_SPEED)
        self.fine_turn_speed = float(self.get_parameter('fine_turn_speed').value or DEFAULT_FINE_TURN_SPEED)
        

        
        # 参数验证和修正
        # 如果参数值小于1，可能是速度值而不是百分比，进行转换
        if self.approach_speed < 1.0:
            self.approach_speed = self.approach_speed * 100.0
            self.get_logger().warn(f'approach_speed 参数已从速度值转换为百分比: {self.approach_speed}%')
        
        if self.turn_speed < 1.0:
            self.turn_speed = self.turn_speed * 100.0
            self.get_logger().warn(f'turn_speed 参数已从速度值转换为百分比: {self.turn_speed}%')
        
        if self.fine_approach_speed < 1.0:
            self.fine_approach_speed = self.fine_approach_speed * 100.0
            self.get_logger().warn(f'fine_approach_speed 参数已从速度值转换为百分比: {self.fine_approach_speed}%')
        
        if self.fine_turn_speed < 1.0:
            self.fine_turn_speed = self.fine_turn_speed * 100.0
            self.get_logger().warn(f'fine_turn_speed 参数已从速度值转换为百分比: {self.fine_turn_speed}%')
        
        # 限制参数范围
        self.approach_speed = max(0.0, min(100.0, self.approach_speed))
        self.turn_speed = max(0.0, min(100.0, self.turn_speed))
        self.fine_approach_speed = max(0.0, min(100.0, self.fine_approach_speed))
        self.fine_turn_speed = max(0.0, min(100.0, self.fine_turn_speed))
        
        self.get_logger().info(
            f'自动采摘控制器参数:\n'
            f'  控制频率: {self.control_rate} Hz\n'
            f'  搜索超时: {self.search_timeout} 秒\n'
            f'  接近速度: {self.approach_speed}%\n'
            f'  转向速度: {self.turn_speed}%\n'
            f'  精细接近速度: {self.fine_approach_speed}%\n'
            f'  精细转向速度: {self.fine_turn_speed}%'
        )
        
        # 创建订阅者
        # 模式控制
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        # 瓶子检测信息
        self.detection_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_callback,
            10
        )
        
        # 最近瓶子距离
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        # 采摘状态 - 修复QoS兼容性问题
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        harvest_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 使用BEST_EFFORT以提高兼容性
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.harvest_status_sub = self.create_subscription(
            String,
            'harvest/status',
            self.harvest_status_callback,
            harvest_status_qos
        )
        

        
        # 创建发布者 - 直接发布到cmd_vel话题（修复自动控制不动问题）
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # 直接发布到cmd_vel，与手动控制一致
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        self.servo_cmd_pub = self.create_publisher(ServoCommand, 'servo/command', 10)
        self.tracking_pub = self.create_publisher(Point, 'servo/tracking_target', 10)
        
        # 瓶子截取请求发布者 - 通知检测节点截取瓶子图像
        self.bottle_crop_request_pub = self.create_publisher(
            String,
            'bottle_detection/crop_request',
            10
        )
        
        # 状态变量 - 修改默认模式为自动
        self.current_mode = MODE_AUTO
        self.auto_harvest_active = True
        self.bottle_visible = False
        self.nearest_distance = None
        self.bottle_cx = 0
        self.bottle_cy = 0
        # 边界框信息变量
        self.bottle_xmin = 0
        self.bottle_ymin = 0
        self.bottle_xmax = 0
        self.bottle_ymax = 0
        self.bottle_width = 0
        self.bottle_height = 0
        self.bottle_area = 0
        self.frame_width = 640
        self.frame_height = 480
        self.harvest_in_progress = False
        self.last_detection_time = time.time()
        self.searching = False
        
        # 全速前进状态变量
        self.full_speed_mode = False  # 是否启用全速前进模式
        self.full_speed_started = False  # 是否已开始全速前进
        self.full_speed_start_time = 0.0  # 全速前进开始时间
        self.full_speed_duration = 0.0  # 计算出的前进时间
        self.initial_target_distance = None  # 首次检测到的目标距离
        
        # 当前运动状态
        self.current_direction = 0x04  # DIR_STOP
        self.current_speed = 50
        
        # 近距离采摘状态管理
        self.near_distance_state = "adjusting"  # adjusting: 机械臂调整中, moving: 缓慢前进中
        self.arm_adjustment_start_time = 0.0
        self.arm_adjustment_timeout = 3.0  # 机械臂调整超时时间（秒）
        self.center_tolerance_x = CENTER_DEADZONE  # X方向中心容忍度
        self.center_tolerance_y = 20  # Y方向中心容忍度（像素）
        self._last_distance_state = None  # 跟踪上一次的距离状态
        
        # 控制锁
        self.control_lock = threading.Lock()
        
        # 创建控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,  # 根据控制频率计算定时器周期
            self.control_loop
        )
        
        self.get_logger().info('自动采摘控制器已启动')
    
    def mode_callback(self, msg):
        """模式更新回调"""
        try:
            data = json.loads(msg.data)
            self.current_mode = data.get("mode", MODE_AUTO)  # 修改默认值为自动模式
            self.auto_harvest_active = data.get("auto_harvest", True)  # 修改默认值为启用自动采摘
            
            self.get_logger().info(
                f'模式更新: {self.current_mode}, '
                f'自动采摘: {self.auto_harvest_active}'
            )
            
            # 切换模式时停止运动并重置状态
            if self.current_mode == MODE_MANUAL or not self.auto_harvest_active:
                self.stop_robot()
                self.reset_full_speed_mode()
                self.reset_near_distance_state()
                self._last_distance_state = None
                
        except Exception as e:
            self.get_logger().error(f'解析模式数据错误: {e}')
    
    def detection_callback(self, msg):
        """瓶子检测信息回调"""
        try:
            data = json.loads(msg.data)
            
            with self.control_lock:
                self.bottle_visible = data.get("bottle_detected", False)
                
                if self.bottle_visible and "nearest_bottle" in data:
                    bottle_info = data["nearest_bottle"]
                    self.bottle_cx = bottle_info.get("pixel_x", 0)
                    self.bottle_cy = bottle_info.get("pixel_y", 0)
                    self.nearest_distance = bottle_info.get("distance", None)
                    
                    # 获取边界框信息进行精准控制
                    bbox = bottle_info.get("bbox", [0, 0, 0, 0])  # [xmin, ymin, xmax, ymax]
                    if len(bbox) >= 4:
                        self.bottle_xmin = bbox[0]
                        self.bottle_ymin = bbox[1] 
                        self.bottle_xmax = bbox[2]
                        self.bottle_ymax = bbox[3]
                        
                        # 计算边界框宽度和高度，用于判断瓶子大小
                        self.bottle_width = self.bottle_xmax - self.bottle_xmin
                        self.bottle_height = self.bottle_ymax - self.bottle_ymin
                        self.bottle_area = self.bottle_width * self.bottle_height
                        
                        self.get_logger().debug(
                            f'边界框信息: [{self.bottle_xmin},{self.bottle_ymin},{self.bottle_xmax},{self.bottle_ymax}], '
                            f'尺寸: {self.bottle_width}x{self.bottle_height}, 面积: {self.bottle_area}'
                        )
                    else:
                        # 如果没有边界框信息，使用默认值
                        self.bottle_xmin = self.bottle_cx - 50
                        self.bottle_ymin = self.bottle_cy - 50
                        self.bottle_xmax = self.bottle_cx + 50
                        self.bottle_ymax = self.bottle_cy + 50
                        self.bottle_width = 100
                        self.bottle_height = 100
                        self.bottle_area = 10000
                    
                    self.last_detection_time = time.time()
                    
        except Exception as e:
            self.get_logger().error(f'解析检测数据错误: {e}')
    
    def distance_callback(self, msg):
        """距离更新回调"""
        if msg.data > 0:
            with self.control_lock:
                self.nearest_distance = msg.data
    
    def harvest_status_callback(self, msg):
        """采摘状态回调"""
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            
            self.get_logger().debug(f'收到采摘状态: {state}')
            
            if state == "completed":
                self.harvest_in_progress = False
                self.get_logger().info('采摘完成，继续搜索下一个目标')
            elif state == "started":
                self.harvest_in_progress = True
                self.get_logger().info('开始采摘操作')
            elif state == "failed":
                self.harvest_in_progress = False
                self.get_logger().warn('采摘失败，继续搜索下一个目标')
            elif state == "aborted":
                self.harvest_in_progress = False
                self.get_logger().warn('采摘被中止')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'采摘状态JSON解析错误: {e}')
            self.get_logger().debug(f'原始消息: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'处理采摘状态时发生未知错误: {e}')
    

    
    def request_bottle_crop(self):
        """请求检测节点截取瓶子图像"""
        try:
            # 构建截取请求消息，包含瓶子的边界框信息
            crop_request = {
                'action': 'crop_bottle',
                'bbox': {
                    'xmin': int(self.bottle_xmin),
                    'ymin': int(self.bottle_ymin), 
                    'xmax': int(self.bottle_xmax),
                    'ymax': int(self.bottle_ymax)
                },
                'center': {
                    'x': int(self.bottle_cx),
                    'y': int(self.bottle_cy)
                },
                'timestamp': time.time(),
                'request_id': f'harvest_{int(time.time()*1000)}'
            }
            
            # 发布截取请求
            request_msg = String()
            request_msg.data = json.dumps(crop_request)
            self.bottle_crop_request_pub.publish(request_msg)
            
            self.get_logger().info(
                f'发送瓶子截取请求: 边界框({self.bottle_xmin},{self.bottle_ymin},{self.bottle_xmax},{self.bottle_ymax}), '
                f'中心点({self.bottle_cx},{self.bottle_cy})'
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'发送瓶子截取请求时出错: {e}')
            return False
    
    def control_loop(self):
        """主控制循环"""
        # 手动模式下不执行自动跟踪，机械臂由用户手动控制
        # if self.current_mode == MODE_MANUAL and self.bottle_visible:
        #     self._manual_servo_control()
        #     return
        
        # 只在自动模式且激活采摘时执行
        if self.current_mode != MODE_AUTO or not self.auto_harvest_active:
            return
        
        # 如果正在采摘，不进行移动控制
        if self.harvest_in_progress:
            return
        
        with self.control_lock:
            current_time = time.time()
            
            # 检查是否超时未检测到瓶子
            if current_time - self.last_detection_time > self.search_timeout:
                if not self.searching:
                    self.get_logger().info('超时未检测到瓶子，开始搜索')
                    self.searching = True
                self.search_for_bottle()
                return
            
            # 如果检测到瓶子
            if self.bottle_visible and self.nearest_distance is not None:
                self.searching = False
                # 检查距离值是否合理
                if self.nearest_distance > MAX_POSSIBLE_DISTANCE:
                    self.get_logger().warn(f'检测到异常距离值: {self.nearest_distance}m, 忽略此次控制')
                    return
                
                # 全速前进控制逻辑
                if self.full_speed_mode:
                    self.full_speed_control()
                else:
                    self.approach_bottle()
            else:
                # 没有检测到瓶子，停止
                self.stop_robot()
    
    def _manual_servo_control(self):
        """手动模式下的舵机控制 - 已禁用，机械臂现在不会在手动模式下自动跟踪物品"""
        # 此方法已被禁用，手动模式下机械臂不再自动跟踪
        # 发布跟踪目标
        # tracking_msg = Point()
        # tracking_msg.x = float(self.bottle_cx)
        # tracking_msg.y = float(self.bottle_cy)
        # tracking_msg.z = float(self.frame_width)  # 传递图像宽度
        # 
        # self.tracking_pub.publish(tracking_msg)
        # self.get_logger().debug(f"手动模式舵机跟踪: 坐标=({self.bottle_cx},{self.bottle_cy})")
        
        self.get_logger().debug("手动模式：机械臂自动跟踪已禁用")
    
    def approach_bottle(self):
        """接近瓶子的控制逻辑"""
        # 首次检测到有效距离时启动全速前进模式
        if self.initial_target_distance is None and self.nearest_distance is not None and self.nearest_distance > 0:
            self.initial_target_distance = self.nearest_distance
            self.full_speed_mode = True
            self.full_speed_started = False
            
            # 根据距离范围计算前进时间
            if self.nearest_distance is not None and self.nearest_distance > DISTANCE_VERY_FAR:
                # 如果距离大于超远距离阈值，前进到远距离阈值处
                effective_distance = max(0.1, self.nearest_distance - DISTANCE_FAR)
                self.full_speed_duration = effective_distance / FULL_SPEED
                self.get_logger().info(
                    f'超远距离模式: 目标距离={self.nearest_distance:.3f}m > {DISTANCE_VERY_FAR}m, '
                    f'前进到{DISTANCE_FAR}m处, 有效距离={effective_distance:.3f}m, '
                    f'计算前进时间: {self.full_speed_duration:.2f}秒'
                )
            else:
                # 在DISTANCE_FAR范围内，执行原来的逻辑
                effective_distance = max(0.1, self.nearest_distance - SAFETY_DISTANCE)
                self.full_speed_duration = effective_distance / FULL_SPEED
                self.get_logger().info(
                    f'常规距离模式: 目标距离={self.nearest_distance:.3f}m <= {DISTANCE_VERY_FAR}m, '
                    f'计算前进时间: {self.full_speed_duration:.2f}秒 '
                    f'(有效距离: {effective_distance:.3f}m, 安全距离: {SAFETY_DISTANCE}m)'
                )
            return
        
        # 原有的分距离控制逻辑（作为备用）
        # 计算偏移 - 与舵机控制保持一致
        center_x = self.frame_width // 2 + 80  # 与servo_control_node保持一致的偏移
        offset_x = center_x - self.bottle_cx
        
        # 根据距离选择控制策略
        if self.nearest_distance is not None:
            if self.nearest_distance > DISTANCE_VERY_FAR:
                # 超远距离：快速接近，大角度调整
                self.get_logger().info(f'距离状态: 超远距离 (>{DISTANCE_VERY_FAR}m)')
                self._last_distance_state = "very_far"
                self.approach_very_far(offset_x)
            elif self.nearest_distance > DISTANCE_FAR:
                # 远距离：使用电机移动
                self.get_logger().info(f'距离状态: 远距离 ({DISTANCE_FAR}m-{DISTANCE_VERY_FAR}m)')
                self._last_distance_state = "far"
                self.approach_far(offset_x)
            elif self.nearest_distance > DISTANCE_NEAR:
                # 中等距离：精细控制
                self.get_logger().info(f'距离状态: 中等距离 ({DISTANCE_NEAR}m-{DISTANCE_FAR}m)')
                self._last_distance_state = "medium"
                self.approach_medium(offset_x)
            elif self.nearest_distance > DISTANCE_HARVEST:
                # 近距离：使用舵机跟踪
                self.get_logger().info(f'距离状态: 近距离 ({DISTANCE_HARVEST}m-{DISTANCE_NEAR}m)')
                # 确保首次进入近距离时重置状态
                if hasattr(self, '_last_distance_state') and self._last_distance_state != "near":
                    self.reset_near_distance_state()
                self._last_distance_state = "near"
                self.approach_near(offset_x)
            else:
                # 采摘距离：停止并采摘
                self.get_logger().info(f'距离状态: 采摘距离 (<{DISTANCE_HARVEST}m)')
                self._last_distance_state = "harvest"
                self.stop_and_harvest(offset_x)
    
    def full_speed_control(self):
        """全速前进控制方法"""
        current_time = time.time()
        
        # 如果还没开始全速前进，开始计时
        if not self.full_speed_started:
            self.full_speed_started = True
            self.full_speed_start_time = current_time
            
            # 开始全速前进
            twist = Twist()
            twist.linear.x = FULL_SPEED
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info(f'开始全速前进: 速度={FULL_SPEED}m/s, 持续时间={self.full_speed_duration:.2f}秒')
            return
        
        # 检查是否到达预定时间
        elapsed_time = current_time - self.full_speed_start_time
        if elapsed_time >= self.full_speed_duration:
            # 时间到达，立即停止
            self.stop_robot()
            self.full_speed_mode = False
            self.full_speed_started = False
            
            self.get_logger().info(
                f'全速前进完成: 实际用时={elapsed_time:.2f}秒, '
                f'预计前进距离={elapsed_time * FULL_SPEED:.3f}m'
            )
            
            # 可以选择继续后续控制逻辑或者直接停止
            # 这里选择停止，等待下一次目标检测
            return
        
        # 继续全速前进
        twist = Twist()
        twist.linear.x = FULL_SPEED
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # 定期输出进度信息
        if int(elapsed_time * 10) % 5 == 0:  # 每0.5秒输出一次
            remaining_time = self.full_speed_duration - elapsed_time
            self.get_logger().info(f'全速前进中: 已用时={elapsed_time:.1f}s, 剩余={remaining_time:.1f}s')

    def approach_very_far(self, offset_x):
        """超远距离接近策略"""
        twist = Twist()
        
        # 超远距离时，放宽居中要求，使用更大的死区
        large_deadzone = CENTER_DEADZONE * 3
        
        if abs(offset_x) > large_deadzone:
            # 需要大角度调整
            if offset_x > 0:
                # 瓶子在左边，向左转
                twist.angular.z = 0.8  # rad/s，更快的转向速度
                self.current_direction = 0x02  # DIR_LEFT
                self.get_logger().info(f'超远距离：瓶子在左侧，快速向左转（偏移:{offset_x}px）')
            else:
                # 瓶子在右边，向右转
                twist.angular.z = -0.8  # rad/s
                self.current_direction = 0x03  # DIR_RIGHT
                self.get_logger().info(f'超远距离：瓶子在右侧，快速向右转（偏移:{offset_x}px）')
            
            # 应用转向速度百分比
            twist.angular.z = twist.angular.z * self.turn_speed / 100.0
        else:
            # 瓶子大致居中，快速前进
            base_speed = 0.6  # m/s，基础速度提高
            # 应用接近速度百分比
            twist.linear.x = base_speed * self.approach_speed / 100.0
            self.current_direction = 0x00  # DIR_FORWARD
            self.get_logger().info(f'超远距离：瓶子居中，快速前进，速度={twist.linear.x:.2f}m/s（{self.approach_speed}%）')
        
        self.cmd_vel_pub.publish(twist)

    def approach_far(self, offset_x):
        """远距离接近策略"""
        twist = Twist()
        
        # 大偏移时先转向
        if abs(offset_x) > CENTER_DEADZONE * 2:
            if offset_x > 0:
                # 瓶子在左边，向左转
                twist.angular.z = 0.5  # rad/s
                self.current_direction = 0x02  # DIR_LEFT
                self.get_logger().info('远距离：瓶子在左侧，向左转')
            else:
                # 瓶子在右边，向右转
                twist.angular.z = -0.5  # rad/s
                self.current_direction = 0x03  # DIR_RIGHT
                self.get_logger().info('远距离：瓶子在右侧，向右转')
            
            # 应用转向速度百分比
            twist.angular.z = twist.angular.z * self.turn_speed / 100.0
        else:
            # 瓶子基本居中，前进
            base_speed = 0.4  # m/s，基础速度提高
            # 应用接近速度百分比
            twist.linear.x = base_speed * self.approach_speed / 100.0
            self.current_direction = 0x00  # DIR_FORWARD
            self.get_logger().info(f'远距离：瓶子居中，前进，速度={twist.linear.x:.2f}m/s（{self.approach_speed}%）')
        
        self.cmd_vel_pub.publish(twist)
    
    def approach_medium(self, offset_x):
        """中等距离接近策略"""
        twist = Twist()
        
        # 更精细的控制
        if abs(offset_x) > CENTER_DEADZONE:
            if offset_x > 0:
                twist.angular.z = 0.5  # rad/s
                self.current_direction = 0x02  # DIR_LEFT
                self.get_logger().info('中等距离：瓶子在左侧，向左微调')
            else:
                twist.angular.z = -0.5  # rad/s
                self.current_direction = 0x03  # DIR_RIGHT
                self.get_logger().info('中等距离：瓶子在右侧，向右微调')
            
            # 应用精细转向速度百分比
            twist.angular.z = twist.angular.z * self.fine_turn_speed / 100.0
        else:
            base_speed = 0.25  # m/s，基础速度提高
            # 应用精细接近速度百分比
            twist.linear.x = base_speed * self.fine_approach_speed / 100.0
            self.current_direction = 0x00  # DIR_FORWARD
            self.get_logger().info(f'中等距离：瓶子居中，缓慢前进，速度={twist.linear.x:.2f}m/s（{self.fine_approach_speed}%）')
        
        self.cmd_vel_pub.publish(twist)
    
    def approach_near(self, offset_x):
        """近距离接近策略 - 状态机控制：先机械臂调整，再电机缓慢前进"""
        twist = Twist()
        current_time = time.time()
        
        # 重新计算画面中心偏移（X和Y方向）- 与舵机控制保持一致
        center_x = self.frame_width // 2 + 50  # 与servo_control_node保持一致的偏移
        center_y = self.frame_height // 2 + 50
        offset_x_center = center_x - self.bottle_cx
        offset_y_center = center_y - self.bottle_cy
        
        # 检查是否在画面中心（X和Y都要满足）
        is_centered = (abs(offset_y_center) <= self.center_tolerance_y)
        
        # 始终发送舵机跟踪命令
        tracking_msg = Point()
        tracking_msg.x = float(center_x)
        tracking_msg.y = float(self.bottle_cy)
        tracking_msg.z = float(self.frame_width)
        self.tracking_pub.publish(tracking_msg)
        
        if self.near_distance_state == "adjusting":
            # 状态1：机械臂调整中，电机停止
            if self.arm_adjustment_start_time == 0.0:
                self.arm_adjustment_start_time = current_time
                self.get_logger().info('近距离：开始机械臂调整阶段，电机停止')
            
            # 电机停止
            self.current_direction = 0x04  # DIR_STOP
            
            # 检查是否已对准中心
            if is_centered:
                # 给机械臂一些调整时间（至少1秒）
                adjustment_time = current_time - self.arm_adjustment_start_time
                if adjustment_time >= 1.0:
                    self.near_distance_state = "moving"
                    self.get_logger().info(
                        f'近距离：机械臂调整完成（用时{adjustment_time:.1f}s），'
                        f'目标已居中（X偏移:{offset_x_center}px, Y偏移:{offset_y_center}px），开始缓慢前进'
                    )
                else:
                    self.get_logger().info(
                        f'近距离：目标已居中，等待机械臂稳定（{1.0-adjustment_time:.1f}s后开始移动）'
                    )
            else:
                # 检查调整超时
                adjustment_time = current_time - self.arm_adjustment_start_time
                if adjustment_time > self.arm_adjustment_timeout:
                    self.get_logger().warn(
                        f'近距离：机械臂调整超时（{adjustment_time:.1f}s），'
                        f'当前偏移（X:{offset_x_center}px, Y:{offset_y_center}px），强制进入移动状态'
                    )
                    self.near_distance_state = "moving"
                else:
                    self.get_logger().info(
                        f'近距离：机械臂调整中（X偏移:{offset_x_center}px, Y偏移:{offset_y_center}px）'
                    )
            
        elif self.near_distance_state == "moving":
            # 状态2：确认对准后缓慢前进
            if is_centered:
                # 目标仍在中心，继续缓慢前进
                base_speed = 0.03  # m/s，非常慢的速度
                twist.linear.x = base_speed
                self.current_direction = 0x00  # DIR_FORWARD
                self.get_logger().info(
                    f'近距离：目标居中，缓慢前进到采摘位置，'
                    f'速度={twist.linear.x:.2f}m/s（X偏移:{offset_x_center}px, Y偏移:{offset_y_center}px）'
                )
            else:
                # 目标偏离中心，退回机械臂调整模式
                self.near_distance_state = "adjusting"
                self.arm_adjustment_start_time = 0.0  # 重置调整时间
                self.current_direction = 0x04  # DIR_STOP
                self.get_logger().info(
                    f'近距离：目标偏离中心，退回机械臂调整模式（X偏移:{offset_x_center}px, Y偏移:{offset_y_center}px）'
                )
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_and_harvest(self, offset_x):
        """停止并执行采摘 - 优化版：增加容错机制"""
        # 停止移动
        self.stop_robot()
        self.current_direction = 0x04  # DIR_STOP
        
        # 放宽对准要求，在采摘距离时允许更大的偏差
        harvest_deadzone = CENTER_DEADZONE * 1.5  # 采摘时允许的偏差更大
        
        if abs(offset_x) < harvest_deadzone:
            if not self.harvest_in_progress:
                self.get_logger().info(f'到达采摘距离 {self.nearest_distance:.3f}m，开始采摘（偏移:{offset_x}px）')
                
                # 先确保舵机对准目标
                tracking_msg = Point()
                tracking_msg.x = float(self.bottle_cx)
                tracking_msg.y = float(self.bottle_cy)
                tracking_msg.z = float(self.frame_width)
                self.tracking_pub.publish(tracking_msg)
                
                # 等待舵机调整完成（可选：添加小延时）
                # time.sleep(0.5)
                
                # 🆕 请求检测节点截取瓶子图像并发布给AI识别系统
                self.get_logger().info('请求检测节点截取瓶子图像进行AI识别...')
                request_success = self.request_bottle_crop()
                if request_success:
                    self.get_logger().info('瓶子截取请求发送成功，检测节点将处理图像截取')
                else:
                    self.get_logger().warn('瓶子截取请求发送失败，仍继续采摘流程')
                
                # 发送采摘命令
                harvest_cmd = HarvestCommand()
                harvest_cmd.header.stamp = self.get_clock().now().to_msg()
                harvest_cmd.start_harvest = True
                self.harvest_cmd_pub.publish(harvest_cmd)
                self.harvest_in_progress = True
        else:
            # 使用舵机微调对准
            self.get_logger().info(f'采摘距离但未对准，继续调整舵机（偏移:{offset_x}px）')
            tracking_msg = Point()
            tracking_msg.x = float(self.bottle_cx)
            tracking_msg.y = float(self.bottle_cy)
            tracking_msg.z = float(self.frame_width)
            self.tracking_pub.publish(tracking_msg)
    
    def search_for_bottle(self):
        """搜索瓶子"""
        # 简单的旋转搜索策略
        twist = Twist()
        twist.angular.z = 0.10  # 慢速旋转
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug('搜索模式：旋转寻找目标')
    
    def reset_full_speed_mode(self):
        """重置全速前进模式状态"""
        self.full_speed_mode = False
        self.full_speed_started = False
        self.full_speed_start_time = 0.0
        self.full_speed_duration = 0.0
        self.initial_target_distance = None
        self.get_logger().debug('全速前进模式状态已重置')
    
    def reset_near_distance_state(self):
        """重置近距离状态机"""
        self.near_distance_state = "adjusting"
        self.arm_adjustment_start_time = 0.0
        self.get_logger().info('近距离状态机已重置为调整模式')
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def destroy_node(self):
        """清理资源"""
        # 停止机器人
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None  # 预先初始化变量
    try:
        node = AutoHarvestController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if node is not None:  # 检查node是否已创建
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()