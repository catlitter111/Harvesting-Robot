#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动采摘控制器 - 重构版
基于四阶段状态机控制：远距离高速接近、近距离姿态校正、舵机对准、最后接近与采摘
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float32, Int32, Bool
from bottle_detection_msgs.msg import HarvestCommand, ServoCommand
import json
import time
import threading
from enum import Enum

# 核心参数定义
FAR_DISTANCE_THRESHOLD = 0.7    # 远距离阈值（米）
SERVO_ADJUST_THRESHOLD = 0.45    # 舵机调整距离阈值（米）
HARVEST_DISTANCE_THRESHOLD = 0.31# 采摘距离阈值（米）
ROBOT_FULL_SPEED = 0.3        # 小车全速（米/秒）
SERVO_ALIGNMENT_TIME = 2.0       # 舵机对准固定时间（秒）

# 图像中心死区（像素）
CENTER_DEADZONE = 30

# 控制模式
MODE_MANUAL = "manual"
MODE_AUTO = "auto"

# 最大可能距离
MAX_POSSIBLE_DISTANCE = 1.5  # 米

# 近距离控制速度
NEAR_APPROACH_SPEED = 0.05  # 近距离接近速度（米/秒）
NEAR_TURN_SPEED = 0.03   # 近距离转向速度（弧度/秒）


class HarvestState(Enum):
    """采摘状态枚举"""
    SEARCHING = "searching"                    # 搜索目标
    FAR_DISTANCE_APPROACH = "far_approach"     # 远距离高速接近
    NEAR_DISTANCE_ADJUST = "near_adjust"       # 近距离姿态校正
    SERVO_ALIGNMENT = "servo_align"            # 舵机精确对准
    FINAL_APPROACH = "final_approach"          # 最后接近
    HARVESTING = "harvesting"                  # 执行采摘


class AutoHarvestController(Node):
    """自动采摘控制器节点 - 重构版"""
    
    def __init__(self):
        super().__init__('auto_harvest_controller')
        
        # 声明参数
        self.declare_parameter('control_rate', 10.0)  # Hz
        self.declare_parameter('search_timeout', 5.0)  # 秒
        
        # 获取参数
        self.control_rate = float(self.get_parameter('control_rate').value or 10.0)
        self.search_timeout = float(self.get_parameter('search_timeout').value or 5.0)
        
        self.get_logger().info(
            f'自动采摘控制器参数:\n'
            f'  控制频率: {self.control_rate} Hz\n'
            f'  搜索超时: {self.search_timeout} 秒\n'
            f'  远距离阈值: {FAR_DISTANCE_THRESHOLD} m\n'
            f'  舵机调整阈值: {SERVO_ADJUST_THRESHOLD} m\n'
            f'  采摘距离阈值: {HARVEST_DISTANCE_THRESHOLD} m\n'
            f'  全速: {ROBOT_FULL_SPEED} m/s'
        )
        
        # 创建订阅者
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.detection_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        # 采摘状态订阅
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        harvest_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.harvest_status_sub = self.create_subscription(
            String,
            'harvest/status',
            self.harvest_status_callback,
            harvest_status_qos
        )
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        self.servo_cmd_pub = self.create_publisher(ServoCommand, 'servo/command', 10)
        self.tracking_pub = self.create_publisher(Point, 'servo/tracking_target', 10)
        self.bottle_crop_request_pub = self.create_publisher(String, 'bottle_detection/crop_request', 10)
        
        # 状态变量
        self.current_mode = MODE_AUTO
        self.auto_harvest_active = True
        self.bottle_visible = False
        self.nearest_distance = None
        self.bottle_cx = 0
        self.bottle_cy = 0
        self.bottle_xmin = 0
        self.bottle_ymin = 0
        self.bottle_xmax = 0
        self.bottle_ymax = 0
        self.bottle_width = 0
        self.bottle_height = 0
        self.bottle_area = 0
        self.frame_width = 640
        self.frame_height = 480
        self.last_detection_time = time.time()
        
        # 状态机变量
        self.harvest_state = HarvestState.SEARCHING
        self.state_start_time = 0.0
        self.far_approach_start_time = 0.0
        self.far_approach_duration = 0.0
        self.servo_alignment_start_time = 0.0
        self.harvest_in_progress = False
        
        # 控制锁
        self.control_lock = threading.Lock()
        
        # 创建控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        self.get_logger().info('自动采摘控制器（重构版）已启动')
    
    def mode_callback(self, msg):
        """模式更新回调"""
        try:
            data = json.loads(msg.data)
            self.current_mode = data.get("mode", MODE_AUTO)
            self.auto_harvest_active = data.get("auto_harvest", True)
            
            self.get_logger().info(
                f'模式更新: {self.current_mode}, '
                f'自动采摘: {self.auto_harvest_active}'
            )
            
            # 切换模式时停止运动并重置状态
            if self.current_mode == MODE_MANUAL or not self.auto_harvest_active:
                self.stop_robot()
                self.reset_state_machine()
                
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
                    
                    # 获取边界框信息
                    bbox = bottle_info.get("bbox", [0, 0, 0, 0])
                    if len(bbox) >= 4:
                        self.bottle_xmin = bbox[0]
                        self.bottle_ymin = bbox[1] 
                        self.bottle_xmax = bbox[2]
                        self.bottle_ymax = bbox[3]
                        self.bottle_width = self.bottle_xmax - self.bottle_xmin
                        self.bottle_height = self.bottle_ymax - self.bottle_ymin
                        self.bottle_area = self.bottle_width * self.bottle_height
                    
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
            
            if state == "completed":
                self.harvest_in_progress = False
                self.harvest_state = HarvestState.SEARCHING
                self.get_logger().info('采摘完成，返回搜索状态')
            elif state == "started":
                self.harvest_in_progress = True
                self.get_logger().info('开始采摘操作')
            elif state == "failed":
                self.harvest_in_progress = False
                self.harvest_state = HarvestState.SEARCHING
                self.get_logger().warn('采摘失败，返回搜索状态')
                
        except Exception as e:
            self.get_logger().error(f'处理采摘状态时发生错误: {e}')
    
    def control_loop(self):
        """主控制循环 - 基于状态机"""
        # 只在自动模式且激活采摘时执行
        if self.current_mode != MODE_AUTO or not self.auto_harvest_active:
            return
        
        with self.control_lock:
            current_time = time.time()
            
            # 根据当前状态执行相应控制
            if self.harvest_state == HarvestState.SEARCHING:
                self.handle_searching_state(current_time)
            elif self.harvest_state == HarvestState.FAR_DISTANCE_APPROACH:
                self.handle_far_approach_state(current_time)
            elif self.harvest_state == HarvestState.NEAR_DISTANCE_ADJUST:
                self.handle_near_adjust_state()
            elif self.harvest_state == HarvestState.SERVO_ALIGNMENT:
                self.handle_servo_alignment_state(current_time)
            elif self.harvest_state == HarvestState.FINAL_APPROACH:
                self.handle_final_approach_state()
            elif self.harvest_state == HarvestState.HARVESTING:
                self.handle_harvesting_state()
    
    def handle_searching_state(self, current_time):
        """处理搜索状态"""
        # 检查是否超时未检测到瓶子
        if current_time - self.last_detection_time > self.search_timeout:
            self.search_for_bottle()
            return
        
        # 如果检测到瓶子且距离有效
        if self.bottle_visible and self.nearest_distance is not None:
            # 检查距离值是否合理
            if self.nearest_distance > MAX_POSSIBLE_DISTANCE:
                self.get_logger().warn(f'检测到异常距离值: {self.nearest_distance}m, 忽略')
                return
            
            # 根据距离进入相应状态
            if self.nearest_distance > FAR_DISTANCE_THRESHOLD:
                self.transition_to_state(HarvestState.FAR_DISTANCE_APPROACH)
            elif self.nearest_distance > SERVO_ADJUST_THRESHOLD:
                self.transition_to_state(HarvestState.NEAR_DISTANCE_ADJUST)
            elif self.nearest_distance > HARVEST_DISTANCE_THRESHOLD:
                self.transition_to_state(HarvestState.SERVO_ALIGNMENT)
            else:
                self.transition_to_state(HarvestState.HARVESTING)
        else:
            # 没有检测到瓶子，停止
            self.stop_robot()
    
    def handle_far_approach_state(self, current_time):
        """处理远距离高速接近状态"""
        if self.far_approach_start_time == 0.0:
            # 检查距离值是否有效
            if self.nearest_distance is None:
                self.get_logger().warn('远距离接近: 距离值无效，返回搜索状态')
                self.transition_to_state(HarvestState.SEARCHING)
                return
            
            # 计算前进时间
            distance_to_travel = self.nearest_distance - FAR_DISTANCE_THRESHOLD
            self.far_approach_duration = (distance_to_travel / ROBOT_FULL_SPEED )/2
            self.far_approach_start_time = current_time
            
            self.get_logger().info(
                f'开始远距离高速接近: 当前距离={self.nearest_distance:.2f}m, '
                f'前进距离={distance_to_travel:.2f}m, '
                f'预计时间={self.far_approach_duration:.2f}s'
            )
        
        # 检查是否完成
        elapsed_time = current_time - self.far_approach_start_time
        if elapsed_time >= self.far_approach_duration:
            # 完成远距离接近，停止并重新评估
            self.stop_robot()
            self.far_approach_start_time = 0.0
            
            # 重新检测距离，决定下一个状态
            if self.nearest_distance is not None:
                if self.nearest_distance > FAR_DISTANCE_THRESHOLD:
                    # 仍然很远，继续远距离接近
                    self.get_logger().info(f'重新评估: 距离={self.nearest_distance:.2f}m, 继续远距离接近')
                elif self.nearest_distance > SERVO_ADJUST_THRESHOLD:
                    self.transition_to_state(HarvestState.NEAR_DISTANCE_ADJUST)
                else:
                    self.transition_to_state(HarvestState.SERVO_ALIGNMENT)
            else:
                self.transition_to_state(HarvestState.SEARCHING)
        else:
            # 继续全速前进
            twist = Twist()
            twist.linear.x = ROBOT_FULL_SPEED
            twist.angular.z = 0.0
            
            remaining_time = self.far_approach_duration - elapsed_time
            extra_info = f"已用时={elapsed_time:.1f}s, 剩余={remaining_time:.1f}s"
            self.publish_cmd_vel_with_debug(twist, "远距离高速接近", extra_info)
    
    def handle_near_adjust_state(self):
        """处理近距离姿态校正状态"""
        # 计算偏移
        center_x = self.frame_width // 2 + 50
        offset_x = center_x - self.bottle_cx
        
        twist = Twist()
        
        # 使用近距离控制算法
        if abs(offset_x) > CENTER_DEADZONE:
            # 需要转向调整
            if offset_x > 0:
                twist.angular.z = NEAR_TURN_SPEED
                extra_info = f"左转调整，偏移={offset_x}px"
            else:
                twist.angular.z = -NEAR_TURN_SPEED
                extra_info = f"右转调整，偏移={offset_x}px"
        else:
            # 基本对准，缓慢前进
            twist.linear.x = NEAR_APPROACH_SPEED
            extra_info = f"基本对准，缓慢前进，偏移={offset_x}px"
        
        self.publish_cmd_vel_with_debug(twist, "近距离姿态校正", extra_info)
        
        # 检查是否到达舵机调整距离
        if self.nearest_distance is not None and self.nearest_distance <= SERVO_ADJUST_THRESHOLD:
            self.transition_to_state(HarvestState.SERVO_ALIGNMENT)
    
    def handle_servo_alignment_state(self, current_time):
        """处理舵机精确对准状态"""
        if self.servo_alignment_start_time == 0.0:
            # 开始舵机对准
            self.stop_robot()
            self.servo_alignment_start_time = current_time
            self.get_logger().info(f'到达舵机调整距离({SERVO_ADJUST_THRESHOLD}m)，开始舵机对准')
        
        # 发送舵机跟踪命令
        tracking_msg = Point()
        tracking_msg.x = float(370)
        tracking_msg.y = float(self.bottle_cy)
        tracking_msg.z = float(self.frame_width)
        self.tracking_pub.publish(tracking_msg)
        
        # 检查是否完成2秒对准时间
        elapsed_time = current_time - self.servo_alignment_start_time
        if elapsed_time >= SERVO_ALIGNMENT_TIME:
            self.get_logger().info(f'舵机对准完成（用时{SERVO_ALIGNMENT_TIME}s），进入最后接近阶段')
            self.servo_alignment_start_time = 0.0
            
            # 检查当前距离，决定下一步
            if self.nearest_distance is not None:
                if self.nearest_distance > HARVEST_DISTANCE_THRESHOLD:
                    self.transition_to_state(HarvestState.FINAL_APPROACH)
                else:
                    self.transition_to_state(HarvestState.HARVESTING)
            else:
                self.transition_to_state(HarvestState.SEARCHING)
        else:
            # 显示对准进度
            remaining_time = SERVO_ALIGNMENT_TIME - elapsed_time
            if int(elapsed_time * 10) % 5 == 0:  # 每0.5秒输出一次
                self.get_logger().debug(f'舵机对准中: {elapsed_time:.1f}s / {SERVO_ALIGNMENT_TIME}s')
    
    def handle_final_approach_state(self):
        """处理最后接近状态（舵机保持不动）"""
        # 计算偏移，但只用于判断是否对准
        center_x = self.frame_width // 2 + 50
        offset_x = center_x - self.bottle_cx
        
        twist = Twist()
        
        # 舵机在此阶段保持不动，只有小车移动
        if abs(offset_x) <= CENTER_DEADZONE * 1.5:  # 允许更大的偏差
            # 缓慢前进
            twist.linear.x = NEAR_APPROACH_SPEED * 0.5  # 更慢的速度
            extra_info = f"舵机保持，缓慢前进，偏移={offset_x}px"
        else:
            # 偏差太大，小幅调整方向
            if offset_x > 0:
                twist.angular.z = NEAR_TURN_SPEED 
                extra_info = f"舵机保持，微调左转，偏移={offset_x}px"
            else:
                twist.angular.z = -NEAR_TURN_SPEED 
                extra_info = f"舵机保持，微调右转，偏移={offset_x}px"
        
        self.publish_cmd_vel_with_debug(twist, "最后接近阶段", extra_info)
        
        # 检查是否到达采摘距离
        if self.nearest_distance is not None and self.nearest_distance <= HARVEST_DISTANCE_THRESHOLD:
            self.transition_to_state(HarvestState.HARVESTING)
    
    def handle_harvesting_state(self):
        """处理采摘状态"""
        # 停止移动
        self.stop_robot()
        
        if not self.harvest_in_progress:
            self.get_logger().info(f'到达采摘距离({self.nearest_distance:.3f}m)，开始采摘')
            
            # 请求截取瓶子图像
            self.request_bottle_crop()
            
            # 发送采摘命令
            harvest_cmd = HarvestCommand()
            harvest_cmd.header.stamp = self.get_clock().now().to_msg()
            harvest_cmd.start_harvest = True
            self.harvest_cmd_pub.publish(harvest_cmd)
            self.harvest_in_progress = True
    
    def transition_to_state(self, new_state):
        """状态转换"""
        if self.harvest_state != new_state:
            self.get_logger().info(f'状态转换: {self.harvest_state.value} -> {new_state.value}')
            self.harvest_state = new_state
            self.state_start_time = time.time()
    
    def search_for_bottle(self):
        """搜索瓶子"""
        twist = Twist()
        twist.angular.z = 0.15  # 慢速旋转
        timeout_elapsed = time.time() - self.last_detection_time
        extra_info = f"旋转寻找目标，超时时间={timeout_elapsed:.1f}s"
        self.publish_cmd_vel_with_debug(twist, "搜索模式", extra_info)
    
    def request_bottle_crop(self):
        """请求检测节点截取瓶子图像"""
        try:
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
                'timestamp': time.time()
            }
            
            request_msg = String()
            request_msg.data = json.dumps(crop_request)
            self.bottle_crop_request_pub.publish(request_msg)
            
            self.get_logger().info('发送瓶子截取请求')
            return True
            
        except Exception as e:
            self.get_logger().error(f'发送瓶子截取请求时出错: {e}')
            return False
    
    def reset_state_machine(self):
        """重置状态机"""
        self.harvest_state = HarvestState.SEARCHING
        self.state_start_time = 0.0
        self.far_approach_start_time = 0.0
        self.far_approach_duration = 0.0
        self.servo_alignment_start_time = 0.0
        self.harvest_in_progress = False
        self.get_logger().info('状态机已重置')
    
    def publish_cmd_vel_with_debug(self, twist, context="", extra_info=""):
        """发布电机命令并输出调试信息"""
        # 发布命令
        self.cmd_vel_pub.publish(twist)
        
        # 构建调试信息
        debug_msg = f"[电机命令] {context}"
        if extra_info:
            debug_msg += f" | {extra_info}"
        
        # 速度信息
        linear_speed = twist.linear.x
        angular_speed = twist.angular.z
        
        if linear_speed != 0 or angular_speed != 0:
            debug_msg += f" | 线速度: {linear_speed:.3f} m/s"
            debug_msg += f" | 角速度: {angular_speed:.3f} rad/s"
            
            # 运动方向描述
            if linear_speed > 0:
                debug_msg += " | 前进"
            elif linear_speed < 0:
                debug_msg += " | 后退"
                
            if angular_speed > 0:
                debug_msg += " | 左转"
            elif angular_speed < 0:
                debug_msg += " | 右转"
        else:
            debug_msg += " | 停止"
        
        # 当前状态信息
        debug_msg += f" | 状态: {self.harvest_state.value}"
        
        # 距离信息
        if self.nearest_distance is not None:
            debug_msg += f" | 距离: {self.nearest_distance:.3f}m"
        
        # 瓶子位置信息
        if self.bottle_visible:
            center_x = self.frame_width // 2 + 80
            offset_x = center_x - self.bottle_cx
            debug_msg += f" | 瓶子位置: ({self.bottle_cx}, {self.bottle_cy})"
            debug_msg += f" | 偏移: {offset_x}px"
        
        self.get_logger().info(debug_msg)
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_cmd_vel_with_debug(twist, "停止机器人")
    
    def destroy_node(self):
        """清理资源"""
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = AutoHarvestController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()