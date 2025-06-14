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
DISTANCE_FAR = 0.6      # 远距离阈值，超过此距离使用电机调整方向
DISTANCE_NEAR = 0.5    # 近距离阈值，低于此距离使用舵机调整方向
DISTANCE_HARVEST = 0.35 # 采摘距离阈值，低于此距离开始采摘
DISTANCE_STOP = 0.5    # 停止距离

# 图像中心死区（像素）
CENTER_DEADZONE = 80

# 控制模式
MODE_MANUAL = "manual"
MODE_AUTO = "auto"

# 最大可能距离
MAX_POSSIBLE_DISTANCE = 10.0  # 米


class AutoHarvestController(Node):
    """自动采摘控制器节点"""
    
    def __init__(self):
        super().__init__('auto_harvest_controller')
        
        # 声明参数 - 确保所有参数都是浮点数
        self.declare_parameter('control_rate', 10.0)  # Hz
        self.declare_parameter('search_timeout', 5.0)  # 秒
        self.declare_parameter('approach_speed', 30.0)  # 百分比 (0-100)
        self.declare_parameter('turn_speed', 20.0)  # 百分比 (0-100)
        self.declare_parameter('fine_approach_speed', 10.0)  # 百分比 (0-100)
        self.declare_parameter('fine_turn_speed', 15.0)  # 百分比 (0-100)
        
        # 获取参数
        self.control_rate = self.get_parameter('control_rate').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.fine_approach_speed = self.get_parameter('fine_approach_speed').value
        self.fine_turn_speed = self.get_parameter('fine_turn_speed').value
        
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
        
        # 采摘状态
        self.harvest_status_sub = self.create_subscription(
            String,
            'harvest/status',
            self.harvest_status_callback,
            10
        )
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)  # 发布到原始话题，由避障控制器处理
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        self.servo_cmd_pub = self.create_publisher(ServoCommand, 'servo/command', 10)
        self.tracking_pub = self.create_publisher(Point, 'servo/tracking_target', 10)
        
        # 采摘图像截取请求发布者
        self.harvest_capture_pub = self.create_publisher(String, 'harvest/capture_request', 10)
        
        # 状态变量
        self.current_mode = MODE_MANUAL
        self.auto_harvest_active = False
        self.bottle_visible = False
        self.nearest_distance = None
        self.bottle_cx = 0
        self.bottle_cy = 0
        self.frame_width = 640
        self.frame_height = 480
        self.harvest_in_progress = False
        self.last_detection_time = time.time()
        self.searching = False
        
        # 采摘图像截取相关
        self.harvest_session_id = None
        self.target_item_index = 0
        self.harvest_image_captured = False
        
        # 当前运动状态
        self.current_direction = 0x04  # DIR_STOP
        self.current_speed = 50
        
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
            self.current_mode = data.get("mode", MODE_MANUAL)
            self.auto_harvest_active = data.get("auto_harvest", False)
            
            self.get_logger().info(
                f'模式更新: {self.current_mode}, '
                f'自动采摘: {self.auto_harvest_active}'
            )
            
            # 切换模式时停止运动
            if self.current_mode == MODE_MANUAL or not self.auto_harvest_active:
                self.stop_robot()
                
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
            status = data.get("status", "")
            
            self.get_logger().info(f"收到采摘状态更新: state={state}, status={status}")
            
            # 处理原有的状态逻辑
            if state == "completed":
                self.harvest_in_progress = False
                self.get_logger().info('采摘完成，继续搜索下一个目标')
                # 重置图像截取状态
                self.harvest_image_captured = False
                self.harvest_session_id = None
            elif state == "started":
                self.harvest_in_progress = True
            
            # 处理新的状态以触发图像截取
            if status == "gripping" and not self.harvest_image_captured:
                # 正在夹取，截取确认图像
                self.request_harvest_image_capture('gripping')
                
            elif status == "confirming":
                # 确认夹取成功，截取最终图像
                self.request_harvest_image_capture('confirming')
                self.harvest_image_captured = True
                
            elif status == "completed":
                # 采摘完成，重置状态
                self.harvest_in_progress = False
                self.harvest_image_captured = False
                self.harvest_session_id = None
                self.get_logger().info("采摘完成，状态已重置")
                
            elif status == "failed":
                # 采摘失败，重置状态
                self.harvest_in_progress = False
                self.harvest_image_captured = False
                self.harvest_session_id = None
                self.get_logger().info("采摘失败，状态已重置")
                
        except json.JSONDecodeError:
            self.get_logger().error("解析采摘状态JSON失败")
        except Exception as e:
            self.get_logger().error(f'解析采摘状态错误: {e}')
    
    def control_loop(self):
        """主控制循环"""
        # 手动模式下的舵机跟踪
        if self.current_mode == MODE_MANUAL and self.bottle_visible:
            self._manual_servo_control()
            return
        
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
                self.approach_bottle()
            else:
                # 没有检测到瓶子，停止
                self.stop_robot()
    
    def _manual_servo_control(self):
        """手动模式下的舵机控制 - 检测到瓶子时直接控制舵机跟踪，不考虑距离"""
        # 发布跟踪目标
        tracking_msg = Point()
        tracking_msg.x = float(self.bottle_cx)
        tracking_msg.y = float(self.bottle_cy)
        tracking_msg.z = float(self.frame_width)  # 传递图像宽度
        
        self.tracking_pub.publish(tracking_msg)
        self.get_logger().debug(f"手动模式舵机跟踪: 坐标=({self.bottle_cx},{self.bottle_cy})")
    
    def approach_bottle(self):
        """接近瓶子的控制逻辑"""
        # 计算偏移
        center_x = self.frame_width // 2
        offset_x = center_x - self.bottle_cx
        
        # 添加调试日志
        self.get_logger().info(
            f'瓶子距离: {self.nearest_distance:.3f}m, '
            f'像素偏移: {offset_x}px, '
            f'坐标: ({self.bottle_cx}, {self.bottle_cy})'
        )
        
        # 根据距离选择控制策略
        if self.nearest_distance > DISTANCE_FAR:
            # 远距离：使用电机移动
            self.get_logger().info(f'距离状态: 远距离 (>{DISTANCE_FAR}m)')
            self.approach_far(offset_x)
        elif self.nearest_distance > DISTANCE_NEAR:
            # 中等距离：精细控制
            self.get_logger().info(f'距离状态: 中等距离 ({DISTANCE_NEAR}m-{DISTANCE_FAR}m)')
            self.approach_medium(offset_x)
        elif self.nearest_distance > DISTANCE_HARVEST:
            # 近距离：使用舵机跟踪
            self.get_logger().info(f'距离状态: 近距离 ({DISTANCE_HARVEST}m-{DISTANCE_NEAR}m)')
            self.approach_near(offset_x)
        else:
            # 采摘距离：停止并采摘
            self.get_logger().info(f'距离状态: 采摘距离 (<{DISTANCE_HARVEST}m)')
            self.stop_and_harvest(offset_x)
    
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
            twist.linear.x = 0.3  # m/s
            # 应用接近速度百分比
            twist.linear.x = twist.linear.x * self.approach_speed / 100.0
            self.current_direction = 0x00  # DIR_FORWARD
            self.get_logger().info(f'远距离：瓶子居中，前进，速度={twist.linear.x:.2f}m/s')
        
        self.cmd_vel_pub.publish(twist)
    
    def approach_medium(self, offset_x):
        """中等距离接近策略"""
        twist = Twist()
        
        # 更精细的控制
        if abs(offset_x) > CENTER_DEADZONE:
            if offset_x > 0:
                twist.angular.z = 0.3  # rad/s
                self.current_direction = 0x02  # DIR_LEFT
                self.get_logger().info('中等距离：瓶子在左侧，向左微调')
            else:
                twist.angular.z = -0.3  # rad/s
                self.current_direction = 0x03  # DIR_RIGHT
                self.get_logger().info('中等距离：瓶子在右侧，向右微调')
            
            # 应用精细转向速度百分比
            twist.angular.z = twist.angular.z * self.fine_turn_speed / 100.0
        else:
            twist.linear.x = 0.1  # m/s
            # 应用精细接近速度百分比
            twist.linear.x = twist.linear.x * self.fine_approach_speed / 100.0
            self.current_direction = 0x00  # DIR_FORWARD
            self.get_logger().info(f'中等距离：瓶子居中，缓慢前进，速度={twist.linear.x:.2f}m/s')
        
        self.cmd_vel_pub.publish(twist)
    
    def approach_near(self, offset_x):
        """近距离接近策略"""
        # 停止移动
        self.stop_robot()
        self.current_direction = 0x04  # DIR_STOP
        self.get_logger().info('近距离：停止车辆，使用舵机微调')
        
        # 使用舵机进行跟踪
        tracking_msg = Point()
        tracking_msg.x = float(self.bottle_cx)
        tracking_msg.y = float(self.bottle_cy)
        tracking_msg.z = float(self.frame_width)  # 传递图像宽度
        
        self.tracking_pub.publish(tracking_msg)
    
    def stop_and_harvest(self, offset_x):
        """停止并执行采摘"""
        # 停止移动
        self.stop_robot()
        self.current_direction = 0x04  # DIR_STOP
        
        # 检查是否对准
        if abs(offset_x) < CENTER_DEADZONE:
            if not self.harvest_in_progress:
                self.get_logger().info('开始采摘')
                
                # 生成采摘会话ID
                import uuid
                self.harvest_session_id = str(uuid.uuid4())[:8]
                
                # 发送采摘命令
                harvest_cmd = HarvestCommand()
                harvest_cmd.header.stamp = self.get_clock().now().to_msg()
                harvest_cmd.start_harvest = True
                self.harvest_cmd_pub.publish(harvest_cmd)
                self.harvest_in_progress = True
                
                # 请求截取采摘图像
                self.request_harvest_image_capture('approaching')
        else:
            # 使用舵机微调对准
            tracking_msg = Point()
            tracking_msg.x = float(self.bottle_cx)
            tracking_msg.y = float(self.bottle_cy)
            tracking_msg.z = float(self.frame_width)
            self.tracking_pub.publish(tracking_msg)
    
    def search_for_bottle(self):
        """搜索瓶子"""
        # 简单的旋转搜索策略
        twist = Twist()
        twist.angular.z = 0.25  # 慢速旋转
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug('搜索模式：旋转寻找目标')
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def request_harvest_image_capture(self, harvest_status):
        """请求截取采摘图像"""
        try:
            request_data = {
                'harvest_session_id': self.harvest_session_id or '',
                'target_item_index': self.target_item_index,
                'harvest_status': harvest_status,
                'timestamp': int(time.time() * 1000),
                'robot_id': 'robot_123'  # 可以从参数获取
            }
            
            request_msg = String()
            request_msg.data = json.dumps(request_data)
            self.harvest_capture_pub.publish(request_msg)
            
            self.get_logger().info(f"已发送图像截取请求 - 状态: {harvest_status}, "
                                 f"会话ID: {self.harvest_session_id}")
                                 
        except Exception as e:
            self.get_logger().error(f"发送图像截取请求失败: {e}")
    
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