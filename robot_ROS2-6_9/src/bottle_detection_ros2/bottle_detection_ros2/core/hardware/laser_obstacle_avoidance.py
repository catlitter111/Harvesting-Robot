#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达避障控制器
整合到AgriSage3系统中，与自动采摘控制器协同工作
提供智能避障功能，确保机器人安全作业
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32, Int32, Bool
import numpy as np
import json
import time
import threading
import math

# 避障参数
DEFAULT_RESPONSE_DISTANCE = 0.8  # 响应距离（米）
DEFAULT_DANGER_DISTANCE = 0.4    # 危险距离（米）
DEFAULT_EMERGENCY_DISTANCE = 0.2 # 紧急距离（米）

# 角度扇区定义（度）
FRONT_ANGLE = 30    # 前方扇区：±30度
SIDE_ANGLE = 60     # 侧面扇区：30-60度

# 避障动作类型
ACTION_NONE = 0
ACTION_SLOW_DOWN = 1
ACTION_TURN_LEFT = 2
ACTION_TURN_RIGHT = 3
ACTION_EMERGENCY_STOP = 4
ACTION_REVERSE = 5

# 控制模式
MODE_MANUAL = "manual"
MODE_AUTO = "auto"


class LaserObstacleAvoidance(Node):
    """激光雷达避障控制器"""
    
    def __init__(self):
        super().__init__('laser_obstacle_avoidance')
        
        # 声明参数
        self._declare_parameters()
        self._get_parameters()
        
        # 创建订阅者
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        self.cmd_vel_input_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',  # 原始速度指令
            self.cmd_vel_input_callback,
            10
        )
        
        # 创建发布者
        self.cmd_vel_output_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.obstacle_info_pub = self.create_publisher(String, 'obstacle/info', 10)
        self.avoidance_status_pub = self.create_publisher(String, 'avoidance/status', 10)
        
        # 状态变量
        self.current_mode = MODE_MANUAL
        self.avoidance_enabled = True
        self.emergency_stop = False
        
        # 激光数据
        self.laser_data = None
        self.obstacle_detected = False
        
        # 扇区障碍物计数
        self.front_obstacles = 0
        self.left_front_obstacles = 0
        self.right_front_obstacles = 0
        self.left_side_obstacles = 0
        self.right_side_obstacles = 0
        
        # 最近障碍物距离
        self.nearest_distance = float('inf')
        self.nearest_angle = 0
        
        # 输入的速度指令
        self.input_cmd_vel = Twist()
        self.last_input_time = time.time()
        
        # 避障状态
        self.avoidance_action = ACTION_NONE
        self.avoidance_start_time = 0
        self.avoidance_duration = 0
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 创建定时器
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info(
            f'激光雷达避障控制器已启动\n'
            f'响应距离: {self.response_distance}m\n'
            f'危险距离: {self.danger_distance}m\n'
            f'紧急距离: {self.emergency_distance}m\n'
            f'避障开关: {self.avoidance_enabled}'
        )
    
    def _declare_parameters(self):
        """声明ROS2参数"""
        self.declare_parameter('avoidance_enabled', True)
        self.declare_parameter('response_distance', DEFAULT_RESPONSE_DISTANCE)
        self.declare_parameter('danger_distance', DEFAULT_DANGER_DISTANCE)
        self.declare_parameter('emergency_distance', DEFAULT_EMERGENCY_DISTANCE)
        self.declare_parameter('front_angle', float(FRONT_ANGLE))
        self.declare_parameter('side_angle', float(SIDE_ANGLE))
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('slow_down_factor', 0.3)
        self.declare_parameter('obstacle_count_threshold', 5)
        self.declare_parameter('cmd_timeout', 2.0)
    
    def _get_parameters(self):
        """获取参数"""
        self.avoidance_enabled = self.get_parameter('avoidance_enabled').value
        self.response_distance = self.get_parameter('response_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        self.front_angle = self.get_parameter('front_angle').value
        self.side_angle = self.get_parameter('side_angle').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.slow_down_factor = self.get_parameter('slow_down_factor').value
        self.obstacle_count_threshold = self.get_parameter('obstacle_count_threshold').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
    
    def laser_callback(self, msg):
        """激光雷达数据回调"""
        with self.lock:
            self.laser_data = msg
            self._analyze_obstacles(msg)
    
    def _analyze_obstacles(self, scan):
        """分析障碍物分布"""
        if not scan.ranges:
            return
            
        # 重置计数
        self.front_obstacles = 0
        self.left_front_obstacles = 0
        self.right_front_obstacles = 0
        self.left_side_obstacles = 0
        self.right_side_obstacles = 0
        
        self.nearest_distance = float('inf')
        self.nearest_angle = 0
        
        ranges = np.array(scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)]
        
        if len(valid_ranges) == 0:
            self.obstacle_detected = False
            return
        
        # 分析每个激光点
        for i, distance in enumerate(ranges):
            if not np.isfinite(distance) or distance <= scan.range_min or distance >= scan.range_max:
                continue
                
            # 计算角度（转换为度）
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)
            
            # 标准化角度到-180到180度
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            
            # 检查是否在响应距离内
            if distance < self.response_distance:
                # 分类到不同扇区
                if abs(angle_deg) <= self.front_angle:
                    self.front_obstacles += 1
                elif self.front_angle < angle_deg <= self.side_angle:
                    self.left_front_obstacles += 1
                elif -self.side_angle <= angle_deg < -self.front_angle:
                    self.right_front_obstacles += 1
                elif self.side_angle < angle_deg <= 90:
                    self.left_side_obstacles += 1
                elif -90 <= angle_deg < -self.side_angle:
                    self.right_side_obstacles += 1
            
            # 更新最近障碍物
            if distance < self.nearest_distance:
                self.nearest_distance = distance
                self.nearest_angle = angle_deg
        
        # 判断是否检测到障碍物
        total_obstacles = (self.front_obstacles + self.left_front_obstacles + 
                          self.right_front_obstacles + self.left_side_obstacles + 
                          self.right_side_obstacles)
        self.obstacle_detected = total_obstacles > 0
        
        # 发布障碍物信息
        self._publish_obstacle_info()
    
    def _publish_obstacle_info(self):
        """发布障碍物信息"""
        info = {
            'obstacle_detected': self.obstacle_detected,
            'nearest_distance': float(self.nearest_distance) if self.nearest_distance != float('inf') else -1,
            'nearest_angle': float(self.nearest_angle),
            'front_obstacles': self.front_obstacles,
            'left_front_obstacles': self.left_front_obstacles,
            'right_front_obstacles': self.right_front_obstacles,
            'left_side_obstacles': self.left_side_obstacles,
            'right_side_obstacles': self.right_side_obstacles,
            'avoidance_action': self.avoidance_action,
            'emergency_stop': self.emergency_stop
        }
        
        msg = String()
        msg.data = json.dumps(info, ensure_ascii=False)
        self.obstacle_info_pub.publish(msg)
    
    def mode_callback(self, msg):
        """模式更新回调"""
        try:
            data = json.loads(msg.data)
            self.current_mode = data.get("mode", MODE_MANUAL)
            self.get_logger().info(f'避障控制器模式更新: {self.current_mode}')
        except Exception as e:
            self.get_logger().error(f'解析模式数据错误: {e}')
    
    def cmd_vel_input_callback(self, msg):
        """原始速度指令回调"""
        with self.lock:
            self.input_cmd_vel = msg
            self.last_input_time = time.time()
    
    def control_loop(self):
        """主控制循环"""
        if not self.avoidance_enabled:
            # 避障关闭，直接转发原始指令
            self.cmd_vel_output_pub.publish(self.input_cmd_vel)
            return
        
        with self.lock:
            # 检查输入指令超时
            current_time = time.time()
            if current_time - self.last_input_time > self.cmd_timeout:
                # 超时，发送停止指令
                stop_cmd = Twist()
                self.cmd_vel_output_pub.publish(stop_cmd)
                return
            
            # 计算避障后的速度指令
            output_cmd = self._compute_avoidance_command()
            self.cmd_vel_output_pub.publish(output_cmd)
            
            # 发布避障状态
            self._publish_avoidance_status()
    
    def _compute_avoidance_command(self):
        """计算避障后的速度指令"""
        output_cmd = Twist()
        
        # 如果没有激光数据，停止
        if self.laser_data is None:
            return output_cmd
        
        # 检查紧急停止条件
        if self.nearest_distance < self.emergency_distance:
            self.emergency_stop = True
            self.avoidance_action = ACTION_EMERGENCY_STOP
            self.get_logger().warn(f'紧急停止！最近障碍物距离: {self.nearest_distance:.2f}m')
            return output_cmd
        
        self.emergency_stop = False
        
        # 决策避障动作
        action = self._decide_avoidance_action()
        
        if action == ACTION_NONE:
            # 无障碍物，直接使用原始指令
            output_cmd = self.input_cmd_vel
        elif action == ACTION_SLOW_DOWN:
            # 减速前进
            output_cmd.linear.x = self.input_cmd_vel.linear.x * self.slow_down_factor
            output_cmd.angular.z = self.input_cmd_vel.angular.z
        elif action == ACTION_TURN_LEFT:
            # 左转避障
            output_cmd.linear.x = max(0, self.input_cmd_vel.linear.x * 0.5)
            output_cmd.angular.z = self.max_angular_speed * 0.8
        elif action == ACTION_TURN_RIGHT:
            # 右转避障
            output_cmd.linear.x = max(0, self.input_cmd_vel.linear.x * 0.5)
            output_cmd.angular.z = -self.max_angular_speed * 0.8
        elif action == ACTION_REVERSE:
            # 后退
            output_cmd.linear.x = -self.max_linear_speed * 0.3
            output_cmd.angular.z = 0
        
        # 限制速度
        output_cmd.linear.x = max(-self.max_linear_speed, 
                                 min(self.max_linear_speed, output_cmd.linear.x))
        output_cmd.angular.z = max(-self.max_angular_speed, 
                                  min(self.max_angular_speed, output_cmd.angular.z))
        
        self.avoidance_action = action
        return output_cmd
    
    def _decide_avoidance_action(self):
        """决策避障动作"""
        # 如果没有障碍物，无需避障
        if not self.obstacle_detected:
            return ACTION_NONE
        
        # 检查前方障碍物
        if self.front_obstacles > self.obstacle_count_threshold:
            if self.nearest_distance < self.danger_distance:
                # 前方危险，选择转向
                if self.left_front_obstacles < self.right_front_obstacles:
                    return ACTION_TURN_LEFT
                else:
                    return ACTION_TURN_RIGHT
            else:
                # 前方有障碍物但不危险，减速
                return ACTION_SLOW_DOWN
        
        # 检查侧前方障碍物
        if self.left_front_obstacles > self.obstacle_count_threshold:
            if self.nearest_distance < self.danger_distance:
                return ACTION_TURN_RIGHT
            else:
                return ACTION_SLOW_DOWN
        
        if self.right_front_obstacles > self.obstacle_count_threshold:
            if self.nearest_distance < self.danger_distance:
                return ACTION_TURN_LEFT
            else:
                return ACTION_SLOW_DOWN
        
        # 检查侧面障碍物
        if (self.left_side_obstacles > self.obstacle_count_threshold and 
            self.right_side_obstacles > self.obstacle_count_threshold and
            self.nearest_distance < self.danger_distance):
            # 两侧都有障碍物，后退
            return ACTION_REVERSE
        
        return ACTION_NONE
    
    def _publish_avoidance_status(self):
        """发布避障状态"""
        action_names = {
            ACTION_NONE: "无避障",
            ACTION_SLOW_DOWN: "减速",
            ACTION_TURN_LEFT: "左转",
            ACTION_TURN_RIGHT: "右转",
            ACTION_EMERGENCY_STOP: "紧急停止",
            ACTION_REVERSE: "后退"
        }
        
        status = {
            'avoidance_enabled': self.avoidance_enabled,
            'current_action': action_names.get(self.avoidance_action, "未知"),
            'emergency_stop': self.emergency_stop,
            'nearest_distance': float(self.nearest_distance) if self.nearest_distance != float('inf') else -1,
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.avoidance_status_pub.publish(msg)
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('激光雷达避障控制器正在关闭...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LaserObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'避障控制器运行错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 