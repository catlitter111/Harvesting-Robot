#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统测试脚本
用于测试各个节点的功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32
from bottle_detection_msgs.msg import RobotCommand, HarvestCommand
import json
import time
import threading


class SystemTester(Node):
    """系统测试节点"""
    
    def __init__(self):
        super().__init__('system_tester')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, 'robot/mode', 10)
        self.robot_cmd_pub = self.create_publisher(RobotCommand, 'robot/command', 10)
        self.harvest_cmd_pub = self.create_publisher(HarvestCommand, 'robot/harvest_command', 10)
        
        # 创建订阅者
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        self.count_sub = self.create_subscription(
            Int32,
            'bottle_detection/count',
            self.count_callback,
            10
        )
        
        self.info_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.info_callback,
            10
        )
        
        # 状态变量
        self.nearest_distance = None
        self.bottle_count = 0
        self.last_info = None
        
        self.get_logger().info('系统测试器已启动')
    
    def distance_callback(self, msg):
        """距离回调"""
        self.nearest_distance = msg.data
        if msg.data > 0:
            self.get_logger().info(f'最近瓶子距离: {msg.data:.2f}m')
    
    def count_callback(self, msg):
        """数量回调"""
        self.bottle_count = msg.data
        self.get_logger().info(f'检测到 {msg.data} 个瓶子')
    
    def info_callback(self, msg):
        """信息回调"""
        try:
            self.last_info = json.loads(msg.data)
        except:
            pass
    
    def test_manual_control(self):
        """测试手动控制"""
        self.get_logger().info('=== 测试手动控制 ===')
        
        # 设置手动模式
        mode_msg = String()
        mode_msg.data = json.dumps({
            "mode": "manual",
            "auto_harvest": False
        })
        self.mode_pub.publish(mode_msg)
        time.sleep(1)
        
        # 测试各个方向
        directions = [
            ("前进", 0.3, 0.0),
            ("后退", -0.3, 0.0),
            ("左转", 0.0, 0.5),
            ("右转", 0.0, -0.5),
            ("停止", 0.0, 0.0)
        ]
        
        for name, linear, angular in directions:
            self.get_logger().info(f'测试{name}...')
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)
            time.sleep(2)
        
        self.get_logger().info('手动控制测试完成')
    
    def test_auto_mode(self):
        """测试自动模式"""
        self.get_logger().info('=== 测试自动模式 ===')
        
        # 设置自动模式
        mode_msg = String()
        mode_msg.data = json.dumps({
            "mode": "auto",
            "auto_harvest": True
        })
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info('已切换到自动模式，观察10秒...')
        time.sleep(10)
        
        # 切回手动模式
        mode_msg.data = json.dumps({
            "mode": "manual",
            "auto_harvest": False
        })
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info('自动模式测试完成')
    
    def test_harvest(self):
        """测试采摘功能"""
        self.get_logger().info('=== 测试采摘功能 ===')
        
        # 发送采摘命令
        harvest_cmd = HarvestCommand()
        harvest_cmd.header.stamp = self.get_clock().now().to_msg()
        harvest_cmd.start_harvest = True
        
        self.get_logger().info('发送采摘命令...')
        self.harvest_cmd_pub.publish(harvest_cmd)
        
        # 等待采摘完成
        self.get_logger().info('等待采摘完成（约10秒）...')
        time.sleep(10)
        
        self.get_logger().info('采摘测试完成')
    
    def run_all_tests(self):
        """运行所有测试"""
        tests = [
            self.test_manual_control,
            self.test_auto_mode,
            self.test_harvest
        ]
        
        for test in tests:
            try:
                test()
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f'测试失败: {e}')
        
        self.get_logger().info('所有测试完成！')
        
        # 显示最终状态
        self.get_logger().info(f'最终状态:')
        self.get_logger().info(f'  - 检测到瓶子数: {self.bottle_count}')
        self.get_logger().info(f'  - 最近距离: {self.nearest_distance}')


def main(args=None):
    rclpy.init(args=args)
    
    tester = SystemTester()
    
    # 在单独线程运行测试
    test_thread = threading.Thread(target=tester.run_all_tests)
    test_thread.start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()