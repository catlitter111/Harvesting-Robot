#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达避障集成测试脚本
用于验证避障功能是否正确集成到AgriSage3系统中
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import time
import threading


class LidarIntegrationTest(Node):
    """激光雷达集成测试节点"""
    
    def __init__(self):
        super().__init__('lidar_integration_test')
        
        # 测试状态
        self.test_results = {}
        self.start_time = time.time()
        
        # 订阅话题
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        self.obstacle_sub = self.create_subscription(
            String, 'obstacle/info', self.obstacle_callback, 10)
        
        self.avoidance_sub = self.create_subscription(
            String, 'avoidance/status', self.avoidance_callback, 10)
        
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # 发布测试指令
        self.cmd_vel_raw_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        
        # 状态变量
        self.laser_data_received = False
        self.obstacle_info_received = False
        self.avoidance_status_received = False
        self.cmd_vel_raw_received = False
        self.cmd_vel_received = False
        
        self.latest_laser_time = None
        self.latest_obstacle_info = None
        self.latest_avoidance_status = None
        
        # 创建测试定时器
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.command_timer = self.create_timer(2.0, self.send_test_commands)
        
        self.get_logger().info('激光雷达集成测试开始...')
        self.get_logger().info('测试将持续30秒')
    
    def laser_callback(self, msg):
        """激光雷达数据回调"""
        self.laser_data_received = True
        self.latest_laser_time = time.time()
        
        # 简单检查数据质量
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        self.test_results['laser_data_quality'] = len(valid_ranges) / len(msg.ranges) if msg.ranges else 0
    
    def obstacle_callback(self, msg):
        """障碍物信息回调"""
        self.obstacle_info_received = True
        try:
            self.latest_obstacle_info = json.loads(msg.data)
            self.test_results['obstacle_detection'] = True
        except Exception as e:
            self.get_logger().error(f'解析障碍物信息失败: {e}')
            self.test_results['obstacle_detection'] = False
    
    def avoidance_callback(self, msg):
        """避障状态回调"""
        self.avoidance_status_received = True
        try:
            self.latest_avoidance_status = json.loads(msg.data)
            self.test_results['avoidance_working'] = True
        except Exception as e:
            self.get_logger().error(f'解析避障状态失败: {e}')
            self.test_results['avoidance_working'] = False
    
    def cmd_vel_raw_callback(self, msg):
        """原始速度指令回调"""
        self.cmd_vel_raw_received = True
        self.test_results['cmd_vel_raw_flow'] = True
    
    def cmd_vel_callback(self, msg):
        """最终速度指令回调"""
        self.cmd_vel_received = True
        self.test_results['cmd_vel_output'] = True
    
    def send_test_commands(self):
        """发送测试指令"""
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.1
        self.cmd_vel_raw_pub.publish(twist)
    
    def run_tests(self):
        """运行测试检查"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 基础连接测试
        tests = {
            '激光雷达数据接收': self.laser_data_received,
            '障碍物信息接收': self.obstacle_info_received,
            '避障状态接收': self.avoidance_status_received,
            '原始指令接收': self.cmd_vel_raw_received,
            '最终指令输出': self.cmd_vel_received,
        }
        
        # 打印测试进度
        print(f"\n{'='*50}")
        print(f"测试进度: {elapsed_time:.1f}/30.0 秒")
        print(f"{'='*50}")
        
        for test_name, result in tests.items():
            status = "✓ 通过" if result else "✗ 失败"
            print(f"{test_name:<20}: {status}")
        
        # 详细信息
        if self.latest_obstacle_info:
            print(f"\n障碍物检测:")
            print(f"  检测到障碍物: {self.latest_obstacle_info.get('obstacle_detected', 'N/A')}")
            print(f"  最近距离: {self.latest_obstacle_info.get('nearest_distance', 'N/A'):.2f}m")
            print(f"  前方障碍物: {self.latest_obstacle_info.get('front_obstacles', 'N/A')} 个")
        
        if self.latest_avoidance_status:
            print(f"\n避障状态:")
            print(f"  当前动作: {self.latest_avoidance_status.get('current_action', 'N/A')}")
            print(f"  紧急停止: {self.latest_avoidance_status.get('emergency_stop', 'N/A')}")
        
        # 数据质量检查
        if 'laser_data_quality' in self.test_results:
            quality = self.test_results['laser_data_quality']
            print(f"\n数据质量:")
            print(f"  激光数据有效率: {quality:.1%}")
            
        # 30秒后结束测试
        if elapsed_time >= 30:
            self.print_final_results()
            rclpy.shutdown()
    
    def print_final_results(self):
        """打印最终测试结果"""
        print(f"\n{'='*60}")
        print("最终测试结果")
        print(f"{'='*60}")
        
        # 核心功能测试
        core_tests = [
            ('激光雷达数据流', self.laser_data_received),
            ('避障控制器运行', self.avoidance_status_received),
            ('指令流转正常', self.cmd_vel_raw_received and self.cmd_vel_received),
            ('障碍物检测功能', self.obstacle_info_received),
        ]
        
        passed = 0
        total = len(core_tests)
        
        for test_name, result in core_tests:
            status = "✓ 通过" if result else "✗ 失败"
            color = "\033[92m" if result else "\033[91m"  # 绿色/红色
            print(f"{color}{test_name:<25}: {status}\033[0m")
            if result:
                passed += 1
        
        print(f"\n总体结果: {passed}/{total} 项测试通过")
        
        if passed == total:
            print("\033[92m🎉 所有测试通过！激光雷达避障功能集成成功！\033[0m")
        else:
            print(f"\033[91m⚠️  有 {total-passed} 项测试失败，请检查系统配置\033[0m")
        
        # 建议
        print(f"\n建议:")
        if not self.laser_data_received:
            print("- 检查激光雷达连接和驱动节点")
        if not self.avoidance_status_received:
            print("- 检查避障控制器节点是否启动")
        if not (self.cmd_vel_raw_received and self.cmd_vel_received):
            print("- 检查话题重定向配置")
        
        print("\n详细文档请参考: LIDAR_INTEGRATION_README.md")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = LidarIntegrationTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 