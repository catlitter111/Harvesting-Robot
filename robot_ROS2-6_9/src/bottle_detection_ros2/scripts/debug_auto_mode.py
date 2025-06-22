#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动模式调试脚本
监控自动采摘系统的各个组件状态，帮助诊断问题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist, Point
from bottle_detection_msgs.msg import HarvestCommand, ServoCommand
import json
import time
import sys


class AutoModeDebugger(Node):
    """自动模式调试器"""
    
    def __init__(self):
        super().__init__('auto_mode_debugger')
        
        # 状态变量
        self.bottle_info = None
        self.nearest_distance = None
        self.robot_mode = None
        self.cmd_vel = None
        self.harvest_command = None
        self.last_update_time = time.time()
        
        # 创建订阅者监控各个话题
        self.bottle_info_sub = self.create_subscription(
            String,
            'bottle_detection/info',
            self.bottle_info_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float32,
            'bottle_detection/nearest_distance',
            self.distance_callback,
            10
        )
        
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_callback,
            10
        )
        
        self.harvest_cmd_sub = self.create_subscription(
            HarvestCommand,
            'robot/harvest_command',
            self.harvest_cmd_callback,
            10
        )
        
        # 创建定时器定期打印状态
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('自动模式调试器已启动')
        self.get_logger().info('监控话题:')
        self.get_logger().info('  - bottle_detection/info')
        self.get_logger().info('  - bottle_detection/nearest_distance')
        self.get_logger().info('  - robot/mode')
        self.get_logger().info('  - cmd_vel_raw')
        self.get_logger().info('  - robot/harvest_command')
    
    def bottle_info_callback(self, msg):
        """瓶子信息回调"""
        try:
            self.bottle_info = json.loads(msg.data)
            self.last_update_time = time.time()
        except Exception as e:
            self.get_logger().error(f'解析瓶子信息错误: {e}')
    
    def distance_callback(self, msg):
        """距离回调"""
        self.nearest_distance = msg.data
        self.last_update_time = time.time()
    
    def mode_callback(self, msg):
        """模式回调"""
        try:
            self.robot_mode = json.loads(msg.data)
            self.last_update_time = time.time()
        except Exception as e:
            self.get_logger().error(f'解析模式信息错误: {e}')
    
    def cmd_vel_callback(self, msg):
        """运动命令回调"""
        self.cmd_vel = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.last_update_time = time.time()
    
    def harvest_cmd_callback(self, msg):
        """采摘命令回调"""
        self.harvest_command = msg.start_harvest
        self.last_update_time = time.time()
    
    def print_status(self):
        """打印系统状态"""
        current_time = time.time()
        
        print("\n" + "="*80)
        print(f"自动模式调试状态 - {time.strftime('%H:%M:%S')}")
        print("="*80)
        
        # 模式状态
        if self.robot_mode:
            mode = self.robot_mode.get('mode', 'unknown')
            auto_harvest = self.robot_mode.get('auto_harvest', False)
            print(f"🤖 机器人模式: {mode}")
            print(f"🔄 自动采摘: {'启用' if auto_harvest else '禁用'}")
        else:
            print("❌ 未收到模式信息")
        
        # 瓶子检测状态
        if self.bottle_info:
            bottle_detected = self.bottle_info.get('bottle_detected', False)
            total_count = self.bottle_info.get('total_count', 0)
            valid_count = self.bottle_info.get('valid_count', 0)
            
            print(f"🍾 瓶子检测: {'检测到' if bottle_detected else '未检测到'}")
            print(f"📊 检测统计: 总数={total_count}, 有效={valid_count}")
            
            if bottle_detected and 'nearest_bottle' in self.bottle_info:
                nearest = self.bottle_info['nearest_bottle']
                pixel_x = nearest.get('pixel_x', 0)
                pixel_y = nearest.get('pixel_y', 0)
                distance = nearest.get('distance', 0)
                status = nearest.get('status', 'unknown')
                
                print(f"🎯 最近瓶子:")
                print(f"   位置: ({pixel_x}, {pixel_y})")
                print(f"   距离: {distance:.2f}m")
                print(f"   状态: {status}")
                
                # 分析为什么车不动
                if distance > 15.0:
                    print("⚠️  问题: 瓶子距离超过最大检测范围(15m)")
                elif distance > 5.0:
                    print("ℹ️  提示: 瓶子在远距离范围，应该快速接近")
                elif distance > 2.0:
                    print("ℹ️  提示: 瓶子在中等距离范围，应该正常接近")
                else:
                    print("ℹ️  提示: 瓶子在近距离范围，应该精细控制")
        else:
            print("❌ 未收到瓶子检测信息")
        
        # 距离信息
        if self.nearest_distance is not None:
            print(f"📏 最近距离: {self.nearest_distance:.2f}m")
        else:
            print("❌ 未收到距离信息")
        
        # 运动状态
        if self.cmd_vel:
            linear = self.cmd_vel['linear_x']
            angular = self.cmd_vel['angular_z']
            
            if abs(linear) > 0.01 or abs(angular) > 0.01:
                print(f"🚗 运动状态: 活跃")
                print(f"   前进速度: {linear:.2f} m/s")
                print(f"   转向速度: {angular:.2f} rad/s")
                
                if linear > 0:
                    print("   方向: 前进")
                elif linear < 0:
                    print("   方向: 后退")
                elif angular > 0:
                    print("   方向: 左转")
                elif angular < 0:
                    print("   方向: 右转")
            else:
                print("🛑 运动状态: 静止")
                
                # 分析为什么静止
                if self.robot_mode and self.robot_mode.get('mode') == 'auto' and self.robot_mode.get('auto_harvest'):
                    if self.bottle_info and self.bottle_info.get('bottle_detected'):
                        print("⚠️  问题: 检测到瓶子但车辆未移动")
                        print("   可能原因:")
                        print("   - 瓶子距离超出控制范围")
                        print("   - 自动控制器未正常工作")
                        print("   - 控制命令未正确发布")
                    else:
                        print("ℹ️  正常: 未检测到瓶子，车辆待命")
                else:
                    print("ℹ️  正常: 非自动模式或自动采摘未启用")
        else:
            print("❌ 未收到运动命令")
        
        # 采摘状态
        if self.harvest_command is not None:
            print(f"🤏 采摘命令: {'启动' if self.harvest_command else '停止'}")
        else:
            print("❌ 未收到采摘命令")
        
        # 数据更新时间
        time_since_update = current_time - self.last_update_time
        if time_since_update > 5.0:
            print(f"⚠️  警告: 数据更新超时 ({time_since_update:.1f}秒)")
        else:
            print(f"✅ 数据更新正常 (最后更新: {time_since_update:.1f}秒前)")
        
        print("="*80)
        
        # 给出建议
        self.print_suggestions()
    
    def print_suggestions(self):
        """打印调试建议"""
        print("\n💡 调试建议:")
        
        # 检查模式
        if not self.robot_mode:
            print("1. 检查机器人模式发布器是否正常工作")
            print("   命令: ros2 topic echo /robot/mode")
        
        # 检查检测
        if not self.bottle_info:
            print("2. 检查瓶子检测节点是否正常工作")
            print("   命令: ros2 topic echo /bottle_detection/info")
        
        # 检查控制器
        if not self.cmd_vel:
            print("3. 检查自动控制器是否正常工作")
            print("   命令: ros2 topic echo /cmd_vel_raw")
        
        # 具体问题分析
        if (self.robot_mode and self.robot_mode.get('mode') == 'auto' and 
            self.robot_mode.get('auto_harvest') and self.bottle_info and 
            self.bottle_info.get('bottle_detected')):
            
            if not self.cmd_vel or (abs(self.cmd_vel['linear_x']) < 0.01 and abs(self.cmd_vel['angular_z']) < 0.01):
                print("4. 自动模式下检测到瓶子但无运动命令，检查:")
                print("   - auto_harvest_controller节点日志")
                print("   - 瓶子距离是否在有效范围内")
                print("   - 控制器参数配置")
        
        print("\n📋 有用的命令:")
        print("   ros2 node list | grep -E '(bottle|harvest|control)'")
        print("   ros2 topic list | grep -E '(bottle|cmd_vel|harvest)'")
        print("   ros2 topic hz /bottle_detection/info")
        print("   ros2 param list | grep harvest")


def main(args=None):
    rclpy.init(args=args)
    
    debugger = None
    try:
        debugger = AutoModeDebugger()
        print("自动模式调试器启动成功！")
        print("按 Ctrl+C 退出")
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        print("\n调试器已停止")
    except Exception as e:
        print(f'调试器运行出错: {e}')
    finally:
        if debugger is not None:
            debugger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 