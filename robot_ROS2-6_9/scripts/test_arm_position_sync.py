#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂位置同步测试脚本
用于验证第一次控制移动角度过大问题的修复
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class ArmControlTester(Node):
    def __init__(self):
        super().__init__('arm_control_tester')
        
        # 创建发布者
        self.arm_control_pub = self.create_publisher(
            String,
            'arm_control/command',
            10
        )
        
        self.get_logger().info('机械臂控制测试器已启动')
        
    def test_position_sync(self):
        """测试位置同步功能"""
        test_commands = [
            {'command': 'arm_up', 'speed': 30},
            {'command': 'arm_down', 'speed': 30},
            {'command': 'arm_left', 'speed': 30},
            {'command': 'arm_right', 'speed': 30},
            {'command': 'arm_stop', 'speed': 0}
        ]
        
        self.get_logger().info('开始机械臂控制测试...')
        self.get_logger().info('测试目的：验证第一次控制是否移动角度正常')
        
        for i, cmd in enumerate(test_commands):
            self.get_logger().info(f'发送测试命令 {i+1}/{len(test_commands)}: {cmd}')
            
            # 构建命令消息
            msg = String()
            msg.data = json.dumps(cmd)
            
            # 发布命令
            self.arm_control_pub.publish(msg)
            
            # 等待执行
            time.sleep(2)
        
        self.get_logger().info('测试完成！请查看servo_debug_node的日志输出')
        self.get_logger().info('重点关注：')
        self.get_logger().info('  1. 是否出现"第一次机械臂控制，正在同步舵机位置..."')
        self.get_logger().info('  2. 是否出现"位置已同步"信息')
        self.get_logger().info('  3. 第一次移动的步长是否正常（应该与后续步长相近）')


def main(args=None):
    rclpy.init(args=args)
    
    tester = ArmControlTester()
    
    try:
        # 等待一段时间确保其他节点已启动
        print("等待3秒，确保servo_debug_node已启动...")
        time.sleep(3)
        
        # 开始测试
        tester.test_position_sync()
        
        # 保持节点运行一段时间
        rclpy.spin_once(tester, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
    except Exception as e:
        tester.get_logger().error(f'测试过程中发生错误: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 