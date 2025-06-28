#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机话题控制测试脚本
演示如何通过ROS话题控制舵机
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ServoTestController(Node):
    """舵机测试控制器"""
    
    def __init__(self):
        super().__init__('servo_test_controller')
        
        # 创建发布者
        self.debug_cmd_pub = self.create_publisher(
            String,
            'servo/debug_command',
            10
        )
        
        self.get_logger().info('舵机话题控制测试节点已启动')
        
    def send_command(self, command):
        """发送控制命令"""
        msg = String()
        msg.data = command
        self.debug_cmd_pub.publish(msg)
        self.get_logger().info(f'发送命令: {command}')

def main():
    rclpy.init()
    
    # 创建控制器
    controller = ServoTestController()
    
    print("\n" + "="*50)
    print("舵机话题控制测试")
    print("="*50)
    print("可用命令:")
    print("  center     - 回到中心位置")
    print("  up/down    - 垂直移动")
    print("  left/right - 水平移动")
    print("  preset_1-9 - 快捷位置(1-9)")
    print("  harvest_test - 测试采摘动作")
    print("  info       - 显示舵机信息")
    print("  quit       - 退出")
    print("="*50)
    
    try:
        while True:
            # 获取用户输入
            command = input("\n请输入命令: ").strip()
            
            if command.lower() in ['quit', 'exit', 'q']:
                break
            elif command:
                # 发送命令
                controller.send_command(command)
                time.sleep(0.1)  # 短暂延迟
            
    except KeyboardInterrupt:
        print("\n收到中断信号，退出程序")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 