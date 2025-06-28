#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动控制修复验证脚本
验证自动采摘控制器是否能正确发布cmd_vel指令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time


class AutoControlTestNode(Node):
    """自动控制测试节点"""
    
    def __init__(self):
        super().__init__('auto_control_test')
        
        # 创建订阅者监控cmd_vel话题
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建发布者模拟模式控制
        self.mode_pub = self.create_publisher(String, 'robot/mode', 10)
        
        # 创建发布者模拟瓶子检测
        self.detection_pub = self.create_publisher(String, 'bottle_detection/info', 10)
        
        # 状态跟踪
        self.cmd_vel_received = False
        self.last_cmd_vel_time = 0
        self.cmd_vel_count = 0
        
        # 创建定时器定期发送测试数据
        self.test_timer = self.create_timer(2.0, self.send_test_data)
        
        # 创建定时器检查结果
        self.check_timer = self.create_timer(1.0, self.check_results)
        
        self.get_logger().info('自动控制测试节点已启动')
        self.get_logger().info('正在监控 /cmd_vel 话题...')
        
        # 启动测试
        self.start_test()
    
    def start_test(self):
        """启动测试"""
        self.get_logger().info('=== 开始自动控制修复测试 ===')
        
        # 发送自动模式激活命令
        time.sleep(1.0)  # 等待节点完全启动
        
        # 激活自动模式
        self.activate_auto_mode()
    
    def activate_auto_mode(self):
        """激活自动模式"""
        mode_msg = String()
        mode_data = {
            "mode": "auto",
            "auto_harvest": True
        }
        mode_msg.data = json.dumps(mode_data)
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info('✅ 已激活自动模式和自动采摘')
    
    def send_test_data(self):
        """发送测试用的瓶子检测数据"""
        detection_msg = String()
        detection_data = {
            "bottle_detected": True,
            "nearest_bottle": {
                "pixel_x": 320,  # 屏幕中心
                "pixel_y": 240,
                "bbox": [250, 180, 390, 300],  # 边界框 [xmin, ymin, xmax, ymax]
                "distance": 1.5,  # 1.5米距离，应该触发远距离控制
                "confidence": 0.8,
                "status": "正常"
            },
            "total_count": 1,
            "fps": 30.0,
            "timestamp": time.time()
        }
        detection_msg.data = json.dumps(detection_data)
        self.detection_pub.publish(detection_msg)
        
        self.get_logger().info(f'📡 发送测试瓶子检测数据: 距离=1.5m, 位置=(320,240), 边界框=[250,180,390,300]')
    
    def cmd_vel_callback(self, msg):
        """cmd_vel回调函数"""
        self.cmd_vel_received = True
        self.last_cmd_vel_time = time.time()
        self.cmd_vel_count += 1
        
        # 分析接收到的命令
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        if linear_x > 0:
            cmd_type = "前进"
        elif linear_x < 0:
            cmd_type = "后退"
        elif angular_z > 0:
            cmd_type = "左转"
        elif angular_z < 0:
            cmd_type = "右转"
        else:
            cmd_type = "停止"
        
        self.get_logger().info(
            f'✅ 接收到cmd_vel指令 #{self.cmd_vel_count}: {cmd_type} '
            f'(线速度={linear_x:.3f}m/s, 角速度={angular_z:.3f}rad/s)'
        )
        
        # 检查速度是否合理
        if linear_x > 0:
            expected_min_speed = 0.15  # 预期最小速度
            if linear_x < expected_min_speed:
                self.get_logger().warn(
                    f'⚠️  速度可能过低: {linear_x:.3f}m/s < {expected_min_speed}m/s'
                )
            else:
                self.get_logger().info(f'✅ 速度正常: {linear_x:.3f}m/s')
    
    def check_results(self):
        """检查测试结果"""
        current_time = time.time()
        
        if self.cmd_vel_received:
            time_since_last = current_time - self.last_cmd_vel_time
            if time_since_last < 3.0:  # 3秒内有指令
                status = "✅ 正常接收"
            else:
                status = f"⚠️  {time_since_last:.1f}秒前"
            
            self.get_logger().info(
                f'📊 测试状态: 接收到 {self.cmd_vel_count} 条指令, '
                f'最后指令: {status}'
            )
        else:
            self.get_logger().warn('❌ 尚未接收到任何cmd_vel指令')
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('=== 测试结束 ===')
        if self.cmd_vel_received:
            self.get_logger().info(f'✅ 测试成功! 总共接收到 {self.cmd_vel_count} 条cmd_vel指令')
            self.get_logger().info('🎉 自动控制修复验证通过!')
        else:
            self.get_logger().error('❌ 测试失败! 未接收到任何cmd_vel指令')
            self.get_logger().error('💡 请检查自动采摘控制器是否正常运行')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = AutoControlTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f'测试出错: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 