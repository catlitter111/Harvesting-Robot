#!/usr/bin/env python3
"""
采摘图像截取功能测试启动脚本
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class HarvestCaptureTestNode(Node):
    """采摘图像截取测试节点"""
    
    def __init__(self):
        super().__init__('harvest_capture_test')
        
        # 创建发布者
        self.capture_request_pub = self.create_publisher(
            String, 'harvest/capture_request', 10)
        
        # 创建定时器，定期发送测试请求
        self.timer = self.create_timer(5.0, self.send_test_request)
        
        self.request_count = 0
        
        self.get_logger().info("采摘图像截取测试节点已启动")
    
    def send_test_request(self):
        """发送测试的采摘图像截取请求"""
        self.request_count += 1
        
        # 模拟不同的采摘状态
        harvest_statuses = ['approaching', 'gripping', 'confirming', 'completed']
        status = harvest_statuses[self.request_count % len(harvest_statuses)]
        
        request_data = {
            'harvest_session_id': f'test_session_{self.request_count}',
            'target_item_index': 0,
            'harvest_status': status,
            'timestamp': int(time.time() * 1000),
            'robot_id': 'test_robot'
        }
        
        request_msg = String()
        request_msg.data = json.dumps(request_data)
        self.capture_request_pub.publish(request_msg)
        
        self.get_logger().info(f"已发送测试截取请求 #{self.request_count} - 状态: {status}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HarvestCaptureTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 