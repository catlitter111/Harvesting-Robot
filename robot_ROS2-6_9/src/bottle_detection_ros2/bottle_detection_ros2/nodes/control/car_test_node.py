 #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车测试节点
功能：给小车发送速度为100%的前进指令，一秒后停止
用于测试robot_control_node的基本移动功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading


class CarTestNode(Node):
    """小车测试节点类"""
    
    def __init__(self):
        super().__init__('car_test_node')
        
        # 创建发布者，发布到cmd_vel话题
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 等待发布者就绪
        time.sleep(1.0)
        
        self.get_logger().info('小车测试节点已启动')
        self.get_logger().info('准备发送测试指令：前进速度100%，持续1秒')
        
        # 开始测试
        self.start_test()
    
    def start_test(self):
        """开始测试"""
        # 在单独线程中执行测试，避免阻塞节点
        test_thread = threading.Thread(target=self.execute_test)
        test_thread.daemon = True
        test_thread.start()
    
    def execute_test(self):
        """执行测试序列"""
        try:
            # 等待其他节点启动完成
            self.get_logger().info('等待2秒，确保其他节点就绪...')
            time.sleep(2.0)
            
            # 发送前进指令
            self.get_logger().info('🚗 发送前进指令：速度100%')
            twist_msg = Twist()
            twist_msg.linear.x = 0.5  # 0.5 m/s，相当于100%速度 (max_linear_speed = 0.5)
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            
            # 发布前进命令
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f'✅ 前进指令已发送：linear.x = {twist_msg.linear.x} m/s')
            
            # 等待1秒
            self.get_logger().info('⏱️ 等待1秒...')
            time.sleep(1.0)
            
            # 发送停止指令
            self.get_logger().info('🛑 发送停止指令')
            stop_msg = Twist()  # 所有值默认为0
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info('✅ 停止指令已发送')
            
            # 测试完成
            self.get_logger().info('🎉 测试完成！小车应该已经前进1秒后停止')
            
            # 额外等待，然后测试其他方向
            # time.sleep(2.0)
            # self.test_other_directions()
            
        except Exception as e:
            self.get_logger().error(f'测试执行出错: {e}')
    
    def test_other_directions(self):
        """测试其他方向（可选）"""
        self.get_logger().info('🔄 开始测试其他方向...')
        
        directions = [
            ('后退', -0.3, 0.0),    # 后退，60%速度
            ('左转', 0.0, 0.5),     # 左转，50%角速度 (max_angular_speed = 1.0)
            ('右转', 0.0, -0.5),    # 右转，50%角速度
        ]
        
        for direction_name, linear_x, angular_z in directions:
            self.get_logger().info(f'🎯 测试{direction_name}...')
            
            # 发送方向指令
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(twist_msg)
            
            # 持续0.5秒
            time.sleep(0.5)
            
            # 停止
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info(f'✅ {direction_name}测试完成')
            
            # 间隔
            time.sleep(1.0)
        
        self.get_logger().info('🏁 所有方向测试完成！')
        
        # 测试完成后的总结
        self.print_test_summary()
    
    def print_test_summary(self):
        """打印测试总结"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('🎯 小车测试总结报告')
        self.get_logger().info('=' * 50)
        self.get_logger().info('✅ 前进测试：速度100%，持续1秒')
        self.get_logger().info('✅ 后退测试：速度60%，持续0.5秒')
        self.get_logger().info('✅ 左转测试：角速度50%，持续0.5秒')
        self.get_logger().info('✅ 右转测试：角速度50%，持续0.5秒')
        self.get_logger().info('🔧 测试指令通过cmd_vel话题发送')
        self.get_logger().info('📡 robot_control_node应该接收并执行这些指令')
        self.get_logger().info('📊 websocket_bridge_node会转发状态信息')
        self.get_logger().info('=' * 50)
        self.get_logger().info('🎉 测试节点任务完成，可以按Ctrl+C退出')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CarTestNode()
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()