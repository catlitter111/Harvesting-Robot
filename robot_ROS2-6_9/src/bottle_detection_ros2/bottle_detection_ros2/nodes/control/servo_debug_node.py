#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂调试节点
用于手动控制机械臂的水平和垂直运动，方便调试
"""

import rclpy
from rclpy.node import Node
from bottle_detection_msgs.msg import ServoCommand, ServoStatus, HarvestCommand
from std_msgs.msg import String
import time
import json
from bottle_detection_ros2.core.qos_profiles import REALTIME_CONTROL_QOS, STATUS_UPDATE_QOS

class ServoDebugNode(Node):
    """机械臂调试节点类"""
    
    def __init__(self):
        super().__init__('servo_debug_node')
        
        # 声明参数
        self.declare_parameter('step_size', 50)  # 每次移动的PWM步长
        self.declare_parameter('time_ms', 200)   # 移动时间（毫秒）
        
        # 获取参数
        step_size_value = self.get_parameter('step_size').value
        self.step_size = int(step_size_value) if step_size_value is not None else 50
        time_ms_value = self.get_parameter('time_ms').value
        self.time_ms = int(time_ms_value) if time_ms_value is not None else 200
        
        # 舵机ID定义
        self.HORIZONTAL_SERVO_ID = 0  # 水平舵机
        self.VERTICAL_SERVO_ID = 1    # 垂直舵机
        
        # 舵机范围定义（从servo_control_node.py获取）
        self.horizontal_range = (500, 2500)  # 水平舵机范围
        self.vertical_range = (500, 1500)    # 垂直舵机范围
        self.horizontal_center = 1500        # 水平中心位置
        self.vertical_center = 600           # 垂直中心位置
        
        # 当前位置（初始化为中心位置）
        self.current_horizontal = self.horizontal_center
        self.current_vertical = self.vertical_center
        
        # 创建发布者
        self.servo_cmd_pub = self.create_publisher(
            ServoCommand,
            'servo/command',
            REALTIME_CONTROL_QOS
        )
        
        self.harvest_cmd_pub = self.create_publisher(
            HarvestCommand,
            'robot/harvest_command',
            REALTIME_CONTROL_QOS
        )
        
        # 创建订阅者（用于获取舵机状态）
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            'servo/status',
            self.servo_status_callback,
            STATUS_UPDATE_QOS
        )
        
        # 新增：订阅机械臂控制命令（来自WebSocket桥接节点）
        self.arm_control_sub = self.create_subscription(
            String,
            'arm_control/command',
            self.arm_control_callback,
            REALTIME_CONTROL_QOS
        )
        
        # 高级控制模式标志
        self.advanced_mode = False
        
        # 节点运行状态
        self.running = True
        
        self.get_logger().info('机械臂调试节点已启动，等待WebSocket控制指令...')
        self.get_logger().info('支持的机械臂控制指令：')
        self.get_logger().info('  - arm_rotate_up: 机械臂向上转')
        self.get_logger().info('  - arm_rotate_down: 机械臂向下转')
        self.get_logger().info('  - arm_rotate_left: 机械臂向左转')
        self.get_logger().info('  - arm_rotate_right: 机械臂向右转')
        self.get_logger().info('  - arm_stop: 停止机械臂')

    
    def move_horizontal(self, delta):
        """移动水平舵机"""
        new_position = self.current_horizontal + delta
        new_position = max(self.horizontal_range[0], min(self.horizontal_range[1], new_position))
        
        if new_position != self.current_horizontal:
            self.current_horizontal = new_position
            self.send_servo_command(self.HORIZONTAL_SERVO_ID, new_position)
    
    def move_vertical(self, delta):
        """移动垂直舵机"""
        new_position = self.current_vertical + delta
        new_position = max(self.vertical_range[0], min(self.vertical_range[1], new_position))
        
        if new_position != self.current_vertical:
            self.current_vertical = new_position
            self.send_servo_command(self.VERTICAL_SERVO_ID, new_position)
    
    def move_to_center(self):
        """移动到中心位置"""
        self.current_horizontal = self.horizontal_center
        self.current_vertical = self.vertical_center
        
        # 同时发送两个舵机的命令
        self.send_servo_command(self.HORIZONTAL_SERVO_ID, self.horizontal_center)
        self.send_servo_command(self.VERTICAL_SERVO_ID, self.vertical_center)
        
        print(f"\n已回到中心位置")
    
    def move_to_preset(self, preset):
        """移动到预设位置"""
        # 定义9个预设位置（3x3网格）
        presets = {
            1: (2500, 1500),      # 左上
            2: (1450, 1500),     # 中上
            3: (500, 1500),     # 右上
            4: (2500, 1000),     # 左中
            5: (1450, 600),     # 中心（特殊的垂直中心位置）
            6: (500, 1000),    # 右中
            7: (2500, 500),     # 左下
            8: (1450, 500),    # 中下
            9: (500, 500),    # 右下
        }
        
        if preset in presets:
            h_pos, v_pos = presets[preset]
            # 确保在范围内
            h_pos = max(self.horizontal_range[0], min(self.horizontal_range[1], h_pos))
            v_pos = max(self.vertical_range[0], min(self.vertical_range[1], v_pos))
            
            self.current_horizontal = h_pos
            self.current_vertical = v_pos
            
            # 发送命令
            self.send_servo_command(self.HORIZONTAL_SERVO_ID, h_pos)
            self.send_servo_command(self.VERTICAL_SERVO_ID, v_pos)
            
            print(f"\n移动到预设位置 {preset}")
    
    def send_servo_command(self, servo_id, position):
        """发送舵机控制命令"""
        msg = ServoCommand()
        msg.servo_id = servo_id
        msg.position = int(position)
        msg.time_ms = self.time_ms
        msg.stop = False
        msg.set_mode = False
        msg.mode = -1
        
        self.servo_cmd_pub.publish(msg)
        self.get_logger().debug(f'发送舵机命令: ID={servo_id}, 位置={position}')
    
    def servo_status_callback(self, msg):
        """舵机状态回调"""
        # 更新当前位置（从实际舵机反馈）
        if len(msg.servo_positions) > self.VERTICAL_SERVO_ID:
            actual_h = msg.servo_positions[self.HORIZONTAL_SERVO_ID]
            actual_v = msg.servo_positions[self.VERTICAL_SERVO_ID]
            
            # 如果实际位置与记录位置相差较大，更新记录
            if abs(actual_h - self.current_horizontal) > 50:
                self.current_horizontal = actual_h
            if abs(actual_v - self.current_vertical) > 50:
                self.current_vertical = actual_v
    
    def arm_control_callback(self, msg):
        """机械臂控制命令回调（来自WebSocket桥接节点）"""
        try:
            # 解析JSON数据
            data = json.loads(msg.data)
            command = data.get('command', '')
            speed = data.get('speed', 50)
            
            self.get_logger().info(f'收到机械臂控制命令: {command}, 速度: {speed}%')
            
            # 根据速度计算步长（基础步长50，速度范围0-100）
            speed_factor = max(0.1, min(2.0, speed / 50.0))  # 0.1到2.0倍
            dynamic_step = int(self.step_size * speed_factor)
            
            # 处理机械臂控制命令
            if command == 'arm_up':
                self.move_vertical(dynamic_step)  # 负值表示向上
                self.get_logger().info(f'机械臂向上转动，步长: {dynamic_step}')
            elif command == 'arm_down':
                self.move_vertical(-dynamic_step)   # 正值表示向下
                self.get_logger().info(f'机械臂向下转动，步长: {dynamic_step}')
            elif command == 'arm_left':
                self.move_horizontal(-dynamic_step) # 正值表示向左
                self.get_logger().info(f'机械臂向左转动，步长: {dynamic_step}')
            elif command == 'arm_right':
                self.move_horizontal(dynamic_step) # 负值表示向右
                self.get_logger().info(f'机械臂向右转动，步长: {dynamic_step}')
            elif command == 'arm_stop':
                # 发送停止命令到舵机控制节点
                for servo_id in [0, 1]:  # 停止水平和垂直舵机
                    stop_msg = ServoCommand()
                    stop_msg.servo_id = servo_id
                    stop_msg.position = -1  # 无效位置
                    stop_msg.time_ms = 0
                    stop_msg.stop = True
                    stop_msg.set_mode = False
                    stop_msg.mode = -1
                    self.servo_cmd_pub.publish(stop_msg)
                self.get_logger().info('机械臂停止命令已发送')
            else:
                self.get_logger().warn(f'未知的机械臂控制命令: {command}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析机械臂控制命令JSON失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理机械臂控制命令失败: {e}')
    
    def print_servo_info(self):
        """打印舵机信息"""
        print("\n" + "="*50)
        print("舵机信息")
        print("="*50)
        print(f"水平舵机 (ID {self.HORIZONTAL_SERVO_ID}):")
        print(f"  当前位置: {self.current_horizontal} PWM")
        print(f"  范围: {self.horizontal_range[0]} - {self.horizontal_range[1]}")
        print(f"  中心: {self.horizontal_center}")
        print(f"\n垂直舵机 (ID {self.VERTICAL_SERVO_ID}):")
        print(f"  当前位置: {self.current_vertical} PWM")
        print(f"  范围: {self.vertical_range[0]} - {self.vertical_range[1]}")
        print(f"  中心: {self.vertical_center}")
        print("="*50 + "\n")
    
    def read_all_servos(self):
        """读取所有舵机位置（发送读取请求）"""
        print("\n正在读取所有舵机位置...")
        for i in range(6):  # 假设有6个舵机
            msg = ServoCommand()
            msg.servo_id = i
            msg.position = -1  # 特殊值，表示读取
            msg.time_ms = 0
            msg.stop = False
            msg.set_mode = False
            msg.mode = -1
            self.servo_cmd_pub.publish(msg)
        print("已发送读取请求，请查看servo_control_node的日志输出\n")
    
    def test_harvest_sequence(self):
        """测试采摘动作序列"""
        print("\n开始测试采摘动作序列...")
        msg = HarvestCommand()
        msg.start_harvest = True
        msg.stop_harvest = False
        self.harvest_cmd_pub.publish(msg)
        print("已发送采摘开始命令\n")
        
        # 等待一下让消息发送出去
        time.sleep(0.1)
    
    def reset_system(self):
        """重置系统到初始状态"""
        print("\n重置系统...")
        
        # 停止任何正在进行的采摘动作
        stop_msg = HarvestCommand()
        stop_msg.start_harvest = False
        stop_msg.stop_harvest = True
        self.harvest_cmd_pub.publish(stop_msg)
        
        # 回到中心位置
        self.move_to_center()
        
        print("系统已重置\n")
    
    def destroy_node(self):
        """清理资源"""
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ServoDebugNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n收到中断信号，正在关闭...')
    except Exception as e:
        print(f'节点运行错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 