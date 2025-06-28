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
import sys
import select
import termios
import tty
import threading
import time
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
        
        # 高级控制模式标志
        self.advanced_mode = False
        
        # 键盘输入设置
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            self.keyboard_enabled = True
        except Exception as e:
            self.get_logger().warn(f'无法初始化键盘输入: {e}')
            self.get_logger().info('将使用ROS话题控制模式')
            self.keyboard_enabled = False
            self.settings = None
        
        # 控制说明
        self.print_help()
        
        # 创建键盘监听线程（仅在键盘可用时）
        self.running = True
        if self.keyboard_enabled:
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.start()
        else:
            # 创建话题控制接口
            self.create_topic_control_interface()
        
        self.get_logger().info('机械臂调试节点已启动')
    
    def create_topic_control_interface(self):
        """创建话题控制接口（当键盘不可用时）"""
        # 创建控制命令订阅者
        self.debug_cmd_sub = self.create_subscription(
            String,
            'servo/debug_command',
            self.debug_command_callback,
            REALTIME_CONTROL_QOS
        )
        
        self.get_logger().info('话题控制接口已创建')
        self.get_logger().info('可以通过发布到 /servo/debug_command 话题来控制舵机')
        self.get_logger().info('可用命令: center, up, down, left, right, preset_1-9, harvest_test')
    
    def debug_command_callback(self, msg):
        """调试命令回调"""
        command = msg.data.strip().lower()
        
        if command == 'center':
            self.move_to_center()
        elif command == 'up':
            self.move_vertical(self.step_size)
        elif command == 'down':
            self.move_vertical(-self.step_size)
        elif command == 'left':
            self.move_horizontal(self.step_size)
        elif command == 'right':
            self.move_horizontal(-self.step_size)
        elif command.startswith('preset_') and len(command) == 8:
            try:
                preset_num = int(command[-1])
                if 1 <= preset_num <= 9:
                    self.move_to_preset(preset_num)
            except ValueError:
                pass
        elif command == 'harvest_test':
            self.test_harvest_sequence()
        elif command == 'info':
            self.print_servo_info()
        else:
            self.get_logger().warn(f'未知命令: {command}')
    
    def print_help(self):
        """打印控制说明"""
        print("\n" + "="*50)
        print("机械臂调试控制")
        print("="*50)
        print("使用方向键控制舵机:")
        print("  ↑/↓ : 控制垂直舵机 (上/下)")
        print("  ←/→ : 控制水平舵机 (左/右)")
        print("\n其他控制键:")
        print("  Space : 回到中心位置")
        print("  +/-   : 增加/减少步长")
        print("  q     : 退出程序")
        print("\n快捷位置:")
        print("  1 : 左上角")
        print("  2 : 正上方")
        print("  3 : 右上角")
        print("  4 : 左边")
        print("  5 : 中心")
        print("  6 : 右边")
        print("  7 : 左下角")
        print("  8 : 正下方")
        print("  9 : 右下角")
        print("\n高级功能:")
        print("  h : 显示/隐藏帮助")
        print("  i : 显示当前舵机信息")
        print("  r : 读取所有舵机位置")
        print("  t : 测试采摘动作序列")
        print("  c : 回到中心并重置采摘系统")
        print("="*50)
        print(f"当前步长: {self.step_size} PWM")
        print(f"当前位置 - 水平: {self.current_horizontal}, 垂直: {self.current_vertical}")
        print("="*50 + "\n")
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # 处理方向键（方向键会产生多个字符）
            if key == '\x1b':  # ESC序列
                additional_chars = sys.stdin.read(2)
                if additional_chars == '[A':
                    return 'UP'
                elif additional_chars == '[B':
                    return 'DOWN'
                elif additional_chars == '[C':
                    return 'RIGHT'
                elif additional_chars == '[D':
                    return 'LEFT'
            return key
        return None
    
    def keyboard_listener(self):
        """键盘监听线程"""
        try:
            while self.running:
                key = self.get_key()
                if key:
                    self.process_key(key)
        except Exception as e:
            self.get_logger().error(f'键盘监听错误: {e}')
        finally:
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def process_key(self, key):
        """处理键盘输入"""
        # 方向键控制
        if key == 'UP':
            self.move_vertical(-self.step_size)
        elif key == 'DOWN':
            self.move_vertical(self.step_size)
        elif key == 'LEFT':
            self.move_horizontal(-self.step_size)
        elif key == 'RIGHT':
            self.move_horizontal(self.step_size)
        
        # 其他控制键
        elif key == ' ':  # 空格键 - 回中心
            self.move_to_center()
        elif key == '+' or key == '=':
            self.step_size = min(200, self.step_size + 10)
            print(f"\n步长增加到: {self.step_size} PWM")
        elif key == '-' or key == '_':
            self.step_size = max(10, self.step_size - 10)
            print(f"\n步长减少到: {self.step_size} PWM")
        elif key == 'q' or key == 'Q':
            print("\n退出程序...")
            self.running = False
            rclpy.shutdown()
        
        # 快捷位置（1-9数字键）
        elif key in '123456789':
            self.move_to_preset(int(key))
        
        # 高级功能
        elif key == 'h' or key == 'H':
            self.print_help()
        elif key == 'i' or key == 'I':
            self.print_servo_info()
        elif key == 'r' or key == 'R':
            self.read_all_servos()
        elif key == 't' or key == 'T':
            self.test_harvest_sequence()
        elif key == 'c' or key == 'C':
            self.reset_system()
        
        # 显示当前位置
        if key in ['UP', 'DOWN', 'LEFT', 'RIGHT', ' '] or key in '123456789':
            print(f"\r当前位置 - 水平: {self.current_horizontal}, 垂直: {self.current_vertical}  ", end='', flush=True)
    
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
        if hasattr(self, 'keyboard_thread'):
            self.keyboard_thread.join()
        
        # 恢复终端设置
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
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