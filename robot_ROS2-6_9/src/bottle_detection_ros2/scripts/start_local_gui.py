#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地控制界面启动脚本
"""

import sys
import os
import traceback

# 添加ROS2包路径
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')

def main():
    """主函数"""
    try:
        # 导入本地控制界面
        from bottle_detection_ros2.gui.local_control_gui import LocalControlGUI
        
        print("正在启动本地控制界面...")
        
        # 创建并运行界面
        gui = LocalControlGUI()
        gui.run()
        
    except ImportError as e:
        print(f"导入模块失败: {str(e)}")
        print("请确保已正确安装所有依赖包:")
        print("- tkinter (通常随Python安装)")
        print("- Pillow (pip install Pillow)")
        print("- opencv-python (pip install opencv-python)")
        print("- rclpy (ROS2包)")
        print(traceback.format_exc())
    except Exception as e:
        print(f"启动界面失败: {str(e)}")
        print(traceback.format_exc())

if __name__ == '__main__':
    main() 