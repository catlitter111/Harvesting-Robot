#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口断开测试脚本
用于测试PID控制程序在串口断开时的行为
"""

import tkinter as tk
from tkinter import messagebox
import threading
import time
import sys
import os

# 添加上位机与服务端目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_disconnection_handling():
    """测试断开连接的处理"""
    print("=== 串口断开测试 ===")
    print("1. 启动程序")
    print("2. 连接串口")
    print("3. 模拟断开")
    print("4. 检查程序响应")
    
    try:
        from PID import RobotControlGUI
        
        root = tk.Tk()
        app = RobotControlGUI(root)
        
        # 设置窗口关闭处理
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        
        # 显示测试说明
        def show_test_instructions():
            messagebox.showinfo(
                "测试说明",
                "串口断开优化测试：\n\n"
                "1. 点击CONNECT连接串口\n"
                "2. 在连接状态下拔掉USB设备\n"
                "3. 观察程序是否：\n"
                "   • 检测到断开\n"
                "   • 弹出提示框\n"
                "   • 正确更新UI状态\n"
                "   • 程序不卡死\n"
                "4. 重新插入设备并测试重连\n\n"
                "优化功能：\n"
                "• 超时检测（5秒无数据）\n"
                "• 增强异常处理\n"
                "• 线程安全退出\n"
                "• 用户友好提示"
            )
        
        # 延迟显示说明，让主窗口先显示
        root.after(1000, show_test_instructions)
        
        print("GUI started successfully")
        root.mainloop()
        
    except ImportError as e:
        print(f"Import error: {e}")
        messagebox.showerror("错误", f"无法导入PID模块: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        messagebox.showerror("错误", f"测试过程中出现错误: {e}")

def simulate_device_disconnection():
    """模拟设备断开的测试函数"""
    print("\n=== 设备断开模拟测试 ===")
    
    # 这里可以添加模拟串口断开的代码
    # 例如：删除/dev/ttyS3文件或模拟串口错误
    
    device_path = "/dev/ttyS3"
    
    if os.path.exists(device_path):
        print(f"发现串口设备: {device_path}")
        print("请手动拔掉USB设备来测试断开处理")
    else:
        print(f"串口设备 {device_path} 不存在")
        print("这正好可以测试设备不存在时的处理")

if __name__ == "__main__":
    print("STM32 Robot Control - 串口断开测试")
    print("=" * 50)
    
    # 检查系统环境
    import platform
    print(f"操作系统: {platform.system()}")
    print(f"Python版本: {platform.python_version()}")
    
    # 检查必要的模块
    try:
        import serial
        print(f"PySerial版本: {serial.__version__}")
    except ImportError:
        print("警告: PySerial 未安装")
    
    try:
        import matplotlib
        print(f"Matplotlib版本: {matplotlib.__version__}")
    except ImportError:
        print("警告: Matplotlib 未安装")
    
    print("\n启动测试...")
    
    # 运行模拟测试
    simulate_device_disconnection()
    
    # 启动GUI测试
    test_disconnection_handling() 