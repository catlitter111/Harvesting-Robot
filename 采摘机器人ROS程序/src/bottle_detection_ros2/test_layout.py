#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
界面布局测试脚本
用于验证界面组件创建和布局是否正确，无需实际显示
"""

import sys
import traceback
import os

# 设置无头模式
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

try:
    from PySide6.QtWidgets import QApplication
    from local_control_interface import SmartAgricultureInterface
    
    def test_interface():
        """测试界面创建"""
        try:
            app = QApplication(sys.argv)
            
            print("正在创建智慧农业采摘系统界面...")
            window = SmartAgricultureInterface()
            
            print("✅ 界面创建成功！")
            print(f"窗口标题: {window.windowTitle()}")
            print(f"窗口尺寸: {window.size().width()} x {window.size().height()}")
            
            # 测试数据更新
            print("\n正在测试数据更新...")
            window.update_data()
            print("✅ 数据更新成功！")
            
            # 测试控制功能
            print("\n正在测试控制功能...")
            window.toggle_system()
            window.set_auto_mode()
            window.toggle_harvest()
            print("✅ 控制功能测试成功！")
            
            print("\n🎉 所有测试通过！界面准备就绪。")
            print("📝 要在实际环境中运行，请确保：")
            print("   1. 安装了完整的图形环境")
            print("   2. 安装了 libxcb-cursor0 包")
            print("   3. 设置了正确的 DISPLAY 环境变量")
            
            return True
            
        except Exception as e:
            print(f"❌ 界面测试失败: {e}")
            traceback.print_exc()
            return False
    
    if __name__ == "__main__":
        success = test_interface()
        sys.exit(0 if success else 1)
        
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print("请确保安装了 PySide6: pip install PySide6")
    sys.exit(1) 