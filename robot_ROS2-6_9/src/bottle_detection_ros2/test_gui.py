#!/usr/bin/env python3
import tkinter as tk
from datetime import datetime

class TestGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("智慧农业采摘系统")
        self.root.geometry("1200x800")
        self.root.configure(bg='#E8F5E8')
        
        # 简单的界面
        header = tk.Frame(self.root, bg='#4CAF50', height=60)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        
        tk.Label(header, text="🌱 智慧农业采摘系统", 
                font=('Arial', 18, 'bold'),
                bg='#4CAF50', fg='white').pack(pady=20)
        
        # 主要内容
        main = tk.Frame(self.root, bg='#E8F5E8')
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左侧面板
        left = tk.Frame(main, bg='white', width=250)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0,10))
        left.pack_propagate(False)
        
        tk.Label(left, text="控制中心", font=('Arial', 14, 'bold'), 
                bg='white').pack(pady=20)
        
        tk.Button(left, text="手动模式", width=15, 
                 command=lambda: print("手动模式")).pack(pady=5)
        tk.Button(left, text="自动模式", width=15,
                 command=lambda: print("自动模式")).pack(pady=5)
        tk.Button(left, text="开始采摘", width=15, bg='#4CAF50', fg='white',
                 command=lambda: print("开始采摘")).pack(pady=10)
        
        # 中间面板
        center = tk.Frame(main, bg='white')
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,10))
        
        tk.Label(center, text="实时监控", font=('Arial', 14, 'bold'),
                bg='white').pack(pady=20)
        
        canvas = tk.Canvas(center, bg='black', height=400)
        canvas.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        canvas.create_text(400, 200, text="等待视频流...", fill='white', font=('Arial', 16))
        
        # 右侧面板
        right = tk.Frame(main, bg='white', width=250)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)
        
        tk.Label(right, text="系统状态", font=('Arial', 14, 'bold'),
                bg='white').pack(pady=20)
        
        status_text = "电池: 75%\nCPU: 52%\n温度: 28°C\n今日采摘: 197个"
        tk.Label(right, text=status_text, font=('Arial', 12),
                bg='white', justify=tk.LEFT).pack(padx=20, pady=20)
        
        # 时间更新
        self.time_label = tk.Label(header, text="", font=('Arial', 12),
                                  bg='#4CAF50', fg='white')
        self.time_label.pack(side=tk.RIGHT, padx=20)
        self.update_time()
        
    def update_time(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_label.config(text=current_time)
        self.root.after(1000, self.update_time)
        
    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    print("启动测试GUI...")
    gui = TestGUI()
    gui.run()
