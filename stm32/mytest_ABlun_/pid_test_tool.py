#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32四轮小车PID测试工具
用于通过串口控制和监控PID测试
"""
import serial,time,struct,sys
class PIDTestTool:
    def __init__(self,port='COM3',baudrate=115200):
        try:
            self.ser=serial.Serial(port,baudrate,timeout=1)
            print(f"串口{port}连接成功")
        except:
            print(f"串口{port}连接失败")
            sys.exit(1)
    def send_cmd(self,cmd,data):
        """发送命令到STM32"""
        packet=[0xAA,0x55,cmd,len(data)]+data
        checksum=sum(packet[2:])&0xFF
        packet.append(checksum)
        self.ser.write(bytes(packet))
        print(f"发送: {' '.join([f'{b:02X}' for b in packet])}")
    def pid_test(self,mode,speed):
        """PID测试命令"""
        self.send_cmd(0x06,[mode,speed])
        print(f"PID测试: 模式{mode}, 速度{speed}%")
    def set_pid_params(self,kp,ki,kd):
        """设置PID参数"""
        data=list(struct.pack('<fff',kp,ki,kd))
        self.send_cmd(0x07,data)
        print(f"设置PID参数: Kp={kp}, Ki={ki}, Kd={kd}")
    def read_data(self,timeout=5):
        """读取串口数据"""
        start=time.time()
        while time.time()-start<timeout:
            if self.ser.in_waiting>0:
                try:
                    line=self.ser.readline().decode('utf-8').strip()
                    if line:print(f"接收: {line}")
                except:pass
            time.sleep(0.01)
    def run_test_sequence(self):
        """运行完整测试序列"""
        tests=[
            (1,30,"前进测试"),
            (2,25,"后退测试"),
            (3,35,"左转测试"),
            (4,40,"转圈测试"),
            (5,50,"加速急停测试")
        ]
        for mode,speed,name in tests:
            print(f"\n=== {name} ===")
            self.pid_test(mode,speed)
            self.read_data(6)
            time.sleep(1)
        print("\n测试序列完成")
    def interactive_mode(self):
        """交互模式"""
        print("\n=== PID测试交互模式 ===")
        print("命令:")
        print("1-前进 2-后退 3-左转 4-转圈 5-加速急停 0-停止")
        print("p-设置PID参数 s-序列测试 q-退出")
        while True:
            try:
                cmd=input("\n输入命令: ").strip().lower()
                if cmd=='q':break
                elif cmd=='s':
                    self.run_test_sequence()
                elif cmd=='p':
                    kp=float(input("Kp: "))
                    ki=float(input("Ki: "))
                    kd=float(input("Kd: "))
                    self.set_pid_params(kp,ki,kd)
                elif cmd in '012345':
                    mode=int(cmd)
                    if mode==0:
                        self.pid_test(0,0)
                    else:
                        speed=int(input("速度(10-100): "))
                        self.pid_test(mode,speed)
                    self.read_data(3)
                else:
                    print("无效命令")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"错误: {e}")
    def close(self):
        """关闭串口"""
        if hasattr(self,'ser'):
            self.ser.close()
            print("串口已关闭")
if __name__=="__main__":
    tool=PIDTestTool()
    try:
        if len(sys.argv)>1 and sys.argv[1]=='auto':
            tool.run_test_sequence()
        else:
            tool.interactive_mode()
    finally:
        tool.close() 