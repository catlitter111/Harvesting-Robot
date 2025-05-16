import serial
import logging
import time
import threading
import serial.tools.list_ports
from typing import Optional, Union, List, Tuple, Callable

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("串口控制")

class UARTController:
    """串口控制类：管理与舵机和机器人底盘的通信"""
    
    def __init__(self, device: str, baudrate: int, timeout: float, chassis_device: str = None, chassis_baudrate: int = 115200):
        """
        初始化串口控制器
        
        Args:
            device: 舵机串口设备路径
            baudrate: 舵机波特率
            timeout: 超时时间
            chassis_device: 底盘串口设备路径，None则自动检测
            chassis_baudrate: 底盘串口波特率
        """
        # 舵机串口配置
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.retry_count = 3  # 通信失败重试次数
        self.retry_delay = 0.1  # 重试间隔时间
        
        # 底盘串口配置
        self.chassis_device = chassis_device
        self.chassis_baudrate = chassis_baudrate
        self.chassis_ser = None
        self.chassis_connected = False
        
        # 线程安全锁
        self.servo_lock = threading.Lock()
        self.chassis_lock = threading.Lock()
        
        # 底盘数据接收回调
        self.chassis_data_callback = None
        
        # 底盘监听线程
        self.chassis_thread = None
        self.running = True
        
    def set_chassis_data_callback(self, callback: Callable):
        """设置底盘数据接收回调函数"""
        self.chassis_data_callback = callback
        
    def connect(self) -> bool:
        """
        连接舵机串口设备
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.ser = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                xonxoff=False,
                rtscts=False
            )
            if not self.ser.is_open:
                self.ser.open()
            logger.info(f"舵机串口 {self.device} 连接成功")
            return True
        except Exception as e:
            logger.error(f"舵机串口连接失败: {e}")
            return False
    
    def connect_chassis(self) -> bool:
        """
        连接底盘串口设备
        
        Returns:
            bool: 连接是否成功
        """
        try:
            # 如果没有指定串口，尝试自动检测
            if self.chassis_device is None:
                ports = list(serial.tools.list_ports.comports())
                if not ports:
                    logger.error("未找到可用的串口设备")
                    return False
                
                # 使用第一个找到的串口（可能需要更智能的选择算法）
                for port in ports:
                    if port.device != self.device:  # 避免与舵机串口冲突
                        self.chassis_device = port.device
                        logger.info(f"自动选择底盘串口: {self.chassis_device}")
                        break
                
                if self.chassis_device is None:
                    logger.error("未找到可用的底盘串口")
                    return False
            
            # 连接底盘串口
            self.chassis_ser = serial.Serial(
                port=self.chassis_device,
                baudrate=self.chassis_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                xonxoff=False,
                rtscts=False
            )
            
            if not self.chassis_ser.is_open:
                self.chassis_ser.open()
                
            logger.info(f"底盘串口 {self.chassis_device} 连接成功, 波特率: {self.chassis_baudrate}")
            self.chassis_connected = True
            
            # 启动底盘串口监听线程
            self.start_chassis_listening()
            
            return True
        except Exception as e:
            logger.error(f"底盘串口连接失败: {e}")
            self.chassis_connected = False
            return False
    
    def start_chassis_listening(self):
        """启动底盘串口监听线程"""
        if self.chassis_thread is not None and self.chassis_thread.is_alive():
            return  # 线程已在运行
            
        self.chassis_thread = threading.Thread(target=self.chassis_listening_thread)
        self.chassis_thread.daemon = True
        self.chassis_thread.start()
        logger.info("底盘串口监听线程已启动")
    
    def chassis_listening_thread(self):
        """底盘串口监听线程函数"""
        while self.running and self.chassis_connected:
            if self.chassis_ser is None or not self.chassis_ser.is_open:
                time.sleep(0.1)
                continue
                
            try:
                with self.chassis_lock:
                    if self.chassis_ser.in_waiting > 0:
                        data = self.chassis_ser.read(self.chassis_ser.in_waiting)
                        if data:
                            # 处理接收到的数据
                            self.parse_chassis_data(data)
            except Exception as e:
                logger.error(f"读取底盘串口数据错误: {e}")
                
            time.sleep(0.01)  # 短暂休眠，减少CPU使用
    
    def parse_chassis_data(self, data: bytes):
        """
        解析从底盘接收到的数据
        
        Args:
            data: 接收到的字节数据
        """
        # 将接收到的数据转换为十六进制字符串便于调试
        hex_data = ' '.join([f"{b:02X}" for b in data])
        logger.debug(f"底盘返回: {hex_data}")
        
        # 寻找帧头
        i = 0
        while i < len(data) - 1:
            if data[i] == 0xAA and data[i + 1] == 0x55:
                # 找到帧头，检查数据包是否完整
                if i + 3 < len(data):
                    cmd = data[i + 2]
                    length = data[i + 3]
                    
                    # 检查数据包长度是否足够
                    if i + 4 + length + 1 <= len(data):
                        # 提取数据部分
                        packet_data = data[i + 4:i + 4 + length]
                        
                        # 计算校验和
                        checksum = cmd + length
                        for b in packet_data:
                            checksum += b
                        checksum &= 0xFF  # 取低8位
                        
                        # 获取接收到的校验和
                        received_checksum = data[i + 4 + length]
                        
                        # 校验
                        if checksum == received_checksum:
                            # 校验通过，处理命令
                            if self.chassis_data_callback:
                                self.chassis_data_callback(cmd, packet_data)
                        else:
                            logger.warning(f"底盘数据校验和错误: 计算={checksum:02X}, 接收={received_checksum:02X}")
                        
                        # 跳过已处理的数据
                        i += 4 + length + 1
                        continue
            
            i += 1
    
    def send(self, data: Union[str, bytes], retry: bool = True) -> int:
        """
        发送数据到舵机串口
        
        Args:
            data: 要发送的数据，可以是字符串或字节
            retry: 是否在失败时重试
            
        Returns:
            int: 发送的字节数，失败返回0
        """
        with self.servo_lock:
            if self.ser is None or not self.ser.is_open:
                logger.error("舵机串口未连接")
                return 0
                
            # 将字符串转换为字节
            if isinstance(data, str):
                data = data.encode('utf-8')
            
            # 重试机制
            for attempt in range(self.retry_count if retry else 1):
                try:
                    written = self.ser.write(data)
                    logger.debug(f"已发送舵机数据 {written} 字节: {data}")
                    return written
                except Exception as e:
                    logger.error(f"发送舵机数据失败(尝试 {attempt+1}/{self.retry_count}): {e}")
                    if attempt < self.retry_count - 1 and retry:
                        time.sleep(self.retry_delay)
                        continue
                    return 0
    
    def send_to_chassis(self, command: bytes) -> bool:
        """
        发送命令到底盘串口
        
        Args:
            command: 要发送的命令字节序列
            
        Returns:
            bool: 是否发送成功
        """
        with self.chassis_lock:
            if not self.chassis_connected or self.chassis_ser is None or not self.chassis_ser.is_open:
                logger.error("底盘串口未连接，无法发送命令")
                return False
                
            try:
                self.chassis_ser.write(command)
                # 将命令转为十六进制字符串便于日志记录
                hex_command = ' '.join([f"{b:02X}" for b in command])
                logger.debug(f"发送底盘命令: {hex_command}")
                return True
            except Exception as e:
                logger.error(f"发送底盘命令失败: {e}")
                return False
    
    def receive(self, size: int = 256, timeout: Optional[float] = None) -> Optional[int]:
        """
        从舵机串口接收数据，特别是舵机角度数据
        
        Args:
            size: 要读取的最大字节数
            timeout: 读取超时时间，None则使用默认值
            
        Returns:
            Optional[int]: 舵机角度，失败或无数据则返回None
        """
        with self.servo_lock:
            if self.ser is None or not self.ser.is_open:
                logger.error("舵机串口未连接")
                return None
            
            # 设置临时超时（如果提供）
            original_timeout = None
            if timeout is not None:
                original_timeout = self.ser.timeout
                self.ser.timeout = timeout
            
            try:
                data = self.ser.read(size)
                
                # 恢复原超时设置
                if original_timeout is not None:
                    self.ser.timeout = original_timeout
                    
                if not data:
                    return None
                    
                # 解析舵机回传的角度信息
                data_str = data.decode('utf-8', errors='ignore').strip()
                logger.debug(f"接收到舵机数据: {data_str}")
                
                # 提取舵机编号和角度
                if data_str.startswith("#") and data_str.endswith("!"):
                    data_content = data_str.strip("#!").strip().strip("\r\n")
                    parts = data_content.split('P')
                    if len(parts) >= 2:
                        servo_id = parts[0]
                        angle_part = parts[1].split('!')[0]  # 去除可能的其他字符
                        try:
                            angle = int(angle_part)
                            logger.debug(f"舵机编号: {servo_id}, 角度: {angle}")
                            return angle
                        except ValueError:
                            logger.warning(f"无法将角度值转换为整数: '{angle_part}'")
                            return None
                else:
                    logger.warning(f"接收到的舵机数据格式不正确: {data_str}")
                    return None
            except Exception as e:
                logger.error(f"接收舵机数据失败: {e}")
                # 恢复原超时设置
                if original_timeout is not None:
                    self.ser.timeout = original_timeout
                return None
    
    def flush(self) -> None:
        """清空舵机串口缓冲区"""
        with self.servo_lock:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                logger.debug("舵机串口缓冲区已清空")
    
    def flush_chassis(self) -> None:
        """清空底盘串口缓冲区"""
        with self.chassis_lock:
            if self.chassis_ser and self.chassis_ser.is_open:
                self.chassis_ser.reset_input_buffer()
                self.chassis_ser.reset_output_buffer()
                logger.debug("底盘串口缓冲区已清空")
    
    def close(self) -> None:
        """关闭所有串口连接"""
        # 停止底盘监听线程
        self.running = False
        
        # 关闭舵机串口
        with self.servo_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("舵机串口已关闭")
        
        # 关闭底盘串口
        with self.chassis_lock:
            if self.chassis_ser and self.chassis_ser.is_open:
                self.chassis_ser.close()
                self.chassis_connected = False
                logger.info("底盘串口已关闭")