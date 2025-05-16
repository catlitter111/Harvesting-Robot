import serial
import logging
import time
from typing import Optional, Union

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("串口控制")

class UARTController:
    """串口控制类：管理与舵机的通信"""
    
    def __init__(self, device: str, baudrate: int, timeout: float):
        """
        初始化串口控制器
        
        Args:
            device: 串口设备路径
            baudrate: 波特率
            timeout: 超时时间
        """
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.retry_count = 3  # 通信失败重试次数
        self.retry_delay = 0.1  # 重试间隔时间
        
    def connect(self) -> bool:
        """
        连接串口设备
        
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
            logger.info(f"串口 {self.device} 连接成功")
            return True
        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            return False
    
    def send(self, data: Union[str, bytes], retry: bool = True) -> int:
        """
        发送数据到串口
        
        Args:
            data: 要发送的数据，可以是字符串或字节
            retry: 是否在失败时重试
            
        Returns:
            int: 发送的字节数，失败返回0
        """
        if self.ser is None or not self.ser.is_open:
            logger.error("串口未连接")
            return 0
            
        # 将字符串转换为字节
        if isinstance(data, str):
            data = data.encode('utf-8')
        
        # 重试机制
        for attempt in range(self.retry_count if retry else 1):
            try:
                written = self.ser.write(data)
                logger.debug(f"已发送 {written} 字节: {data}")
                return written
            except Exception as e:
                logger.error(f"发送失败(尝试 {attempt+1}/{self.retry_count}): {e}")
                if attempt < self.retry_count - 1 and retry:
                    time.sleep(self.retry_delay)
                    continue
                return 0
    
    def receive(self, size: int = 256, timeout: Optional[float] = None) -> Optional[int]:
        """
        从串口接收数据，特别是舵机角度数据
        
        Args:
            size: 要读取的最大字节数
            timeout: 读取超时时间，None则使用默认值
            
        Returns:
            Optional[int]: 舵机角度，失败或无数据则返回None
        """
        if self.ser is None or not self.ser.is_open:
            logger.error("串口未连接")
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
            logger.debug(f"接收到数据: {data_str}")
            
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
                logger.warning(f"接收到的数据格式不正确: {data_str}")
                return None
        except Exception as e:
            logger.error(f"接收数据失败: {e}")
            # 恢复原超时设置
            if original_timeout is not None:
                self.ser.timeout = original_timeout
            return None
    
    def flush(self) -> None:
        """清空串口缓冲区"""
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            logger.debug("串口缓冲区已清空")
    
    def close(self) -> None:
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")