import logging
import time
from typing import Dict, Optional, Any, List
from .uart_controller import UARTController

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("跟随控制")

class FollowController:
    """跟随控制类：管理目标跟随和抓取逻辑"""
    
    def __init__(self, uart_controller: UARTController, config: Any):
        """
        初始化跟随控制器
        
        Args:
            uart_controller: 串口控制器
            config: 配置参数
        """
        self.uart = uart_controller
        self.config = config
        
        # 控制状态
        self.pid_start_x = 0
        self.pid_start_y = 0
        self.stop_flag_x = 1
        self.read_flag_x = 1
        self.stop_flag_y = 1
        self.read_flag_y = 1
        self.send_left = 1
        self.send_right = 1
        self.send_up = 1
        self.send_down = 1
        
        # 抓取状态
        self.identify_flag = 1  # 识别模式标志
        self.catch_process = 0  # 抓取过程状态
        self.send_catch_flag = 1
        self.catch_flag = 1
        self.distance_count = 0
        self.current_distance = 1000  # 初始距离设为一个大值
        
        # 预定义的指令集
        self.commands = {
            "read0": "#000PRAD!",
            "read1": "#001PRAD!",
            "read5": "#005PRAD!",
            "get_mod": "#000PMOD!",
            "rt_start": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P2000T1500!#005P1200T2000!",
            "rt_catch1": "#000P1250T2000!#001P0900T2000!#002P1750T2000!#003P1200T2000!#004P1500T2000!#005P1750T2000!",
            "rt_catch2": "#000P2500T2000!#001P1400T2000!#002P1850T2000!#003P1700T2000!#004P1500T2000!#005P1750T2000!",
            "rt_catch3": "#000P2500T1500!#001P1300T1500!#002P2000T1500!#003P1700T1500!#004P1500T1500!#005P1200T1500!",
            "rt_catch4": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T2000!#005P1200T2000!",
            "DST": "#001PDST!",
            "init_position": "#000P1250T1500!#001P0900T1500!#002P2000T1500!#003P0800T1500!#004P1500T1500!#005P1200T1500!"
        }
        
        # 舵机角度限制
        self.servo_limits = {
            0: {"min": 833, "max": 2167},  # 水平方向舵机
            1: {"min": 882, "max": 1500},  # 垂直方向舵机
            2: {"min": 1750, "max": 2000},
            3: {"min": 800, "max": 1700},
            4: {"min": 1500, "max": 2000},
            5: {"min": 1200, "max": 1790}
        }
    
    def initialize(self) -> None:
        """初始化机械臂位置"""
        logger.info("初始化机械臂位置")
        self.uart.send(self.commands["init_position"])
        time.sleep(1)  # 等待舵机到位
    
    def update_target(self, center_x: Optional[int], center_y: Optional[int], distance: float) -> None:
        """
        更新目标位置并执行跟随
        
        Args:
            center_x: 目标中心点x坐标
            center_y: 目标中心点y坐标
            distance: 目标距离（厘米）
        """
        if center_x is None or center_y is None:
            return
        
        self.current_distance = distance
        
        # 目标足够近时，开始计数准备抓取
        if self.current_distance <= self.config.CATCH_DISTANCE:
            self.distance_count += 1
            if self.distance_count >= 10 and self.identify_flag == 1:
                logger.info(f"目标距离 {self.current_distance:.1f}cm, 开始抓取流程")
                self.identify_flag = 0  # 停止识别，开始抓取
        else:
            self.distance_count = 0  # 重置计数
        
        # 如果在识别模式，执行跟随
        if self.identify_flag == 1:
            self.follow_x(center_x)
            self.follow_y(center_y)
        # 否则执行抓取
        else:
            self.catch_process_handler()
    
    def follow_x(self, target_x: int) -> None:
        """
        水平方向跟随控制
        
        Args:
            target_x: 目标中心点x坐标
        """
        # 如果目标位置与中心点误差在阈值内，停止运动
        if abs(self.config.CENTERX - target_x) <= 30:
            if self.stop_flag_x == 1:
                command = "#{:03d}PDST!".format(0)
                self.uart.send(command)
                self.stop_flag_x = 0
                self.read_flag_x = 1
                self.send_left = 1
                self.send_right = 1
        else:
            # 否则根据目标位置调整舵机
            self.stop_flag_x = 1
            if self.read_flag_x == 1:
                # 读取当前舵机位置
                command = "#{:03d}PRAD!".format(0)
                self.uart.send(command)
                self.pid_start_x = self.uart.receive()
                self.read_flag_x = 0
            else:
                # 目标在右侧，向右转
                if target_x - self.config.CENTERX > 30:
                    if self.pid_start_x is not None:
                        if self.pid_start_x > self.servo_limits[0]["max"] - 67:  # 留一定余量
                            command = "#{:03d}PDST!".format(0)
                        else:
                            target_position = self.servo_limits[0]["max"]
                            temp = int((target_position - self.pid_start_x) * self.config.SPEED)
                            if temp < 4000:
                                temp = 4000
                            command = "#{:03d}P{:04d}T{:04d}!".format(0, target_position, temp)
                        if self.send_left == 1:
                            self.uart.send(command)
                            self.send_left = 0
                            self.send_right = 1
                # 目标在左侧，向左转
                elif self.config.CENTERX - target_x > 30:
                    if self.pid_start_x is not None:
                        if self.pid_start_x < self.servo_limits[0]["min"] + 67:  # 留一定余量
                            command = "#{:03d}PDST!".format(0)
                        else:
                            target_position = self.servo_limits[0]["min"]
                            temp = int((self.pid_start_x - target_position) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(0, target_position, temp)
                        if self.send_right == 1:
                            self.uart.send(command)
                            self.send_right = 0
                            self.send_left = 1
    
    def follow_y(self, target_y: int) -> None:
        """
        垂直方向跟随控制
        
        Args:
            target_y: 目标中心点y坐标
        """
        # 如果目标位置与中心点误差在阈值内，停止运动
        if abs(self.config.CENTERY - target_y) <= 30:
            if self.stop_flag_y == 1:
                command = "#{:03d}PDST!".format(1)
                self.uart.send(command)
                self.stop_flag_y = 0
                self.read_flag_y = 1
                self.send_up = 1
                self.send_down = 1
        else:
            # 否则根据目标位置调整舵机
            self.stop_flag_y = 1
            if self.read_flag_y == 1:
                # 读取当前舵机位置
                command = "#{:03d}PRAD!".format(1)
                self.uart.send(command)
                self.pid_start_y = self.uart.receive()
                self.read_flag_y = 0
            else:
                # 目标在下方，向下调整
                if target_y - self.config.CENTERY > 30:
                    if self.pid_start_y is not None:
                        if self.pid_start_y > self.servo_limits[1]["max"] - 20:
                            command = "#{:03d}PDST!".format(1)
                        else:
                            target_position = self.servo_limits[1]["max"]
                            temp = int((target_position - self.pid_start_y) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(1, target_position, temp)
                        if self.send_up == 1:
                            self.uart.send(command)
                            self.send_up = 0
                            self.send_down = 1
                # 目标在上方，向上调整
                elif self.config.CENTERY - target_y > 30:
                    if self.pid_start_y is not None:
                        if self.pid_start_y < self.servo_limits[1]["min"] + 20:
                            command = "#{:03d}PDST!".format(1)
                        else:
                            target_position = self.servo_limits[1]["min"]
                            temp = int((self.pid_start_y - target_position) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(1, target_position, temp)
                        if self.send_down == 1:
                            self.uart.send(command)
                            self.send_down = 0
                            self.send_up = 1
    
    def catch_process_handler(self) -> None:
        """处理抓取流程的状态机"""
        # 状态0：准备抓取
        if self.catch_process == 0:
            if self.catch_flag == 1:
                self.uart.send(self.commands["read0"])
                catch0_pid0 = self.uart.receive()
                if catch0_pid0 is not None:
                    self.catch_flag = 0
            
            if self.catch_flag == 0:
                if self.send_catch_flag == 1:
                    # 使用当前水平舵机位置
                    current_position = self.pid_start_x if self.pid_start_x else 1250
                    command = f"#000P{current_position:04d}T1250!#001P0900T1500!#002P1750T1500!#003P1200T1500!#004P1500T1500!#005P1790T1500!"
                    self.uart.send(command)
                    self.send_catch_flag = 0
                    logger.info("抓取阶段0: 准备抓取姿势")
                else:
                    self.uart.send(self.commands["read5"])
                    catch0_pid5 = self.uart.receive()
                    if catch0_pid5 is not None and catch0_pid5 > 1720:
                        self.catch_process = 1
                        self.send_catch_flag = 1
                        self.catch_flag = 1
                        logger.info("抓取流程进入阶段1: 爪子已打开")
        
        # 状态1：抓取动作1 - 伸向目标
        elif self.catch_process == 1:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch2"])
                self.send_catch_flag = 0
                logger.info("抓取阶段1: 伸向目标")
            else:
                self.uart.send(self.commands["read0"])
                catch1_pid0 = self.uart.receive()
                if catch1_pid0 is not None and catch1_pid0 > 2450:
                    self.catch_process = 2
                    self.send_catch_flag = 1
                    logger.info("抓取流程进入阶段2: 已伸出")
        
        # 状态2：抓取动作2 - 闭合爪子
        elif self.catch_process == 2:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch3"])
                self.send_catch_flag = 0
                logger.info("抓取阶段2: 闭合爪子")
            else:
                self.uart.send(self.commands["read5"])
                catch2_pid5 = self.uart.receive()
                if catch2_pid5 is not None and catch2_pid5 < 1208:
                    self.catch_process = 3
                    self.send_catch_flag = 1
                    logger.info("抓取流程进入阶段3: 爪子已闭合")
        
        # 状态3：抓取动作3 - 回到初始位置
        elif self.catch_process == 3:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch4"])
                self.send_catch_flag = 0
                logger.info("抓取阶段3: 返回初始位置")
            else:
                self.uart.send(self.commands["read0"])
                catch3_pid0 = self.uart.receive()
                if catch3_pid0 is not None and catch3_pid0 < 1260 and catch3_pid0 > 1245:
                    self.catch_process = 0
                    self.send_catch_flag = 1
                    self.identify_flag = 1
                    self.distance_count = 0
                    self.current_distance = 1000
                    self.read_flag_y = 1
                    self.read_flag_x = 1
                    logger.info("抓取完成，恢复识别模式")
    
    def reset_to_identify_mode(self) -> None:
        """重置为识别模式"""
        self.identify_flag = 1
        self.catch_process = 0
        self.send_catch_flag = 1
        self.catch_flag = 1
        self.distance_count = 0
        self.current_distance = 1000
        self.read_flag_y = 1
        self.read_flag_x = 1
        logger.info("手动重置为识别模式")
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取当前控制状态
        
        Returns:
            Dict: 包含当前状态信息的字典
        """
        return {
            "mode": "识别跟踪" if self.identify_flag == 1 else "抓取",
            "catch_process": self.catch_process,
            "distance": self.current_distance,
            "distance_count": self.distance_count,
            "pid_x": self.pid_start_x,
            "pid_y": self.pid_start_y
        }