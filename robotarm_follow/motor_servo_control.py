
    #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机与舵机控制模块
负责小车电机与机械臂舵机控制，单独线程运行，防止视频卡顿
"""

import threading
import queue
import time
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("motor_servo_control")

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04

# 距离阈值
DISTANCE_FAR = 1.2  # 远距离阈值，超过此距离使用电机调整方向
DISTANCE_NEAR = 0.8  # 近距离阈值，低于此距离使用舵机调整方向
DISTANCE_HARVEST = 0.5  # 采摘距离阈值，低于此距离开始采摘
DISTANCE_STOP = 0.3  # 停止距离，低于此距离停止移动

# 采摘状态
HARVEST_IDLE = 0     # 空闲状态
HARVEST_STARTED = 1  # 开始采摘
HARVEST_STEP1 = 2    # 采摘步骤1
HARVEST_STEP2 = 3    # 采摘步骤2
HARVEST_STEP3 = 4    # 采摘步骤3
HARVEST_STEP4 = 5    # 采摘步骤4
HARVEST_COMPLETE = 6 # 采摘完成

# 图像中心区域的死区大小
CENTER_DEADZONE = 80  # 像素值，左右方向

# 机械臂抓取动作指令
ARM_COMMANDS = {
    "rt_start": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T1500!#005P1200T2000!",
    "rt_catch1": "#000P1250T2000!#001P0900T2000!#002P1750T2000!#003P1200T2000!#004P1500T2000!#005P1850T2000!",
    "rt_catch2": "#000P2500T2000!#001P1400T2000!#002P1850T2000!#003P1700T2000!#004P1500T2000!#005P1850T2000!",
    "rt_catch3": "#000P2500T1500!#001P1300T1500!#002P2000T1500!#003P1700T1500!#004P1500T1500!#005P1200T1500!",
    "rt_catch4": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T2000!#005P1200T2000!"
}

class MotorServoController:
    """电机和舵机控制器类"""
    
    def __init__(self, robot_controller, servo_controller, control_queue, status_callback=None):
        """
        初始化控制器
        
        参数:
        robot_controller -- 机器人控制器实例
        servo_controller -- 舵机控制器实例
        control_queue -- 控制命令队列
        status_callback -- 状态更新回调函数
        """
        self.robot = robot_controller
        self.servo = servo_controller
        self.queue = control_queue
        self.status_callback = status_callback
        
        self.running = False
        self.thread = None
        
        # 控制状态
        self.current_mode = "manual"  # 当前模式: manual或auto
        self.auto_harvest_active = False  # 自动采摘是否激活
        self.current_speed = 50  # 当前速度百分比
        self.current_direction = DIR_STOP  # 当前方向
        
        # 瓶子跟踪相关
        self.bottle_visible = False  # 瓶子是否可见
        self.nearest_bottle_distance = None  # 最近瓶子距离
        self.bottle_cx = 0  # 瓶子中心x坐标
        self.bottle_cy = 0  # 瓶子中心y坐标
        self.frame_width = 640  # 画面宽度
        self.frame_height = 480  # 画面高度
        
        # 采摘相关
        self.harvest_state = HARVEST_IDLE  # 采摘状态
        self.harvest_step_time = 0  # 采摘步骤时间点
        self.harvested_count = 0  # 采摘计数
        
        # 舵机位置跟踪
        self.current_servo_position = 1500  # 当前舵机位置
        
        # 调试模式
        self.debug = True
    
    def start(self):
        """启动控制线程"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning("控制线程已经在运行")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        logger.info("电机与舵机控制线程已启动")
    
    def stop(self):
        """停止控制线程"""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        
        # 停止机器人
        if self.robot:
            self.robot.stop()
        
        # 重置舵机位置
        if self.servo and self.servo.serial:
            self.servo.center_servo(0)  # 主舵机居中
            self.servo.set_initial_position()  # 机械臂回到初始位置
            
        logger.info("电机与舵机控制线程已停止")
    
    def _control_loop(self):
        """控制线程主循环"""
        last_log_time = time.time()
        last_auto_control_time = time.time()
        last_servo_control_time = time.time()
        auto_control_interval = 0.1  # 自动控制的时间间隔
        servo_control_interval = 0.05  # 舵机控制的时间间隔
        
        while self.running:
            try:
                # 检查队列中的新命令
                try:
                    cmd = self.queue.get(block=False)
                    self._process_command(cmd)
                except queue.Empty:
                    pass
                
                current_time = time.time()
                
                # 自动模式下的控制逻辑
                if self.current_mode == "auto" and self.auto_harvest_active:
                    # 控制频率限制，避免过于频繁的控制命令
                    if current_time - last_auto_control_time >= auto_control_interval:
                        last_auto_control_time = current_time
                        self._auto_control()
                
                # 手动模式下的舵机控制 - 在检测到瓶子时控制舵机跟踪瓶子，不考虑距离
                elif self.current_mode == "manual" and self.bottle_visible:
                    if current_time - last_servo_control_time >= servo_control_interval:
                        last_servo_control_time = current_time
                        self._manual_servo_control()
                
                # 采摘状态机处理
                if self.harvest_state != HARVEST_IDLE:
                    self._harvest_state_machine(current_time)
                
                # 定期日志记录（调试用）
                if self.debug and current_time - last_log_time >= 5.0:
                    last_log_time = current_time
                    mode_str = "AUTO" if self.current_mode == "auto" and self.auto_harvest_active else "MANUAL"
                    logger.debug(f"状态: 模式={mode_str}, 方向={self.current_direction}, 速度={self.current_speed}%, " +
                              f"瓶子距离={self.nearest_bottle_distance}m, 采摘状态={self.harvest_state}")
                
                # 睡眠减少CPU使用
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"控制线程错误: {e}")

    def _manual_servo_control(self):
        """手动模式下的舵机控制 - 检测到瓶子时直接控制舵机跟踪，不考虑距离"""
        # 只有在检测到瓶子时才控制舵机
        if not self.bottle_visible:
            return
            
        # 使用舵机进行跟踪
        if self.servo and self.servo.serial:
            # 这里直接使用舵机控制器的track_object方法，不考虑距离阈值
            self.current_servo_position = self.servo.track_object(
                self.frame_width,
                self.frame_height,
                self.bottle_cx,
                self.bottle_cy,
                self.current_servo_position
            )
            logger.debug(f"手动模式舵机跟踪: 坐标=({self.bottle_cx},{self.bottle_cy}), 当前位置={self.current_servo_position}")
    
    
    def _process_command(self, cmd):
        """处理控制命令"""
        cmd_type = cmd.get("type")
        
        if cmd_type == "set_mode":
            # 设置操作模式
            self.current_mode = cmd.get("mode", "manual")
            self.auto_harvest_active = cmd.get("harvest", False)
            logger.info(f"模式已设置为 {self.current_mode}, 自动采摘: {self.auto_harvest_active}")
            
            # 切换模式时停止运动
            if self.robot:
                self.robot.stop()
                self.current_direction = DIR_STOP
            
        elif cmd_type == "move":
            # 手动移动命令
            if self.current_mode != "manual":
                logger.debug(f"忽略移动命令，当前处于{self.current_mode}模式")
                return
                
            direction = cmd.get("direction", DIR_STOP)
            speed = cmd.get("speed", self.current_speed)
            
            if self.robot:
                self.robot.move(direction, speed)
                self.current_direction = direction
                self.current_speed = speed
                logger.debug(f"手动移动: 方向={direction}, 速度={speed}%")
        
        elif cmd_type == "stop":
            # 停止命令
            if self.robot:
                self.robot.stop()
                self.current_direction = DIR_STOP
                logger.debug("停止移动")
        
        elif cmd_type == "bottle_update":
            # 瓶子检测更新
            self.bottle_visible = cmd.get("visible", False)
            self.nearest_bottle_distance = cmd.get("distance")
            self.bottle_cx = cmd.get("cx", 0)
            self.bottle_cy = cmd.get("cy", 0)
            self.frame_width = cmd.get("frame_width", 640)
            self.frame_height = cmd.get("frame_height", 480)
        
        elif cmd_type == "harvest":
            # 手动触发采摘
            if self.current_mode == "manual" and self.harvest_state == HARVEST_IDLE:
                self._start_harvest()
                logger.info("手动触发采摘")
        
        elif cmd_type == "emergency_stop":
            # 紧急停止
            if self.robot:
                self.robot.stop()
                self.current_direction = DIR_STOP
            
            # 重置采摘状态
            self.harvest_state = HARVEST_IDLE
            
            # 如果在自动模式，关闭自动采摘
            if self.current_mode == "auto":
                self.auto_harvest_active = False
            
            logger.warning("紧急停止！所有操作已终止")
    
    def _auto_control(self):
        """自动模式下的控制逻辑"""
        # 没有检测到瓶子，执行搜索
        if not self.bottle_visible or self.nearest_bottle_distance is None:
            if self.current_direction != DIR_STOP:
                # 停止机器人
                if self.robot:
                    self.robot.stop()
                    self.current_direction = DIR_STOP
                    logger.debug("未检测到瓶子，停止移动")
            # 这里可以添加搜索逻辑，比如随机转向等
            return
        
        # 检查距离值是否合理 - 添加上限检查，防止异常大的距离值
        MAX_POSSIBLE_DISTANCE = 10.0  # 设置一个合理的最大距离（米）
        if self.nearest_bottle_distance > MAX_POSSIBLE_DISTANCE:
            logger.warning(f"检测到异常距离值: {self.nearest_bottle_distance}m, 忽略此次控制")
            return
        
        # 计算瓶子偏离图像中心的程度
        center_x = self.frame_width // 2
        offset_x = center_x - self.bottle_cx
        
        # 判断距离区间，执行不同策略
        if self.nearest_bottle_distance > DISTANCE_FAR:
            # 远距离：使用电机调整方向，前进靠近瓶子
            self._control_approach_far(offset_x)
        elif self.nearest_bottle_distance > DISTANCE_NEAR:
            # 中等距离：使用电机微调方向，缓慢靠近
            self._control_approach_medium(offset_x)
        elif self.nearest_bottle_distance > DISTANCE_HARVEST:
            # 近距离：使用舵机调整方向，准备采摘
            self._control_approach_near(offset_x)
        else:
            # 采摘距离：停止移动，开始采摘
            if self.current_direction != DIR_STOP:
                if self.robot:
                    self.robot.stop()
                    self.current_direction = DIR_STOP
            
            # 如果居中且尚未开始采摘，启动采摘流程
            if abs(offset_x) < CENTER_DEADZONE and self.harvest_state == HARVEST_IDLE:
                self._start_harvest()

    
    def _control_approach_far(self, offset_x):
        """远距离接近控制"""
        # 大偏移时，先调整方向
        if abs(offset_x) > CENTER_DEADZONE * 2:
            if offset_x > 0:  # 瓶子在左边
                new_direction = DIR_LEFT
                logger.debug("远距离：瓶子在左侧，向左转")
            else:  # 瓶子在右边
                new_direction = DIR_RIGHT
                logger.debug("远距离：瓶子在右侧，向右转")
                
            # 仅当方向改变时才发送命令，减少频繁控制
            if self.current_direction != new_direction:
                if self.robot:
                    # 使用较低的转向速度，避免突然快速转动
                    turn_speed = max(30, min(self.current_speed - 20, 50))  # 限制最大转向速度为50%
                    self.robot.move(new_direction, turn_speed)
                    self.current_direction = new_direction
                    logger.debug(f"转向速度设置为{turn_speed}%")
        else:
            # 瓶子基本居中，直线前进
            if self.current_direction != DIR_FORWARD:
                if self.robot:
                    # 限制前进速度，防止远距离时速度过快
                    approach_speed = min(self.current_speed, 60)  # 限制远距离接近速度最大为60%
                    self.robot.move(DIR_FORWARD, approach_speed)
                    self.current_direction = DIR_FORWARD
                    logger.debug(f"远距离：瓶子居中，前进，速度={approach_speed}%")
    
    def _control_approach_medium(self, offset_x):
        """中等距离接近控制"""
        # 中等距离时，需要更精确的方向控制
        if abs(offset_x) > CENTER_DEADZONE:
            if offset_x > 0:  # 瓶子在左边
                new_direction = DIR_LEFT
                logger.debug("中等距离：瓶子在左侧，向左微调")
            else:  # 瓶子在右边
                new_direction = DIR_RIGHT
                logger.debug("中等距离：瓶子在右侧，向右微调")
                
            # 使用更低的速度进行精确调整
            if self.current_direction != new_direction:
                if self.robot:
                    turn_speed = max(20, self.current_speed - 30)  # 更低的转向速度
                    self.robot.move(new_direction, turn_speed)
                    self.current_direction = new_direction
        else:
            # 瓶子基本居中，缓慢前进
            if self.current_direction != DIR_FORWARD:
                if self.robot:
                    slow_speed = max(15, self.current_speed - 35)  # 缓慢靠近速度
                    self.robot.move(DIR_FORWARD, slow_speed)
                    self.current_direction = DIR_FORWARD
                    logger.debug(f"中等距离：瓶子居中，缓慢前进，速度={slow_speed}%")
    
    def _control_approach_near(self, offset_x):
        """近距离接近控制，主要使用舵机微调"""
        # 近距离时，停止车辆移动，使用舵机调整
        if self.current_direction != DIR_STOP:
            if self.robot:
                self.robot.stop()
                self.current_direction = DIR_STOP
                logger.debug("近距离：停止车辆，使用舵机微调")
        
        # 在自动模式下使用舵机进行微调 (手动模式下舵机控制由_manual_servo_control处理)
        if self.current_mode == "auto" and self.servo and self.servo.serial:
            # 使用舵机控制器的track_object方法
            self.current_servo_position = self.servo.track_object(
                self.frame_width,
                self.frame_height,
                self.bottle_cx,
                self.bottle_cy,
                self.current_servo_position
            )
    
    def _start_harvest(self):
        """开始采摘流程"""
        if self.harvest_state != HARVEST_IDLE:
            logger.warning("已有采摘任务在进行中")
            return
            
        logger.info("开始采摘")
        self.harvest_state = HARVEST_STARTED
        self.harvest_step_time = time.time()
        
        # 首先确保机器人停止
        if self.robot:
            self.robot.stop()
            self.current_direction = DIR_STOP
    
    def _harvest_state_machine(self, current_time):
        """采摘状态机"""
        # 刚开始采摘
        if self.harvest_state == HARVEST_STARTED:
            # 发送初始指令
            if self.servo and self.servo.serial:
                self.servo.send_command(ARM_COMMANDS["rt_start"])
                logger.info("采摘步骤1: 机械臂就位")
                
            self.harvest_state = HARVEST_STEP1
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP1 and current_time - self.harvest_step_time > 2.0:
            # 第一步完成，发送第二步指令
            if self.servo and self.servo.serial:
                self.servo.send_command(ARM_COMMANDS["rt_catch1"])
                logger.info("采摘步骤2: 准备抓取")
                
            self.harvest_state = HARVEST_STEP2
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP2 and current_time - self.harvest_step_time > 2.0:
            # 第二步完成，发送第三步指令
            if self.servo and self.servo.serial:
                self.servo.send_command(ARM_COMMANDS["rt_catch2"])
                logger.info("采摘步骤3: 抓取目标")
                
            self.harvest_state = HARVEST_STEP3
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP3 and current_time - self.harvest_step_time > 2.0:
            # 第三步完成，发送第四步指令
            if self.servo and self.servo.serial:
                self.servo.send_command(ARM_COMMANDS["rt_catch3"])
                logger.info("采摘步骤4: 抬升目标")
                
            self.harvest_state = HARVEST_STEP4
            self.harvest_step_time = current_time
            
        elif self.harvest_state == HARVEST_STEP4 and current_time - self.harvest_step_time > 2.0:
            # 第四步完成，发送第五步指令
            if self.servo and self.servo.serial:
                self.servo.send_command(ARM_COMMANDS["rt_catch4"])
                logger.info("采摘步骤5: 返回初始位置")
                
            self.harvest_state = HARVEST_COMPLETE
            self.harvest_step_time = current_time
            
            # 增加采摘计数
            self.harvested_count += 1
            
            # 通知状态更新
            if self.status_callback:
                self.status_callback({
                    "harvested_count": self.harvested_count,
                    "harvest_completed": True
                })
                
        elif self.harvest_state == HARVEST_COMPLETE and current_time - self.harvest_step_time > 2.0:
            # 采摘完成，重置状态
            logger.info(f"采摘完成，采摘总数: {self.harvested_count}")
            self.harvest_state = HARVEST_IDLE