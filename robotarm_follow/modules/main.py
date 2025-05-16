#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
双目视觉目标跟随抓取系统主程序
"""

import cv2
import numpy as np
import time
import threading
import queue
import logging
import argparse
import os
import sys
from typing import Tuple, Optional

# 添加父目录到路径，确保可以导入模块
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from modules.config import Config
from modules.uart_controller import UARTController
from modules.object_detector import ObjectDetector
from modules.depth_processor import DepthProcessor
from modules.follow_controller import FollowController
from modules.network_client import NetworkClient
from modules.robot_controller import RobotController

# 配置日志
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('robot_arm.log')
    ]
)
logger = logging.getLogger("主程序")

class FollowSystem:
    """系统主类：整合所有模块并运行主循环"""
    
    def __init__(self, config_file: str = "config/config.yaml", server_url: str = None, robot_id: str = None):
        """
        初始化跟随系统
        
        Args:
            config_file: 配置文件路径
            server_url: WebSocket服务器URL，如果为None则不启用网络功能
            robot_id: 机器人ID
        """
        # 加载配置
        self.config = Config(config_file)
        
        # 网络功能标志
        self.enable_network = server_url is not None
        
        # 初始化串口控制器
        self.uart_controller = UARTController(
            self.config.UART_DEVICE,
            self.config.BAUD_RATE,
            self.config.TIMEOUT,
            chassis_device=self.config.CHASSIS_UART_DEVICE if hasattr(self.config, 'CHASSIS_UART_DEVICE') else None,
            chassis_baudrate=self.config.CHASSIS_BAUD_RATE if hasattr(self.config, 'CHASSIS_BAUD_RATE') else 115200
        )
        
        # 初始化目标检测器
        self.object_detector = ObjectDetector(
            self.config.RKNN_MODEL,
            self.config.MODEL_SIZE,
            self.config.OBJ_THRESH,
            self.config.NMS_THRESH,
            self.config.CLASSES
        )
        
        # 初始化深度处理器
        self.depth_processor = DepthProcessor()
        
        # 初始化跟随控制器
        self.follow_controller = FollowController(
            self.uart_controller,
            self.config
        )
        
        # 如果启用网络功能，初始化相关模块
        if self.enable_network:
            # 初始化机器人控制器
            self.robot_controller = RobotController(self.uart_controller)
            
            # 初始化网络客户端
            self.network_client = NetworkClient(
                server_url=server_url,
                initial_preset="medium",
                robot_id=robot_id
            )
            
            # 设置命令回调
            self.network_client.set_command_callback(self.handle_network_command)
        else:
            self.robot_controller = None
            self.network_client = None
        
        # 线程安全的队列，用于在线程间传递深度图
        self.depth_queue = queue.Queue(maxsize=1)
        
        # 性能监控变量
        self.start_time = time.time()
        self.frame_count = 0
        
        # 摄像头对象
        self.cap = None
        
        # 显示设置
        self.show_depth = False  # 是否显示深度图
        self.running = True  # 程序运行标志
    
    def handle_network_command(self, command_data):
        """
        处理从网络收到的命令
        
        Args:
            command_data: 命令数据
        """
        if self.robot_controller:
            self.robot_controller.handle_command(command_data)
    
    def initialize(self) -> bool:
        """
        初始化系统
        
        Returns:
            bool: 是否成功初始化
        """
        # 连接串口
        if not self.uart_controller.connect():
            logger.error("串口连接失败，程序退出")
            return False
        
        # 加载模型
        if not self.object_detector.load_model():
            logger.error("模型加载失败，程序退出")
            return False
        
        # 初始化机械臂位置
        self.follow_controller.initialize()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.config.CAMERA_ID)
        self.cap.set(3, self.config.CAMERA_WIDTH)
        self.cap.set(4, self.config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        if not self.cap.isOpened():
            logger.error("无法打开摄像头，程序退出")
            return False
        
        # 如果启用网络功能，启动网络客户端
        if self.enable_network and self.network_client:
            if not self.network_client.start():
                logger.error("网络客户端启动失败")
                return False
            
            # 设置初始位置
            if self.robot_controller:
                # 设置默认位置(可根据实际情况修改)
                self.robot_controller.set_position(39.9042, 116.4074)
        
        logger.info("系统初始化完成")
        return True
    
    def process_depth_thread(self, left_frame: np.ndarray, right_frame: np.ndarray) -> None:
        """
        深度处理线程函数
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
        """
        try:
            if self.show_depth:
                threeD, vis_image = self.depth_processor.compute_with_visualization(left_frame, right_frame)
                # 显示深度可视化图
                if vis_image is not None:
                    cv2.imshow('深度图', vis_image)
            else:
                threeD = self.depth_processor.compute_depth(left_frame, right_frame)
            
            # 如果队列已满，清空队列
            if self.depth_queue.full():
                try:
                    self.depth_queue.get_nowait()
                except queue.Empty:
                    pass
            
            if threeD is not None:
                self.depth_queue.put(threeD)
        except Exception as e:
            logger.error(f"深度处理线程异常: {e}")
    
    def draw_fps(self, image: np.ndarray, fps: float) -> None:
        """
        在图像上绘制帧率
        
        Args:
            image: 输入图像
            fps: 帧率
        """
        cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    def run(self) -> None:
        """运行主循环"""
        if not self.initialize():
            return
        
        try:
            while self.running:
                # 读取摄像头帧
                ret, frame = self.cap.read()
                if not ret:
                    logger.error("无法获取摄像头帧")
                    break
                
                # 分割左右图像
                left_frame = frame[0:480, 0:640]
                right_frame = frame[0:480, 640:1280]
                
                # 如果启用网络功能，发送左相机图像到网络客户端
                if self.enable_network and self.network_client:
                    self.network_client.add_frame_to_queue(left_frame.copy())
                
                # 如果处于识别模式，执行目标检测和跟随
                if self.follow_controller.identify_flag == 1:
                    # 启动深度计算线程
                    depth_thread = threading.Thread(
                        target=self.process_depth_thread, 
                        args=(left_frame, right_frame)
                    )
                    depth_thread.daemon = True
                    depth_thread.start()
                    
                    # 进行目标检测
                    boxes, classes, scores = self.object_detector.detect(left_frame)
                    
                    # 等待深度图计算完成
                    try:
                        threeD = self.depth_queue.get(timeout=0.1)
                        
                        # 绘制检测结果并获取目标信息
                        if boxes is not None:
                            target_info = self.object_detector.draw_detections(
                                left_frame, boxes, classes, scores, threeD,
                                target_class_id=self.config.TARGET_CLASS_ID
                            )
                            
                            # 更新跟随控制器的目标位置
                            self.follow_controller.update_target(
                                target_info["center_x"],
                                target_info["center_y"],
                                target_info["distance"]
                            )
                    except queue.Empty:
                        logger.warning("深度图计算超时")
                else:
                    # 处于抓取模式，执行抓取流程
                    self.follow_controller.catch_process_handler()
                
                # 计算并显示帧率
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time
                self.draw_fps(left_frame, fps)
                
                # 显示当前距离和状态
                status = "抓取中" if not self.follow_controller.identify_flag else "跟踪中"
                distance_text = f"距离: {self.follow_controller.current_distance:.1f}cm"
                catch_phase = f"阶段: {self.follow_controller.catch_process}" if not self.follow_controller.identify_flag else ""
                
                cv2.putText(left_frame, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(left_frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if catch_phase:
                    cv2.putText(left_frame, catch_phase, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # 显示处理后的图像
                cv2.imshow('目标跟随系统', left_frame)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                
                # 按q退出
                if key == ord('q'):
                    logger.info("用户按下q键，程序退出")
                    self.running = False
                    break
                
                # 按r重置为识别模式
                elif key == ord('r'):
                    self.follow_controller.reset_to_identify_mode()
                    logger.info("用户按下r键，重置为识别模式")
                
                # 按d切换深度图显示
                elif key == ord('d'):
                    self.show_depth = not self.show_depth
                    logger.info(f"用户按下d键，{'显示' if self.show_depth else '隐藏'}深度图")
                    if not self.show_depth and cv2.getWindowProperty('深度图', cv2.WND_PROP_VISIBLE) > 0:
                        cv2.destroyWindow('深度图')
        
        finally:
            # 释放资源
            self.cleanup()
    
    def cleanup(self) -> None:
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        
        # 停止网络客户端
        if self.enable_network and self.network_client:
            self.network_client.stop()
        
        cv2.destroyAllWindows()
        self.object_detector.release()
        self.uart_controller.close()
        logger.info("程序正常退出，资源已释放")


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='双目视觉目标跟随抓取系统')
    parser.add_argument('--config', type=str, default='config/config.yaml',
                        help='配置文件路径')
    parser.add_argument('--show-depth', action='store_true',
                        help='显示深度图')
    parser.add_argument('--server-url', type=str, default=None,
                        help='WebSocket服务器URL，例如ws://example.com:1234')
    parser.add_argument('--robot-id', type=str, default='robot_123',
                        help='机器人ID')
    return parser.parse_args()


if __name__ == '__main__':
    # 解析命令行参数
    args = parse_arguments()
    
    # 创建并运行系统
    system = FollowSystem(
        args.config, 
        server_url=args.server_url,
        robot_id=args.robot_id
    )
    system.show_depth = args.show_depth
    system.run()

    