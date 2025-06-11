#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
集成瓶子检测节点 - 带完整错误追踪版本
整合了异步瓶子检测、双目深度估计、视频质量控制等功能
支持手动和自动两种工作模式
"""

import rclpy
import os
import traceback
import sys
import logging
from functools import wraps
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String, Float32, Int32, Header
from bottle_detection_msgs.msg import BottleDetection, ServoCommand
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import json
from queue import Queue, Empty
from bottle_detection_ros2.core.vision.stereo_camera import StereoCamera
from bottle_detection_ros2.core.processing.bottle_rknn_pool import BottleRKNNPoolExecutor
from bottle_detection_ros2.core.vision.bottle_detector_async import detect_bottle_async, draw_detections
from bottle_detection_ros2.utils.utils import MedianFilter

# 设置详细日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - [%(filename)s:%(lineno)d] - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('/tmp/bottle_detection_debug.log')
    ]
)
logger = logging.getLogger(__name__)

# 设置全局异常处理
def exception_handler(exc_type, exc_value, exc_traceback):
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return
    
    logger.error("未捕获的异常:", exc_info=(exc_type, exc_value, exc_traceback))

sys.excepthook = exception_handler

# 错误追踪装饰器
def trace_errors(func):
    """为方法添加详细的错误追踪"""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            logger.debug(f">>> 进入方法: {func.__name__}")
            result = func(self, *args, **kwargs)
            logger.debug(f"<<< 成功退出方法: {func.__name__}")
            return result
        except Exception as e:
            logger.error(f"!!! 方法 {func.__name__} 发生错误 !!!")
            logger.error(f"错误类型: {type(e).__name__}")
            logger.error(f"错误信息: {str(e)}")
            logger.error(f"参数信息: args数量={len(args)}, kwargs={list(kwargs.keys())}")
            
            # 打印参数详情（小心大数据）
            for i, arg in enumerate(args[:3]):  # 只打印前3个参数
                if hasattr(arg, 'shape'):  # numpy数组
                    logger.error(f"  args[{i}]: numpy数组, shape={arg.shape}, dtype={arg.dtype}")
                elif isinstance(arg, list) and len(arg) > 0:
                    logger.error(f"  args[{i}]: 列表, 长度={len(arg)}, 第一个元素类型={type(arg[0])}")
                else:
                    arg_str = str(arg)
                    if len(arg_str) > 100:
                        arg_str = arg_str[:100] + '...'
                    logger.error(f"  args[{i}]: {arg_str}")
            
            logger.error("详细堆栈信息:")
            logger.error(traceback.format_exc())
            logger.error("-" * 80)
            
            raise
    return wrapper

class IntegratedBottleDetectionNode(Node):
    """集成瓶子检测节点类"""
    
    @trace_errors
    def __init__(self):
        super().__init__('integrated_bottle_detection_node')
        logger.info("="*80)
        logger.info("初始化集成瓶子检测节点...")
        logger.info(f"Python版本: {sys.version}")
        logger.info(f"NumPy版本: {np.__version__}")
        logger.info(f"OpenCV版本: {cv2.__version__}")
        logger.info("="*80)
        
        # 声明参数
        self._declare_parameters()
        self._get_parameters()
        
        # 初始化CV Bridge
        self.cv_bridge = CvBridge()
        
        # 初始化双目相机
        self.stereo_camera = StereoCamera(
            self.camera_id,
            self.camera_width,
            self.camera_height
        )
        
        # 初始化异步瓶子检测器
        try:
            self.bottle_detector_pool = BottleRKNNPoolExecutor(
                model_path=self.model_path,
                detector_func=detect_bottle_async,
                thread_num=self.thread_num,
                queue_size=self.queue_size
            )
            self.get_logger().info(f'异步检测器初始化成功，线程数: {self.thread_num}')
        except Exception as e:
            self.get_logger().error(f'异步检测器初始化失败: {e}')
            raise RuntimeError('异步检测器初始化失败')
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建发布者
        self._create_publishers(qos_profile)
        
        # 创建订阅者
        self._create_subscribers()
        
        # 初始化系统
        self._initialize_system()
        
        # 创建处理定时器
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback)
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 状态变量
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        self.current_mode = "manual"
        self.auto_harvest_active = False
        
        # 帧数据缓存
        self.frame_data_cache = {}
        self.max_cache_size = self.thread_num * 3
        
        # 瓶子检测结果
        self.bottle_detections_with_distance = []
        self.nearest_bottle_distance = None
        
        # 距离滤波器
        self.distance_filter = MedianFilter(window_size=5)
        
        # 视频质量控制
        self.quality_presets = {
            "high": {"resolution": (640, 480), "quality": 80},
            "medium": {"resolution": (480, 360), "quality": 70},
            "low": {"resolution": (320, 240), "quality": 60},
            "very_low": {"resolution": (240, 180), "quality": 50},
            "minimum": {"resolution": (160, 120), "quality": 40}
        }
        self.current_quality = "medium"
        
        # 舵机跟踪控制
        self.enable_servo_tracking = True
        self.servo_tracking_active = False
        
        # 预填充处理管道
        self._prefill_pipeline()
        
        self.get_logger().info(
            f'集成瓶子检测节点已启动\n'
            f'相机ID: {self.camera_id}\n'
            f'模型路径: {self.model_path}\n'
            f'线程数: {self.thread_num}\n'
            f'发布频率: {self.publish_rate} Hz'
        )
    
    @trace_errors
    def _declare_parameters(self):
        """声明ROS2参数"""
        logger.debug("声明ROS2参数...")
        # 相机参数
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('calibration_file', '')
        
        # 模型参数
        self.declare_parameter('model_path', 'yolo11n.rknn')
        self.declare_parameter('model_size', [640, 640])
        
        # 检测参数
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 发布参数
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # 显示参数
        self.declare_parameter('show_display', True)
        
        # 异步处理参数
        self.declare_parameter('thread_num', 3)
        self.declare_parameter('queue_size', 10)
        
        # 舵机跟踪参数
        self.declare_parameter('enable_servo_tracking', True)
    
    @trace_errors
    def _get_parameters(self):
        """获取参数值"""
        logger.debug("获取参数值...")
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        self.model_path = self.get_parameter('model_path').value
        self.model_size = self.get_parameter('model_size').value
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.show_display = self.get_parameter('show_display').value
        
        self.thread_num = self.get_parameter('thread_num').value
        self.queue_size = self.get_parameter('queue_size').value
        
        self.enable_servo_tracking = self.get_parameter('enable_servo_tracking').value
        
        logger.debug(f"参数: camera_id={self.camera_id}, model_path={self.model_path}, "
                    f"confidence_threshold={self.confidence_threshold}")
    
    @trace_errors
    def _create_publishers(self, qos_profile):
        """创建发布者"""
        logger.debug("创建发布者...")
        # 图像发布者
        self.left_image_pub = self.create_publisher(
            Image, 'camera/left/image_raw', qos_profile)
        self.right_image_pub = self.create_publisher(
            Image, 'camera/right/image_raw', qos_profile)
        self.annotated_image_pub = self.create_publisher(
            Image, 'bottle_detection/annotated_image', qos_profile)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 'bottle_detection/compressed_image', qos_profile)
        
        # 检测结果发布者
        self.bottle_position_pub = self.create_publisher(
            PointStamped, 'bottle_detection/nearest_position', 10)
        self.bottle_distance_pub = self.create_publisher(
            Float32, 'bottle_detection/nearest_distance', 10)
        self.bottle_count_pub = self.create_publisher(
            Int32, 'bottle_detection/count', 10)
        self.detection_info_pub = self.create_publisher(
            String, 'bottle_detection/info', 10)
        
        # 自定义消息发布者
        self.bottle_detection_pub = self.create_publisher(
            BottleDetection, 'bottle_detection/detection', 10)
        
        # 舵机跟踪目标发布者
        self.tracking_target_pub = self.create_publisher(
            Point, 'servo/tracking_target', 10)
    
    @trace_errors
    def _create_subscribers(self):
        """创建订阅者"""
        logger.debug("创建订阅者...")
        # 模式控制订阅
        self.mode_sub = self.create_subscription(
            String,
            'robot/mode',
            self.mode_callback,
            10
        )
        
        # 视频质量控制订阅
        self.quality_sub = self.create_subscription(
            String,
            'video/quality_preset',
            self.quality_callback,
            10
        )
    
    @trace_errors
    def _initialize_system(self):
        """初始化系统组件"""
        logger.debug("初始化系统组件...")
        # 打开相机
        if not self.stereo_camera.open_camera():
            self.get_logger().error('无法打开相机，节点将退出')
            raise RuntimeError('相机初始化失败')
        
        # 加载相机参数
        if self.calibration_file:
            self.stereo_camera.load_camera_params(self.calibration_file)
        else:
            self.stereo_camera.load_camera_params()
        
        # 设置双目校正
        self.stereo_camera.setup_stereo_rectification()
        
        # 设置距离范围
        self.stereo_camera.set_distance_range(self.min_distance, self.max_distance)
        
        self.get_logger().info('系统初始化完成')
    
    @trace_errors
    def _prefill_pipeline(self):
        """预填充处理管道"""
        self.get_logger().info('预填充处理管道...')
        for i in range(self.thread_num):
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                continue
            
            frame_id = self.bottle_detector_pool.put(frame_left)
            
            self.frame_data_cache[frame_id] = {
                'left': frame_left.copy(),
                'right': frame_right.copy(),
                'timestamp': self.get_clock().now().to_msg()
            }
    
    @trace_errors
    def timer_callback(self):
        """定时器回调函数"""
        try:
            with self.lock:
                # 读取双目图像
                frame_left, frame_right = self.stereo_camera.capture_frame()
                if frame_left is None or frame_right is None:
                    self.get_logger().warn('无法读取相机图像', throttle_duration_sec=1.0)
                    return
                
                timestamp = self.get_clock().now().to_msg()
                
                # 提交新帧到异步处理队列
                if not self.bottle_detector_pool.is_full():
                    frame_id = self.bottle_detector_pool.put(frame_left)
                    
                    self.frame_data_cache[frame_id] = {
                        'left': frame_left.copy(),
                        'right': frame_right.copy(),
                        'timestamp': timestamp
                    }
                    
                    # 清理过期缓存
                    if len(self.frame_data_cache) > self.max_cache_size:
                        oldest_id = min(self.frame_data_cache.keys())
                        del self.frame_data_cache[oldest_id]
                
                # 尝试获取处理结果
                result_frame_id, result, success = self.bottle_detector_pool.get(timeout=0.001)
                
                if success and result is not None:
                    _, _, bottle_detections = result
                    
                    if result_frame_id in self.frame_data_cache:
                        frame_data = self.frame_data_cache[result_frame_id]
                        cached_left = frame_data['left']
                        cached_right = frame_data['right']
                        cached_timestamp = frame_data['timestamp']
                        
                        del self.frame_data_cache[result_frame_id]
                        
                        # 处理检测结果
                        self._process_detection_result(
                            cached_left, cached_right, bottle_detections, cached_timestamp)
                
                # 更新FPS
                self._update_fps()
                
        except Exception as e:
            logger.error(f"timer_callback异常: {e}")
            logger.error(traceback.format_exc())
            # 继续运行，不让节点崩溃
            self.get_logger().error(f"处理帧时出错，跳过此帧: {e}")
    
    def _safe_to_scalar(self, value, name="value", as_int=False):
        """安全地将值转换为标量，带详细日志"""
        try:
            logger.debug(f"转换 {name}: 输入类型={type(value)}, 值={value}")
            
            # 检查是否已经是标量
            if isinstance(value, (int, float)):
                result = int(value) if as_int else float(value)
                logger.debug(f"  {name} 已是标量，结果={result}")
                return result
            
            # 检查是否是numpy标量
            if isinstance(value, np.generic):
                result = int(value.item()) if as_int else float(value.item())
                logger.debug(f"  {name} 是numpy标量，结果={result}")
                return result
            
            # 检查是否是numpy数组
            if isinstance(value, np.ndarray):
                logger.debug(f"  {name} 是numpy数组: shape={value.shape}, dtype={value.dtype}")
                if value.size == 1:
                    result = int(value.item()) if as_int else float(value.item())
                    logger.debug(f"  {name} 单元素数组，结果={result}")
                    return result
                elif value.size > 1:
                    # 多元素数组，取第一个元素
                    logger.warning(f"  {name} 多元素数组 shape={value.shape}，取第一个元素")
                    result = int(value.flat[0]) if as_int else float(value.flat[0])
                    return result
                else:
                    logger.error(f"  {name} 空数组!")
                    return 0 if as_int else 0.0
            
            # 尝试直接转换
            result = int(value) if as_int else float(value)
            logger.debug(f"  {name} 直接转换结果={result}")
            return result
            
        except Exception as e:
            logger.error(f"转换 {name} 失败: {e}")
            logger.error(f"值详情: type={type(value)}, value={value}")
            if hasattr(value, '__dict__'):
                logger.error(f"  对象属性: {value.__dict__}")
            return 0 if as_int else 0.0
    
    @trace_errors
    def _process_detection_result(self, frame_left, frame_right, bottle_detections, timestamp):
        """处理检测结果并发布 - 修改版：显示所有检测到的瓶子"""
        logger.debug(f"处理检测结果: 检测数量={len(bottle_detections) if bottle_detections else 0}")
        
        try:
            # 校正图像
            frame_left_rectified, img_left_gray, img_right_gray = \
                self.stereo_camera.rectify_stereo_images(frame_left, frame_right)
            
            # 计算视差和3D点云
            disparity, disp_normalized = self.stereo_camera.compute_disparity(
                img_left_gray, img_right_gray)
            threeD = self.stereo_camera.compute_3d_points(disparity)
            
            # 处理检测结果
            annotated_image = frame_left.copy()
            nearest_bottle = None
            min_distance = float('inf')
            
            # 存储所有检测结果（包括距离无效的）
            all_detections = []
            valid_detections = []  # 仅存储距离有效的检测
            
            for idx, detection in enumerate(bottle_detections):
                try:
                    logger.debug(f"\n处理检测 {idx}:")
                    logger.debug(f"  原始数据: {detection}")
                    
                    # 解包检测结果
                    if len(detection) < 7:
                        logger.warning(f"检测结果长度不足: {len(detection)} < 7")
                        continue
                    
                    left, top, right, bottom, score, cx, cy = detection[:7]
                    
                    # 安全转换为标量
                    score = self._safe_to_scalar(score, "score")
                    cx = self._safe_to_scalar(cx, "cx", as_int=True)
                    cy = self._safe_to_scalar(cy, "cy", as_int=True)
                    left = self._safe_to_scalar(left, "left", as_int=True)
                    top = self._safe_to_scalar(top, "top", as_int=True)
                    right = self._safe_to_scalar(right, "right", as_int=True)
                    bottom = self._safe_to_scalar(bottom, "bottom", as_int=True)
                    
                    # 检查置信度
                    if score < self.confidence_threshold:
                        logger.debug(f"  跳过低置信度检测: {score} < {self.confidence_threshold}")
                        continue
                    
                    # 计算距离
                    distance = self.stereo_camera.get_bottle_distance(threeD, cx, cy)
                    logger.debug(f"  原始距离: type={type(distance)}, value={distance}")
                    
                    # 获取3D位置
                    position_3d = None
                    if threeD is not None:
                        try:
                            h, w = threeD.shape[:2]
                            if 0 <= cy < h and 0 <= cx < w:
                                position_3d = threeD[cy][cx].copy()
                        except Exception as e:
                            logger.error(f"  获取3D位置失败: {e}")
                    
                    # 创建检测字典
                    detection_dict = {
                        'detection': (int(left), int(top), int(right), int(bottom), 
                                    float(score), int(cx), int(cy)),
                        'distance': None,  # 初始为None
                        'distance_raw': distance,  # 保存原始距离值
                        '3d_position': position_3d,
                        'valid_distance': False,  # 标记距离是否有效
                        'status': 'unknown'  # 状态描述
                    }
                    
                    # 处理距离
                    if distance is not None:
                        distance = self._safe_to_scalar(distance, "distance")
                        detection_dict['distance_raw'] = distance
                        
                        # 检查距离范围
                        if self.min_distance <= distance <= self.max_distance:
                            # 距离有效
                            filtered_distance = self.distance_filter.update(distance)
                            detection_dict['distance'] = float(filtered_distance)
                            detection_dict['valid_distance'] = True
                            detection_dict['status'] = 'valid'
                            
                            # 添加到有效检测列表
                            valid_detections.append(detection_dict)
                            
                            # 更新最近瓶子
                            if filtered_distance < min_distance:
                                min_distance = filtered_distance
                                nearest_bottle = detection_dict
                        else:
                            # 距离超出范围但有值
                            detection_dict['distance'] = float(distance)
                            detection_dict['valid_distance'] = False
                            if distance < self.min_distance:
                                detection_dict['status'] = 'too_close'
                            else:
                                detection_dict['status'] = 'too_far'
                    else:
                        # 距离无效（无法计算）
                        detection_dict['valid_distance'] = False
                        detection_dict['status'] = 'no_distance'
                    
                    # 添加到所有检测列表（无论距离是否有效）
                    all_detections.append(detection_dict)
                    logger.debug(f"  检测状态: {detection_dict['status']}, 距离: {detection_dict['distance']}")
                    
                except Exception as e:
                    logger.error(f"处理检测 {idx} 时出错: {e}")
                    logger.error(traceback.format_exc())
                    continue
            
            logger.debug(f"总检测数: {len(all_detections)}, 有效检测数: {len(valid_detections)}")
            
            # 更新全局状态（仅包含距离有效的）
            self.bottle_detections_with_distance = valid_detections
            self.nearest_bottle_distance = min_distance if nearest_bottle else None
            
            # 在图像上绘制所有检测结果（包括距离无效的）
            self._draw_all_detections_on_image(annotated_image, all_detections)
            
            # 添加状态信息
            self._draw_status_info_enhanced(annotated_image, len(all_detections), len(valid_detections))
            
            # 发布结果
            self._publish_raw_images(frame_left, frame_right, timestamp)
            self._publish_annotated_image(annotated_image, timestamp)
            self._publish_detection_results(nearest_bottle, len(valid_detections), timestamp)
            
            # 手动模式下的舵机跟踪（仅跟踪有效距离的瓶子）
            if self.current_mode == "manual" and self.enable_servo_tracking and nearest_bottle:
                self._publish_tracking_target(nearest_bottle)
            
            # 显示图像
            if self.show_display:
                self._display_images(annotated_image, disp_normalized)
                
        except Exception as e:
            logger.error("_process_detection_result 主体出错:")
            logger.error(f"错误类型: {type(e).__name__}")
            logger.error(f"错误信息: {str(e)}")
            logger.error(traceback.format_exc())
            raise

    @trace_errors
    def _draw_all_detections_on_image(self, image, all_detections):
        """在图像上绘制所有检测结果，使用不同颜色区分状态"""
        # 定义不同状态的颜色
        colors = {
            'valid': (0, 255, 0),       # 绿色 - 有效距离
            'too_close': (0, 165, 255),  # 橙色 - 太近
            'too_far': (255, 0, 0),      # 蓝色 - 太远  
            'no_distance': (128, 128, 128),  # 灰色 - 无距离数据
            'unknown': (255, 255, 255)   # 白色 - 未知状态
        }
        
        for det in all_detections:
            detection = det['detection']
            distance = det['distance']
            status = det['status']
            valid = det['valid_distance']
            left, top, right, bottom, score, cx, cy = detection
            
            # 选择颜色
            color = colors.get(status, colors['unknown'])
            thickness = 3 if valid else 2  # 有效距离的框更粗
            
            # 绘制边界框
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, thickness)
            
            # 绘制标签
            label = f"bottle: {score:.2f}"
            cv2.putText(image, label, (int(left), int(top-10)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制距离或状态
            if distance is not None:
                distance_text = f"{distance:.2f}m"
            else:
                # 根据状态显示不同的文本
                status_text = {
                    'no_distance': "NO DIST",
                    'too_close': "TOO CLOSE",
                    'too_far': "TOO FAR",
                    'unknown': "UNKNOWN"
                }
                distance_text = status_text.get(status, "N/A")
            
            # 添加状态标记
            if not valid:
                distance_text = f"[{distance_text}]"  # 用方括号标记无效距离
            
            cv2.putText(image, distance_text, (int(cx-30), int(cy)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 绘制中心点
            cv2.circle(image, (int(cx), int(cy)), 5, color, -1)
            
            # 如果是最近的有效瓶子，添加特殊标记
            if valid and det.get('is_nearest', False):
                # 绘制一个更大的圆圈
                cv2.circle(image, (int(cx), int(cy)), 20, (0, 255, 255), 2)  # 黄色圆圈

    @trace_errors  
    def _draw_status_info_enhanced(self, image, total_detections, valid_detections):
        """在图像上绘制增强的状态信息"""
        # FPS
        cv2.putText(image, f"FPS: {self.current_fps:.1f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 模式
        mode_text = f"Mode: {self.current_mode.upper()}"
        if self.current_mode == "auto" and self.auto_harvest_active:
            mode_text += " (HARVEST)"
        cv2.putText(image, mode_text, (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 队列大小
        queue_text = f"Queue: {self.bottle_detector_pool.get_queue_size()}"
        cv2.putText(image, queue_text, (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # 检测统计
        detection_text = f"Detections: {total_detections} total, {valid_detections} valid"
        cv2.putText(image, detection_text, (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 图例
        legend_y = image.shape[0] - 100
        cv2.putText(image, "Legend:", (10, legend_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 各种状态的颜色说明
        legends = [
            ("Valid", (0, 255, 0)),
            ("Too Close", (0, 165, 255)),
            ("Too Far", (255, 0, 0)),
            ("No Distance", (128, 128, 128))
        ]
        
        for i, (text, color) in enumerate(legends):
            y_pos = legend_y + 20 + i * 20
            cv2.rectangle(image, (10, y_pos-10), (25, y_pos), color, -1)
            cv2.putText(image, text, (30, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    
    @trace_errors
    def _draw_detections_on_image(self, image, valid_detections):
        """在图像上绘制检测结果"""
        for valid_det in valid_detections:
            detection = valid_det['detection']
            distance = valid_det['distance']
            left, top, right, bottom, score, cx, cy = detection
            
            # 绘制边界框
            color = (0, 255, 0)  # 绿色
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
            
            # 绘制标签
            label = f"bottle: {score:.2f}"
            cv2.putText(image, label, (int(left), int(top-10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制距离
            distance_text = f"{distance:.2f}m"
            cv2.putText(image, distance_text, (int(cx-20), int(cy)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 绘制中心点
            cv2.circle(image, (int(cx), int(cy)), 5, (255, 0, 0), -1)
    
    @trace_errors
    def _draw_status_info(self, image):
        """在图像上绘制状态信息"""
        # FPS
        cv2.putText(image, f"FPS: {self.current_fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 模式
        mode_text = f"Mode: {self.current_mode.upper()}"
        if self.current_mode == "auto" and self.auto_harvest_active:
            mode_text += " (HARVEST)"
        cv2.putText(image, mode_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 队列大小
        queue_text = f"Queue: {self.bottle_detector_pool.get_queue_size()}"
        cv2.putText(image, queue_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    @trace_errors
    def _publish_raw_images(self, left_image, right_image, timestamp):
        """发布原始图像"""
        try:
            # 左相机图像
            left_msg = self.cv_bridge.cv2_to_imgmsg(left_image, 'bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_camera'
            self.left_image_pub.publish(left_msg)
            
            # 右相机图像
            right_msg = self.cv_bridge.cv2_to_imgmsg(right_image, 'bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_camera'
            self.right_image_pub.publish(right_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布原始图像失败: {e}')
            logger.error(traceback.format_exc())
    
    @trace_errors
    def _publish_annotated_image(self, image, timestamp):
        """发布标注图像"""
        try:
            # 根据质量设置调整大小
            quality_config = self.quality_presets[self.current_quality]
            resized = cv2.resize(image, quality_config["resolution"])
            
            # 发布原始格式
            msg = self.cv_bridge.cv2_to_imgmsg(resized, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'left_camera'
            self.annotated_image_pub.publish(msg)
            
            # 发布压缩格式
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = cv2.imencode(
                    '.jpg', resized,
                    [cv2.IMWRITE_JPEG_QUALITY, quality_config["quality"]])[1].tostring()
                self.compressed_image_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布标注图像失败: {e}')
            logger.error(traceback.format_exc())
    
    @trace_errors
    def _publish_detection_results(self, nearest_bottle, bottle_count, timestamp):
        """发布检测结果"""
        try:
            # 发布瓶子数量
            count_msg = Int32()
            count_msg.data = bottle_count
            self.bottle_count_pub.publish(count_msg)
            
            if nearest_bottle:
                detection = nearest_bottle['detection']
                distance = nearest_bottle['distance']
                position_3d = nearest_bottle['3d_position']
                left, top, right, bottom, score, cx, cy = detection
                
                # 添加调试日志
                logger.debug(f"发布检测结果 - position_3d类型: {type(position_3d)}")
                if position_3d is not None:
                    logger.debug(f"  position_3d shape: {getattr(position_3d, 'shape', 'N/A')}")
                    logger.debug(f"  position_3d value: {position_3d}")
                
                # 发布距离
                distance_msg = Float32()
                distance_msg.data = distance
                self.bottle_distance_pub.publish(distance_msg)
                
                # 发布3D位置
                if position_3d is not None:
                    try:
                        position_msg = PointStamped()
                        position_msg.header.stamp = timestamp
                        position_msg.header.frame_id = 'left_camera'
                        
                        # 安全地提取坐标值
                        if isinstance(position_3d, np.ndarray) and position_3d.size >= 3:
                            position_msg.point.x = float(position_3d[0]) / 1000.0
                            position_msg.point.y = float(position_3d[1]) / 1000.0
                            position_msg.point.z = float(position_3d[2]) / 1000.0
                        else:
                            logger.warning(f"position_3d格式不正确: {position_3d}")
                            position_msg.point.x = 0.0
                            position_msg.point.y = 0.0
                            position_msg.point.z = 0.0
                        
                        self.bottle_position_pub.publish(position_msg)
                    except Exception as e:
                        logger.error(f"处理3D位置时出错: {e}")
                        logger.error(f"position_3d详情: type={type(position_3d)}, value={position_3d}")
                
                # 发布详细信息
                info = {
                    'bottle_detected': True,
                    'nearest_bottle': {
                        'pixel_x': int(cx),
                        'pixel_y': int(cy),
                        'bbox': [int(left), int(top), int(right), int(bottom)],
                        'confidence': float(score),
                        'distance': float(distance),
                        'status': self._get_distance_status(distance)
                    },
                    'total_count': bottle_count,
                    'fps': float(self.current_fps),
                    'timestamp': time.time()
                }
                
                # 发布自定义消息
                detection_msg = BottleDetection()
                detection_msg.header.stamp = timestamp
                detection_msg.header.frame_id = 'left_camera'
                detection_msg.bottle_detected = True
                detection_msg.bottle_count = bottle_count
                detection_msg.nearest_bottle_x = int(cx)
                detection_msg.nearest_bottle_y = int(cy)
                detection_msg.bbox_left = int(left)
                detection_msg.bbox_top = int(top)
                detection_msg.bbox_right = int(right)
                detection_msg.bbox_bottom = int(bottom)
                detection_msg.confidence = float(score)
                detection_msg.distance = float(distance)
                
                # 修复：正确处理 position_3d
                if position_3d is not None:
                    try:
                        if isinstance(position_3d, np.ndarray) and position_3d.size >= 3:
                            detection_msg.position_x = float(position_3d[0]) / 1000.0
                            detection_msg.position_y = float(position_3d[1]) / 1000.0
                            detection_msg.position_z = float(position_3d[2]) / 1000.0
                        else:
                            detection_msg.position_x = 0.0
                            detection_msg.position_y = 0.0
                            detection_msg.position_z = 0.0
                    except Exception as e:
                        logger.error(f"转换position_3d失败: {e}")
                        detection_msg.position_x = 0.0
                        detection_msg.position_y = 0.0
                        detection_msg.position_z = 0.0
                else:
                    detection_msg.position_x = 0.0
                    detection_msg.position_y = 0.0
                    detection_msg.position_z = 0.0
                
                detection_msg.image_width = self.camera_width // 2
                detection_msg.image_height = self.camera_height
                detection_msg.status = self._get_distance_status(distance)
                self.bottle_detection_pub.publish(detection_msg)
            else:
                # 没有检测到瓶子
                distance_msg = Float32()
                distance_msg.data = -1.0
                self.bottle_distance_pub.publish(distance_msg)
                
                info = {
                    'bottle_detected': False,
                    'total_count': 0,
                    'fps': float(self.current_fps),
                    'timestamp': time.time()
                }
                
                # 发布空的检测消息
                detection_msg = BottleDetection()
                detection_msg.header.stamp = timestamp
                detection_msg.header.frame_id = 'left_camera'
                detection_msg.bottle_detected = False
                detection_msg.bottle_count = 0
                detection_msg.distance = -1.0
                detection_msg.status = "未检测到目标"
                self.bottle_detection_pub.publish(detection_msg)
            
            # 发布JSON信息
            info_msg = String()
            info_msg.data = json.dumps(info, ensure_ascii=False)
            self.detection_info_pub.publish(info_msg)
            
        except Exception as e:
            logger.error(f"发布检测结果失败: {e}")
            logger.error(traceback.format_exc())

    
    @trace_errors
    def _publish_tracking_target(self, nearest_bottle):
        """发布舵机跟踪目标"""
        detection = nearest_bottle['detection']
        _, _, _, _, _, cx, cy = detection
        
        tracking_msg = Point()
        tracking_msg.x = float(cx)
        tracking_msg.y = float(cy)
        tracking_msg.z = float(self.camera_width // 2)  # 传递图像宽度
        
        self.tracking_target_pub.publish(tracking_msg)
    
    @trace_errors
    def _get_distance_status(self, distance):
        """根据距离返回状态描述"""
        # 确保distance是标量
        distance = self._safe_to_scalar(distance, "distance_status")
        
        if distance < 0.5:
            return "距离过近"
        elif distance < 0.8:
            return "采摘距离"
        elif distance < 1.5:
            return "正常"
        elif distance < 3.0:
            return "距离较远"
        else:
            return "距离过远"
    
    @trace_errors
    def _update_fps(self):
        """更新FPS计算"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    @trace_errors
    def _display_images(self, annotated_image, disparity):
        """显示图像窗口"""
        cv2.imshow('Bottle Detection', annotated_image)
        cv2.imshow('Disparity', disparity)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('用户请求退出')
            rclpy.shutdown()
    
    @trace_errors
    def mode_callback(self, msg):
        """模式更新回调"""
        try:
            data = json.loads(msg.data)
            self.current_mode = data.get("mode", "manual")
            self.auto_harvest_active = data.get("auto_harvest", False)
            
            self.get_logger().info(
                f'模式更新: {self.current_mode}, '
                f'自动采摘: {self.auto_harvest_active}'
            )
        except Exception as e:
            self.get_logger().error(f'解析模式数据错误: {e}')
            logger.error(traceback.format_exc())
    
    @trace_errors
    def quality_callback(self, msg):
        """视频质量更新回调"""
        quality = msg.data
        if quality in self.quality_presets:
            self.current_quality = quality
            self.get_logger().info(f'视频质量设置为: {quality}')
        else:
            self.get_logger().warn(f'未知的质量预设: {quality}')
    
    @trace_errors
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        
        if hasattr(self, 'stereo_camera'):
            self.stereo_camera.close_camera()
        
        if hasattr(self, 'bottle_detector_pool'):
            self.bottle_detector_pool.release()
        
        if self.show_display:
            cv2.destroyAllWindows()
        
        super().destroy_node()


def main(args=None):
    logger.info("="*80)
    logger.info("启动瓶子检测主程序...")
    logger.info("="*80)
    
    rclpy.init(args=args)
    
    try:
        node = IntegratedBottleDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("用户中断 (Ctrl+C)")
    except Exception as e:
        logger.critical("主程序发生致命错误!")
        logger.critical(f"错误类型: {type(e).__name__}")
        logger.critical(f"错误信息: {str(e)}")
        logger.critical("详细堆栈:")
        logger.critical(traceback.format_exc())
        
        # 打印系统信息
        import platform
        logger.critical(f"Python版本: {sys.version}")
        logger.critical(f"平台: {platform.platform()}")
        logger.critical(f"NumPy版本: {np.__version__}")
        logger.critical(f"OpenCV版本: {cv2.__version__}")
        logger.critical(f"ROS2分发版: {os.environ.get('ROS_DISTRO', 'unknown')}")
        
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        logger.info("程序已退出")


if __name__ == '__main__':
    main()