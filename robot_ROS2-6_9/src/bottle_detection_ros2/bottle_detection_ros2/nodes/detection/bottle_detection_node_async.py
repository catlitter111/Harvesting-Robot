#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
异步瓶子检测节点
使用异步处理提高检测性能
"""

import rclpy
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

class BottleDetectionNodeAsync(Node):
    """异步瓶子检测主节点类"""
    
    def __init__(self):
        super().__init__('bottle_detection_node_async')
        
        # 声明ROS2参数
        self._declare_parameters()
        
        # 获取参数值
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
        
        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建发布者
        self.left_image_pub = self.create_publisher(
            Image, 'camera/left/image_raw', qos_profile)
        self.right_image_pub = self.create_publisher(
            Image, 'camera/right/image_raw', qos_profile)
        self.annotated_image_pub = self.create_publisher(
            Image, 'bottle_detection/annotated_image', qos_profile)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 'bottle_detection/compressed_image', qos_profile)
        
        # 使用标准消息类型发布检测结果
        self.bottle_position_pub = self.create_publisher(
            PointStamped, 'bottle_detection/nearest_position', 10)
        self.bottle_distance_pub = self.create_publisher(
            Float32, 'bottle_detection/nearest_distance', 10)
        self.bottle_count_pub = self.create_publisher(
            Int32, 'bottle_detection/count', 10)
        self.detection_info_pub = self.create_publisher(
            String, 'bottle_detection/info', 10)
        
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
        
        # 帧数据缓存 - 存储帧ID对应的双目数据
        self.frame_data_cache = {}
        self.max_cache_size = self.thread_num * 3
        
        # 预填充处理管道
        self._prefill_pipeline()
        
        self.get_logger().info(
            f'异步瓶子检测节点已启动\n'
            f'相机ID: {self.camera_id}\n'
            f'模型路径: {self.model_path}\n'
            f'线程数: {self.thread_num}\n'
            f'发布频率: {self.publish_rate} Hz'
        )
    
    def _declare_parameters(self):
        """声明ROS2参数"""
        # 相机参数
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('calibration_file', '')
        
        # 模型参数
        self.declare_parameter('model_path', 
            '/home/elf/Desktop/project_/need_margin/new_project/yolo11n.rknn')
        self.declare_parameter('model_size', [640, 640])
        
        # 检测参数
        self.declare_parameter('min_distance', 0.2)  # 最小有效距离（米）
        self.declare_parameter('max_distance', 5.0)  # 最大有效距离（米）
        self.declare_parameter('confidence_threshold', 0.5)  # 置信度阈值
        
        # 发布参数
        self.declare_parameter('publish_rate', 30.0)  # 发布频率
        self.declare_parameter('publish_compressed', True)  # 是否发布压缩图像
        self.declare_parameter('jpeg_quality', 80)  # JPEG压缩质量
        
        # 显示参数
        self.declare_parameter('show_display', False)  # 是否显示窗口
        
        # 异步处理参数
        self.declare_parameter('thread_num', 3)  # 线程数量
        self.declare_parameter('queue_size', 10)  # 队列大小
    
    def _get_parameters(self):
        """获取参数值"""
        # 相机参数
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # 模型参数
        self.model_path = self.get_parameter('model_path').value
        self.model_size = self.get_parameter('model_size').value
        
        # 检测参数
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # 发布参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # 显示参数
        self.show_display = self.get_parameter('show_display').value
        
        # 异步处理参数
        self.thread_num = self.get_parameter('thread_num').value
        self.queue_size = self.get_parameter('queue_size').value
    
    def _initialize_system(self):
        """初始化系统组件"""
        # 打开相机
        if not self.stereo_camera.open_camera():
            self.get_logger().error('无法打开相机，节点将退出')
            raise RuntimeError('相机初始化失败')
        
        # 加载相机参数
        if self.calibration_file:
            self.stereo_camera.load_camera_params(self.calibration_file)
        else:
            self.stereo_camera.load_camera_params()  # 使用默认参数
        
        # 设置双目校正
        self.stereo_camera.setup_stereo_rectification()
        
        self.get_logger().info('系统初始化完成')
    
    def _prefill_pipeline(self):
        """预填充处理管道"""
        self.get_logger().info('预填充处理管道...')
        for i in range(self.thread_num):
            # 读取双目图像
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                continue
            
            # 提交到异步处理队列
            frame_id = self.bottle_detector_pool.put(frame_left)
            
            # 缓存帧数据
            self.frame_data_cache[frame_id] = {
                'left': frame_left.copy(),
                'right': frame_right.copy(),
                'timestamp': self.get_clock().now().to_msg()
            }
    
    def timer_callback(self):
        """定时器回调函数 - 主处理循环"""
        with self.lock:
            # 读取双目图像
            frame_left, frame_right = self.stereo_camera.capture_frame()
            if frame_left is None or frame_right is None:
                self.get_logger().warn('无法读取相机图像', throttle_duration_sec=1.0)
                return
            
            # 获取时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 提交新帧到异步处理队列（如果队列未满）
            if not self.bottle_detector_pool.is_full():
                frame_id = self.bottle_detector_pool.put(frame_left)
                
                # 缓存帧数据
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
                
                # 从缓存获取对应的帧数据
                if result_frame_id in self.frame_data_cache:
                    frame_data = self.frame_data_cache[result_frame_id]
                    cached_left = frame_data['left']
                    cached_right = frame_data['right']
                    cached_timestamp = frame_data['timestamp']
                    
                    # 删除已使用的缓存
                    del self.frame_data_cache[result_frame_id]
                    
                    # 处理检测结果
                    self._process_detection_result(
                        cached_left, cached_right, bottle_detections, cached_timestamp)
                else:
                    self.get_logger().warn(f'帧 {result_frame_id} 的缓存数据不存在')
            
            # 更新FPS
            self._update_fps()
    
    def _process_detection_result(self, frame_left, frame_right, bottle_detections, timestamp):
        """处理检测结果并发布"""
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
        
        # 计算每个瓶子的距离
        valid_detections = []
        for detection in bottle_detections:
            left, top, right, bottom, score, cx, cy = detection
            
            # 过滤低置信度检测
            if score < self.confidence_threshold:
                continue
            
            # 计算距离
            distance = self.stereo_camera.get_bottle_distance(threeD, cx, cy)
            
            if distance and self.min_distance <= distance <= self.max_distance:
                valid_detections.append({
                    'detection': detection,
                    'distance': distance,
                    '3d_position': threeD[cy][cx] if threeD is not None else None
                })
                
                # 更新最近瓶子
                if distance < min_distance:
                    min_distance = distance
                    nearest_bottle = valid_detections[-1]
        
        # 在图像上绘制检测结果
        draw_detections(annotated_image, bottle_detections)
        
        # 为有效检测添加距离信息
        for valid_det in valid_detections:
            detection = valid_det['detection']
            distance = valid_det['distance']
            cx, cy = int(detection[5]), int(detection[6])
            
            # 在图像上添加距离信息
            distance_text = f"{distance:.2f}m"
            cv2.putText(annotated_image, distance_text, (cx-20, cy), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 发布原始图像
        self._publish_raw_images(frame_left, frame_right, timestamp)
        
        # 发布标注图像
        self._publish_annotated_image(annotated_image, timestamp)
        
        # 发布瓶子检测结果
        self._publish_detection_results(
            nearest_bottle, len(valid_detections), timestamp)
        
        # 显示图像（如果启用）
        if self.show_display:
            self._display_images(annotated_image, disp_normalized)
    
    def _publish_raw_images(self, left_image, right_image, timestamp):
        """发布原始左右相机图像"""
        try:
            # 发布左相机图像
            left_msg = self.cv_bridge.cv2_to_imgmsg(left_image, 'bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_camera'
            self.left_image_pub.publish(left_msg)
            
            # 发布右相机图像
            right_msg = self.cv_bridge.cv2_to_imgmsg(right_image, 'bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_camera'
            self.right_image_pub.publish(right_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布原始图像失败: {str(e)}')
    
    def _publish_annotated_image(self, image, timestamp):
        """发布标注后的图像"""
        try:
            # 添加FPS和队列信息
            info_text = f'FPS: {self.current_fps:.1f} | Queue: {self.bottle_detector_pool.get_queue_size()}'
            cv2.putText(image, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 发布原始格式
            msg = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = 'left_camera'
            self.annotated_image_pub.publish(msg)
            
            # 发布压缩格式（如果启用）
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = cv2.imencode(
                    '.jpg', image, 
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])[1].tostring()
                self.compressed_image_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布标注图像失败: {str(e)}')
    
    def _publish_detection_results(self, nearest_bottle, bottle_count, timestamp):
        """发布瓶子检测结果（使用标准消息类型）"""
        
        # 发布瓶子数量
        count_msg = Int32()
        count_msg.data = bottle_count
        self.bottle_count_pub.publish(count_msg)
        
        if nearest_bottle:
            detection = nearest_bottle['detection']
            distance = nearest_bottle['distance']
            position_3d = nearest_bottle['3d_position']
            
            # 发布最近瓶子距离
            distance_msg = Float32()
            distance_msg.data = distance
            self.bottle_distance_pub.publish(distance_msg)
            
            # 发布最近瓶子位置
            if position_3d is not None:
                position_msg = PointStamped()
                position_msg.header.stamp = timestamp
                position_msg.header.frame_id = 'left_camera'
                position_msg.point.x = float(position_3d[0] / 1000.0)  # 转换为米
                position_msg.point.y = float(position_3d[1] / 1000.0)
                position_msg.point.z = float(position_3d[2] / 1000.0)
                self.bottle_position_pub.publish(position_msg)
            
            # 发布详细信息（JSON格式）
            left, top, right, bottom, score, cx, cy = detection
            info = {
                'bottle_detected': True,
                'nearest_bottle': {
                    'pixel_x': int(cx),
                    'pixel_y': int(cy),
                    'bbox': [int(left), int(top), int(right), int(bottom)],
                    'confidence': float(score),
                    'distance': float(distance),
                    'status': '正常' if 0.5 <= distance <= 3.0 else ('距离过近' if distance < 0.5 else '距离较远')
                },
                'total_count': bottle_count,
                'fps': float(self.current_fps),
                'queue_size': self.bottle_detector_pool.get_queue_size(),
                'timestamp': time.time()
            }
            
            info_msg = String()
            info_msg.data = json.dumps(info, ensure_ascii=False)
            self.detection_info_pub.publish(info_msg)
        else:
            # 没有检测到瓶子
            distance_msg = Float32()
            distance_msg.data = -1.0
            self.bottle_distance_pub.publish(distance_msg)
            
            info = {
                'bottle_detected': False,
                'total_count': 0,
                'fps': float(self.current_fps),
                'queue_size': self.bottle_detector_pool.get_queue_size(),
                'timestamp': time.time()
            }
            info_msg = String()
            info_msg.data = json.dumps(info, ensure_ascii=False)
            self.detection_info_pub.publish(info_msg)
    
    def _update_fps(self):
        """更新FPS计算"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
            
            # 记录性能信息
            self.get_logger().info(
                f'FPS: {self.current_fps:.1f}, '
                f'队列大小: {self.bottle_detector_pool.get_queue_size()}, '
                f'缓存大小: {len(self.frame_data_cache)}',
                throttle_duration_sec=5.0
            )
    
    def _display_images(self, annotated_image, disparity):
        """显示图像窗口"""
        cv2.imshow('Bottle Detection (Async)', annotated_image)
        cv2.imshow('Disparity', disparity)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('用户请求退出')
            rclpy.shutdown()
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        
        # 关闭相机
        if hasattr(self, 'stereo_camera'):
            self.stereo_camera.close_camera()
        
        # 释放异步检测器
        if hasattr(self, 'bottle_detector_pool'):
            self.bottle_detector_pool.release()
        
        # 关闭显示窗口
        if self.show_display:
            cv2.destroyAllWindows()
        
        super().destroy_node()

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = BottleDetectionNodeAsync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()