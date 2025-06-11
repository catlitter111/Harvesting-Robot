#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测ROS2主节点
集成双目相机读取、瓶子检测、距离计算和结果发布功能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32, Int32, Header
from bottle_detection_msgs.msg import BottleDetection, ServoCommand
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import json
from bottle_detection_ros2.core.vision.stereo_camera import StereoCamera
from bottle_detection_ros2.core.vision.bottle_detector import BottleDetector

class BottleDetectionNode(Node):
    """瓶子检测主节点类"""
    
    def __init__(self):
        super().__init__('bottle_detection_node')
        
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
        
        # 初始化瓶子检测器
        self.bottle_detector = BottleDetector(
            self.model_path,
            tuple(self.model_size)
        )
        
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
        
        # 瓶子检测结果缓存
        self.latest_detection = None
        
        self.get_logger().info(
            f'瓶子检测节点已启动\n'
            f'相机ID: {self.camera_id}\n'
            f'模型路径: {self.model_path}\n'
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
        
        # 加载检测模型
        if not self.bottle_detector.load_model():
            self.get_logger().error('无法加载RKNN模型，节点将退出')
            raise RuntimeError('模型加载失败')
        
        self.get_logger().info('系统初始化完成')
    
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
            
            # 校正图像
            frame_left_rectified, img_left_gray, img_right_gray = \
                self.stereo_camera.rectify_stereo_images(frame_left, frame_right)
            
            # 计算视差和3D点云
            disparity, disp_normalized = self.stereo_camera.compute_disparity(
                img_left_gray, img_right_gray)
            threeD = self.stereo_camera.compute_3d_points(disparity)
            
            # 检测瓶子
            bottle_detections = self.bottle_detector.detect(frame_left)
            
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
                self.bottle_detector.draw_detection(
                    annotated_image, detection, distance)
            
            # 发布原始图像
            self._publish_raw_images(frame_left, frame_right, timestamp)
            
            # 发布标注图像
            self._publish_annotated_image(annotated_image, timestamp)
            
            # 发布瓶子检测结果
            self._publish_detection_results(
                nearest_bottle, len(valid_detections), timestamp)
            
            # 更新FPS
            self._update_fps()
            
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
            # 添加FPS信息
            cv2.putText(image, f'FPS: {self.current_fps:.1f}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (0, 255, 0), 2)
            
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
    
    def _display_images(self, annotated_image, disparity):
        """显示图像窗口"""
        cv2.imshow('Bottle Detection', annotated_image)
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
        
        # 释放模型
        if hasattr(self, 'bottle_detector'):
            self.bottle_detector.release_model()
        
        # 关闭显示窗口
        if self.show_display:
            cv2.destroyAllWindows()
        
        super().destroy_node()

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = BottleDetectionNode()
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