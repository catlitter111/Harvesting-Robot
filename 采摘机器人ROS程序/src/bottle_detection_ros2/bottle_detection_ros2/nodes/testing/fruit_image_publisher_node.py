#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
水果图片发布节点
定期发布指定文件夹中的水果照片，供AI识别系统处理
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import cv2
import os
import glob
import time
import threading
from pathlib import Path


class FruitImagePublisherNode(Node):
    """水果图片发布节点"""
    
    def __init__(self):
        super().__init__('fruit_image_publisher_node')
        
        # 声明参数
        self.declare_parameter('image_folder_path', '/home/robot/fruit_images')
        self.declare_parameter('publish_interval', 5.0)  # 发布间隔（秒）
        self.declare_parameter('loop_images', True)  # 是否循环发布
        self.declare_parameter('image_quality', 80)  # JPEG压缩质量
        self.declare_parameter('max_image_size', 1024)  # 最大图片尺寸（像素）
        self.declare_parameter('supported_formats', ['jpg', 'jpeg', 'png', 'bmp'])
        
        # 获取参数
        self.image_folder_path = self.get_parameter('image_folder_path').value
        self.publish_interval = self.get_parameter('publish_interval').value
        self.loop_images = self.get_parameter('loop_images').value
        self.image_quality = self.get_parameter('image_quality').value
        self.max_image_size = self.get_parameter('max_image_size').value
        self.supported_formats = self.get_parameter('supported_formats').value
        
        # 创建发布者
        self.image_pub = self.create_publisher(
            CompressedImage,
            'fruit_detection/raw_image',
            10
        )
        
        # 图片列表
        self.image_files = []
        self.current_index = 0
        
        # 加载图片文件列表
        self.load_image_files()
        
        # 如果有图片文件，开始发布
        if self.image_files:
            self.get_logger().info(f'找到 {len(self.image_files)} 张图片，开始发布')
            # 创建定时器
            self.timer = self.create_timer(self.publish_interval, self.publish_next_image)
        else:
            self.get_logger().warn(f'在文件夹 {self.image_folder_path} 中未找到支持的图片文件')
    
    def load_image_files(self):
        """加载图片文件列表"""
        try:
            # 检查文件夹是否存在
            if not os.path.exists(self.image_folder_path):
                self.get_logger().error(f'图片文件夹不存在: {self.image_folder_path}')
                return
            
            # 支持的图片格式
            image_extensions = []
            for fmt in self.supported_formats:
                image_extensions.extend([f'*.{fmt}', f'*.{fmt.upper()}'])
            
            # 搜索所有支持的图片文件
            self.image_files = []
            for extension in image_extensions:
                pattern = os.path.join(self.image_folder_path, extension)
                files = glob.glob(pattern)
                self.image_files.extend(files)
            
            # 按文件名排序
            self.image_files.sort()
            
            # 记录找到的文件
            if self.image_files:
                self.get_logger().info(f'加载了 {len(self.image_files)} 张图片:')
                for i, img_file in enumerate(self.image_files[:5]):  # 只显示前5个
                    filename = os.path.basename(img_file)
                    self.get_logger().info(f'  {i+1}. {filename}')
                if len(self.image_files) > 5:
                    self.get_logger().info(f'  ... 还有 {len(self.image_files)-5} 张图片')
            else:
                self.get_logger().warn(f'在 {self.image_folder_path} 中未找到支持的图片文件')
                self.get_logger().info(f'支持的格式: {", ".join(self.supported_formats)}')
                
        except Exception as e:
            self.get_logger().error(f'加载图片文件失败: {e}')
    
    def resize_image_if_needed(self, image):
        """如果图片太大，调整大小"""
        height, width = image.shape[:2]
        
        # 如果图片尺寸超过限制，按比例缩放
        if max(height, width) > self.max_image_size:
            scale = self.max_image_size / max(height, width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            self.get_logger().debug(f'图片已调整大小: {width}x{height} -> {new_width}x{new_height}')
        
        return image
    
    def publish_next_image(self):
        """发布下一张图片"""
        if not self.image_files:
            self.get_logger().warn('没有可发布的图片')
            return
        
        try:
            # 获取当前图片文件路径
            current_image_path = self.image_files[self.current_index]
            filename = os.path.basename(current_image_path)
            
            self.get_logger().info(f'发布图片: {filename} ({self.current_index + 1}/{len(self.image_files)})')
            
            # 读取图片
            image = cv2.imread(current_image_path)
            if image is None:
                self.get_logger().error(f'无法读取图片: {current_image_path}')
                self.move_to_next_image()
                return
            
            # 调整图片大小（如果需要）
            image = self.resize_image_if_needed(image)
            
            # 压缩为JPEG格式
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.image_quality]
            success, encoded_image = cv2.imencode('.jpg', image, encode_param)
            
            if not success:
                self.get_logger().error(f'图片编码失败: {current_image_path}')
                self.move_to_next_image()
                return
            
            # 创建CompressedImage消息
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'fruit_camera'
            msg.format = 'jpeg'
            msg.data = encoded_image.tobytes()
            
            # 添加自定义信息到header（通过frame_id传递文件名）
            # 格式: "fruit_camera|filename.jpg"
            msg.header.frame_id = f'fruit_camera|{filename}'
            
            # 发布图片
            self.image_pub.publish(msg)
            
            # 计算文件大小
            file_size_kb = len(encoded_image) / 1024
            self.get_logger().info(f'图片发布成功: {filename}, 大小: {file_size_kb:.1f}KB')
            
            # 移动到下一张图片
            self.move_to_next_image()
            
        except Exception as e:
            self.get_logger().error(f'发布图片时出错: {e}')
            self.move_to_next_image()
    
    def move_to_next_image(self):
        """移动到下一张图片"""
        self.current_index += 1
        
        # 如果到达列表末尾
        if self.current_index >= len(self.image_files):
            if self.loop_images:
                # 循环到第一张
                self.current_index = 0
                self.get_logger().info('已循环到第一张图片')
            else:
                # 停止发布
                self.get_logger().info('所有图片已发布完成，停止发布')
                if hasattr(self, 'timer'):
                    self.timer.cancel()
    
    def get_image_info(self):
        """获取图片信息"""
        if not self.image_files:
            return "无图片文件"
        
        total = len(self.image_files)
        current = self.current_index + 1 if self.current_index < total else total
        
        return f"图片 {current}/{total}"
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FruitImagePublisherNode()
        
        # 打印节点信息
        node.get_logger().info('=== 水果图片发布节点已启动 ===')
        node.get_logger().info(f'图片文件夹: {node.image_folder_path}')
        node.get_logger().info(f'发布间隔: {node.publish_interval}秒')
        node.get_logger().info(f'循环发布: {"是" if node.loop_images else "否"}')
        node.get_logger().info(f'图片质量: {node.image_quality}%')
        node.get_logger().info(f'最大尺寸: {node.max_image_size}像素')
        node.get_logger().info(f'当前状态: {node.get_image_info()}')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    except Exception as e:
        node.get_logger().error(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 