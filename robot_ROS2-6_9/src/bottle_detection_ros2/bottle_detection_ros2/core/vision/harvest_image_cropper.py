#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
采摘图像截取器
负责根据检测结果截取水果图像，并生成采摘记录图片
"""

import cv2
import numpy as np
import base64
import time
import uuid
from typing import Tuple, Optional, Dict, Any
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import logging

logger = logging.getLogger(__name__)


class HarvestImageCropper:
    """采摘图像截取器类"""
    
    def __init__(self, 
                 crop_margin_ratio: float = 0.3,
                 min_crop_size: int = 100,
                 max_crop_size: int = 400,
                 jpeg_quality: int = 85):
        """
        初始化图像截取器
        
        Args:
            crop_margin_ratio: 截取边距比例（相对于检测框大小）
            min_crop_size: 最小截取尺寸
            max_crop_size: 最大截取尺寸
            jpeg_quality: JPEG压缩质量
        """
        self.crop_margin_ratio = crop_margin_ratio
        self.min_crop_size = min_crop_size
        self.max_crop_size = max_crop_size
        self.jpeg_quality = jpeg_quality
        
        logger.info(f"采摘图像截取器初始化完成 - 边距比例: {crop_margin_ratio}, "
                   f"尺寸范围: {min_crop_size}-{max_crop_size}")
    
    def crop_harvest_image(self, 
                          full_image: np.ndarray,
                          detection_box: Tuple[int, int, int, int],
                          item_type: str = "unknown",
                          confidence: float = 0.0,
                          distance: float = 0.0) -> Dict[str, Any]:
        """
        截取采摘图像
        
        Args:
            full_image: 完整的原始图像
            detection_box: 检测框 (x1, y1, x2, y2)
            item_type: 物品类型
            confidence: 检测置信度
            distance: 物品距离
            
        Returns:
            dict: 包含截取结果的字典
        """
        try:
            if full_image is None or full_image.size == 0:
                logger.error("输入图像为空")
                return None
            
            # 解析检测框
            x1, y1, x2, y2 = detection_box
            img_h, img_w = full_image.shape[:2]
            
            # 计算检测框中心和尺寸
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            box_w = x2 - x1
            box_h = y2 - y1
            
            # 计算截取区域（添加边距）
            margin_w = int(box_w * self.crop_margin_ratio)
            margin_h = int(box_h * self.crop_margin_ratio)
            
            # 计算最终截取尺寸（保持正方形）
            crop_size = max(box_w + margin_w * 2, box_h + margin_h * 2)
            crop_size = max(self.min_crop_size, min(crop_size, self.max_crop_size))
            
            # 计算截取区域边界
            crop_x1 = max(0, center_x - crop_size // 2)
            crop_y1 = max(0, center_y - crop_size // 2)
            crop_x2 = min(img_w, crop_x1 + crop_size)
            crop_y2 = min(img_h, crop_y1 + crop_size)
            
            # 调整起始位置以保持尺寸
            if crop_x2 - crop_x1 < crop_size:
                crop_x1 = max(0, crop_x2 - crop_size)
            if crop_y2 - crop_y1 < crop_size:
                crop_y1 = max(0, crop_y2 - crop_size)
            
            # 截取图像
            cropped_image = full_image[crop_y1:crop_y2, crop_x1:crop_x2]
            
            # 如果截取的图像太小，进行resize
            actual_h, actual_w = cropped_image.shape[:2]
            if actual_h < self.min_crop_size or actual_w < self.min_crop_size:
                target_size = max(self.min_crop_size, min(actual_w, actual_h))
                cropped_image = cv2.resize(cropped_image, (target_size, target_size))
                logger.info(f"图像resize到 {target_size}x{target_size}")
            
            # 在截取的图像上绘制标注
            annotated_crop = self._annotate_cropped_image(
                cropped_image.copy(), 
                item_type, 
                confidence, 
                distance
            )
            
            # 压缩为JPEG格式
            cropped_compressed = self._compress_image(annotated_crop)
            full_compressed = self._compress_image(full_image)
            
            # 生成采摘会话ID
            harvest_session_id = str(uuid.uuid4())[:8]
            
            # 构建结果
            result = {
                'harvest_session_id': harvest_session_id,
                'cropped_image': cropped_compressed,
                'full_image': full_compressed,
                'crop_region': {
                    'x': crop_x1,
                    'y': crop_y1,
                    'width': crop_x2 - crop_x1,
                    'height': crop_y2 - crop_y1
                },
                'item_info': {
                    'type': item_type,
                    'confidence': confidence,
                    'distance': distance,
                    'center_position': Point(x=float(center_x), y=float(center_y), z=0.0)
                },
                'timestamp': int(time.time() * 1000)
            }
            
            logger.info(f"成功截取采摘图像 - 会话ID: {harvest_session_id}, "
                       f"物品: {item_type}, 置信度: {confidence:.2f}, "
                       f"截取区域: {crop_x1},{crop_y1} -> {crop_x2},{crop_y2}")
            
            return result
            
        except Exception as e:
            logger.error(f"截取采摘图像失败: {e}")
            return None
    
    def _annotate_cropped_image(self, 
                               image: np.ndarray, 
                               item_type: str, 
                               confidence: float, 
                               distance: float) -> np.ndarray:
        """在截取的图像上添加标注信息"""
        try:
            h, w = image.shape[:2]
            
            # 绘制边框
            cv2.rectangle(image, (5, 5), (w-5, h-5), (0, 255, 0), 2)
            
            # 添加文字标注
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            
            # 物品类型
            text1 = f"Type: {item_type}"
            (tw1, th1), _ = cv2.getTextSize(text1, font, font_scale, thickness)
            cv2.rectangle(image, (8, 8), (8 + tw1 + 8, 8 + th1 + 8), (0, 0, 0), -1)
            cv2.putText(image, text1, (12, 8 + th1), font, font_scale, (0, 255, 0), thickness)
            
            # 置信度
            text2 = f"Conf: {confidence:.2f}"
            (tw2, th2), _ = cv2.getTextSize(text2, font, font_scale, thickness)
            cv2.rectangle(image, (8, 25), (8 + tw2 + 8, 25 + th2 + 8), (0, 0, 0), -1)
            cv2.putText(image, text2, (12, 25 + th2), font, font_scale, (255, 255, 0), thickness)
            
            # 距离
            if distance > 0:
                text3 = f"Dist: {distance:.2f}m"
                (tw3, th3), _ = cv2.getTextSize(text3, font, font_scale, thickness)
                cv2.rectangle(image, (8, 42), (8 + tw3 + 8, 42 + th3 + 8), (0, 0, 0), -1)
                cv2.putText(image, text3, (12, 42 + th3), font, font_scale, (255, 0, 255), thickness)
            
            # 时间戳
            timestamp_str = time.strftime("%H:%M:%S", time.localtime())
            text4 = f"Time: {timestamp_str}"
            (tw4, th4), _ = cv2.getTextSize(text4, font, font_scale, thickness)
            cv2.rectangle(image, (8, h - th4 - 16), (8 + tw4 + 8, h - 8), (0, 0, 0), -1)
            cv2.putText(image, text4, (12, h - 12), font, font_scale, (255, 255, 255), thickness)
            
            return image
            
        except Exception as e:
            logger.error(f"图像标注失败: {e}")
            return image
    
    def _compress_image(self, image: np.ndarray) -> CompressedImage:
        """压缩图像为ROS CompressedImage消息"""
        try:
            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, encoded_img = cv2.imencode('.jpg', image, encode_param)
            
            # 创建CompressedImage消息
            compressed_msg = CompressedImage()
            compressed_msg.header = Header()
            compressed_msg.header.stamp.sec = int(time.time())
            compressed_msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_img.tobytes()
            
            return compressed_msg
            
        except Exception as e:
            logger.error(f"图像压缩失败: {e}")
            return CompressedImage()
    
    def create_harvest_summary_image(self, 
                                   cropped_images: list, 
                                   session_info: dict) -> Optional[np.ndarray]:
        """
        创建采摘总结图像（多个截取图像的拼接）
        
        Args:
            cropped_images: 截取的图像列表
            session_info: 会话信息
            
        Returns:
            拼接后的总结图像
        """
        try:
            if not cropped_images:
                return None
            
            # 计算拼接布局
            num_images = len(cropped_images)
            cols = min(3, num_images)
            rows = (num_images + cols - 1) // cols
            
            # 统一图像尺寸
            target_size = 200
            resized_images = []
            
            for img in cropped_images:
                if img.size > 0:
                    resized = cv2.resize(img, (target_size, target_size))
                    resized_images.append(resized)
            
            if not resized_images:
                return None
            
            # 创建拼接画布
            canvas_w = cols * target_size + (cols + 1) * 10
            canvas_h = rows * target_size + (rows + 1) * 10 + 60  # 额外空间用于标题
            canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
            canvas.fill(50)  # 深灰色背景
            
            # 添加标题
            title = f"Harvest Summary - {session_info.get('count', len(resized_images))} items"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            thickness = 2
            (tw, th), _ = cv2.getTextSize(title, font, font_scale, thickness)
            title_x = (canvas_w - tw) // 2
            cv2.putText(canvas, title, (title_x, 35), font, font_scale, (255, 255, 255), thickness)
            
            # 拼接图像
            for i, img in enumerate(resized_images):
                row = i // cols
                col = i % cols
                
                x = col * target_size + (col + 1) * 10
                y = row * target_size + (row + 1) * 10 + 60
                
                canvas[y:y+target_size, x:x+target_size] = img
                
                # 添加编号
                number = str(i + 1)
                cv2.circle(canvas, (x + 20, y + 20), 15, (0, 255, 0), -1)
                cv2.putText(canvas, number, (x + 15, y + 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            return canvas
            
        except Exception as e:
            logger.error(f"创建采摘总结图像失败: {e}")
            return None


def test_harvest_image_cropper():
    """测试函数"""
    # 创建测试图像
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    detection_box = (200, 150, 350, 300)  # x1, y1, x2, y2
    
    # 创建截取器
    cropper = HarvestImageCropper()
    
    # 测试截取
    result = cropper.crop_harvest_image(
        test_image, 
        detection_box,
        item_type="apple",
        confidence=0.95,
        distance=1.2
    )
    
    if result:
        print(f"测试成功 - 会话ID: {result['harvest_session_id']}")
        print(f"截取区域: {result['crop_region']}")
        print(f"时间戳: {result['timestamp']}")
    else:
        print("测试失败")


if __name__ == "__main__":
    test_harvest_image_cropper() 