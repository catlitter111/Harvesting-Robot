#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工具函数模块
提供图像处理、数据转换等通用功能
"""

import cv2
import numpy as np
import time
import math
import json
import base64
from datetime import datetime

# === 计算和数学相关函数 ===

def calculate_3d_position(disparity, u, v, camera_matrix, baseline=0.12):
    """
    根据视差计算3D位置
    
    参数:
    disparity: 视差值
    u, v: 像素坐标
    camera_matrix: 相机内参矩阵
    baseline: 基线距离（米）
    
    返回:
    (x, y, z) 3D坐标
    """
    if disparity <= 0:
        return None, None, None
    
    # 相机焦距
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    # 计算深度
    Z = (fx * baseline) / disparity
    
    # 计算3D坐标
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    
    return X, Y, Z

def calculate_distance(x1, y1, x2, y2):
    """
    计算两点之间的欧几里得距离
    
    参数:
    x1, y1: 第一个点的坐标
    x2, y2: 第二个点的坐标
    
    返回:
    两点之间的距离
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def clamp_value(value, min_val, max_val):
    """
    将值限制在指定范围内
    
    参数:
    value: 输入值
    min_val: 最小值
    max_val: 最大值
    
    返回:
    限制后的值
    """
    return max(min_val, min(value, max_val))

def map_value(value, in_min, in_max, out_min, out_max):
    """
    将值从一个范围映射到另一个范围
    
    参数:
    value: 输入值
    in_min, in_max: 输入范围
    out_min, out_max: 输出范围
    
    返回:
    映射后的值
    """
    if in_max == in_min:
        return out_min
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def convert_depth_to_distance(depth_value, depth_scale=1.0):
    """
    将深度值转换为实际距离
    
    参数:
    depth_value: 深度值
    depth_scale: 深度缩放因子
    
    返回:
    实际距离（米）
    """
    if depth_value <= 0:
        return None
    return depth_value * depth_scale

# === 检测相关函数 ===

def validate_detection_box(left, top, right, bottom, img_width, img_height):
    """
    验证检测框是否有效
    
    参数:
    left, top, right, bottom: 检测框坐标
    img_width, img_height: 图像尺寸
    
    返回:
    True如果有效，否则False
    """
    return (0 <= left < right <= img_width and 
            0 <= top < bottom <= img_height and
            (right - left) > 10 and 
            (bottom - top) > 10)

def filter_detections_by_distance(detections, min_distance=0.2, max_distance=5.0):
    """
    根据距离过滤检测结果
    
    参数:
    detections: 检测结果列表，每个元素包含距离信息
    min_distance: 最小距离
    max_distance: 最大距离
    
    返回:
    过滤后的检测结果
    """
    filtered = []
    for detection in detections:
        if len(detection) >= 8:  # 假设第8个元素是距离
            distance = detection[7]
            if min_distance <= distance <= max_distance:
                filtered.append(detection)
        else:
            filtered.append(detection)  # 如果没有距离信息，保留
    return filtered

def merge_nearby_detections(detections, distance_threshold=50):
    """
    合并近距离的检测结果
    
    参数:
    detections: 检测结果列表
    distance_threshold: 距离阈值
    
    返回:
    合并后的检测结果
    """
    if len(detections) <= 1:
        return detections
    
    merged = []
    used = [False] * len(detections)
    
    for i, det1 in enumerate(detections):
        if used[i]:
            continue
            
        group = [det1]
        used[i] = True
        
        cx1, cy1 = det1[5], det1[6]  # 假设第6、7个元素是中心坐标
        
        for j, det2 in enumerate(detections[i+1:], i+1):
            if used[j]:
                continue
                
            cx2, cy2 = det2[5], det2[6]
            distance = calculate_distance(cx1, cy1, cx2, cy2)
            
            if distance < distance_threshold:
                group.append(det2)
                used[j] = True
        
        # 如果有多个检测结果，选择置信度最高的
        if len(group) > 1:
            best_detection = max(group, key=lambda x: x[4])  # 第5个元素是置信度
            merged.append(best_detection)
        else:
            merged.append(group[0])
    
    return merged

def calculate_iou(box1, box2):
    """
    计算两个边界框的IoU（交并比）
    
    参数:
    box1, box2: 边界框 (left, top, right, bottom)
    
    返回:
    IoU值
    """
    left1, top1, right1, bottom1 = box1
    left2, top2, right2, bottom2 = box2
    
    # 计算交集
    left_i = max(left1, left2)
    top_i = max(top1, top2)
    right_i = min(right1, right2)
    bottom_i = min(bottom1, bottom2)
    
    if left_i >= right_i or top_i >= bottom_i:
        return 0.0
    
    intersection = (right_i - left_i) * (bottom_i - top_i)
    
    # 计算并集
    area1 = (right1 - left1) * (bottom1 - top1)
    area2 = (right2 - left2) * (bottom2 - top2)
    union = area1 + area2 - intersection
    
    return intersection / union if union > 0 else 0.0

def calculate_detection_area(detection):
    """
    计算检测框的面积
    
    参数:
    detection: 检测结果 (left, top, right, bottom, ...)
    
    返回:
    面积
    """
    left, top, right, bottom = detection[:4]
    return (right - left) * (bottom - top)

def normalize_detection_confidence(detections):
    """
    归一化检测置信度
    
    参数:
    detections: 检测结果列表
    
    返回:
    归一化后的检测结果
    """
    if not detections:
        return detections
    
    confidences = [det[4] for det in detections]  # 第5个元素是置信度
    max_conf = max(confidences)
    min_conf = min(confidences)
    
    if max_conf == min_conf:
        return detections
    
    normalized = []
    for det in detections:
        new_det = list(det)
        new_det[4] = (det[4] - min_conf) / (max_conf - min_conf)
        normalized.append(tuple(new_det))
    
    return normalized

def apply_detection_nms(detections, iou_threshold=0.3):
    """
    对检测结果应用非极大值抑制
    
    参数:
    detections: 检测结果列表
    iou_threshold: IoU阈值
    
    返回:
    NMS后的检测结果
    """
    if len(detections) <= 1:
        return detections
    
    # 按置信度排序
    sorted_detections = sorted(detections, key=lambda x: x[4], reverse=True)
    
    keep = []
    while sorted_detections:
        current = sorted_detections.pop(0)
        keep.append(current)
        
        # 移除与当前检测IoU过高的检测
        filtered = []
        for det in sorted_detections:
            iou = calculate_iou(current[:4], det[:4])
            if iou < iou_threshold:
                filtered.append(det)
        sorted_detections = filtered
    
    return keep

# === 图像处理和显示函数 ===

def draw_fps_info(image, fps, position=(10, 30)):
    """
    在图像上绘制FPS信息
    
    参数:
    image: 输入图像
    fps: 帧率值
    position: 文字位置
    """
    text = f"FPS: {fps:.1f}"
    cv2.putText(image, text, position, 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

def draw_detection_info(image, detection_count, nearest_distance=None, position=(10, 70)):
    """
    在图像上绘制检测信息
    
    参数:
    image: 输入图像
    detection_count: 检测数量
    nearest_distance: 最近距离
    position: 文字位置
    """
    text = f"Bottles: {detection_count}"
    cv2.putText(image, text, position, 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    if nearest_distance is not None and nearest_distance > 0:
        text = f"Nearest: {nearest_distance:.2f}m"
        cv2.putText(image, text, (position[0], position[1] + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

# === 数据处理函数 ===

def compress_image_to_bytes(image, quality=80):
    """
    将图像压缩为字节数据
    
    参数:
    image: 输入图像
    quality: JPEG质量
    
    返回:
    压缩后的字节数据
    """
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    _, buffer = cv2.imencode('.jpg', image, encode_param)
    return buffer.tobytes()

def decompress_image_from_bytes(byte_data):
    """
    从字节数据解压缩图像
    
    参数:
    byte_data: 字节数据
    
    返回:
    解压缩的图像
    """
    nparr = np.frombuffer(byte_data, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

def get_current_time_str():
    """
    获取当前时间字符串
    
    返回:
    格式化的时间字符串
    """
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def format_detection_info(detections):
    """
    格式化检测信息为JSON字符串
    
    参数:
    detections: 检测结果列表
    
    返回:
    JSON格式的字符串
    """
    info = {
        "timestamp": get_current_time_str(),
        "count": len(detections),
        "detections": []
    }
    
    for i, det in enumerate(detections):
        det_info = {
            "id": i,
            "left": int(det[0]),
            "top": int(det[1]),
            "right": int(det[2]),
            "bottom": int(det[3]),
            "confidence": float(det[4]),
            "center_x": int(det[5]),
            "center_y": int(det[6])
        }
        
        if len(det) > 7:
            det_info["distance"] = float(det[7])
        
        info["detections"].append(det_info)
    
    return json.dumps(info, indent=2)

def log_detection_stats(detections, logger=None):
    """
    记录检测统计信息
    
    参数:
    detections: 检测结果列表
    logger: 日志记录器（可选）
    """
    if logger:
        logger.info(f"检测到 {len(detections)} 个瓶子")
        if detections:
            confidences = [det[4] for det in detections]
            logger.info(f"置信度范围: {min(confidences):.2f} - {max(confidences):.2f}")
    else:
        print(f"检测到 {len(detections)} 个瓶子")

# === 原有的兼容性函数 ===

def calculate_fps(frame_count, start_time):
    """
    计算帧率
    
    参数:
    frame_count: 帧数
    start_time: 开始时间
    
    返回:
    fps值
    """
    elapsed_time = time.time() - start_time
    if elapsed_time > 0:
        return frame_count / elapsed_time
    return 0.0

def draw_fps(image, fps, position=(10, 30)):
    """在图像上绘制FPS（兼容性函数）"""
    return draw_fps_info(image, fps, position)

def draw_crosshair(image, center_x, center_y, size=20, color=(0, 255, 0), thickness=2):
    """
    绘制十字准星
    
    参数:
    image: 输入图像
    center_x, center_y: 中心坐标
    size: 准星大小
    color: 颜色
    thickness: 线条粗细
    """
    # 水平线
    cv2.line(image, 
             (center_x - size, center_y), 
             (center_x + size, center_y), 
             color, thickness)
    
    # 垂直线
    cv2.line(image, 
             (center_x, center_y - size), 
             (center_x, center_y + size), 
             color, thickness)

def draw_distance_bar(image, distance, max_distance=5.0, 
                     position=(50, 100), width=200, height=20):
    """
    绘制距离条
    
    参数:
    image: 输入图像
    distance: 当前距离
    max_distance: 最大距离
    position: 条形图位置
    width: 条形图宽度
    height: 条形图高度
    """
    x, y = position
    
    # 绘制背景框
    cv2.rectangle(image, (x, y), (x + width, y + height), (255, 255, 255), 2)
    
    if distance is not None and distance > 0:
        # 计算填充长度
        fill_width = int((distance / max_distance) * width)
        fill_width = min(fill_width, width)
        
        # 选择颜色（近距离绿色，远距离红色）
        if distance < 1.0:
            color = (0, 255, 0)  # 绿色
        elif distance < 3.0:
            color = (0, 255, 255)  # 黄色
        else:
            color = (0, 0, 255)  # 红色
        
        # 绘制填充
        cv2.rectangle(image, (x, y), (x + fill_width, y + height), color, -1)
        
        # 绘制距离文本
        text = f"{distance:.2f}m"
        cv2.putText(image, text, (x + width + 10, y + height - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def calculate_center_offset(image_width, object_x):
    """
    计算物体相对于图像中心的偏移
    
    参数:
    image_width: 图像宽度
    object_x: 物体x坐标
    
    返回:
    偏移量（正值表示物体在右侧，负值表示在左侧）
    """
    center_x = image_width // 2
    return object_x - center_x

# === 滤波器类 ===

class MovingAverage:
    """移动平均滤波器"""
    
    def __init__(self, window_size=5):
        """
        初始化移动平均滤波器
        
        参数:
        window_size: 窗口大小
        """
        self.window_size = window_size
        self.values = []
    
    def update(self, value):
        """
        更新滤波器并返回滤波后的值
        
        参数:
        value: 新的值
        
        返回:
        滤波后的值
        """
        if value is not None:
            self.values.append(value)
            if len(self.values) > self.window_size:
                self.values.pop(0)
        
        if self.values:
            return sum(self.values) / len(self.values)
        return None
    
    def reset(self):
        """重置滤波器"""
        self.values = []

class MedianFilter:
    """中值滤波器"""
    
    def __init__(self, window_size=5):
        """
        初始化中值滤波器
        
        参数:
        window_size: 窗口大小
        """
        self.window_size = window_size
        self.values = []
    
    def update(self, value):
        """
        更新滤波器并返回滤波后的值
        
        参数:
        value: 新的值
        
        返回:
        滤波后的值
        """
        if value is not None:
            self.values.append(value)
            if len(self.values) > self.window_size:
                self.values.pop(0)
        
        if self.values:
            sorted_values = sorted(self.values)
            n = len(sorted_values)
            if n % 2 == 0:
                return (sorted_values[n//2 - 1] + sorted_values[n//2]) / 2
            else:
                return sorted_values[n//2]
        return None
    
    def reset(self):
        """重置滤波器"""
        self.values = []