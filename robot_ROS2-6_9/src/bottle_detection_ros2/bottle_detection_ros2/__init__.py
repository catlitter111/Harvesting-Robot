#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测ROS2包

集成了瓶子检测、机器人控制、舵机控制等功能模块
支持RKNN模型加速和异步处理
"""

# 版本信息
__version__ = '1.0.0'

# 核心视觉模块
from .core.vision.stereo_camera import StereoCamera
from .core.vision.bottle_detector import BottleDetector
from .utils.utils import (
    calculate_3d_position,
    calculate_distance,
    MedianFilter,
    draw_fps_info,
    draw_detection_info,
    compress_image_to_bytes,
    decompress_image_from_bytes,
    get_current_time_str,
    clamp_value,
    map_value,
    calculate_center_offset,
    convert_depth_to_distance,
    validate_detection_box,
    filter_detections_by_distance,
    merge_nearby_detections,
    calculate_iou,
    calculate_detection_area,
    normalize_detection_confidence,
    apply_detection_nms,
    format_detection_info,
    log_detection_stats
)

# 核心处理模块
from .core.processing.bottle_rknn_pool import BottleRKNNPoolExecutor

# 节点模块的公共接口可以在需要时导入
# 避免在包级别导入节点类，因为它们通常作为独立的可执行文件运行

__all__ = [
    'StereoCamera',
    'BottleDetector',
    'BottleRKNNPoolExecutor',
    'MedianFilter',
    'calculate_3d_position',
    'calculate_distance',
    'draw_fps_info',
    'draw_detection_info',
    'compress_image_to_bytes',
    'decompress_image_from_bytes',
    'get_current_time_str',
    'clamp_value',
    'map_value',
    'calculate_center_offset',
    'convert_depth_to_distance',
    'validate_detection_box',
    'filter_detections_by_distance',
    'merge_nearby_detections',
    'calculate_iou',
    'calculate_detection_area',
    'normalize_detection_confidence',
    'apply_detection_nms',
    'format_detection_info',
    'log_detection_stats'
]