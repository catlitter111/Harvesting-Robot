#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QoS配置文件
针对不同场景优化的QoS配置
"""

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# 高频率低延迟QoS配置（舵机控制专用）
HIGH_FREQUENCY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 最佳努力，降低延迟
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,  # 只保留最新数据
    durability=QoSDurabilityPolicy.VOLATILE  # 易失性，减少开销
)

# 实时控制QoS配置（关键控制信号）
REALTIME_CONTROL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=3,  # 小缓冲区
    durability=QoSDurabilityPolicy.VOLATILE
)

# 状态更新QoS配置（状态信息）
STATUS_UPDATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE
) 