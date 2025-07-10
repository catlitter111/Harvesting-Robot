#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测RKNN线程池管理器
基于多线程异步处理提高检测帧率
"""

from queue import Queue
from rknnlite.api import RKNNLite
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import logging

logger = logging.getLogger("bottle_rknn_pool")

def init_bottle_rknn(model_path, id=0):
    """
    初始化单个RKNN实例
    
    参数:
    model_path -- RKNN模型路径
    id -- NPU核心ID (0, 1, 2)
    """
    rknn_lite = RKNNLite()
    ret = rknn_lite.load_rknn(model_path)
    if ret != 0:
        logger.error(f"加载RKNN模型失败: {model_path}")
        return None
    
    # 根据ID分配到不同的NPU核心
    if id == 0:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    elif id == 1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
    elif id == 2:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
    else:
        ret = rknn_lite.init_runtime()
    
    if ret != 0:
        logger.error("初始化运行时环境失败")
        return None
    
    logger.info(f"成功初始化RKNN实例 (NPU核心: {id})")
    return rknn_lite

def init_bottle_rknn_pool(model_path, thread_num=3):
    """
    初始化RKNN实例池
    
    参数:
    model_path -- RKNN模型路径
    thread_num -- 线程数量
    """
    rknn_list = []
    for i in range(thread_num):
        rknn = init_bottle_rknn(model_path, i % 3)  # 循环分配到3个NPU核心
        if rknn is None:
            # 如果初始化失败，释放已创建的实例
            for r in rknn_list:
                r.release()
            return None
        rknn_list.append(rknn)
    return rknn_list

class BottleRKNNPoolExecutor:
    """瓶子检测RKNN线程池执行器"""
    
    def __init__(self, model_path, detector_func, thread_num=3, queue_size=10):
        """
        初始化线程池执行器
        
        参数:
        model_path -- RKNN模型路径
        detector_func -- 检测处理函数
        thread_num -- 线程数量
        queue_size -- 队列大小
        """
        self.model_path = model_path
        self.thread_num = thread_num
        self.queue = Queue(maxsize=queue_size)
        self.result_queue = Queue()
        
        # 初始化RKNN实例池
        self.rknn_pool = init_bottle_rknn_pool(model_path, thread_num)
        if self.rknn_pool is None:
            raise RuntimeError("初始化RKNN池失败")
        
        # 创建线程池
        self.pool = ThreadPoolExecutor(max_workers=thread_num)
        self.detector_func = detector_func
        self.task_id = 0
        self.frame_id = 0
        
        logger.info(f"线程池初始化完成，线程数: {thread_num}")
    
    def put(self, frame):
        """
        提交一帧图像到处理队列
        
        参数:
        frame -- 输入图像帧
        
        返回:
        frame_id -- 帧ID，用于结果匹配
        """
        frame_id = self.frame_id
        self.frame_id += 1
        
        # 选择RKNN实例
        rknn_instance = self.rknn_pool[self.task_id % self.thread_num]
        self.task_id += 1
        
        # 提交任务到线程池
        future = self.pool.submit(self.detector_func, rknn_instance, frame, frame_id)
        self.queue.put((frame_id, future))
        
        return frame_id
    
    def get(self, timeout=None):
        """
        获取处理完成的结果
        
        参数:
        timeout -- 超时时间（秒）
        
        返回:
        (frame_id, result, success) -- 帧ID、检测结果、是否成功
        """
        if self.queue.empty():
            return None, None, False
        
        try:
            frame_id, future = self.queue.get(timeout=timeout)
            result = future.result(timeout=timeout)
            return frame_id, result, True
        except Exception as e:
            logger.error(f"获取结果失败: {e}")
            return None, None, False
    
    def get_latest(self):
        """
        获取最新的处理结果，丢弃旧的结果
        
        返回:
        (frame_id, result, success) -- 最新的检测结果
        """
        latest_result = None, None, False
        
        # 获取所有已完成的结果
        while not self.queue.empty():
            try:
                frame_id, future = self.queue.get_nowait()
                if future.done():
                    result = future.result()
                    latest_result = frame_id, result, True
            except:
                break
        
        return latest_result
    
    def release(self):
        """释放所有资源"""
        logger.info("正在释放线程池资源...")
        
        # 关闭线程池
        self.pool.shutdown(wait=True)
        
        # 释放所有RKNN实例
        for rknn in self.rknn_pool:
            rknn.release()
        
        logger.info("线程池资源已释放")
    
    def get_queue_size(self):
        """获取当前队列大小"""
        return self.queue.qsize()
    
    def is_full(self):
        """检查队列是否已满"""
        return self.queue.full()