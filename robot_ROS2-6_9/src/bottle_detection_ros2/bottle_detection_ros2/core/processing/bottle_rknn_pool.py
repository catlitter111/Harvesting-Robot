#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测BPU线程池管理器
基于多线程异步处理提高检测帧率
适配RDK X5板端
"""

from queue import Queue
from hobot_dnn import pyeasy_dnn as dnn
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import logging
import time

logger = logging.getLogger("bottle_rknn_pool")

def init_bottle_rknn(model_path, id=0):
    """
    初始化单个BPU实例
    
    参数:
    model_path -- BPU模型路径 (.bin格式)
    id -- 实例ID (用于标识，RDK X5不支持多核心分配)
    """
    try:
        begin_time = time.time()
        # 加载BPU模型
        quantize_model = dnn.load(model_path)
        load_time = time.time() - begin_time
        
        logger.info(f"✓ 成功初始化BPU实例 {id}: {load_time*1000:.2f}ms")
        
        # 创建一个包装器，保持与原API的兼容性
        class BPUWrapper:
            def __init__(self, model):
                self.model = model
                self.id = id
                
                # 准备反量化系数
                self.s_bboxes_scale = model[0].outputs[1].properties.scale_data[np.newaxis, :]
                self.m_bboxes_scale = model[0].outputs[3].properties.scale_data[np.newaxis, :]
                self.l_bboxes_scale = model[0].outputs[5].properties.scale_data[np.newaxis, :]
                
                # DFL求期望的系数
                self.weights_static = np.array([i for i in range(16)]).astype(np.float32)[np.newaxis, np.newaxis, :]
                
                # 生成anchors
                self._generate_anchors()
                
                # 获取模型输入尺寸
                self.input_H, self.input_W = model[0].inputs[0].properties.shape[2:4]
                
                # 检测参数
                self.SCORE_THRESHOLD = 0.02
                self.NMS_THRESHOLD = 0.45
                self.CONF_THRES_RAW = -np.log(1/self.SCORE_THRESHOLD - 1)
                self.REG = 16
                self.CLASSES_NUM = 80
            
            def _generate_anchors(self):
                """生成anchor点"""
                self.s_anchor = np.stack([
                    np.tile(np.linspace(0.5, 79.5, 80), reps=80), 
                    np.repeat(np.arange(0.5, 80.5, 1), 80)
                ], axis=0).transpose(1, 0)
                
                self.m_anchor = np.stack([
                    np.tile(np.linspace(0.5, 39.5, 40), reps=40), 
                    np.repeat(np.arange(0.5, 40.5, 1), 40)
                ], axis=0).transpose(1, 0)
                
                self.l_anchor = np.stack([
                    np.tile(np.linspace(0.5, 19.5, 20), reps=20), 
                    np.repeat(np.arange(0.5, 20.5, 1), 20)
                ], axis=0).transpose(1, 0)
            
            def inference(self, input_data):
                """执行推理，保持与RKNN API兼容"""
                # input_data是list格式，取第一个元素
                input_tensor = input_data[0]
                
                # BPU推理
                outputs = self.model[0].forward(input_tensor)
                
                # 转换为numpy数组列表，保持与RKNN格式兼容
                return [dnnTensor.buffer for dnnTensor in outputs]
            
            def release(self):
                """释放BPU资源"""
                # BPU模型会自动释放
                self.model = None
                logger.info(f"BPU实例 {self.id} 已释放")
        
        return BPUWrapper(quantize_model)
        
    except Exception as e:
        logger.error(f"❌ 初始化BPU实例 {id} 失败: {e}")
        return None

def init_bottle_rknn_pool(model_path, thread_num=3):
    """
    初始化BPU实例池
    
    参数:
    model_path -- BPU模型路径
    thread_num -- 线程数量
    """
    rknn_list = []
    for i in range(thread_num):
        rknn = init_bottle_rknn(model_path, i)
        if rknn is None:
            # 如果初始化失败，释放已创建的实例
            for r in rknn_list:
                r.release()
            return None
        rknn_list.append(rknn)
    
    logger.info(f"✓ BPU实例池初始化完成，共 {len(rknn_list)} 个实例")
    return rknn_list

class BottleRKNNPoolExecutor:
    """瓶子检测BPU线程池执行器"""
    
    def __init__(self, model_path, detector_func, thread_num=3, queue_size=10):
        """
        初始化线程池执行器
        
        参数:
        model_path -- BPU模型路径
        detector_func -- 检测处理函数
        thread_num -- 线程数量
        queue_size -- 队列大小
        """
        self.model_path = model_path
        self.thread_num = thread_num
        self.queue = Queue(maxsize=queue_size)
        self.result_queue = Queue()
        
        # 初始化BPU实例池
        self.rknn_pool = init_bottle_rknn_pool(model_path, thread_num)
        if self.rknn_pool is None:
            raise RuntimeError("初始化BPU池失败")
        
        # 创建线程池
        self.pool = ThreadPoolExecutor(max_workers=thread_num)
        self.detector_func = detector_func
        self.task_id = 0
        self.frame_id = 0
        
        logger.info(f"✓ BPU线程池初始化完成，线程数: {thread_num}")
    
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
        
        # 轮询选择BPU实例
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
        logger.info("正在释放BPU线程池资源...")
        
        # 关闭线程池
        self.pool.shutdown(wait=True)
        
        # 释放所有BPU实例
        for rknn in self.rknn_pool:
            rknn.release()
        
        logger.info("✓ BPU线程池资源已释放")
    
    def get_queue_size(self):
        """获取当前队列大小"""
        return self.queue.qsize()
    
    def is_full(self):
        """检查队列是否已满"""
        return self.queue.full()