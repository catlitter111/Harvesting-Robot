#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像传递桥接器
提供高效的函数调用方式图像传递，减少ROS2话题订阅的延迟
同时保持完全的向后兼容性
"""

import threading
import time
import logging
from typing import Optional, Callable, Dict, Any
import weakref

logger = logging.getLogger(__name__)

class ImageBridgeSingleton:
    """图像传递桥接器单例类"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
            
        # 回调函数注册表 - 使用弱引用避免循环引用
        self._image_callbacks: Dict[str, Callable] = {}
        self._callback_lock = threading.RLock()
        
        # 统计信息
        self._stats = {
            'total_images_sent': 0,
            'total_callbacks_executed': 0,
            'last_image_time': 0,
            'average_callback_time': 0.0,
            'max_callback_time': 0.0,
            'failed_callbacks': 0
        }
        
        # 状态
        self._active = True
        
        self._initialized = True
        logger.info("ImageBridge 单例实例已创建")
    
    def register_image_callback(self, callback_id: str, callback_func: Callable[[bytes, str, float], None]) -> bool:
        """
        注册图像回调函数
        
        Args:
            callback_id: 回调函数的唯一标识
            callback_func: 回调函数，参数为 (image_data: bytes, format: str, timestamp: float)
        
        Returns:
            bool: 注册是否成功
        """
        try:
            with self._callback_lock:
                if callback_id in self._image_callbacks:
                    logger.warning(f"回调ID '{callback_id}' 已存在，将被覆盖")
                
                self._image_callbacks[callback_id] = callback_func
                logger.info(f"图像回调已注册: {callback_id}")
                return True
                
        except Exception as e:
            logger.error(f"注册图像回调失败 {callback_id}: {e}")
            return False
    
    def unregister_image_callback(self, callback_id: str) -> bool:
        """
        取消注册图像回调函数
        
        Args:
            callback_id: 要取消的回调函数ID
            
        Returns:
            bool: 取消注册是否成功
        """
        try:
            with self._callback_lock:
                if callback_id in self._image_callbacks:
                    del self._image_callbacks[callback_id]
                    logger.info(f"图像回调已取消注册: {callback_id}")
                    return True
                else:
                    logger.warning(f"要取消的回调ID不存在: {callback_id}")
                    return False
                    
        except Exception as e:
            logger.error(f"取消注册图像回调失败 {callback_id}: {e}")
            return False
    
    def send_image(self, image_data: bytes, image_format: str = "jpeg", 
                   timestamp: Optional[float] = None, source: str = "unknown") -> int:
        """
        发送图像数据给所有注册的回调函数
        
        Args:
            image_data: 压缩后的图像数据
            image_format: 图像格式 (jpeg, png等)
            timestamp: 时间戳，None则使用当前时间
            source: 图像来源标识
            
        Returns:
            int: 成功执行的回调函数数量
        """
        if not self._active:
            return 0
            
        if timestamp is None:
            timestamp = time.time()
        
        executed_count = 0
        total_callback_time = 0.0
        
        # 更新统计信息
        self._stats['total_images_sent'] += 1
        self._stats['last_image_time'] = timestamp
        
        # 获取回调函数列表的副本，避免在执行过程中被修改
        with self._callback_lock:
            callbacks = list(self._image_callbacks.items())
        
        if not callbacks:
            logger.debug(f"没有注册的图像回调函数，跳过图像发送 (来源: {source})")
            return 0
        
        logger.debug(f"发送图像给 {len(callbacks)} 个回调函数 (来源: {source}, 大小: {len(image_data)} bytes)")
        
        # 执行所有回调函数
        for callback_id, callback_func in callbacks:
            try:
                start_time = time.time()
                
                # 调用回调函数
                callback_func(image_data, image_format, timestamp)
                
                end_time = time.time()
                callback_time = end_time - start_time
                total_callback_time += callback_time
                
                # 更新统计信息
                if callback_time > self._stats['max_callback_time']:
                    self._stats['max_callback_time'] = callback_time
                
                executed_count += 1
                logger.debug(f"回调函数 '{callback_id}' 执行成功 (耗时: {callback_time*1000:.2f}ms)")
                
            except Exception as e:
                logger.error(f"执行图像回调函数失败 '{callback_id}': {e}")
                self._stats['failed_callbacks'] += 1
        
        # 更新平均回调时间
        self._stats['total_callbacks_executed'] += executed_count
        if executed_count > 0:
            avg_time = total_callback_time / executed_count
            # 使用指数移动平均
            if self._stats['average_callback_time'] == 0:
                self._stats['average_callback_time'] = avg_time
            else:
                alpha = 0.1  # 平滑因子
                self._stats['average_callback_time'] = (
                    alpha * avg_time + (1 - alpha) * self._stats['average_callback_time']
                )
        
        if executed_count > 0:
            logger.debug(f"图像发送完成: {executed_count}/{len(callbacks)} 个回调成功执行")
        
        return executed_count
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._callback_lock:
            stats = self._stats.copy()
            stats['registered_callbacks'] = len(self._image_callbacks)
            stats['callback_ids'] = list(self._image_callbacks.keys())
        
        return stats
    
    def get_registered_callbacks(self) -> list:
        """获取已注册的回调函数ID列表"""
        with self._callback_lock:
            return list(self._image_callbacks.keys())
    
    def clear_stats(self):
        """清空统计信息"""
        self._stats = {
            'total_images_sent': 0,
            'total_callbacks_executed': 0,
            'last_image_time': 0,
            'average_callback_time': 0.0,
            'max_callback_time': 0.0,
            'failed_callbacks': 0
        }
        logger.info("ImageBridge 统计信息已清空")
    
    def set_active(self, active: bool):
        """设置桥接器是否激活"""
        self._active = active
        status = "激活" if active else "停用"
        logger.info(f"ImageBridge 已{status}")
    
    def is_active(self) -> bool:
        """检查桥接器是否激活"""
        return self._active
    
    def cleanup(self):
        """清理资源"""
        logger.info("正在清理 ImageBridge 资源...")
        
        with self._callback_lock:
            callback_count = len(self._image_callbacks)
            self._image_callbacks.clear()
        
        self._active = False
        logger.info(f"ImageBridge 清理完成，移除了 {callback_count} 个回调函数")


# 全局单例实例
def get_image_bridge() -> ImageBridgeSingleton:
    """获取图像桥接器全局单例实例"""
    return ImageBridgeSingleton()


# 便捷函数
def register_image_callback(callback_id: str, callback_func: Callable[[bytes, str, float], None]) -> bool:
    """注册图像回调函数的便捷函数"""
    return get_image_bridge().register_image_callback(callback_id, callback_func)


def unregister_image_callback(callback_id: str) -> bool:
    """取消注册图像回调函数的便捷函数"""
    return get_image_bridge().unregister_image_callback(callback_id)


def send_image(image_data: bytes, image_format: str = "jpeg", 
               timestamp: Optional[float] = None, source: str = "unknown") -> int:
    """发送图像的便捷函数"""
    return get_image_bridge().send_image(image_data, image_format, timestamp, source)


def get_bridge_stats() -> Dict[str, Any]:
    """获取桥接器统计信息的便捷函数"""
    return get_image_bridge().get_stats()


if __name__ == "__main__":
    # 测试代码
    import json
    
    def test_callback(image_data: bytes, format: str, timestamp: float):
        print(f"收到图像: 大小={len(image_data)} bytes, 格式={format}, 时间戳={timestamp}")
    
    # 注册回调
    bridge = get_image_bridge()
    bridge.register_image_callback("test_callback", test_callback)
    
    # 发送测试图像
    test_data = b"test_image_data"
    result = bridge.send_image(test_data, "jpeg", source="test")
    print(f"发送结果: {result}")
    
    # 显示统计信息
    stats = bridge.get_stats()
    print("统计信息:")
    print(json.dumps(stats, indent=2, ensure_ascii=False))
    
    # 清理
    bridge.cleanup() 