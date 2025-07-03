#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
异步瓶子检测器 - 优化版
使用BPU模型池进行高效的异步检测
解决帧数跳变问题，提供平滑的性能表现
"""

import cv2
import numpy as np
import logging
import time
import sys
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from scipy.special import softmax
from collections import deque
import threading

# 导入BPU线程池管理器
try:
    from bottle_detection_ros2.core.processing.bottle_rknn_pool import BottleRKNNPoolExecutor
except ImportError:
    print("错误: 无法导入bottle_rknn_pool模块")
    print("请确保bottle_rknn_pool.py文件在同一目录下")
    sys.exit(1)

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("bottle_detector_async")

# YOLO检测参数
OBJ_THRESH = 0.02  # 目标置信度阈值
NMS_THRESH = 0.5  # 非极大值抑制阈值
MODEL_SIZE = (640, 640)  # 模型输入尺寸

# COCO类别
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
           'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
           'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
           'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
           'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
           'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
           'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
           'hair drier', 'toothbrush']

# 颜色调色板
color_palette = np.random.uniform(0, 255, size=(len(CLASSES), 3))

class FPSCounter:
    """平滑FPS计算器"""
    
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.frame_times = deque(maxlen=window_size)
        self.last_time = time.time()
        self.fps = 0.0
        self.lock = threading.Lock()
    
    def update(self):
        """更新FPS计算"""
        current_time = time.time()
        with self.lock:
            delta = current_time - self.last_time
            if delta > 0:
                self.frame_times.append(delta)
                self.last_time = current_time
                
                # 计算平滑FPS
                if len(self.frame_times) > 1:
                    avg_delta = sum(self.frame_times) / len(self.frame_times)
                    self.fps = 1.0 / avg_delta if avg_delta > 0 else 0.0
    
    def get_fps(self):
        """获取当前FPS"""
        with self.lock:
            return self.fps
    
    def reset(self):
        """重置计数器"""
        with self.lock:
            self.frame_times.clear()
            self.last_time = time.time()
            self.fps = 0.0

class FrameBuffer:
    """帧缓冲管理器"""
    
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.buffer = {}
        self.lock = threading.Lock()
        self.next_display_id = 0
    
    def put(self, frame_id, frame):
        """添加帧到缓冲区"""
        with self.lock:
            self.buffer[frame_id] = frame
            # 清理过期帧
            if len(self.buffer) > self.max_size:
                # 删除最旧的帧
                oldest_id = min(self.buffer.keys())
                if oldest_id < self.next_display_id:
                    del self.buffer[oldest_id]
    
    def get(self, frame_id):
        """获取指定帧"""
        with self.lock:
            return self.buffer.pop(frame_id, None)
    
    def get_next_available(self):
        """获取下一个可用帧（按顺序）"""
        with self.lock:
            # 尝试获取期望的下一帧
            if self.next_display_id in self.buffer:
                frame = self.buffer.pop(self.next_display_id)
                self.next_display_id += 1
                return self.next_display_id - 1, frame
            
            # 如果没有期望帧，获取最早的可用帧
            if self.buffer:
                available_ids = sorted(self.buffer.keys())
                # 跳过过早的帧，获取最接近期望ID的帧
                for frame_id in available_ids:
                    if frame_id >= self.next_display_id:
                        frame = self.buffer.pop(frame_id)
                        self.next_display_id = frame_id + 1
                        return frame_id, frame
                
                # 如果所有帧都太早，获取最新的
                latest_id = max(available_ids)
                frame = self.buffer.pop(latest_id)
                self.next_display_id = latest_id + 1
                return latest_id, frame
            
            return None, None
    
    def size(self):
        """获取缓冲区大小"""
        with self.lock:
            return len(self.buffer)

def bgr2nv12(bgr_img):
    """BGR转NV12格式，适配BPU输入要求"""
    height, width = bgr_img.shape[0], bgr_img.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
    
    nv12 = np.zeros_like(yuv420p)
    nv12[:height * width] = y
    nv12[height * width:] = uv_packed
    
    return nv12

def letter_box(im, new_shape=(640, 640), pad_color=(114, 114, 114)):
    """调整图像大小并保持宽高比，使用padding填充"""
    shape = im.shape[:2]  # 当前形状 [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
        
    # 计算缩放比例
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    
    # 计算新的未填充尺寸
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # 宽高填充
    dw /= 2  # 两边均匀填充
    dh /= 2
    
    if shape[::-1] != new_unpad:  # 调整大小
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        
    # 添加边框
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)
    
    return im, r, (dw, dh)

def bpu_post_process(outputs, bpu_instance, img_w, img_h, scale, pad):
    """BPU输出后处理，适配多尺度输出"""
    try:
        # Reshape输出 - 适配YOLO11的6个输出
        s_clses = outputs[0].reshape(-1, bpu_instance.CLASSES_NUM)
        s_bboxes = outputs[1].reshape(-1, bpu_instance.REG * 4)
        m_clses = outputs[2].reshape(-1, bpu_instance.CLASSES_NUM)
        m_bboxes = outputs[3].reshape(-1, bpu_instance.REG * 4)
        l_clses = outputs[4].reshape(-1, bpu_instance.CLASSES_NUM)
        l_bboxes = outputs[5].reshape(-1, bpu_instance.REG * 4)

        # 分类阈值筛选
        s_max_scores = np.max(s_clses, axis=1)
        s_valid_indices = np.flatnonzero(s_max_scores >= bpu_instance.CONF_THRES_RAW)
        s_ids = np.argmax(s_clses[s_valid_indices, :], axis=1) if len(s_valid_indices) > 0 else np.array([])
        s_scores = s_max_scores[s_valid_indices]

        m_max_scores = np.max(m_clses, axis=1)
        m_valid_indices = np.flatnonzero(m_max_scores >= bpu_instance.CONF_THRES_RAW)
        m_ids = np.argmax(m_clses[m_valid_indices, :], axis=1) if len(m_valid_indices) > 0 else np.array([])
        m_scores = m_max_scores[m_valid_indices]

        l_max_scores = np.max(l_clses, axis=1)
        l_valid_indices = np.flatnonzero(l_max_scores >= bpu_instance.CONF_THRES_RAW)
        l_ids = np.argmax(l_clses[l_valid_indices, :], axis=1) if len(l_valid_indices) > 0 else np.array([])
        l_scores = l_max_scores[l_valid_indices]

        # 如果没有有效检测，返回None
        if len(s_scores) == 0 and len(m_scores) == 0 and len(l_scores) == 0:
            return None, None, None

        # Sigmoid激活
        s_scores = 1 / (1 + np.exp(-s_scores)) if len(s_scores) > 0 else np.array([])
        m_scores = 1 / (1 + np.exp(-m_scores)) if len(m_scores) > 0 else np.array([])
        l_scores = 1 / (1 + np.exp(-l_scores)) if len(l_scores) > 0 else np.array([])

        # 反量化
        s_bboxes_float32 = s_bboxes[s_valid_indices,:].astype(np.float32) * bpu_instance.s_bboxes_scale if len(s_valid_indices) > 0 else np.array([]).reshape(0, 64)
        m_bboxes_float32 = m_bboxes[m_valid_indices,:].astype(np.float32) * bpu_instance.m_bboxes_scale if len(m_valid_indices) > 0 else np.array([]).reshape(0, 64)
        l_bboxes_float32 = l_bboxes[l_valid_indices,:].astype(np.float32) * bpu_instance.l_bboxes_scale if len(l_valid_indices) > 0 else np.array([]).reshape(0, 64)

        # dist2bbox - DFL解码
        all_dbboxes = []
        all_scores = []
        all_ids = []

        # 处理small尺度
        if len(s_valid_indices) > 0:
            s_ltrb_indices = np.sum(softmax(s_bboxes_float32.reshape(-1, 4, 16), axis=2) * bpu_instance.weights_static, axis=2)
            s_anchor_indices = bpu_instance.s_anchor[s_valid_indices, :]
            s_x1y1 = s_anchor_indices - s_ltrb_indices[:, 0:2]
            s_x2y2 = s_anchor_indices + s_ltrb_indices[:, 2:4]
            s_dbboxes = np.hstack([s_x1y1, s_x2y2]) * 8
            all_dbboxes.append(s_dbboxes)
            all_scores.append(s_scores)
            all_ids.append(s_ids)

        # 处理medium尺度
        if len(m_valid_indices) > 0:
            m_ltrb_indices = np.sum(softmax(m_bboxes_float32.reshape(-1, 4, 16), axis=2) * bpu_instance.weights_static, axis=2)
            m_anchor_indices = bpu_instance.m_anchor[m_valid_indices, :]
            m_x1y1 = m_anchor_indices - m_ltrb_indices[:, 0:2]
            m_x2y2 = m_anchor_indices + m_ltrb_indices[:, 2:4]
            m_dbboxes = np.hstack([m_x1y1, m_x2y2]) * 16
            all_dbboxes.append(m_dbboxes)
            all_scores.append(m_scores)
            all_ids.append(m_ids)

        # 处理large尺度
        if len(l_valid_indices) > 0:
            l_ltrb_indices = np.sum(softmax(l_bboxes_float32.reshape(-1, 4, 16), axis=2) * bpu_instance.weights_static, axis=2)
            l_anchor_indices = bpu_instance.l_anchor[l_valid_indices,:]
            l_x1y1 = l_anchor_indices - l_ltrb_indices[:, 0:2]
            l_x2y2 = l_anchor_indices + l_ltrb_indices[:, 2:4]
            l_dbboxes = np.hstack([l_x1y1, l_x2y2]) * 32
            all_dbboxes.append(l_dbboxes)
            all_scores.append(l_scores)
            all_ids.append(l_ids)

        if len(all_dbboxes) == 0:
            return None, None, None

        # 拼接所有检测结果
        dbboxes = np.concatenate(all_dbboxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        ids = np.concatenate(all_ids, axis=0)

        # 转换为xywh格式用于NMS
        hw = (dbboxes[:,2:4] - dbboxes[:,0:2])
        xyhw = np.hstack([dbboxes[:,0:2], hw])

        # 分类别NMS
        boxes, classes, scores_out = [], [], []
        for i in range(bpu_instance.CLASSES_NUM):
            id_indices = ids == i
            if not np.any(id_indices):
                continue
                
            current_boxes = xyhw[id_indices,:]
            current_scores = scores[id_indices]
            
            if len(current_boxes) == 0:
                continue
                
            # 使用OpenCV的NMS
            indices = cv2.dnn.NMSBoxes(
                current_boxes.tolist(), 
                current_scores.tolist(), 
                bpu_instance.SCORE_THRESHOLD, 
                bpu_instance.NMS_THRESHOLD
            )
            
            if len(indices) == 0:
                continue
                
            # 处理indices的格式
            if isinstance(indices, np.ndarray):
                if indices.ndim == 2:
                    indices = indices.flatten()
            elif isinstance(indices, list):
                indices = np.array(indices).flatten()
            
            for indic in indices:
                x1, y1, x2, y2 = dbboxes[id_indices,:][indic]
                
                # 坐标转换回原图（letterbox逆变换）
                dw, dh = pad
                x1 = (x1 - dw) / scale
                y1 = (y1 - dh) / scale
                x2 = (x2 - dw) / scale
                y2 = (y2 - dh) / scale

                # 边界限制
                x1 = max(0, min(x1, img_w))
                x2 = max(0, min(x2, img_w))
                y1 = max(0, min(y1, img_h))
                y2 = max(0, min(y2, img_h))

                # 确保边界框有效
                if x2 > x1 and y2 > y1:
                    boxes.append([x1, y1, x2, y2])
                    classes.append(i)
                    scores_out.append(current_scores[indic])

        if len(boxes) == 0:
            return None, None, None
            
        return np.array(boxes), np.array(classes), np.array(scores_out)
    
    except Exception as e:
        logger.error(f"后处理出错: {e}")
        return None, None, None

def detect_bottle_async(rknn_lite, image, frame_id):
    """异步瓶子检测函数"""
    try:
        # 获取原始图像尺寸
        img_h, img_w = image.shape[:2]
        
        # 图像预处理
        img, scale, pad = letter_box(image, MODEL_SIZE)
        
        # 转换为NV12格式
        img_nv12 = bgr2nv12(img)
        
        # 执行BPU推理
        outputs = rknn_lite.inference([img_nv12])
        
        # 后处理
        boxes, classes, scores = bpu_post_process(outputs, rknn_lite, img_w, img_h, scale, pad)
        
        bottle_detections = []
        if boxes is not None:
            # 筛选出目标类别的检测结果
            target_classes = ['bottle', 'sports ball', 'apple', 'orange']
            for i, (box, score, cl) in enumerate(zip(boxes, scores, classes)):
                if CLASSES[cl] in target_classes:
                    x1, y1, x2, y2 = [float(_b) for _b in box]
                    
                    left = int(x1)
                    top = int(y1)
                    right = int(x2)
                    bottom = int(y2)
                    
                    # 确保边界框有效
                    if right > left and bottom > top:
                        # 计算中心点
                        center_x = (left + right) // 2
                        center_y = (top + bottom) // 2
                        
                        bottle_detections.append((left, top, right, bottom, score, center_x, center_y))
        
        return frame_id, image, bottle_detections
    
    except Exception as e:
        logger.error(f"检测过程出错: {e}")
        return frame_id, image, []

def draw_detections(image, detections):
    """在图像上绘制检测结果"""
    try:
        bottle_class_id = CLASSES.index('bottle')
        color = tuple(map(int, color_palette[bottle_class_id]))
        
        for detection in detections:
            left, top, right, bottom, score = detection[:5]
            
            # 绘制边界框
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
            
            # 添加标签
            label = f"orange: {score:.2f}"
            
            # 计算文本尺寸
            (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_x = int(left)
            label_y = int(top) - 10 if int(top) - 10 > label_height else int(top) + 10
            
            # 绘制文本背景
            cv2.rectangle(image, (label_x, label_y - label_height - baseline), 
                         (label_x + label_width, label_y + baseline), color, cv2.FILLED)
            
            # 绘制文本
            cv2.putText(image, label, (label_x, label_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            
            # 在中心绘制圆点
            if len(detection) >= 7:
                cx, cy = int(detection[5]), int(detection[6])
                cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)
    except Exception as e:
        logger.error(f"绘制检测结果出错: {e}")

def test_video_async(model_path, video_path, thread_num=3):
    """使用异步处理测试视频 - 优化版本"""
    logger.info(f"🚀 开始异步检测（优化版）")
    logger.info(f"模型: {model_path}")
    logger.info(f"视频源: {video_path}")
    logger.info(f"线程数: {thread_num}")
    
    # 创建线程池执行器
    try:
        pool = BottleRKNNPoolExecutor(
            model_path=model_path,
            detector_func=detect_bottle_async,
            thread_num=thread_num,
            queue_size=thread_num * 2  # 适当增加队列大小
        )
        logger.info("✓ BPU线程池创建成功")
    except Exception as e:
        logger.error(f"❌ 创建线程池失败: {e}")
        return
    
    # 打开视频源
    try:
        if video_path in ["0", "1", "2"]:
            cap = cv2.VideoCapture(int(video_path))
            logger.info(f"✓ 使用摄像头 {video_path}")
        else:
            cap = cv2.VideoCapture(video_path)
            logger.info(f"✓ 使用视频文件: {video_path}")
        
        if not cap.isOpened():
            logger.error("❌ 无法打开视频源")
            pool.release()
            return
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲延迟
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"✓ 视频参数: {actual_width}x{actual_height}@{actual_fps}fps")
        
    except Exception as e:
        logger.error(f"❌ 打开视频源失败: {e}")
        pool.release()
        return
    
    # 初始化性能监控
    fps_counter = FPSCounter(window_size=30)  # 30帧滑动平均
    frame_buffer = FrameBuffer(max_size=thread_num * 2)
    
    # 性能统计
    total_frames = 0
    start_time = time.time()
    stats_interval = 30  # 30帧输出一次统计
    
    # 创建显示窗口
    window_name = "RDK Orange Detection (Optimized)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    logger.info("🎯 开始优化异步处理，按 'q' 或 'ESC' 键退出")
    
    # 预填充管道（减少初始化延迟）
    logger.info("正在预填充处理管道...")
    for i in range(min(thread_num, 3)):  # 只预填充少量帧
        ret, frame = cap.read()
        if ret:
            frame_id = pool.put(frame)
            frame_buffer.put(frame_id, frame.copy())
    
    try:
        while cap.isOpened():
            # 读取新帧
            ret, frame = cap.read()
            if not ret:
                logger.warning("视频读取结束或失败")
                break
            
            # 提交新帧到处理队列
            if not pool.is_full():
                frame_id = pool.put(frame)
                frame_buffer.put(frame_id, frame.copy())
            
            # 获取处理完成的结果
            result_frame_id, result, success = pool.get(timeout=0.01)  # 减少等待时间
            
            display_frame = None
            detections = []
            
            if success and result is not None:
                _, _, detections = result
                # 获取对应的原始帧
                display_frame = frame_buffer.get(result_frame_id)
                if display_frame is None:
                    # 如果没有对应帧，使用最新帧
                    _, display_frame = frame_buffer.get_next_available()
            
            # 如果没有获取到结果，使用缓冲区中的下一个可用帧
            if display_frame is None:
                _, display_frame = frame_buffer.get_next_available()
            
            # 如果还是没有帧，使用当前读取的帧
            if display_frame is None:
                display_frame = frame.copy()
            
            # 绘制检测结果
            if len(detections) > 0:
                draw_detections(display_frame, detections)
            
            # 更新FPS计算
            fps_counter.update()
            total_frames += 1
            
            # 显示性能信息
            current_fps = fps_counter.get_fps()
            
            # 绘制平滑的性能信息
            info_text = f"FPS: {current_fps:.1f} | Threads: {thread_num} | Buffer: {frame_buffer.size()}"
            cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示检测信息
            detection_text = f"Orange: {len(detections)}"
            cv2.putText(display_frame, detection_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 显示队列状态
            queue_text = f"Queue: {pool.get_queue_size()}/{thread_num*2}"
            cv2.putText(display_frame, queue_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 显示总帧数
            frame_text = f"Total: {total_frames}"
            cv2.putText(display_frame, frame_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # 显示结果
            cv2.imshow(window_name, display_frame)
            
            # 定期输出统计信息
            if total_frames % stats_interval == 0:
                elapsed = time.time() - start_time
                avg_fps = total_frames / elapsed
                logger.info(f"📊 统计: {total_frames} 帧, 平均FPS: {avg_fps:.2f}, 当前FPS: {current_fps:.2f}")
            
            # 检查退出键
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), 27]:  # q或ESC
                logger.info("用户请求退出")
                break
            elif key == ord('s'):  # s保存截图
                screenshot_name = f"screenshot_{int(time.time())}.jpg"
                cv2.imwrite(screenshot_name, display_frame)
                logger.info(f"✓ 截图已保存: {screenshot_name}")
            elif key == ord('r'):  # r重置FPS计数器
                fps_counter.reset()
                logger.info("FPS计数器已重置")
    
    except KeyboardInterrupt:
        logger.info("检测被用户中断")
    except Exception as e:
        logger.error(f"检测过程出错: {e}")
        import traceback
        logger.error(traceback.format_exc())
    
    finally:
        # 计算最终统计
        total_time = time.time() - start_time
        final_avg_fps = total_frames / total_time if total_time > 0 else 0
        
        logger.info("=" * 70)
        logger.info("🏁 检测完成统计信息:")
        logger.info(f"总帧数: {total_frames}")
        logger.info(f"总时间: {total_time:.2f} 秒")
        logger.info(f"平均FPS: {final_avg_fps:.2f}")
        logger.info(f"最终FPS: {fps_counter.get_fps():.2f}")
        logger.info("=" * 70)
        
        # 释放资源
        logger.info("正在释放资源...")
        cap.release()
        cv2.destroyAllWindows()
        pool.release()
        logger.info("✓ 所有资源已释放")

def compare_performance(model_path, video_path):
    """性能对比测试"""
    thread_configs = [1, 2, 3, 4]
    results = {}
    
    logger.info("🔄 开始性能对比测试...")
    logger.info("=" * 70)
    
    for thread_num in thread_configs:
        logger.info(f"\n📊 测试配置: {thread_num} 线程")
        logger.info("-" * 50)
        
        # 运行30秒测试
        test_video_async_limited(model_path, video_path, thread_num, test_duration=30)
        time.sleep(3)  # 等待资源完全释放
    
    logger.info("\n🏆 性能对比测试完成！")

def test_video_async_limited(model_path, video_path, thread_num=3, test_duration=30):
    """限时版本的异步测试，用于性能对比"""
    logger.info(f"开始 {test_duration} 秒限时测试 ({thread_num} 线程)...")
    
    try:
        pool = BottleRKNNPoolExecutor(
            model_path=model_path,
            detector_func=detect_bottle_async,
            thread_num=thread_num
        )
    except Exception as e:
        logger.error(f"创建线程池失败: {e}")
        return
    
    if video_path in ["0", "1", "2"]:
        cap = cv2.VideoCapture(int(video_path))
    else:
        cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        logger.error("无法打开视频源")
        pool.release()
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    fps_counter = FPSCounter()
    frame_buffer = FrameBuffer()
    
    frames = 0
    start_time = time.time()
    
    try:
        while time.time() - start_time < test_duration:
            ret, frame = cap.read()
            if not ret:
                break
            
            if not pool.is_full():
                frame_id = pool.put(frame)
                frame_buffer.put(frame_id, frame.copy())
            
            result_frame_id, result, success = pool.get(timeout=0.01)
            
            if success and result is not None:
                frames += 1
                fps_counter.update()
                frame_buffer.get(result_frame_id)
    
    except Exception as e:
        logger.error(f"限时测试出错: {e}")
    
    finally:
        actual_duration = time.time() - start_time
        avg_fps = frames / actual_duration if actual_duration > 0 else 0
        smooth_fps = fps_counter.get_fps()
        
        logger.info(f"✓ 测试结果: {frames} 帧 / {actual_duration:.2f} 秒")
        logger.info(f"  平均FPS: {avg_fps:.2f}")
        logger.info(f"  平滑FPS: {smooth_fps:.2f}")
        
        cap.release()
        pool.release()

def main():
    """主函数 - 命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RDK X5异步瓶子检测（优化版）')
    parser.add_argument('--model', '-m', type=str, 
                       default='/home/sunrise/project/my_main/models/yolo11n_detect_bayese_640x640_nv12_modified.bin',
                       help='BPU模型路径')
    parser.add_argument('--source', '-s', type=str, default='0',
                       help='视频源 (0/1/2为摄像头，或视频文件路径)')
    parser.add_argument('--threads', '-t', type=int, default=3,
                       help='线程数量 (1-6)')
    parser.add_argument('--compare', '-c', action='store_true',
                       help='性能对比模式')
    parser.add_argument('--debug', '-d', action='store_true',
                       help='启用调试输出')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    if args.threads < 1 or args.threads > 6:
        logger.error("线程数必须在1-6之间")
        return
    
    if not os.path.exists(args.model):
        logger.error(f"模型文件不存在: {args.model}")
        return
    
    logger.info("🚀 RDK X5异步瓶子检测系统（优化版）")
    logger.info(f"模型: {args.model}")
    logger.info(f"视频源: {args.source}")
    logger.info(f"线程数: {args.threads}")
    
    try:
        if args.compare:
            compare_performance(args.model, args.source)
        else:
            test_video_async(args.model, args.source, args.threads)
    except KeyboardInterrupt:
        logger.info("程序被用户中断")
    except Exception as e:
        logger.error(f"程序运行出错: {e}")

if __name__ == '__main__':
    # 检查依赖
    try:
        import cv2
        import numpy as np
        from scipy.special import softmax
        logger.info("✓ 所有依赖库导入成功")
    except ImportError as e:
        logger.error(f"❌ 缺少依赖库: {e}")
        sys.exit(1)
    
    # 运行主程序
    if len(sys.argv) > 1:
        main()
    else:
        MODEL_PATH = "/home/sunrise/project/my_main/models/yolo11n_detect_bayese_640x640_nv12_modified.bin"
        logger.info("使用默认参数运行...")
        test_video_async(MODEL_PATH, "0", 6)