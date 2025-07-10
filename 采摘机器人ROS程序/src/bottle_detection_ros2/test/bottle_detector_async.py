#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
异步瓶子检测主程序
使用多线程异步处理提高检测帧率
"""

import cv2
import numpy as np
import logging
import time
import sys
import os
from bottle_rknn_pool import BottleRKNNPoolExecutor

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("bottle_detector_async")

# YOLO检测参数
OBJ_THRESH = 0.2  # 目标置信度阈值
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

# YOLO后处理辅助函数
def sigmoid(x):
    """Sigmoid激活函数"""
    return 1 / (1 + np.exp(-x))

def letter_box(im, new_shape=(640, 640), pad_color=(255, 255, 255)):
    """
    调整图像大小并保持宽高比，使用padding填充
    """
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
    
    return im

def filter_boxes(boxes, box_confidences, box_class_probs):
    """根据置信度阈值筛选检测框"""
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape
    
    # 获取每个框的最大类别概率和对应类别
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    
    # 筛选出置信度大于阈值的框
    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]
    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    
    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """非极大值抑制，去除重叠的检测框"""
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
    
    areas = w * h
    order = scores.argsort()[::-1]  # 按得分降序排序
    
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        
        # 计算交集
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
        
        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1
        
        # 计算IoU
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        
        # 保留IoU小于阈值的框
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
        
    keep = np.array(keep)
    return keep

def softmax(x, axis=None):
    """Softmax函数"""
    x = x - x.max(axis=axis, keepdims=True)
    y = np.exp(x)
    return y / y.sum(axis=axis, keepdims=True)

def dfl(position):
    """Distribution Focal Loss (DFL) 用于边界框回归"""
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)
    y = softmax(y, 2)
    acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y

def box_process(position):
    """处理边界框预测"""
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([MODEL_SIZE[1] // grid_h, MODEL_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)
    
    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
    
    return xyxy

def post_process(input_data):
    """YOLO输出后处理"""
    boxes, scores, classes_conf = [], [], []
    defualt_branch = 3  # YOLOv8有3个检测头
    pair_per_branch = len(input_data) // defualt_branch
    
    # 处理每个检测头的输出
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))
    
    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)
    
    # 展平所有检测结果
    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]
    
    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)
    
    # 筛选框
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)
    
    # 对每个类别执行NMS
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)
        
        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])
    
    if not nclasses and not nscores:
        return None, None, None
    
    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    
    return boxes, classes, scores

def detect_bottle_async(rknn_lite, image, frame_id):
    """
    异步瓶子检测函数
    
    参数:
    rknn_lite -- RKNN实例
    image -- 输入图像 (BGR格式)
    frame_id -- 帧ID
    
    返回:
    (frame_id, image, detections) -- 帧ID、原图像、检测结果
    """
    try:
        # 图像预处理
        img = letter_box(image, MODEL_SIZE)
        input_tensor = np.expand_dims(img, axis=0)
        
        # 执行推理
        outputs = rknn_lite.inference([input_tensor])
        
        # 后处理
        boxes, classes, scores = post_process(outputs)
        
        bottle_detections = []
        if boxes is not None:
            # 获取原始图像尺寸
            img_h, img_w = image.shape[:2]
            # 计算缩放因子
            x_factor = img_w / MODEL_SIZE[0]
            y_factor = img_h / MODEL_SIZE[1]
            
            # 筛选出瓶子类别的检测结果
            for box, score, cl in zip(boxes, scores, classes):
                if CLASSES[cl] == 'bottle':  # 只保留瓶子类别
                    x1, y1, x2, y2 = [int(_b) for _b in box]
                    
                    # 将坐标映射回原始图像尺寸
                    left = int(x1 * x_factor)
                    top = int(y1 * y_factor)
                    right = int(x2 * x_factor)
                    bottom = int(y2 * y_factor)
                    
                    # 计算瓶子中心点
                    center_x = (left + right) // 2
                    center_y = (top + bottom) // 2
                    
                    bottle_detections.append((left, top, right, bottom, score, center_x, center_y))
        
        return frame_id, image, bottle_detections
    
    except Exception as e:
        logger.error(f"检测过程出错: {e}")
        return frame_id, image, []

def draw_detections(image, detections):
    """在图像上绘制检测结果"""
    bottle_class_id = CLASSES.index('bottle')
    color = color_palette[bottle_class_id]
    
    for detection in detections:
        left, top, right, bottom, score = detection[:5]
        
        # 绘制边界框
        cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
        
        # 添加标签
        label = f"bottle: {score:.2f}"
        
        # 计算文本尺寸
        (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_x = left
        label_y = top - 10 if top - 10 > label_height else top + 10
        
        # 绘制文本背景
        cv2.rectangle(image, (int(label_x), int(label_y - label_height)), 
                     (int(label_x + label_width), int(label_y + label_height)), color, cv2.FILLED)
        # 绘制文本
        cv2.putText(image, label, (int(label_x), int(label_y)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

def test_video_async(model_path, video_path, thread_num=3):
    """
    使用异步处理测试视频
    
    参数:
    model_path -- RKNN模型路径
    video_path -- 视频路径或摄像头ID
    thread_num -- 线程数量
    """
    # 创建线程池执行器
    try:
        pool = BottleRKNNPoolExecutor(
            model_path=model_path,
            detector_func=detect_bottle_async,
            thread_num=thread_num
        )
    except Exception as e:
        logger.error(f"创建线程池失败: {e}")
        return
    
    # 打开视频源
    if video_path == "1":
        cap = cv2.VideoCapture(21, cv2.CAP_V4L2)
        logger.info("使用摄像头进行测试")
    else:
        cap = cv2.VideoCapture(video_path)
        logger.info(f"测试视频: {video_path}")
    
    if not cap.isOpened():
        logger.error("无法打开视频源")
        pool.release()
        return
    
    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # 预填充管道
    logger.info("预填充处理管道...")
    for i in range(thread_num + 1):
        ret, frame = cap.read()
        if not ret:
            logger.error("视频读取失败")
            cap.release()
            pool.release()
            return
        pool.put(frame)
    
    # FPS计算相关变量
    frames = 0
    loop_time = time.time()
    init_time = time.time()
    fps_list = []
    frame_dict = {}  # 存储帧ID对应的帧
    last_display_time = time.time()
    
    logger.info("开始异步处理，按 'q' 键退出")
    
    while cap.isOpened():
        # 读取新帧
        ret, frame = cap.read()
        if not ret:
            break
        
        # 提交新帧到处理队列（如果队列未满）
        if not pool.is_full():
            frame_id = pool.put(frame)
            frame_dict[frame_id] = frame.copy()
            logger.debug(f"提交帧 {frame_id} 到处理队列")
        
        # 尝试获取处理结果
        result_frame_id, result, success = pool.get(timeout=0.1)
        
        if success and result is not None:
            _, original_frame, detections = result
            logger.debug(f"获取到帧 {result_frame_id} 的处理结果")
            
            # 使用保存的原始帧
            if result_frame_id in frame_dict:
                display_frame = frame_dict[result_frame_id]
                del frame_dict[result_frame_id]  # 删除已使用的帧
            else:
                display_frame = original_frame
            
            # 绘制检测结果
            draw_detections(display_frame, detections)
            
            # 计算FPS
            frames += 1
            current_time = time.time()
            
            if frames % 30 == 0:
                avg_fps = 30 / (current_time - loop_time)
                fps_list.append(avg_fps)
                logger.info(f"30帧平均帧率: {avg_fps:.2f} FPS, 队列大小: {pool.get_queue_size()}")
                loop_time = current_time
            
            # 显示信息
            if current_time - last_display_time > 0:
                instant_fps = 1 / (current_time - last_display_time)
            else:
                instant_fps = 0
            last_display_time = current_time
            
            info_text = f"FPS: {instant_fps:.1f} | Threads: {thread_num} | Queue: {pool.get_queue_size()}"
            cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示检测数量
            detection_text = f"Bottles: {len(detections)}"
            cv2.putText(display_frame, detection_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 显示结果
            cv2.imshow("Bottle Detection (Async)", display_frame)
        else:
            # 如果没有获取到结果，显示最新的原始帧
            if len(frame_dict) > 0:
                latest_frame_id = max(frame_dict.keys())
                display_frame = frame_dict[latest_frame_id].copy()
                cv2.putText(display_frame, "Processing...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Bottle Detection (Async)", display_frame)
        
        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # 清理过期的帧
        if len(frame_dict) > thread_num * 3:
            oldest_id = min(frame_dict.keys())
            del frame_dict[oldest_id]
    
    # 计算总体统计
    total_time = time.time() - init_time
    total_avg_fps = frames / total_time
    
    logger.info("=" * 50)
    logger.info(f"处理完成！")
    logger.info(f"总帧数: {frames}")
    logger.info(f"总时间: {total_time:.2f} 秒")
    logger.info(f"总平均帧率: {total_avg_fps:.2f} FPS")
    if fps_list:
        logger.info(f"30帧平均帧率: {np.mean(fps_list):.2f} FPS")
    logger.info("=" * 50)
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    pool.release()

def compare_performance(model_path, video_path):
    """
    比较不同线程数的性能
    
    参数:
    model_path -- RKNN模型路径
    video_path -- 视频路径
    """
    thread_configs = [1, 2, 3, 4, 5, 6]
    
    logger.info("开始性能对比测试...")
    logger.info("=" * 60)
    
    for thread_num in thread_configs:
        logger.info(f"\n测试 {thread_num} 线程配置...")
        test_video_async(model_path, video_path, thread_num)
        time.sleep(2)  # 等待资源完全释放
    
    logger.info("\n性能对比测试完成！")

if __name__ == '__main__':
    """主函数 - 测试异步瓶子检测"""
    
    # 默认模型路径（需要根据实际情况修改）
    MODEL_PATH = "/home/elf/Desktop/robot_ROS2/src/bottle_detection_ros2/data/yolo11n.rknn"
    
    # 运行模式选择
    if len(sys.argv) > 1:
        if sys.argv[1] == "compare":
            # 性能对比模式
            video_source = sys.argv[2] if len(sys.argv) > 2 else "1"
            compare_performance(MODEL_PATH, video_source)
        else:
            # 普通测试模式
            video_source = sys.argv[1]
            thread_num = int(sys.argv[2]) if len(sys.argv) > 2 else 3
            test_video_async(MODEL_PATH, video_source, thread_num)
    else:
        # 默认使用摄像头，3个线程
        test_video_async(MODEL_PATH, "1", 3)