#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测模块
处理瓶子检测相关功能，使用RKNN模型进行目标检测
"""

import cv2
import numpy as np
import logging
import time
import sys
import os
from rknnlite.api import RKNNLite

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("bottle_detector")

# YOLO检测参数
OBJ_THRESH = 0.2  # 目标置信度阈值
NMS_THRESH = 0.5  # 非极大值抑制阈值

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

class BottleDetector:
    def __init__(self, model_path, model_size=(640, 640)):
        """
        初始化瓶子检测器
        
        参数:
        model_path -- RKNN模型路径
        model_size -- 模型输入尺寸 (宽度, 高度)
        """
        self.model_path = model_path
        self.model_size = model_size
        self.rknn = None
        
    def load_model(self):
        """加载RKNN模型"""
        try:
            self.rknn = RKNNLite()
            # 加载RKNN模型文件
            if self.rknn.load_rknn(self.model_path) != 0:
                logger.error('加载RKNN模型失败')
                return False
                
            # 初始化运行时环境，目标设备为RK3588
            if self.rknn.init_runtime() != 0:
                logger.error('初始化运行时环境失败!')
                return False
                
            logger.info(f"成功加载模型: {self.model_path}")
            return True
        except Exception as e:
            logger.error(f"加载模型出错: {e}")
            return False
    
    def release_model(self):
        """释放RKNN模型资源"""
        if self.rknn:
            self.rknn.release()
            logger.info("模型资源已释放")
    
    def detect(self, image):
        """
        检测图像中的瓶子
        
        参数:
        image -- 输入图像 (BGR格式)
        
        返回:
        瓶子检测结果列表: [(left, top, right, bottom, score, center_x, center_y), ...]
        """
        if self.rknn is None:
            logger.error("模型未加载")
            return []
            
        # 图像预处理：调整大小并保持宽高比
        img = self._letter_box(image, self.model_size)
        input_tensor = np.expand_dims(img, axis=0)
        
        # 执行推理
        outputs = self.rknn.inference([input_tensor])
        
        # 后处理：解析检测结果
        boxes, classes, scores = self._post_process(outputs)
        
        bottle_detections = []
        if boxes is not None:
            # 获取原始图像尺寸
            img_h, img_w = image.shape[:2]
            # 计算缩放因子
            x_factor = img_w / self.model_size[0]
            y_factor = img_h / self.model_size[1]
            
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
        
        return bottle_detections
    
    def draw_detection(self, image, detection, distance=None):
        """
        在图像上绘制瓶子检测结果
        
        参数:
        image -- 要绘制的图像
        detection -- 检测结果 (left, top, right, bottom, score, cx, cy)
        distance -- 距离信息，如果有的话
        """
        left, top, right, bottom, score = detection[:5]
        bottle_class_id = CLASSES.index('bottle')
        color = color_palette[bottle_class_id]
        
        # 绘制边界框
        cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
        
        # 添加标签和距离信息
        if distance is not None:
            label = f"bottle: {score:.2f}, distance: {distance:.2f}m"
        else:
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
    
    # YOLO检测相关辅助函数
    def _sigmoid(self, x):
        """Sigmoid激活函数"""
        return 1 / (1 + np.exp(-x))
    
    def _letter_box(self, im, new_shape, pad_color=(255, 255, 255), info_need=False):
        """
        调整图像大小并保持宽高比，使用padding填充
        
        参数:
        im -- 输入图像
        new_shape -- 目标尺寸
        pad_color -- 填充颜色
        info_need -- 是否返回额外信息
        """
        shape = im.shape[:2]  # 当前形状 [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
            
        # 计算缩放比例
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r
        
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
        
        if info_need is True:
            return im, ratio, (dw, dh)
        else:
            return im
    
    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
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
    
    def _nms_boxes(self, boxes, scores):
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
    
    def _softmax(self, x, axis=None):
        """Softmax函数"""
        x = x - x.max(axis=axis, keepdims=True)
        y = np.exp(x)
        return y / y.sum(axis=axis, keepdims=True)
    
    def _dfl(self, position):
        """Distribution Focal Loss (DFL) 用于边界框回归"""
        n, c, h, w = position.shape
        p_num = 4
        mc = c // p_num
        y = position.reshape(n, p_num, mc, h, w)
        y = self._softmax(y, 2)
        acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(2)
        return y
    
    def _box_process(self, position):
        """处理边界框预测"""
        grid_h, grid_w = position.shape[2:4]
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)
        stride = np.array([self.model_size[1] // grid_h, self.model_size[0] // grid_w]).reshape(1, 2, 1, 1)
        
        position = self._dfl(position)
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
        
        return xyxy
    
    def _post_process(self, input_data):
        """YOLO输出后处理"""
        boxes, scores, classes_conf = [], [], []
        defualt_branch = 3  # YOLOv8有3个检测头
        pair_per_branch = len(input_data) // defualt_branch
        
        # 处理每个检测头的输出
        for i in range(defualt_branch):
            boxes.append(self._box_process(input_data[pair_per_branch * i]))
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
        boxes, classes, scores = self._filter_boxes(boxes, scores, classes_conf)
        
        # 对每个类别执行NMS
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]
            keep = self._nms_boxes(b, s)
            
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


# 测试函数
def test_image(detector, image_path):
    """测试单张图片"""
    logger.info(f"测试图片: {image_path}")
    
    # 读取图片
    image = cv2.imread(image_path)
    if image is None:
        logger.error(f"无法读取图片: {image_path}")
        return
    
    # 执行检测
    start_time = time.time()
    detections = detector.detect(image)
    end_time = time.time()
    
    # 计算处理时间
    process_time = (end_time - start_time) * 1000  # 转换为毫秒
    fps = 1000 / process_time
    
    # 绘制检测结果
    for detection in detections:
        detector.draw_detection(image, detection)
    
    # 在图像上显示FPS和检测数量
    info_text = f"FPS: {fps:.1f} | 检测到 {len(detections)} 个瓶子 | 处理时间: {process_time:.1f}ms"
    cv2.putText(image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 显示结果
    cv2.imshow("瓶子检测结果", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 保存结果
    output_path = "detection_result.jpg"
    cv2.imwrite(output_path, image)
    logger.info(f"检测结果已保存到: {output_path}")


def test_video(detector, video_path):
    """测试视频或摄像头"""
    if video_path == "1":
        # 使用摄像头
        cap = cv2.VideoCapture(1,cv2.CAP_V4L2)
        logger.info("使用摄像头进行测试")
    else:
        # 使用视频文件
        cap = cv2.VideoCapture(video_path)
        logger.info(f"测试视频: {video_path}")
    
    if not cap.isOpened():
        logger.error("无法打开视频源")
        return
    
    # 获取视频信息
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # 创建视频写入器（可选）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output_detection.mp4', fourcc, fps, (width, height))
    
    # FPS计算相关变量
    fps_list = []
    frame_count = 0
    
    logger.info("按 'q' 键退出")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 执行检测
        start_time = time.time()
        detections = detector.detect(frame)
        end_time = time.time()
        
        # 计算FPS
        process_time = (end_time - start_time) * 1000  # 毫秒
        current_fps = 1000 / process_time
        fps_list.append(current_fps)
        
        # 保持最近30帧的FPS用于计算平均值
        if len(fps_list) > 30:
            fps_list.pop(0)
        avg_fps = sum(fps_list) / len(fps_list)
        
        # 绘制检测结果
        for detection in detections:
            detector.draw_detection(frame, detection)
        
        # 显示信息
        frame_count += 1
        info_text = f"FPS: {current_fps:.1f}"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示处理时间
        time_text = f"time: {process_time:.1f}ms"
        cv2.putText(frame, time_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 显示结果
        cv2.imshow("bottle", frame)
        
        # 写入输出视频
        out.write(frame)
        
        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    logger.info(f"处理完成，共处理 {frame_count} 帧")
    logger.info(f"平均FPS: {avg_fps:.1f}")


if __name__ == '__main__':
    """主函数 - 测试瓶子检测模块"""
    
    # 默认模型路径（需要根据实际情况修改）
    MODEL_PATH = "/home/monster/download/AgriSage3/src/bottle_detection_ros2/data/yolo11n.rknn"  # 请替换为实际的RKNN模型路径
    
    
    
    # 创建检测器实例
    detector = BottleDetector(MODEL_PATH)
    
    # 加载模型
    if not detector.load_model():
        logger.error("模型加载失败")
        sys.exit(1)
    
    test_video(detector, "1")

    # 释放模型资源
    detector.release_model()
    logger.info("程序结束")