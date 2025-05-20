#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测模块
处理瓶子检测相关功能，使用RKNN模型进行目标检测
"""

import cv2
import numpy as np
import logging
from rknn.api import RKNN

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("bottle_detector")

# YOLO检测参数
OBJ_THRESH = 0.2
NMS_THRESH = 0.5

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
        model_size -- 模型输入尺寸
        """
        self.model_path = model_path
        self.model_size = model_size
        self.rknn = None
        
    def load_model(self):
        """加载RKNN模型"""
        try:
            self.rknn = RKNN()
            if self.rknn.load_rknn(self.model_path) != 0:
                logger.error('加载RKNN模型失败')
                return False
                
            # 初始化运行时环境
            if self.rknn.init_runtime(target='rk3588', device_id=0) != 0:
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
        image -- 输入图像
        
        返回:
        瓶子检测结果列表: [(left, top, right, bottom, score, center_x, center_y), ...]
        """
        if self.rknn is None:
            logger.error("模型未加载")
            return []
            
        img = self._letter_box(image, self.model_size)
        input_tensor = np.expand_dims(img, axis=0)
        outputs = self.rknn.inference([input_tensor])
        boxes, classes, scores = self._post_process(outputs)
        
        bottle_detections = []
        if boxes is not None:
            img_h, img_w = image.shape[:2]
            x_factor = img_w / self.model_size[0]
            y_factor = img_h / self.model_size[1]
            
            for box, score, cl in zip(boxes, scores, classes):
                if CLASSES[cl] == 'bottle':  # 只保留瓶子类别
                    x1, y1, x2, y2 = [int(_b) for _b in box]
                    
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
            
        (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_x = left
        label_y = top - 10 if top - 10 > label_height else top + 10
        cv2.rectangle(image, (int(label_x), int(label_y - label_height)), 
                     (int(label_x + label_width), int(label_y + label_height)), color, cv2.FILLED)
        cv2.putText(image, label, (int(label_x), int(label_y)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    
    # YOLO检测相关辅助函数
    def _sigmoid(self, x):
        return 1 / (1 + np.exp(-x))
    
    def _letter_box(self, im, new_shape, pad_color=(255, 255, 255), info_need=False):
        shape = im.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)
        if info_need is True:
            return im, ratio, (dw, dh)
        else:
            return im
    
    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape
        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
        scores = (class_max_score * box_confidences)[_class_pos]
        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
        return boxes, classes, scores
    
    def _nms_boxes(self, boxes, scores):
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]
        areas = w * h
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= NMS_THRESH)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep
    
    def _softmax(self, x, axis=None):
        x = x - x.max(axis=axis, keepdims=True)
        y = np.exp(x)
        return y / y.sum(axis=axis, keepdims=True)
    
    def _dfl(self, position):
        n, c, h, w = position.shape
        p_num = 4
        mc = c // p_num
        y = position.reshape(n, p_num, mc, h, w)
        y = self._softmax(y, 2)
        acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(2)
        return y
    
    def _box_process(self, position):
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
        boxes, scores, classes_conf = [], [], []
        defualt_branch = 3
        pair_per_branch = len(input_data) // defualt_branch
        for i in range(defualt_branch):
            boxes.append(self._box_process(input_data[pair_per_branch * i]))
            classes_conf.append(input_data[pair_per_branch * i + 1])
            scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))
        
        def sp_flatten(_in):
            ch = _in.shape[1]
            _in = _in.transpose(0, 2, 3, 1)
            return _in.reshape(-1, ch)
        
        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]
        
        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)
        
        boxes, classes, scores = self._filter_boxes(boxes, scores, classes_conf)
        
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