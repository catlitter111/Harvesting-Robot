#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测器类
使用RKNN模型进行YOLO目标检测，专门检测瓶子
"""

import cv2
import numpy as np
from rknnlite.api import RKNNLite

# YOLO检测参数
OBJ_THRESH = 0.25  # 目标检测阈值
NMS_THRESH = 0.45  # 非极大值抑制阈值

# COCO数据集类别名称
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
           'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
           'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
           'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
           'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
           'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
           'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
           'hair drier', 'toothbrush']

class BottleDetector:
    """瓶子检测器类"""
    
    def __init__(self, model_path, model_size=(640, 640)):
        """
        初始化瓶子检测器
        
        参数:
        model_path: RKNN模型文件路径
        model_size: 模型输入尺寸 (width, height)
        """
        self.model_path = model_path
        self.model_size = model_size
        self.rknn = None
        
        # 颜色调色板，用于绘制检测框
        self.color_palette = np.random.uniform(0, 255, size=(len(CLASSES), 3))
        
        # 瓶子类别索引
        self.bottle_class_id = CLASSES.index('bottle')
    
    def load_model(self):
        """
        加载RKNN模型
        
        返回:
        成功返回True，失败返回False
        """
        try:
            # 创建RKNN对象
            self.rknn = RKNNLite()
            
            # 加载RKNN模型
            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                print(f'加载RKNN模型失败: {self.model_path}')
                return False
            
            # 初始化运行时环境
            ret = self.rknn.init_runtime()
            if ret != 0:
                print('初始化RKNN运行时环境失败')
                return False
            
            print(f'成功加载RKNN模型: {self.model_path}')
            return True
            
        except Exception as e:
            print(f'加载模型出错: {e}')
            return False
    
    def release_model(self):
        """释放RKNN模型资源"""
        if self.rknn:
            self.rknn.release()
            print('RKNN模型资源已释放')
    
    def detect(self, image):
        """
        检测图像中的瓶子
        
        参数:
        image: 输入图像 (BGR格式)
        
        返回:
        检测结果列表: [(left, top, right, bottom, score, center_x, center_y), ...]
        """
        if self.rknn is None:
            print('错误：模型未加载')
            return []
        
        try:
            # 图像预处理
            img = self._preprocess_image(image)
            
            # 添加批次维度
            input_tensor = np.expand_dims(img, axis=0)
            
            # 执行推理
            outputs = self.rknn.inference([input_tensor])
            
            # 后处理
            boxes, classes, scores = self._postprocess(outputs)
            
            # 提取瓶子检测结果
            bottle_detections = []
            if boxes is not None:
                # 计算缩放因子
                img_h, img_w = image.shape[:2]
                x_factor = img_w / self.model_size[0]
                y_factor = img_h / self.model_size[1]
                
                # 遍历所有检测结果
                for box, score, cl in zip(boxes, scores, classes):
                    # 只保留瓶子类别
                    if CLASSES[cl] == 'bottle':
                        # 将坐标转换回原始图像尺寸
                        x1, y1, x2, y2 = box
                        left = int(x1 * x_factor)
                        top = int(y1 * y_factor)
                        right = int(x2 * x_factor)
                        bottom = int(y2 * y_factor)
                        
                        # 计算中心点
                        center_x = (left + right) // 2
                        center_y = (top + bottom) // 2
                        
                        # 添加到检测结果
                        bottle_detections.append(
                            (left, top, right, bottom, score, center_x, center_y)
                        )
            
            return bottle_detections
            
        except Exception as e:
            print(f'检测过程出错: {e}')
            return []
    
    def draw_detection(self, image, detection, distance=None):
        """
        在图像上绘制检测结果
        
        参数:
        image: 要绘制的图像
        detection: 检测结果 (left, top, right, bottom, score, cx, cy)
        distance: 瓶子距离（可选）
        """
        left, top, right, bottom, score = detection[:5]
        
        # 使用瓶子类别的颜色
        color = self.color_palette[self.bottle_class_id]
        
        # 绘制边界框
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        
        # 准备标签文本
        if distance is not None and distance > 0:
            label = f'bottle: {score:.2f}, {distance:.2f}m'
        else:
            label = f'bottle: {score:.2f}'
        
        # 计算标签背景尺寸
        (label_width, label_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        
        # 确定标签位置
        label_x = left
        label_y = top - 10 if top - 10 > label_height else top + 20
        
        # 绘制标签背景
        cv2.rectangle(
            image,
            (label_x, label_y - label_height - baseline),
            (label_x + label_width, label_y),
            color, 
            cv2.FILLED
        )
        
        # 绘制标签文本
        cv2.putText(
            image, 
            label, 
            (label_x, label_y - baseline),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (0, 0, 0),  # 黑色文字
            1, 
            cv2.LINE_AA
        )
        
        # 在瓶子中心绘制一个小圆点
        if len(detection) >= 7:
            cx, cy = int(detection[5]), int(detection[6])
            cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)
    
    def _preprocess_image(self, image):
        """
        图像预处理，保持宽高比的letterbox变换
        
        参数:
        image: 输入图像
        
        返回:
        预处理后的图像
        """
        # 获取图像尺寸
        shape = image.shape[:2]  # [height, width]
        new_shape = self.model_size
        
        # 计算缩放比例
        r = min(new_shape[0] / shape[1], new_shape[1] / shape[0])
        
        # 计算新的图像尺寸
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[0] - new_unpad[0], new_shape[1] - new_unpad[1]
        
        # 计算padding
        dw /= 2
        dh /= 2
        
        # 调整图像大小
        if shape[::-1] != new_unpad:
            image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        # 添加边框
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        image = cv2.copyMakeBorder(
            image, top, bottom, left, right, 
            cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )
        
        return image
    
    def _postprocess(self, outputs):
        """
        YOLO后处理
        
        参数:
        outputs: 模型输出
        
        返回:
        (boxes, classes, scores) 检测框、类别和置信度
        """
        boxes, scores, classes_conf = [], [], []
        defualt_branch = 3
        pair_per_branch = len(outputs) // defualt_branch
        
        # 处理三个检测头的输出
        for i in range(defualt_branch):
            boxes.append(self._process_boxes(outputs[pair_per_branch * i]))
            classes_conf.append(outputs[pair_per_branch * i + 1])
            scores.append(np.ones_like(outputs[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))
        
        # 展平处理
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
        
        # 过滤低置信度检测
        boxes, classes, scores = self._filter_boxes(boxes, scores, classes_conf)
        
        # 非极大值抑制
        if boxes is not None:
            boxes, classes, scores = self._nms(boxes, classes, scores)
        
        return boxes, classes, scores
    
    def _process_boxes(self, position):
        """处理边界框输出"""
        grid_h, grid_w = position.shape[2:4]
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)
        stride = np.array([self.model_size[0] // grid_w, self.model_size[1] // grid_h]).reshape(1, 2, 1, 1)
        
        position = self._dfl(position)
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
        
        return xyxy
    
    def _dfl(self, position):
        """Distribution Focal Loss处理"""
        n, c, h, w = position.shape
        p_num = 4
        mc = c // p_num
        y = position.reshape(n, p_num, mc, h, w)
        y = self._softmax(y, 2)
        acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(2)
        return y
    
    def _softmax(self, x, axis=None):
        """Softmax函数"""
        x = x - x.max(axis=axis, keepdims=True)
        y = np.exp(x)
        return y / y.sum(axis=axis, keepdims=True)
    
    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """过滤低置信度的检测框"""
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape
        
        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        
        _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
        scores = (class_max_score * box_confidences)[_class_pos]
        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
        
        return boxes, classes, scores
    
    def _nms(self, boxes, classes, scores):
        """非极大值抑制"""
        nboxes, nclasses, nscores = [], [], []
        
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c_arr = classes[inds]
            s = scores[inds]
            keep = self._nms_boxes(b, s)
            
            if len(keep) != 0:
                nboxes.append(b[keep])
                nclasses.append(c_arr[keep])
                nscores.append(s[keep])
        
        if not nclasses and not nscores:
            return None, None, None
        
        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)
        
        return boxes, classes, scores
    
    def _nms_boxes(self, boxes, scores):
        """对单个类别执行NMS"""
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
        
        return np.array(keep)