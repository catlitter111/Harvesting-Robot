#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瓶子检测器类
使用D-Robotics BPU模型进行YOLO目标检测，专门检测瓶子
适配RDK X3/X5板卡
"""

import cv2
import numpy as np
from scipy.special import softmax
from hobot_dnn import pyeasy_dnn as dnn
import time
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("RDK_BottleDetector")

# YOLO检测参数
OBJ_THRESH = 0.02 # 目标检测阈值
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
        model_path: BPU模型文件路径 (.bin格式)
        model_size: 模型输入尺寸 (width, height)
        """
        self.model_path = model_path
        self.model_size = model_size
        self.rknn = None  # 保持变量名兼容性，实际存储BPU模型
        
        # 检测参数
        self.SCORE_THRESHOLD = OBJ_THRESH
        self.NMS_THRESHOLD = NMS_THRESH
        self.CONF_THRES_RAW = -np.log(1/self.SCORE_THRESHOLD - 1)
        self.REG = 16  # DFL回归参数
        self.CLASSES_NUM = len(CLASSES)
        
        # 颜色调色板，用于绘制检测框
        self.color_palette = np.random.uniform(0, 255, size=(len(CLASSES), 3))
        
        # 瓶子类别索引
        self.bottle_class_id = CLASSES.index('bottle')
        
        # 缩放参数
        self.x_scale = 1.0
        self.y_scale = 1.0
        self.x_shift = 0
        self.y_shift = 0
        
        # 原始图像尺寸
        self.img_h = 0
        self.img_w = 0
    
    def load_model(self):
        """
        加载BPU模型
        
        返回:
        成功返回True，失败返回False
        """
        try:
            begin_time = time.time()
            # 加载BPU的bin模型
            self.rknn = dnn.load(self.model_path)
            load_time = time.time() - begin_time
            logger.info(f"✓ 成功加载BPU模型: {self.model_path}")
            logger.info(f"✓ 加载时间: {load_time*1000:.2f}ms")
            
            # 打印模型信息
            logger.info(f"输入形状: {self.rknn[0].inputs[0].properties.shape}")
            logger.info(f"输出数量: {len(self.rknn[0].outputs)}")
            
            # 准备反量化系数
            self.s_bboxes_scale = self.rknn[0].outputs[1].properties.scale_data[np.newaxis, :]
            self.m_bboxes_scale = self.rknn[0].outputs[3].properties.scale_data[np.newaxis, :]
            self.l_bboxes_scale = self.rknn[0].outputs[5].properties.scale_data[np.newaxis, :]
            
            # DFL求期望的系数
            self.weights_static = np.array([i for i in range(self.REG)]).astype(np.float32)[np.newaxis, np.newaxis, :]
            
            # 生成anchors
            self._generate_anchors()
            
            # 获取模型输入尺寸
            self.input_H, self.input_W = self.rknn[0].inputs[0].properties.shape[2:4]
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 加载模型失败: {e}")
            return False
    
    def _generate_anchors(self):
        """生成anchor点"""
        # 生成三个尺度的anchor点
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
    
    def release_model(self):
        """释放BPU模型资源"""
        if self.rknn:
            # BPU模型会自动释放，这里主要是清理变量
            self.rknn = None
            logger.info('BPU模型资源已释放')
    
    def detect(self, image):
        """
        检测图像中的瓶子
        
        参数:
        image: 输入图像 (BGR格式)
        
        返回:
        检测结果列表: [(left, top, right, bottom, score, center_x, center_y), ...]
        """
        if self.rknn is None:
            logger.error('错误：模型未加载')
            return []
        
        try:
            # 图像预处理
            img = self._preprocess_image(image)
            
            # BPU推理
            outputs = self.rknn[0].forward(img)
            
            # 转换为numpy数组
            outputs = [dnnTensor.buffer for dnnTensor in outputs]
            
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
            logger.error(f'检测过程出错: {e}')
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
        color = tuple(map(int, self.color_palette[self.bottle_class_id]))
        
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
        self.img_h, self.img_w = image.shape[0:2]
        
        # 计算缩放比例，保持宽高比
        self.x_scale = min(1.0 * self.input_H / self.img_h, 1.0 * self.input_W / self.img_w)
        self.y_scale = self.x_scale
        
        # 计算新的图像尺寸
        new_w = int(self.img_w * self.x_scale)
        new_h = int(self.img_h * self.y_scale)
        
        # 计算padding
        self.x_shift = (self.input_W - new_w) // 2
        x_other = self.input_W - new_w - self.x_shift
        
        self.y_shift = (self.input_H - new_h) // 2
        y_other = self.input_H - new_h - self.y_shift
        
        # 调整图像大小
        input_tensor = cv2.resize(image, (new_w, new_h))
        
        # 添加边框填充
        input_tensor = cv2.copyMakeBorder(
            input_tensor, 
            self.y_shift, y_other, 
            self.x_shift, x_other, 
            cv2.BORDER_CONSTANT, 
            value=[114, 114, 114]
        )
        
        # 转换为NV12格式
        input_tensor = self._bgr2nv12(input_tensor)
        
        return input_tensor
    
    def _bgr2nv12(self, bgr_img):
        """BGR转NV12格式"""
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
    
    def _postprocess(self, outputs):
        """
        BPU输出后处理
        
        参数:
        outputs: BPU模型输出
        
        返回:
        (boxes, classes, scores) 检测框、类别和置信度
        """
        # Reshape输出
        s_clses = outputs[0].reshape(-1, self.CLASSES_NUM)
        s_bboxes = outputs[1].reshape(-1, self.REG * 4)
        m_clses = outputs[2].reshape(-1, self.CLASSES_NUM)
        m_bboxes = outputs[3].reshape(-1, self.REG * 4)
        l_clses = outputs[4].reshape(-1, self.CLASSES_NUM)
        l_bboxes = outputs[5].reshape(-1, self.REG * 4)

        # 分类阈值筛选
        s_max_scores = np.max(s_clses, axis=1)
        s_valid_indices = np.flatnonzero(s_max_scores >= self.CONF_THRES_RAW)
        s_ids = np.argmax(s_clses[s_valid_indices, :], axis=1)
        s_scores = s_max_scores[s_valid_indices]

        m_max_scores = np.max(m_clses, axis=1)
        m_valid_indices = np.flatnonzero(m_max_scores >= self.CONF_THRES_RAW)
        m_ids = np.argmax(m_clses[m_valid_indices, :], axis=1)
        m_scores = m_max_scores[m_valid_indices]

        l_max_scores = np.max(l_clses, axis=1)
        l_valid_indices = np.flatnonzero(l_max_scores >= self.CONF_THRES_RAW)
        l_ids = np.argmax(l_clses[l_valid_indices, :], axis=1)
        l_scores = l_max_scores[l_valid_indices]

        # Sigmoid激活
        s_scores = 1 / (1 + np.exp(-s_scores))
        m_scores = 1 / (1 + np.exp(-m_scores))
        l_scores = 1 / (1 + np.exp(-l_scores))

        # 反量化
        s_bboxes_float32 = s_bboxes[s_valid_indices,:].astype(np.float32) * self.s_bboxes_scale
        m_bboxes_float32 = m_bboxes[m_valid_indices,:].astype(np.float32) * self.m_bboxes_scale
        l_bboxes_float32 = l_bboxes[l_valid_indices,:].astype(np.float32) * self.l_bboxes_scale

        # dist2bbox - DFL解码
        s_ltrb_indices = np.sum(softmax(s_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        s_anchor_indices = self.s_anchor[s_valid_indices, :]
        s_x1y1 = s_anchor_indices - s_ltrb_indices[:, 0:2]
        s_x2y2 = s_anchor_indices + s_ltrb_indices[:, 2:4]
        s_dbboxes = np.hstack([s_x1y1, s_x2y2]) * 8

        m_ltrb_indices = np.sum(softmax(m_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        m_anchor_indices = self.m_anchor[m_valid_indices, :]
        m_x1y1 = m_anchor_indices - m_ltrb_indices[:, 0:2]
        m_x2y2 = m_anchor_indices + m_ltrb_indices[:, 2:4]
        m_dbboxes = np.hstack([m_x1y1, m_x2y2]) * 16

        l_ltrb_indices = np.sum(softmax(l_bboxes_float32.reshape(-1, 4, 16), axis=2) * self.weights_static, axis=2)
        l_anchor_indices = self.l_anchor[l_valid_indices,:]
        l_x1y1 = l_anchor_indices - l_ltrb_indices[:, 0:2]
        l_x2y2 = l_anchor_indices + l_ltrb_indices[:, 2:4]
        l_dbboxes = np.hstack([l_x1y1, l_x2y2]) * 32

        # 拼接所有检测结果
        dbboxes = np.concatenate((s_dbboxes, m_dbboxes, l_dbboxes), axis=0)
        scores = np.concatenate((s_scores, m_scores, l_scores), axis=0)
        ids = np.concatenate((s_ids, m_ids, l_ids), axis=0)

        # 转换为xywh格式用于NMS
        hw = (dbboxes[:,2:4] - dbboxes[:,0:2])
        xyhw = np.hstack([dbboxes[:,0:2], hw])

        # 分类别NMS
        boxes, classes, scores_out = [], [], []
        for i in range(self.CLASSES_NUM):
            id_indices = ids == i
            if not np.any(id_indices):
                continue
                
            indices = cv2.dnn.NMSBoxes(
                xyhw[id_indices,:], 
                scores[id_indices], 
                self.SCORE_THRESHOLD, 
                self.NMS_THRESHOLD
            )
            
            if len(indices) == 0:
                continue
                
            for indic in indices:
                x1, y1, x2, y2 = dbboxes[id_indices,:][indic]
                
                # 坐标转换回原图（letterbox逆变换）
                x1 = (x1 - self.x_shift) / self.x_scale
                y1 = (y1 - self.y_shift) / self.y_scale
                x2 = (x2 - self.x_shift) / self.x_scale
                y2 = (y2 - self.y_shift) / self.y_scale

                # 边界限制
                x1 = max(0, min(x1, self.img_w))
                x2 = max(0, min(x2, self.img_w))
                y1 = max(0, min(y1, self.img_h))
                y2 = max(0, min(y2, self.img_h))

                boxes.append([x1, y1, x2, y2])
                classes.append(i)
                scores_out.append(scores[id_indices][indic])

        if len(boxes) == 0:
            return None, None, None
            
        return np.array(boxes), np.array(classes), np.array(scores_out)
    
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


# 使用示例
def main():
    """主函数 - 测试用"""
    # 模型路径
    model_path = '/home/sunrise/project/my_main/models/yolo11n_detect_bayese_640x640_nv12_modified.bin'
    
    # 创建检测器
    detector = BottleDetector(model_path)
    
    # 加载模型
    if not detector.load_model():
        logger.error("模型加载失败")
        return
    
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("无法打开摄像头")
        detector.release_model()
        return
    
    logger.info("开始检测... 按 'q' 退出")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 检测瓶子
            detections = detector.detect(frame)
            
            # 绘制结果
            for detection in detections:
                detector.draw_detection(frame, detection)
            
            # 显示
            cv2.imshow('Bottle Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        logger.info("检测被中断")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        detector.release_model()


if __name__ == "__main__":
    main()