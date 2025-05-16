import cv2
import numpy as np
import logging
from typing import Tuple, List, Dict, Optional, Any
from rknn.api import RKNN

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("目标检测")

class ObjectDetector:
    """目标检测类：处理RKNN模型加载和目标检测"""
    
    def __init__(self, model_path: str, model_size: Tuple[int, int], obj_thresh: float, nms_thresh: float, classes: List[str]):
        """
        初始化目标检测器
        
        Args:
            model_path: RKNN模型路径
            model_size: 模型输入尺寸 (width, height)
            obj_thresh: 目标检测置信度阈值
            nms_thresh: 非极大值抑制阈值
            classes: 类别列表
        """
        self.model_path = model_path
        self.model_size = model_size
        self.obj_thresh = obj_thresh
        self.nms_thresh = nms_thresh
        self.classes = classes
        self.rknn = None
        self.color_palette = np.random.uniform(0, 255, size=(len(classes), 3))
        
    def load_model(self) -> bool:
        """
        加载RKNN模型
        
        Returns:
            bool: 是否成功加载模型
        """
        try:
            self.rknn = RKNN()
            logger.info('正在加载RKNN模型...')
            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                logger.error('加载RKNN模型失败')
                return False
                
            logger.info('初始化运行时环境...')
            ret = self.rknn.init_runtime(target='rk3588', device_id=0)
            if ret != 0:
                logger.error('初始化运行时环境失败!')
                return False
                
            logger.info('RKNN模型加载成功')
            return True
        except Exception as e:
            logger.error(f"加载模型过程中发生错误: {e}")
            return False
    
    def release(self) -> None:
        """释放RKNN模型资源"""
        if self.rknn:
            self.rknn.release()
            logger.info("RKNN模型资源已释放")
    
    def detect(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        执行目标检测
        
        Args:
            image: 输入图像
            
        Returns:
            tuple: (boxes, classes, scores) 检测结果，未检测到目标则返回(None, None, None)
        """
        try:
            # 检查模型是否已加载
            if self.rknn is None:
                logger.error("模型未加载，无法执行检测")
                return None, None, None
                
            # 预处理图像
            img = self._letter_box(image, self.model_size)
            input_tensor = np.expand_dims(img, axis=0)
            
            # RKNN推理
            outputs = self.rknn.inference([input_tensor])
            
            # 后处理
            return self._post_process(outputs)
        except Exception as e:
            logger.error(f"目标检测过程中发生错误: {e}")
            return None, None, None
    
    def draw_detections(self, image: np.ndarray, boxes: np.ndarray, classes: np.ndarray, 
                        scores: np.ndarray, threeD: np.ndarray, target_class_id: Optional[int] = None) -> Dict[str, Any]:
        """
        在图像上绘制检测结果并返回目标信息
        
        Args:
            image: 输入图像
            boxes: 边界框
            classes: 类别ID
            scores: 置信度
            threeD: 深度图
            target_class_id: 目标类别ID，如果指定，只返回该类别的中心点
            
        Returns:
            Dict: 包含目标中心点和距离的字典
        """
        img_h, img_w = image.shape[:2]
        x_factor = img_w / self.model_size[0]
        y_factor = img_h / self.model_size[1]
        
        target_info = {"center_x": None, "center_y": None, "distance": float('inf')}
        closest_dist = float('inf')
        
        # 按照距离排序绘制所有目标
        target_objects = []
        
        for box, score, cl in zip(boxes, scores, classes):
            if target_class_id is not None and cl != target_class_id:
                continue
                
            x1, y1, x2, y2 = [int(_b) for _b in box]
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            distance = float('inf')
            if 0 <= center_y < threeD.shape[0] and 0 <= center_x < threeD.shape[1]:
                point = threeD[center_y, center_x]
                distance = np.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) / 1000.0  # 转换为米
                
                if distance < 10:  # 10米内的有效目标
                    target_objects.append({
                        "box": box, 
                        "score": score, 
                        "class": cl, 
                        "center_x": center_x, 
                        "center_y": center_y, 
                        "distance": distance
                    })
                    
                    # 更新最近的目标
                    if distance < closest_dist:
                        closest_dist = distance
                        target_info = {
                            "center_x": center_x, 
                            "center_y": center_y, 
                            "distance": distance * 100  # 转换为厘米
                        }
        
        # 绘制所有检测到的目标
        for obj in target_objects:
            x1, y1, x2, y2 = [int(_b) for _b in obj["box"]]
            left = int(x1 * x_factor)
            top = int(y1 * y_factor) - 10
            right = int(x2 * x_factor)
            bottom = int(y2 * y_factor) + 10
            
            color = self.color_palette[obj["class"]]
            
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), color, 2)
            
            if obj["class"] == target_class_id:
                label = f"{self.classes[obj['class']]}: {obj['score']:.2f} {obj['distance']*100:.0f}cm"
            else:
                label = f"{self.classes[obj['class']]}: {obj['score']:.2f}"
                
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_x = left
            label_y = top - 10 if top - 10 > label_height else top + 10
            
            cv2.rectangle(image, (label_x, label_y - label_height), 
                         (label_x + label_width, label_y + label_height), color, cv2.FILLED)
            cv2.putText(image, label, (label_x, label_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            
            # 绘制目标中心点
            if obj["class"] == target_class_id:
                cv2.circle(image, (int(obj["center_x"]), int(obj["center_y"])), 5, (0, 255, 0), -1)
        
        # 在图像中心绘制参考十字线
        cv2.line(image, (target_info["center_x"] - 20 if target_info["center_x"] else img_w//2 - 20, 
                         target_info["center_y"] if target_info["center_y"] else img_h//2),
                (target_info["center_x"] + 20 if target_info["center_x"] else img_w//2 + 20, 
                 target_info["center_y"] if target_info["center_y"] else img_h//2),
                (0, 0, 255), 2)
        cv2.line(image, (target_info["center_x"] if target_info["center_x"] else img_w//2, 
                         target_info["center_y"] - 20 if target_info["center_y"] else img_h//2 - 20),
                (target_info["center_x"] if target_info["center_x"] else img_w//2, 
                 target_info["center_y"] + 20 if target_info["center_y"] else img_h//2 + 20),
                (0, 0, 255), 2)
        
        return target_info
    
    def _letter_box(self, im: np.ndarray, new_shape: Tuple[int, int], pad_color: Tuple[int, int, int] = (255, 255, 255)) -> np.ndarray:
        """
        将图像调整为指定尺寸，保持宽高比
        
        Args:
            im: 输入图像
            new_shape: 目标尺寸
            pad_color: 填充颜色
            
        Returns:
            调整后的图像
        """
        shape = im.shape[:2]  # 当前图像尺寸 [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
            
        # 计算缩放比例
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
            
        # 计算填充尺寸
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
            
        dw /= 2  # 填充均分两侧
        dh /= 2
            
        if shape[::-1] != new_unpad:  # 调整大小
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)
            
        return im
    
    def _sigmoid(self, x: np.ndarray) -> np.ndarray:
        """Sigmoid激活函数"""
        return 1 / (1 + np.exp(-x))
    
    def _softmax(self, x: np.ndarray, axis: Optional[int] = None) -> np.ndarray:
        """Softmax激活函数"""
        x = x - x.max(axis=axis, keepdims=True)
        y = np.exp(x)
        return y / y.sum(axis=axis, keepdims=True)
    
    def _dfl(self, position: np.ndarray) -> np.ndarray:
        """Distribution Focal Loss (DFL)处理函数"""
        n, c, h, w = position.shape
        p_num = 4
        mc = c // p_num
        y = position.reshape(n, p_num, mc, h, w)
        y = self._softmax(y, 2)
        acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(2)
        return y
    
    def _box_process(self, position: np.ndarray) -> np.ndarray:
        """边界框处理函数"""
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
    
    def _filter_boxes(self, boxes: np.ndarray, box_confidences: np.ndarray, box_class_probs: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """过滤低置信度的边界框"""
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape
            
        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
            
        _class_pos = np.where(class_max_score * box_confidences >= self.obj_thresh)
        scores = (class_max_score * box_confidences)[_class_pos]
            
        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
            
        return boxes, classes, scores
    
    def _nms_boxes(self, boxes: np.ndarray, scores: np.ndarray) -> np.ndarray:
        """非极大值抑制"""
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
            inds = np.where(ovr <= self.nms_thresh)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep
    
    def _post_process(self, input_data: List[np.ndarray]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        YOLO检测结果后处理
        
        Args:
            input_data: RKNN模型输出
            
        Returns:
            tuple: (boxes, classes, scores) 检测结果
        """
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
            
        if boxes.size == 0:
            return None, None, None
                
        # 非极大值抑制
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
                
        if not nclasses:
            return None, None, None
                
        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)
            
        return boxes, classes, scores