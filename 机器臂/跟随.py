import cv2
import numpy as np
from rknn.api import RKNN
import time
import math
import read as camera_config  # 双目校准参数
import threading
import queue
import serial
import logging
import yaml
from typing import Tuple, List, Dict, Optional, Union, Any

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("跟随抓取系统")

class Config:
    """配置类：加载和管理系统配置"""
    
    def __init__(self, config_file="config.yaml"):
        """
        初始化配置类
        
        Args:
            config_file: 配置文件路径
        """
        # 默认配置
        self.default_config = {
            "camera": {
                "id": 21,
                "width": 1280,
                "height": 480,
                "fps": 30
            },
            "model": {
                "path": "./my-main/yolo11n.rknn",
                "size": [640, 640],
                "obj_thresh": 0.4,
                "nms_thresh": 0.6
            },
            "uart": {
                "device": "/dev/ttyS9",
                "baudrate": 115200,
                "timeout": 0.1
            },
            "control": {
                "center_x": 400,
                "center_y": 280,
                "speed": 7.5,
                "catch_distance": 35
            }
        }
        
        # 尝试加载配置文件
        try:
            with open(config_file, 'r') as f:
                user_config = yaml.safe_load(f)
                if user_config:
                    # 递归更新配置
                    self._update_config(self.default_config, user_config)
        except Exception as e:
            logger.warning(f"加载配置文件失败: {e}，使用默认配置")
        
        # 设置实例属性
        self.CAMERA_ID = self.default_config["camera"]["id"]
        self.CAMERA_WIDTH = self.default_config["camera"]["width"]
        self.CAMERA_HEIGHT = self.default_config["camera"]["height"]
        self.RKNN_MODEL = self.default_config["model"]["path"]
        self.MODEL_SIZE = tuple(self.default_config["model"]["size"])
        self.OBJ_THRESH = self.default_config["model"]["obj_thresh"]
        self.NMS_THRESH = self.default_config["model"]["nms_thresh"]
        self.UART_DEVICE = self.default_config["uart"]["device"]
        self.BAUD_RATE = self.default_config["uart"]["baudrate"]
        self.TIMEOUT = self.default_config["uart"]["timeout"]
        self.CENTERX = self.default_config["control"]["center_x"]
        self.CENTERY = self.default_config["control"]["center_y"]
        self.SPEED = self.default_config["control"]["speed"]
        self.CATCH_DISTANCE = self.default_config["control"]["catch_distance"]
        
        # COCO数据集类别
        self.CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
               'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
               'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
               'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
               'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
               'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
               'hair drier', 'toothbrush']
        
        # 目标类别ID (默认跟踪瓶子-bottle类别)
        self.TARGET_CLASS_ID = 39  # bottle的类别ID
    
    def _update_config(self, default_config, user_config):
        """
        递归更新配置字典
        
        Args:
            default_config: 默认配置字典
            user_config: 用户配置字典
        """
        for key, value in user_config.items():
            if key in default_config:
                if isinstance(value, dict) and isinstance(default_config[key], dict):
                    self._update_config(default_config[key], value)
                else:
                    default_config[key] = value


class UARTController:
    """串口控制类：管理与舵机的通信"""
    
    def __init__(self, device, baudrate, timeout):
        """
        初始化串口控制器
        
        Args:
            device: 串口设备路径
            baudrate: 波特率
            timeout: 超时时间
        """
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
    def connect(self) -> bool:
        """
        连接串口设备
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.ser = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                xonxoff=False,
                rtscts=False
            )
            if not self.ser.is_open:
                self.ser.open()
            logger.info(f"串口 {self.device} 连接成功")
            return True
        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            return False
    
    def send(self, data) -> int:
        """
        发送数据到串口
        
        Args:
            data: 要发送的数据，可以是字符串或字节
            
        Returns:
            int: 发送的字节数，失败返回0
        """
        if self.ser is None or not self.ser.is_open:
            logger.error("串口未连接")
            return 0
            
        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            written = self.ser.write(data)
            logger.debug(f"已发送 {written} 字节: {data}")
            return written
        except Exception as e:
            logger.error(f"发送失败: {e}")
            return 0
    
    def receive(self) -> Optional[int]:
        """
        从串口接收数据，特别是舵机角度数据
        
        Returns:
            Optional[int]: 舵机角度，失败或无数据则返回None
        """
        if self.ser is None or not self.ser.is_open:
            logger.error("串口未连接")
            return None
            
        try:
            data = self.ser.read(256)
            if not data:
                return None
                
            # 解析舵机回传的角度信息
            data_str = data.decode('utf-8').strip()
            logger.debug(f"接收到数据: {data_str}")
            
            # 提取舵机编号和角度
            if data_str.startswith("#") and data_str.endswith("!"):
                data_content = data_str.strip("#!").strip().strip("\r\n")
                parts = data_content.split('P')
                if len(parts) >= 2:
                    servo_id = parts[0]
                    angle = int(parts[1].split('!')[0])  # 去除可能的其他字符
                    logger.debug(f"舵机编号: {servo_id}, 角度: {angle}")
                    return angle
            else:
                logger.warning("接收到的数据格式不正确")
                return None
        except Exception as e:
            logger.error(f"接收数据失败: {e}")
            return None
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")


class ObjectDetector:
    """目标检测类：处理RKNN模型加载和目标检测"""
    
    def __init__(self, model_path, model_size, obj_thresh, nms_thresh, classes):
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
    
    def release(self):
        """释放RKNN模型资源"""
        if self.rknn:
            self.rknn.release()
            logger.info("RKNN模型资源已释放")
    
    def detect(self, image):
        """
        执行目标检测
        
        Args:
            image: 输入图像
            
        Returns:
            tuple: (boxes, classes, scores) 检测结果
        """
        # 预处理图像
        img = self._letter_box(image, self.model_size)
        input_tensor = np.expand_dims(img, axis=0)
        
        # RKNN推理
        outputs = self.rknn.inference([input_tensor])
        
        # 后处理
        return self._post_process(outputs)
    
    def draw_detections(self, image, boxes, classes, scores, threeD, target_class_id=None):
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
            tuple: (center_x, center_y, distance) 目标的中心点和距离
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
                distance = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) / 1000.0  # 转换为米
                
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
    
    def _letter_box(self, im, new_shape, pad_color=(255, 255, 255)):
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
    
    def _sigmoid(self, x):
        """Sigmoid激活函数"""
        return 1 / (1 + np.exp(-x))
    
    def _softmax(self, x, axis=None):
        """Softmax激活函数"""
        x = x - x.max(axis=axis, keepdims=True)
        y = np.exp(x)
        return y / y.sum(axis=axis, keepdims=True)
    
    def _dfl(self, position):
        """Distribution Focal Loss (DFL)处理函数"""
        n, c, h, w = position.shape
        p_num = 4
        mc = c // p_num
        y = position.reshape(n, p_num, mc, h, w)
        y = self._softmax(y, 2)
        acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(2)
        return y
    
    def _box_process(self, position):
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
    
    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
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
    
    def _nms_boxes(self, boxes, scores):
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
    
    def _post_process(self, input_data):
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


class FollowController:
    """跟随控制类：管理目标跟随和抓取逻辑"""
    
    def __init__(self, uart_controller, config):
        """
        初始化跟随控制器
        
        Args:
            uart_controller: 串口控制器
            config: 配置参数
        """
        self.uart = uart_controller
        self.config = config
        
        # 控制状态
        self.pid_start_x = 0
        self.pid_start_y = 0
        self.stop_flag_x = 1
        self.read_flag_x = 1
        self.stop_flag_y = 1
        self.read_flag_y = 1
        self.send_left = 1
        self.send_right = 1
        self.send_up = 1
        self.send_down = 1
        
        # 抓取状态
        self.identify_flag = 1
        self.catch_process = 0
        self.send_catch_flag = 1
        self.catch_flag = 1
        self.distance_count = 0
        self.current_distance = 1000  # 初始距离设为一个大值
        
        # 预定义的指令集
        self.commands = {
            "read0": "#000PRAD!",
            "read5": "#005PRAD!",
            "get_mod": "#000PMOD!",
            "rt_start": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P2000T1500!#005P1200T2000!",
            "rt_catch1": "#000P1250T2000!#001P0900T2000!#002P1750T2000!#003P1200T2000!#004P1500T2000!#005P1750T2000!",
            "rt_catch2": "#000P2500T2000!#001P1400T2000!#002P1850T2000!#003P1700T2000!#004P1500T2000!#005P1750T2000!",
            "rt_catch3": "#000P2500T1500!#001P1300T1500!#002P2000T1500!#003P1700T1500!#004P1500T1500!#005P1200T1500!",
            "rt_catch4": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T2000!#005P1200T2000!",
            "DST": "#001PDST!",
            "init_position": "#000P1250T1500!#001P0900T1500!#002P2000T1500!#003P0800T1500!#004P1500T1500!#005P1200T1500!"
        }
    
    def initialize(self):
        """初始化机械臂位置"""
        logger.info("初始化机械臂位置")
        self.uart.send(self.commands["init_position"])
        time.sleep(1)  # 等待舵机到位
    
    def update_target(self, center_x, center_y, distance):
        """
        更新目标位置并执行跟随
        
        Args:
            center_x: 目标中心点x坐标
            center_y: 目标中心点y坐标
            distance: 目标距离（厘米）
        """
        if center_x is None or center_y is None:
            return
        
        self.current_distance = distance
        
        # 目标足够近时，开始计数准备抓取
        if self.current_distance <= self.config.CATCH_DISTANCE:
            self.distance_count += 1
            if self.distance_count >= 10 and self.identify_flag == 1:
                logger.info(f"目标距离 {self.current_distance:.1f}cm, 开始抓取流程")
                self.identify_flag = 0  # 停止识别，开始抓取
        else:
            self.distance_count = 0  # 重置计数
        
        # 如果在识别模式，执行跟随
        if self.identify_flag == 1:
            self.follow_x(center_x)
            self.follow_y(center_y)
        # 否则执行抓取
        else:
            self.catch_process_handler()
    
    def follow_x(self, target_x):
        """
        水平方向跟随控制
        
        Args:
            target_x: 目标中心点x坐标
        """
        # 如果目标位置与中心点误差在阈值内，停止运动
        if abs(self.config.CENTERX - target_x) <= 30:
            if self.stop_flag_x == 1:
                command = "#{:03d}PDST!".format(0)
                self.uart.send(command)
                self.stop_flag_x = 0
                self.read_flag_x = 1
                self.send_left = 1
                self.send_right = 1
        else:
            # 否则根据目标位置调整舵机
            self.stop_flag_x = 1
            if self.read_flag_x == 1:
                # 读取当前舵机位置
                command = "#{:03d}PRAD!".format(0)
                self.uart.send(command)
                self.pid_start_x = self.uart.receive()
                self.read_flag_x = 0
            else:
                # 目标在右侧，向右转
                if target_x - self.config.CENTERX > 30:
                    if self.pid_start_x is not None:
                        if self.pid_start_x > 2100:
                            command = "#{:03d}PDST!".format(0)
                        else:
                            temp = int((2167 - self.pid_start_x) * self.config.SPEED)
                            if temp < 4000:
                                temp = 4000
                            command = "#{:03d}P{:04d}T{:04d}!".format(0, 2167, temp)
                        if self.send_left == 1:
                            self.uart.send(command)
                            self.send_left = 0
                            self.send_right = 1
                # 目标在左侧，向左转
                elif self.config.CENTERX - target_x > 30:
                    if self.pid_start_x is not None:
                        if self.pid_start_x < 900:
                            command = "#{:03d}PDST!".format(0)
                        else:
                            temp = int((self.pid_start_x - 833) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(0, 833, temp)
                        if self.send_right == 1:
                            self.uart.send(command)
                            self.send_right = 0
                            self.send_left = 1
    
    def follow_y(self, target_y):
        """
        垂直方向跟随控制
        
        Args:
            target_y: 目标中心点y坐标
        """
        # 如果目标位置与中心点误差在阈值内，停止运动
        if abs(self.config.CENTERY - target_y) <= 30:
            if self.stop_flag_y == 1:
                command = "#{:03d}PDST!".format(1)
                self.uart.send(command)
                self.stop_flag_y = 0
                self.read_flag_y = 1
                self.send_up = 1
                self.send_down = 1
        else:
            # 否则根据目标位置调整舵机
            self.stop_flag_y = 1
            if self.read_flag_y == 1:
                # 读取当前舵机位置
                command = "#{:03d}PRAD!".format(1)
                self.uart.send(command)
                self.pid_start_y = self.uart.receive()
                self.read_flag_y = 0
            else:
                # 目标在下方，向下调整
                if target_y - self.config.CENTERY > 30:
                    if self.pid_start_y is not None:
                        if self.pid_start_y > 1480:
                            command = "#{:03d}PDST!".format(1)
                        else:
                            temp = int((1500 - self.pid_start_y) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(1, 1500, temp)
                        if self.send_up == 1:
                            self.uart.send(command)
                            self.send_up = 0
                            self.send_down = 1
                # 目标在上方，向上调整
                elif self.config.CENTERY - target_y > 30:
                    if self.pid_start_y is not None:
                        if self.pid_start_y < 882:
                            command = "#{:03d}PDST!".format(1)
                        else:
                            temp = int((self.pid_start_y - 882) * self.config.SPEED)
                            if temp < 3000:
                                temp = 3000
                            command = "#{:03d}P{:04d}T{:04d}!".format(1, 882, temp)
                        if self.send_down == 1:
                            self.uart.send(command)
                            self.send_down = 0
                            self.send_up = 1
    
    def catch_process_handler(self):
        """处理抓取流程的状态机"""
        # 状态0：准备抓取
        if self.catch_process == 0:
            if self.catch_flag == 1:
                self.uart.send(self.commands["read0"])
                catch0_pid0 = self.uart.receive()
                if catch0_pid0 is not None:
                    self.catch_flag = 0
            
            if self.catch_flag == 0:
                if self.send_catch_flag == 1:
                    command = "#000P{:04d}T1250!#001P0900T1500!#002P1750T1500!#003P1200T1500!#004P1500T1500!#005P1790T1500!".format(self.pid_start_x if self.pid_start_x else 1250)
                    self.uart.send(command)
                    self.send_catch_flag = 0
                else:
                    self.uart.send(self.commands["read5"])
                    catch0_pid5 = self.uart.receive()
                    if catch0_pid5 is not None and catch0_pid5 > 1720:
                        self.catch_process = 1
                        self.send_catch_flag = 1
                        self.catch_flag = 1
                        logger.info("抓取流程进入阶段1")
        
        # 状态1：抓取动作1
        elif self.catch_process == 1:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch2"])
                self.send_catch_flag = 0
            else:
                self.uart.send(self.commands["read0"])
                catch1_pid0 = self.uart.receive()
                if catch1_pid0 is not None and catch1_pid0 > 2450:
                    self.catch_process = 2
                    self.send_catch_flag = 1
                    logger.info("抓取流程进入阶段2")
        
        # 状态2：抓取动作2
        elif self.catch_process == 2:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch3"])
                self.send_catch_flag = 0
            else:
                self.uart.send(self.commands["read5"])
                catch2_pid5 = self.uart.receive()
                if catch2_pid5 is not None and catch2_pid5 < 1208:
                    self.catch_process = 3
                    self.send_catch_flag = 1
                    logger.info("抓取流程进入阶段3")
        
        # 状态3：抓取动作3（回到初始位置）
        elif self.catch_process == 3:
            if self.send_catch_flag == 1:
                self.uart.send(self.commands["rt_catch4"])
                self.send_catch_flag = 0
            else:
                self.uart.send(self.commands["read0"])
                catch3_pid0 = self.uart.receive()
                if catch3_pid0 is not None and catch3_pid0 < 1260 and catch3_pid0 > 1245:
                    self.catch_process = 0
                    self.send_catch_flag = 1
                    self.identify_flag = 1
                    self.distance_count = 0
                    self.current_distance = 1000
                    self.read_flag_y = 1
                    self.read_flag_x = 1
                    logger.info("抓取完成，恢复识别模式")


class DepthProcessor:
    """深度处理类：计算双目深度图"""
    
    def __init__(self):
        """初始化深度处理器"""
        pass
    
    def compute_depth(self, left_frame, right_frame):
        """
        计算双目深度图
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
            
        Returns:
            np.ndarray: 3D深度图
        """
        # 转换为灰度图
        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
        
        # 立体校正
        rect_left = cv2.remap(gray_left, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
        rect_right = cv2.remap(gray_right, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)
        
        # 创建StereoBM对象并计算视差
        stereo = cv2.StereoBM_create(numDisparities=96, blockSize=13)
        stereo.setMinDisparity(4)
        disparity = stereo.compute(rect_left, rect_right)
        
        # 将视差图转换为3D点云
        threeD = cv2.reprojectImageTo3D(disparity, camera_config.Q, handleMissingValues=True)
        threeD = threeD * 16  # 根据校准参数调整
        
        return threeD


class FollowSystem:
    """系统主类：整合所有模块并运行主循环"""
    
    def __init__(self):
        """初始化跟随系统"""
        # 加载配置
        self.config = Config()
        
        # 初始化串口控制器
        self.uart_controller = UARTController(
            self.config.UART_DEVICE,
            self.config.BAUD_RATE,
            self.config.TIMEOUT
        )
        
        # 初始化目标检测器
        self.object_detector = ObjectDetector(
            self.config.RKNN_MODEL,
            self.config.MODEL_SIZE,
            self.config.OBJ_THRESH,
            self.config.NMS_THRESH,
            self.config.CLASSES
        )
        
        # 初始化深度处理器
        self.depth_processor = DepthProcessor()
        
        # 初始化跟随控制器
        self.follow_controller = FollowController(
            self.uart_controller,
            self.config
        )
        
        # 线程安全的队列，用于在线程间传递深度图
        self.depth_queue = queue.Queue(maxsize=1)
        
        # 性能监控变量
        self.start_time = time.time()
        self.frame_count = 0
    
    def initialize(self):
        """初始化系统"""
        # 连接串口
        if not self.uart_controller.connect():
            logger.error("串口连接失败，程序退出")
            return False
        
        # 加载模型
        if not self.object_detector.load_model():
            logger.error("模型加载失败，程序退出")
            return False
        
        # 初始化机械臂位置
        self.follow_controller.initialize()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.config.CAMERA_ID)
        self.cap.set(3, self.config.CAMERA_WIDTH)
        self.cap.set(4, self.config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        if not self.cap.isOpened():
            logger.error("无法打开摄像头，程序退出")
            return False
        
        logger.info("系统初始化完成")
        return True
    
    def process_depth_thread(self, left_frame, right_frame):
        """
        深度处理线程函数
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
        """
        try:
            threeD = self.depth_processor.compute_depth(left_frame, right_frame)
            
            # 如果队列已满，清空队列
            if self.depth_queue.full():
                try:
                    self.depth_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.depth_queue.put(threeD)
        except Exception as e:
            logger.error(f"深度处理线程异常: {e}")
    
    def draw_fps(self, image, fps):
        """
        在图像上绘制帧率
        
        Args:
            image: 输入图像
            fps: 帧率
        """
        cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    def run(self):
        """运行主循环"""
        if not self.initialize():
            return
        
        try:
            while True:
                # 读取摄像头帧
                ret, frame = self.cap.read()
                if not ret:
                    logger.error("无法获取摄像头帧")
                    break
                
                # 分割左右图像
                left_frame = frame[0:480, 0:640]
                right_frame = frame[0:480, 640:1280]
                
                # 如果处于识别模式，执行目标检测和跟随
                if self.follow_controller.identify_flag == 1:
                    # 启动深度计算线程
                    depth_thread = threading.Thread(
                        target=self.process_depth_thread, 
                        args=(left_frame, right_frame)
                    )
                    depth_thread.daemon = True
                    depth_thread.start()
                    
                    # 进行目标检测
                    boxes, classes, scores = self.object_detector.detect(left_frame)
                    
                    # 等待深度图计算完成
                    try:
                        threeD = self.depth_queue.get(timeout=0.1)
                        
                        # 绘制检测结果并获取目标信息
                        if boxes is not None:
                            target_info = self.object_detector.draw_detections(
                                left_frame, boxes, classes, scores, threeD,
                                target_class_id=self.config.TARGET_CLASS_ID
                            )
                            
                            # 更新跟随控制器的目标位置
                            self.follow_controller.update_target(
                                target_info["center_x"],
                                target_info["center_y"],
                                target_info["distance"]
                            )
                    except queue.Empty:
                        logger.warning("深度图计算超时")
                else:
                    # 处于抓取模式，执行抓取流程
                    self.follow_controller.catch_process_handler()
                
                # 计算并显示帧率
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time
                self.draw_fps(left_frame, fps)
                
                # 显示当前距离和状态
                status = "抓取中" if not self.follow_controller.identify_flag else "跟踪中"
                distance_text = f"距离: {self.follow_controller.current_distance:.1f}cm"
                cv2.putText(left_frame, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(left_frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # 显示处理后的图像
                cv2.imshow('目标跟随系统', left_frame)
                
                # 按q退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 按r重置为识别模式
                if cv2.waitKey(1) & 0xFF == ord('r'):
                    self.follow_controller.identify_flag = 1
                    self.follow_controller.distance_count = 0
                    logger.info("手动重置为识别模式")
        
        finally:
            # 释放资源
            self.cap.release()
            cv2.destroyAllWindows()
            self.object_detector.release()
            self.uart_controller.close()
            logger.info("程序正常退出，资源已释放")


if __name__ == '__main__':
    # 创建并运行系统
    system = FollowSystem()
    system.run()


    