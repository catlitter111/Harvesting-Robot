import yaml
import os
import logging
from typing import Dict, Any, List, Tuple

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("配置管理")

class Config:
    """配置类：加载和管理系统配置"""
    
    def __init__(self, config_file="config/config.yaml"):
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
                "path": "models/yolo11n.rknn",
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
            },
            "chassis": {
                "uart_device": None,  # 自动检测
                "baudrate": 115200,
                "default_speed": 50
            },
            "network": {
                "server_url": "ws://101.201.150.96:1234",
                "robot_id": "robot_123",
                "video_preset": "medium",
                "enable": False,
                "reconnect_attempts": 5,
                "reconnect_interval": 3
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
        
        # 底盘控制参数
        self.CHASSIS_UART_DEVICE = self.default_config["chassis"]["uart_device"]
        self.CHASSIS_BAUD_RATE = self.default_config["chassis"]["baudrate"]
        self.CHASSIS_DEFAULT_SPEED = self.default_config["chassis"]["default_speed"]
        
        # 网络参数
        self.NETWORK_SERVER_URL = self.default_config["network"]["server_url"]
        self.NETWORK_ROBOT_ID = self.default_config["network"]["robot_id"]
        self.NETWORK_VIDEO_PRESET = self.default_config["network"]["video_preset"]
        self.NETWORK_ENABLE = self.default_config["network"]["enable"]
        self.NETWORK_RECONNECT_ATTEMPTS = self.default_config["network"]["reconnect_attempts"]
        self.NETWORK_RECONNECT_INTERVAL = self.default_config["network"]["reconnect_interval"]
        
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
    
    def _update_config(self, default_config: Dict, user_config: Dict) -> None:
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
    
    def save_config(self, config_file: str = "config/config.yaml") -> bool:
        """
        保存当前配置到文件
        
        Args:
            config_file: 配置文件路径
            
        Returns:
            bool: 是否保存成功
        """
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            
            with open(config_file, 'w') as f:
                yaml.dump(self.default_config, f, default_flow_style=False)
                
            logger.info(f"配置已保存到: {config_file}")
            return True
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False