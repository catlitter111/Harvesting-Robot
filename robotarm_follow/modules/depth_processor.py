import cv2
import numpy as np
import logging
from typing import Optional, Tuple
from .camera_calibration import calibration

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("深度处理")

class DepthProcessor:
    """深度处理类：计算双目深度图"""
    
    def __init__(self, num_disparities: int = 96, block_size: int = 13, min_disparity: int = 4):
        """
        初始化深度处理器
        
        Args:
            num_disparities: 视差计算中的最大视差值
            block_size: 匹配块大小
            min_disparity: 最小视差值
        """
        self.num_disparities = num_disparities
        self.block_size = block_size
        self.min_disparity = min_disparity
        
        # 创建StereoBM对象
        self.stereo = cv2.StereoBM_create(
            numDisparities=self.num_disparities,
            blockSize=self.block_size
        )
        self.stereo.setMinDisparity(self.min_disparity)
        
        # 使用相机校准参数或创建默认值
        if calibration is not None:
            self.left_map1 = calibration.left_map1
            self.left_map2 = calibration.left_map2
            self.right_map1 = calibration.right_map1
            self.right_map2 = calibration.right_map2
            self.Q = calibration.Q
        else:
            # 默认值
            size = (640, 480)
            self.left_map1 = np.zeros((size[1], size[0]), dtype=np.float32)
            self.left_map2 = np.zeros((size[1], size[0]), dtype=np.float32)
            self.right_map1 = np.zeros((size[1], size[0]), dtype=np.float32)
            self.right_map2 = np.zeros((size[1], size[0]), dtype=np.float32)
            self.Q = np.eye(4, dtype=np.float64)
            logger.warning("使用默认校准参数，深度估计可能不准确")
    
    def compute_depth(self, left_frame: np.ndarray, right_frame: np.ndarray) -> Optional[np.ndarray]:
        """
        计算双目深度图
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
            
        Returns:
            Optional[np.ndarray]: 3D深度图，如果计算失败则返回None
        """
        try:
            # 转换为灰度图
            gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            
            # 立体校正
            rect_left = cv2.remap(gray_left, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            rect_right = cv2.remap(gray_right, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
            
            # 计算视差
            disparity = self.stereo.compute(rect_left, rect_right)
            
            # 将视差图转换为3D点云
            threeD = cv2.reprojectImageTo3D(disparity, self.Q, handleMissingValues=True)
            threeD = threeD * 16  # 根据校准参数调整
            
            return threeD
            
        except Exception as e:
            logger.error(f"计算深度图失败: {e}")
            return None
    
    def get_point_distance(self, threeD: np.ndarray, x: int, y: int) -> Optional[float]:
        """
        获取指定点的距离
        
        Args:
            threeD: 3D深度图
            x: x坐标
            y: y坐标
            
        Returns:
            Optional[float]: 距离（米），如果无效则返回None
        """
        try:
            if 0 <= y < threeD.shape[0] and 0 <= x < threeD.shape[1]:
                point = threeD[y, x]
                distance = np.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) / 1000.0  # 转换为米
                
                # 过滤异常值
                if distance > 0 and distance < 10:  # 距离在0到10米之间
                    return distance
            return None
        except Exception as e:
            logger.error(f"计算点距离失败: {e}")
            return None
    
    def visualize_depth(self, disparity: np.ndarray, max_disparity: int = 96) -> np.ndarray:
        """
        将视差图可视化为彩色图像
        
        Args:
            disparity: 视差图
            max_disparity: 最大视差值
            
        Returns:
            np.ndarray: 可视化后的彩色图像
        """
        norm_disparity = disparity.astype(np.float32) / max_disparity
        norm_disparity = np.clip(norm_disparity, 0, 1)
        
        # 转换为彩色热图
        depth_colormap = cv2.applyColorMap(
            (norm_disparity * 255).astype(np.uint8), 
            cv2.COLORMAP_JET
        )
        
        return depth_colormap
    
    def compute_with_visualization(self, left_frame: np.ndarray, right_frame: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        计算深度图并生成可视化结果
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
            
        Returns:
            Tuple[Optional[np.ndarray], Optional[np.ndarray]]: (3D深度图, 可视化图像)
        """
        try:
            # 转换为灰度图
            gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            
            # 立体校正
            rect_left = cv2.remap(gray_left, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            rect_right = cv2.remap(gray_right, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
            
            # 计算视差
            disparity = self.stereo.compute(rect_left, rect_right)
            
            # 将视差图转换为3D点云
            threeD = cv2.reprojectImageTo3D(disparity, self.Q, handleMissingValues=True)
            threeD = threeD * 16  # 根据校准参数调整
            
            # 生成可视化结果
            vis_image = self.visualize_depth(disparity, self.num_disparities)
            
            return threeD, vis_image
            
        except Exception as e:
            logger.error(f"计算深度图失败: {e}")
            return None, None