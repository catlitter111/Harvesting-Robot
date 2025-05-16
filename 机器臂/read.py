import cv2
import numpy as np
import pandas as pd
import logging
import os

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("双目相机校准")

class CameraCalibration:
    """
    双目相机校准类：加载和处理双目相机校准参数
    """
    
    def __init__(self, calibration_file="机器臂/out.xls"):
        """
        初始化相机校准参数
        
        Args:
            calibration_file: 校准参数文件路径
        """
        self.calibration_file = calibration_file
        
        # 相机参数
        self.left_camera_matrix = None
        self.left_distortion = None
        self.right_camera_matrix = None
        self.right_distortion = None
        self.R = None  # 旋转矩阵
        self.T = None  # 平移向量
        
        # 校正映射
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.Q = None  # 重投影矩阵
        
        # 图像尺寸
        self.size = (640, 480)
        
        # 加载校准参数
        self._load_calibration()
        
        # 计算校正映射
        self._compute_rectification_maps()
    
    def _load_calibration(self):
        """从Excel文件加载校准参数"""
        try:
            if not os.path.exists(self.calibration_file):
                logger.error(f"校准文件不存在: {self.calibration_file}")
                raise FileNotFoundError(f"校准文件不存在: {self.calibration_file}")
            
            logger.info(f"正在加载校准文件: {self.calibration_file}")
            df = pd.read_excel(self.calibration_file, header=None)
            
            # 提取相机内参和畸变系数
            self.left_camera_matrix = np.array(df.iloc[0:3, 1:4], dtype=np.float64)
            self.left_distortion = np.array(df.iloc[5, 1:6], dtype=np.float64).reshape(1, 5)
            self.right_camera_matrix = np.array(df.iloc[6:9, 1:4], dtype=np.float64)
            self.right_distortion = np.array(df.iloc[11, 1:6], dtype=np.float64).reshape(1, 5)
            
            # 提取相机之间的旋转和平移
            self.T = np.array(df.iloc[12, 1:4], dtype=np.float64)
            self.R = np.array(df.iloc[13:16, 1:4], dtype=np.float64)
            
            logger.info("校准参数加载成功")
            
            # 校验相机矩阵有效性
            if not self._validate_camera_matrix():
                logger.warning("相机矩阵可能存在问题，请检查校准文件")
        
        except Exception as e:
            logger.error(f"加载校准参数错误: {e}")
            raise
    
    def _validate_camera_matrix(self):
        """
        验证相机矩阵的有效性
        
        Returns:
            bool: 是否有效
        """
        # 检查焦距是否合理
        fx_left = self.left_camera_matrix[0, 0]
        fy_left = self.left_camera_matrix[1, 1]
        fx_right = self.right_camera_matrix[0, 0]
        fy_right = self.right_camera_matrix[1, 1]
        
        # 焦距通常应为正值且与图像尺寸同一数量级
        if min(fx_left, fy_left, fx_right, fy_right) <= 0:
            logger.warning("相机矩阵中存在非正焦距，校准可能不正确")
            return False
        
        # 主点通常应在图像中心附近
        cx_left = self.left_camera_matrix[0, 2]
        cy_left = self.left_camera_matrix[1, 2]
        cx_right = self.right_camera_matrix[0, 2]
        cy_right = self.right_camera_matrix[1, 2]
        
        if (not (0 < cx_left < self.size[0]) or 
            not (0 < cy_left < self.size[1]) or
            not (0 < cx_right < self.size[0]) or
            not (0 < cy_right < self.size[1])):
            logger.warning("相机主点不在图像范围内，校准可能不正确")
            return False
        
        return True
    
    def _compute_rectification_maps(self):
        """计算双目校正的映射参数"""
        try:
            # 计算立体校正参数
            R1, R2, P1, P2, self.Q, roi1, roi2 = cv2.stereoRectify(
                self.left_camera_matrix, self.left_distortion,
                self.right_camera_matrix, self.right_distortion,
                self.size, self.R, self.T
            )
            
            # 计算校正映射
            self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
                self.left_camera_matrix, self.left_distortion, 
                R1, P1, self.size, cv2.CV_16SC2
            )
            
            self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
                self.right_camera_matrix, self.right_distortion, 
                R2, P2, self.size, cv2.CV_16SC2
            )
            
            logger.info("立体校正映射计算完成")
        
        except Exception as e:
            logger.error(f"计算校正映射错误: {e}")
            raise
    
    def rectify_images(self, left_img, right_img):
        """
        对双目图像进行校正
        
        Args:
            left_img: 左相机图像
            right_img: 右相机图像
            
        Returns:
            tuple: (左校正图像, 右校正图像)
        """
        if self.left_map1 is None or self.right_map1 is None:
            logger.error("校正映射未初始化")
            return None, None
        
        rect_left = cv2.remap(left_img, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        rect_right = cv2.remap(right_img, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
        
        return rect_left, rect_right


# 初始化全局相机校准对象
try:
    camera_calibration = CameraCalibration()
    
    # 导出必要的变量供其他模块使用
    left_map1 = camera_calibration.left_map1
    left_map2 = camera_calibration.left_map2
    right_map1 = camera_calibration.right_map1
    right_map2 = camera_calibration.right_map2
    Q = camera_calibration.Q
    
except Exception as e:
    logger.critical(f"相机校准初始化失败: {e}")
    # 创建虚拟校准数据作为备用
    logger.warning("使用备用校准参数")
    size = (640, 480)
    left_map1 = np.zeros((size[1], size[0]), dtype=np.float32)
    left_map2 = np.zeros((size[1], size[0]), dtype=np.float32)
    right_map1 = np.zeros((size[1], size[0]), dtype=np.float32)
    right_map2 = np.zeros((size[1], size[0]), dtype=np.float32)
    Q = np.eye(4, dtype=np.float64)

# 测试代码
if __name__ == "__main__":
    calibration = CameraCalibration()
    print("左相机矩阵:")
    print(calibration.left_camera_matrix)
    print("右相机矩阵:")
    print(calibration.right_camera_matrix)
    print("重投影矩阵 Q:")
    print(calibration.Q)

