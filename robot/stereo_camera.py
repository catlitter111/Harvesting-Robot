#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机模块
处理双目相机相关的功能，包括相机初始化、参数加载、立体校正和深度图计算
"""

import cv2
import numpy as np
import pandas as pd
import logging
import math

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("stereo_camera")

# 定义距离有效范围
MIN_VALID_DISTANCE = 0.2  # 最小有效距离（米）
MAX_VALID_DISTANCE = 5.0  # 最大有效距离（米）

class StereoCamera:
    def __init__(self, camera_id=21, width=1280, height=480):
        """
        初始化双目相机
        
        参数:
        camera_id -- 相机ID
        width -- 画面宽度
        height -- 画面高度
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        self.stereo = None
        
        # 立体相机参数
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.Q = None
        self.valid_roi1 = None
        self.valid_roi2 = None
        
    def load_camera_params(self, file_path=None):
        """加载相机参数，可以从文件读取或使用硬编码值"""
        if file_path:
            try:
                df = pd.read_excel(file_path, header=None)
                
                left_camera_matrix = np.array(df.iloc[0:3, 1:4], dtype=np.float64)
                left_distortion = np.array(df.iloc[5, 1:6], dtype=np.float64).reshape(1, 5)
                right_camera_matrix = np.array(df.iloc[6:9, 1:4], dtype=np.float64)
                right_distortion = np.array(df.iloc[11, 1:6], dtype=np.float64).reshape(1, 5)
                T = np.array(df.iloc[12, 1:4], dtype=np.float64)
                R = np.array(df.iloc[13:16, 1:4], dtype=np.float64)
                
                logger.info("已从文件加载相机参数")
            except Exception as e:
                logger.error(f"无法从文件加载相机参数: {e}")
                logger.info("使用硬编码的相机参数")
                # 使用硬编码的相机参数
                left_camera_matrix = np.array([[479.511022870591, -0.276113089875797, 325.165562307888],
                                            [0., 482.402195086215, 267.117105422009],
                                            [0., 0., 1.]])
                left_distortion = np.array([[0.0544639674308284, -0.0266591889115199, 0.00955609439715649, -0.0026033932373644, 0]])
                right_camera_matrix = np.array([[478.352067946262, 0.544542937907123, 314.900427485172],
                                                [0., 481.875120562091, 267.794159848602],
                                                [0., 0., 1.]])
                right_distortion = np.array([[0.069434162778783, -0.115882071309996, 0.00979426351016958, -0.000953149415242267, 0]])
                R = np.array([[0.999896877234412, -0.00220178317092368, -0.0141910904351714],
                            [0.00221406478831849, 0.999997187880575, 0.00084979294881938],
                            [0.0141891794683169, -0.000881125309460678, 0.999898940295571]])
                T = np.array([[-60.8066968317226], [0.142395217396486], [-1.92683450371277]])
        else:
            # 使用硬编码的相机参数
            left_camera_matrix = np.array([[479.511022870591, -0.276113089875797, 325.165562307888],
                                        [0., 482.402195086215, 267.117105422009],
                                        [0., 0., 1.]])
            left_distortion = np.array([[0.0544639674308284, -0.0266591889115199, 0.00955609439715649, -0.0026033932373644, 0]])
            right_camera_matrix = np.array([[478.352067946262, 0.544542937907123, 314.900427485172],
                                            [0., 481.875120562091, 267.794159848602],
                                            [0., 0., 1.]])
            right_distortion = np.array([[0.069434162778783, -0.115882071309996, 0.00979426351016958, -0.000953149415242267, 0]])
            R = np.array([[0.999896877234412, -0.00220178317092368, -0.0141910904351714],
                        [0.00221406478831849, 0.999997187880575, 0.00084979294881938],
                        [0.0141891794683169, -0.000881125309460678, 0.999898940295571]])
            T = np.array([[-60.8066968317226], [0.142395217396486], [-1.92683450371277]])
        
        # 设置为类属性
        self.left_camera_matrix = left_camera_matrix
        self.left_distortion = left_distortion
        self.right_camera_matrix = right_camera_matrix
        self.right_distortion = right_distortion
        self.R = R
        self.T = T
        
        # 返回参数供其他方法使用
        return left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, R, T
    
    def setup_stereo_rectification(self, size=(640, 480)):
        """设置双目校正参数"""
        if not hasattr(self, 'left_camera_matrix'):
            logger.error("请先加载相机参数")
            return False
        
        # 进行立体校正
        R1, R2, P1, P2, self.Q, self.valid_roi1, self.valid_roi2 = cv2.stereoRectify(
            self.left_camera_matrix, self.left_distortion,
            self.right_camera_matrix, self.right_distortion,
            size, self.R, self.T)
        
        # 计算更正map
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            self.left_camera_matrix, self.left_distortion, R1, P1, size, cv2.CV_16SC2)
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            self.right_camera_matrix, self.right_distortion, R2, P2, size, cv2.CV_16SC2)
        
        # 设置立体匹配算法
        numberOfDisparities = 96
        self.stereo = cv2.StereoBM_create(numDisparities=18, blockSize=5)
        self.stereo.setROI1(self.valid_roi1)
        self.stereo.setROI2(self.valid_roi2)
        self.stereo.setPreFilterCap(31)
        self.stereo.setBlockSize(15)
        self.stereo.setMinDisparity(18)
        self.stereo.setNumDisparities(numberOfDisparities)
        self.stereo.setTextureThreshold(50)
        self.stereo.setUniquenessRatio(18)
        self.stereo.setSpeckleWindowSize(83)
        self.stereo.setSpeckleRange(32)
        self.stereo.setDisp12MaxDiff(1)
        
        logger.info("双目校正参数设置完成")
        return True
    
    def open_camera(self):
        """打开相机"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(3, self.width)   # 设置宽度
            self.cap.set(4, self.height)  # 设置高度
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            if not self.cap.isOpened():
                logger.error("无法打开相机")
                return False
                
            logger.info(f"成功打开相机 ID={self.camera_id}, 分辨率={self.width}x{self.height}")
            return True
        except Exception as e:
            logger.error(f"打开相机出错: {e}")
            return False
    
    def close_camera(self):
        """关闭相机"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logger.info("相机已关闭")
    
    def capture_frame(self):
        """捕获一帧图像，返回左右相机图像"""
        if not self.cap or not self.cap.isOpened():
            logger.error("相机未打开")
            return None, None
        
        ret, frame = self.cap.read()
        if not ret:
            logger.warning("无法接收帧")
            return None, None
        
        # 分割左右相机图像
        frame_left = frame[0:480, 0:640]
        frame_right = frame[0:480, 640:1280]
        
        return frame_left, frame_right
    
    def rectify_stereo_images(self, frame_left, frame_right):
        """校正左右相机图像"""
        if self.left_map1 is None or self.right_map1 is None:
            logger.error("请先设置双目校正参数")
            return None, None
        
        # 转换为灰度图
        imgL_gray = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        imgR_gray = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
        
        # 校正图像
        img_left_rectified = cv2.remap(imgL_gray, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        img_right_rectified = cv2.remap(imgR_gray, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
        
        # 将灰度图转换回彩色图用于检测和显示
        frame_left_rectified = cv2.cvtColor(img_left_rectified, cv2.COLOR_GRAY2BGR)
        
        return frame_left_rectified, img_left_rectified, img_right_rectified
    
    def compute_disparity(self, img_left_rectified, img_right_rectified):
        """计算视差图"""
        if self.stereo is None:
            logger.error("请先设置立体匹配算法")
            return None
        
        disparity = self.stereo.compute(img_left_rectified, img_right_rectified)
        
        # 归一化视差图以便显示
        disp_normalized = cv2.normalize(disparity, disparity, alpha=100, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        return disparity, disp_normalized
    
    def compute_3d_points(self, disparity):
        """计算三维坐标"""
        if self.Q is None:
            logger.error("请先设置双目校正参数")
            return None
        
        # 计算三维坐标
        threeD = cv2.reprojectImageTo3D(disparity, self.Q, handleMissingValues=True)
        threeD = threeD * 16  # 根据实际情况调整
        
        return threeD
    
    def calculate_distance(self, point_3d):
        """计算3D点到相机的欧氏距离(米)"""
        if point_3d is None:
            return None
            
        x, y, z = point_3d
        
        # 检查坐标值是否合理
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
            logger.warning(f"检测到无效3D点坐标: x={x}, y={y}, z={z}")
            return None
            
        # 计算距离（毫米转米）
        distance = math.sqrt(x**2 + y**2 + z**2) / 1000.0
        
        # 距离有效性验证
        if distance < MIN_VALID_DISTANCE or distance > MAX_VALID_DISTANCE:
            # logger.warning(f"检测到异常距离值: {distance}m（超出有效范围{MIN_VALID_DISTANCE}-{MAX_VALID_DISTANCE}m）")
            return None
            
        return distance
        
    def get_bottle_distance(self, threeD, cx, cy):
        """获取瓶子中心点的距离"""
        try:
            # 获取中心点及周围区域的平均距离，使距离计算更稳定
            radius = 3  # 取中心点周围半径为3的区域
            distances = []
            
            # 遍历中心点周围区域
            for y in range(max(0, cy-radius), min(threeD.shape[0], cy+radius+1)):
                for x in range(max(0, cx-radius), min(threeD.shape[1], cx+radius+1)):
                    point_3d = threeD[y][x]
                    distance = self.calculate_distance(point_3d)
                    if distance is not None:
                        distances.append(distance)
            
            # 如果收集到有效距离，计算中位数（比平均值更稳定）
            if distances:
                median_distance = sorted(distances)[len(distances)//2]
                return median_distance
            else:
                return None
                
        except Exception as e:
            logger.error(f"计算瓶子距离时出错: {e}")
            return None
        
        