#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机类 - 修复V2版本
避免GStreamer问题，直接使用V4L2
"""

import cv2
import numpy as np
import pandas as pd
import math
import os

class StereoCamera:
    """双目相机类"""
    
    def __init__(self, camera_id=21, width=1280, height=480):
        """
        初始化双目相机
        
        参数:
        camera_id: 相机设备ID
        width: 图像总宽度（左右相机合并）
        height: 图像高度
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
        
        # 相机内参和畸变参数
        self.left_camera_matrix = None
        self.left_distortion = None
        self.right_camera_matrix = None
        self.right_distortion = None
        self.R = None
        self.T = None
        
        # 距离计算参数
        self.min_valid_distance = 0.2  # 最小有效距离（米）
        self.max_valid_distance = 5.0  # 最大有效距离（米）
    
    def open_camera(self):
        """
        打开相机 - 简化版本，避免GStreamer问题
        
        返回:
        成功返回True，失败返回False
        """
        try:
            print(f"正在打开相机 ID={self.camera_id}")
            
            # 方法1：尝试使用V4L2后端
            print("尝试使用V4L2后端...")
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                print("V4L2后端失败，尝试默认后端...")
                # 方法2：使用默认后端
                self.cap = cv2.VideoCapture(self.camera_id)
            
            if self.cap.isOpened():
                print("相机已打开，设置参数...")
                
                # 设置缓冲区大小为1，减少延迟
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # 尝试设置分辨率
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                
                # 读取实际的分辨率
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = self.cap.get(cv2.CAP_PROP_FPS)
                
                print(f"相机参数: {actual_width}x{actual_height} @ {fps}fps")
                
                # 测试读取一帧
                print("测试读取帧...")
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    print(f"成功读取测试帧，尺寸: {frame.shape}")
                    print("相机初始化成功！")
                    return True
                else:
                    print("无法读取测试帧")
                    self.cap.release()
                    self.cap = None
                    return False
            else:
                print("无法打开相机")
                return False
                
        except Exception as e:
            print(f"打开相机时出错: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None
            return False
    
    def capture_frame(self):
        """
        捕获一帧图像
        
        返回:
        (left_frame, right_frame) 左右相机图像，失败返回(None, None)
        """
        if not self.cap or not self.cap.isOpened():
            return None, None
        
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None, None
        
        # 获取图像尺寸
        h, w = frame.shape[:2]
        
        # 如果是双目相机（宽度大于高度的1.5倍）
        if w > h * 1.5:
            # 分割左右相机图像
            mid = w // 2
            frame_left = frame[:, :mid]
            frame_right = frame[:, mid:]
        else:
            # 如果不是双目相机格式，返回相同图像
            frame_left = frame
            frame_right = frame.copy()
        
        return frame_left, frame_right
    
    def close_camera(self):
        """关闭相机"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("相机已关闭")
    
    def load_camera_params(self, file_path=None):
        """
        加载相机参数
        
        参数:
        file_path: 标定文件路径，如果为None则使用默认参数
        
        返回:
        成功返回True，失败返回False
        """
        if file_path and os.path.exists(file_path):
            try:
                # 从Excel文件读取参数
                df = pd.read_excel(file_path, header=None)
                
                self.left_camera_matrix = np.array(df.iloc[0:3, 1:4], dtype=np.float64)
                self.left_distortion = np.array(df.iloc[5, 1:6], dtype=np.float64).reshape(1, 5)
                self.right_camera_matrix = np.array(df.iloc[6:9, 1:4], dtype=np.float64)
                self.right_distortion = np.array(df.iloc[11, 1:6], dtype=np.float64).reshape(1, 5)
                self.T = np.array(df.iloc[12, 1:4], dtype=np.float64)
                self.R = np.array(df.iloc[13:16, 1:4], dtype=np.float64)
                
                print(f"成功从文件加载相机参数: {file_path}")
            except Exception as e:
                print(f"从文件加载相机参数失败: {e}")
                self._load_default_params()
        else:
            self._load_default_params()
        
        return True
    
    def _load_default_params(self):
        """加载默认的相机参数"""
        # 左相机内参矩阵
        self.left_camera_matrix = np.array([
            [479.511022870591, -0.276113089875797, 325.165562307888],
            [0., 482.402195086215, 267.117105422009],
            [0., 0., 1.]
        ])
        
        # 左相机畸变系数
        self.left_distortion = np.array([
            [0.0544639674308284, -0.0266591889115199, 
             0.00955609439715649, -0.0026033932373644, 0]
        ])
        
        # 右相机内参矩阵
        self.right_camera_matrix = np.array([
            [478.352067946262, 0.544542937907123, 314.900427485172],
            [0., 481.875120562091, 267.794159848602],
            [0., 0., 1.]
        ])
        
        # 右相机畸变系数
        self.right_distortion = np.array([
            [0.069434162778783, -0.115882071309996, 
             0.00979426351016958, -0.000953149415242267, 0]
        ])
        
        # 旋转矩阵
        self.R = np.array([
            [0.999896877234412, -0.00220178317092368, -0.0141910904351714],
            [0.00221406478831849, 0.999997187880575, 0.00084979294881938],
            [0.0141891794683169, -0.000881125309460678, 0.999898940295571]
        ])
        
        # 平移向量
        self.T = np.array([
            [-60.8066968317226], 
            [0.142395217396486], 
            [-1.92683450371277]
        ])
        
        print("使用默认相机参数")
    
    def setup_stereo_rectification(self, size=(640, 480)):
        """
        设置立体校正参数
        
        参数:
        size: 图像大小(宽, 高)
        
        返回:
        成功返回True，失败返回False
        """
        if self.left_camera_matrix is None:
            print("错误：请先加载相机参数")
            return False
        
        try:
            # 立体校正
            R1, R2, P1, P2, self.Q, self.valid_roi1, self.valid_roi2 = cv2.stereoRectify(
                self.left_camera_matrix, self.left_distortion,
                self.right_camera_matrix, self.right_distortion,
                size, self.R, self.T
            )
            
            # 计算校正查找表
            self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
                self.left_camera_matrix, self.left_distortion, 
                R1, P1, size, cv2.CV_16SC2
            )
            
            self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
                self.right_camera_matrix, self.right_distortion, 
                R2, P2, size, cv2.CV_16SC2
            )
            
            # 设置立体匹配算法（StereoBM）
            self.stereo = cv2.StereoBM_create(numDisparities=96, blockSize=15)
            
            # 设置StereoBM参数
            self.stereo.setROI1(self.valid_roi1)
            self.stereo.setROI2(self.valid_roi2)
            self.stereo.setPreFilterCap(31)
            self.stereo.setBlockSize(15)
            self.stereo.setMinDisparity(18)
            self.stereo.setNumDisparities(96)
            self.stereo.setTextureThreshold(50)
            self.stereo.setUniquenessRatio(18)
            self.stereo.setSpeckleWindowSize(83)
            self.stereo.setSpeckleRange(32)
            self.stereo.setDisp12MaxDiff(1)
            
            print("立体校正参数设置完成")
            return True
            
        except Exception as e:
            print(f"设置立体校正参数失败: {e}")
            return False
    
    def rectify_stereo_images(self, frame_left, frame_right):
        """
        校正左右相机图像
        
        参数:
        frame_left: 左相机图像
        frame_right: 右相机图像
        
        返回:
        (frame_left_rectified, img_left_gray, img_right_gray)
        校正后的左彩色图像，左灰度图像，右灰度图像
        """
        if self.left_map1 is None or self.right_map1 is None:
            print("错误：请先设置立体校正参数")
            return None, None, None
        
        # 转换为灰度图
        imgL_gray = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        imgR_gray = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
        
        # 应用校正映射
        img_left_rectified = cv2.remap(imgL_gray, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        img_right_rectified = cv2.remap(imgR_gray, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
        
        # 校正彩色图像用于显示
        frame_left_rectified = cv2.remap(frame_left, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        
        return frame_left_rectified, img_left_rectified, img_right_rectified
    
    def compute_disparity(self, img_left_rectified, img_right_rectified):
        """
        计算视差图
        
        参数:
        img_left_rectified: 校正后的左相机灰度图
        img_right_rectified: 校正后的右相机灰度图
        
        返回:
        (disparity, disp_normalized) 原始视差图和归一化视差图
        """
        if self.stereo is None:
            print("错误：立体匹配器未初始化")
            return None, None
        
        # 计算视差
        disparity = self.stereo.compute(img_left_rectified, img_right_rectified)
        
        # 归一化视差图用于显示
        disp_normalized = cv2.normalize(
            disparity, None, alpha=0, beta=255, 
            norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        
        return disparity, disp_normalized
    
    def compute_3d_points(self, disparity):
        """
        从视差图计算3D点云
        
        参数:
        disparity: 视差图
        
        返回:
        3D点云数组
        """
        if self.Q is None:
            print("错误：Q矩阵未初始化")
            return None
        
        # 将视差图转换为3D点云
        # handleMissingValues=True 会将无效点设置为(0,0,0)
        threeD = cv2.reprojectImageTo3D(disparity, self.Q, handleMissingValues=True)
        
        # 根据实际情况调整比例
        threeD = threeD * 16
        
        return threeD
    
    def calculate_distance(self, point_3d):
        """
        计算3D点到相机的欧氏距离
        
        参数:
        point_3d: 3D坐标点 (x, y, z)
        
        返回:
        距离（米），无效点返回None
        """
        if point_3d is None:
            return None
        
        x, y, z = point_3d
        
        # 检查坐标是否有效
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
            return None
        
        # 计算欧氏距离（毫米转米）
        distance = math.sqrt(x**2 + y**2 + z**2) / 1000.0
        
        # 验证距离是否在合理范围内
        if distance < self.min_valid_distance or distance > self.max_valid_distance:
            return None
        
        return distance
    
    def get_bottle_distance(self, threeD, cx, cy):
        """
        获取瓶子中心点的距离
        
        参数:
        threeD: 3D点云数组
        cx: 瓶子中心x坐标
        cy: 瓶子中心y坐标
        
        返回:
        距离（米），失败返回None
        """
        if threeD is None:
            return None
        
        try:
            # 确保坐标在有效范围内
            h, w = threeD.shape[:2]
            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                return None
            
            # 获取中心点周围区域的距离，提高稳定性
            radius = 3
            distances = []
            
            for dy in range(max(0, cy-radius), min(h, cy+radius+1)):
                for dx in range(max(0, cx-radius), min(w, cx+radius+1)):
                    point_3d = threeD[dy][dx]
                    distance = self.calculate_distance(point_3d)
                    if distance is not None:
                        distances.append(distance)
            
            # 使用中位数作为最终距离（更稳定）
            if distances:
                distances.sort()
                return distances[len(distances)//2]
            else:
                return None
                
        except Exception as e:
            print(f"计算瓶子距离失败: {e}")
            return None
    
    def set_distance_range(self, min_distance, max_distance):
        """
        设置有效距离范围
        
        参数:
        min_distance: 最小有效距离（米）
        max_distance: 最大有效距离（米）
        """
        self.min_valid_distance = min_distance
        self.max_valid_distance = max_distance