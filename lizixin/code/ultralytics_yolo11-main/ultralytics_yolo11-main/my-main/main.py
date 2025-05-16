# -*- coding: utf-8 -*-
 
import numpy as np
import cv2
# import camera_config
import random
import math
import cv2
import numpy as np

# 左相机内参
left_camera_matrix = np.array([[479.511022870591, -0.276113089875797, 325.165562307888],
                               [0., 482.402195086215, 267.117105422009],
                               [0., 0., 1.]])

# 左相机畸变系数:[k1, k2, p1, p2, k3]
left_distortion = np.array([[0.0544639674308284, -0.0266591889115199, 0.00955609439715649, -0.0026033932373644, 0]])

# 右相机内参
right_camera_matrix = np.array([[478.352067946262, 0.544542937907123, 314.900427485172],
                                [0., 481.875120562091, 267.794159848602],
                                [0., 0., 1.]])
# 右相机畸变系数:[k1, k2, p1, p2, k3]
right_distortion = np.array([[0.069434162778783, -0.115882071309996, 0.00979426351016958, -0.000953149415242267, 0]])

# om = np.array([-0.00009, 0.02300, -0.00372])
# R = cv2.Rodrigues(om)[0]

# 旋转矩阵
R = np.array([[0.999896877234412, -0.00220178317092368, -0.0141910904351714],
              [0.00221406478831849, 0.999997187880575, 0.00084979294881938],
              [0.0141891794683169, -0.000881125309460678, 0.999898940295571]])

# 平移向量
T = np.array([[-60.8066968317226], [0.142395217396486], [-1.92683450371277]])

size = (640, 640)

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)

left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)



cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 480)
 
 
# 鼠标回调函数
def onmouse_pick_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        threeD = param
        distance = math.sqrt( threeD[y][x][0] **2 + threeD[y][x][1] **2 + threeD[y][x][2] **2 ) 
        distance = distance / 1000.0  # mm -> m
        print("距离是：", distance, "m")
 
WIN_NAME = 'Deep disp'
cv2.namedWindow(WIN_NAME,  cv2.WINDOW_AUTOSIZE)
 
while True:
  ret, frame = cap.read()
  frame1 = frame[0:480, 0:640]
  frame2 = frame[0:640, 640:1280]  #割开双目图像
 
  imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)  # 将BGR格式转换成灰度图片
  imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
 
  # cv2.remap 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
  # 依据MATLAB测量数据重建无畸变图片
  img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
  img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
 
  imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)  
  imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)
 
  # BM 核心算法
  #numberOfDisparities = ((640 // 8) + 15) & -16  # 640对应是分辨率的宽
  numberOfDisparities=160
  stereo = cv2.StereoBM_create(numDisparities=16, blockSize=9)  #立体匹配
  stereo.setROI1(validPixROI1)
  stereo.setROI2(validPixROI2)
  stereo.setPreFilterCap(31)
  stereo.setBlockSize(15)
  stereo.setMinDisparity(4)#最小视差
  stereo.setNumDisparities(numberOfDisparities)
  stereo.setTextureThreshold(50)
  stereo.setUniquenessRatio(15)
  stereo.setSpeckleWindowSize(100)
  stereo.setSpeckleRange(32)
  stereo.setDisp12MaxDiff(1)
 
  disparity = stereo.compute(img1_rectified, img2_rectified) # 计算视差
 
  disp = cv2.normalize(disparity, disparity, alpha=100, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)  #归一化函数算法
 
  threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)  #计算三维坐标数据值
  threeD = threeD * 16 
 
  cv2.setMouseCallback(WIN_NAME, onmouse_pick_points, threeD)
 
  cv2.imshow("left", frame1)
  cv2.imshow("right", frame2)
  # cv2.imshow("left_r", imgL)
  # cv2.imshow("right_r", imgR)
  cv2.imshow(WIN_NAME, disp) 
 
  key = cv2.waitKey(1)
  if key == ord("q"):
    break
 
cap.release()
cv2.destroyAllWindows()