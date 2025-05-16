import cv2
import numpy as np
 
# 左相机内参
left_camera_matrix = np.array([[479.511022870591, -0.276113089875797,325.165562307888],
                                         [0., 482.402195086215, 267.117105422009],
                                         [0., 0., 1.]])
 
# 左相机畸变系数:[k1, k2, p1, p2, k3]
left_distortion = np.array([[0.0544639674308284, -0.0266591889115199, 0.00955609439715649, -0.0026033932373644, 0]])
 
# 右相机内参
right_camera_matrix = np.array([[478.352067946262, 0.544542937907123,314.900427485172],
                                          [0., 481.875120562091, 267.794159848602],
                                            [0., 0., 1.]])
# 右相机畸变系数:[k1, k2, p1, p2, k3]                                          
right_distortion = np.array([[0.069434162778783, -0.115882071309996, 0.00979426351016958, -0.000953149415242267, 0]])
 
# om = np.array([-0.00009, 0.02300, -0.00372])
# R = cv2.Rodrigues(om)[0]
 
# 旋转矩阵
R = np.array([[0.999896877234412,-0.00220178317092368, -0.0141910904351714],
                           [0.00221406478831849, 0.999997187880575, 0.00084979294881938],
                           [ 0.0141891794683169, -0.000881125309460678, 0.999898940295571]])
 
# 平移向量
T = np.array([[-60.8066968317226], [0.142395217396486], [-1.92683450371277]])
 
size = (640, 640)
 
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)
 
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)