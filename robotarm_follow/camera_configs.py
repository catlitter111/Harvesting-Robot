# author: young
import cv2
import numpy as np

# # k1,k2,p1,p2,k3
# left_camera_matrix = np.array([[446.0063,0,289.9169], [0,445.9054,250.9938], [0, 0, 1]])
# left_distortion = np.array([[-0.0503,-0.0135,0,0,0]])
#
# right_camera_matrix = np.array([[446.5372,0,315.8924], [0,446.4056,239.0667], [0, 0, 1]])
# right_distortion = np.array([[-0.0535,-0.0145,0,0,0]])
#
# R = np.array([[1,0.00035115,-0.0038],
#               [-0.00034087801403453,1,0.0027],
#               [0.0038,-0.0027,1]])
# T = np.array([-60.9025,0.1096,-0.5221])
#
# size = (640, 480)  # open windows size
# # R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
# R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
#                                                                   right_camera_matrix, right_distortion, size, R, T)
#
# # 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
# left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
# right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
#
# print(Q)
#
# Q = [[1.0, 0.0, 0.0, -295.263828],
#      [0.0, 1.0, 0.0, -245.693567],
#      [0.0, 0.0, 0.0, 446.1555],
#      [0.0, 0.0, 0.0164190573, 0.0]]

# k1,k2,p1,p2,k3
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

size = (640, 480)  # open windows size
# R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

print(Q)


# 左相机内参
# left_camera_matrix = np.array([[416.841180253704, 0.0, 338.485167779639],
#                                [0., 416.465934495134, 230.419201769346],
#                                [0., 0., 1.]])

# # 左相机畸变系数:[k1, k2, p1, p2, k3]
# left_distortion = np.array([[-0.0170280933781798, 0.0643596519467521, -0.00161785356900972, -0.00330684695473645, 0]])
#
# # 右相机内参
# right_camera_matrix = np.array([[417.765094485395, 0.0, 315.061245379892],
#                                 [0., 417.845058291483, 238.181766936442],
#                                 [0., 0., 1.]])
# # 右相机畸变系数:[k1, k2, p1, p2, k3]
# right_distortion = np.array([[-0.0394089328586398, 0.131112076868352, -0.00133793245429668, -0.00188957913931929, 0]])
#
# # om = np.array([-0.00009, 0.02300, -0.00372])
# # R = cv2.Rodrigues(om)[0]
#
# # 旋转矩阵
# R = np.array([[0.999962872853149, 0.00187779299260463, -0.00840992323112715],
#               [-0.0018408858041373, 0.999988651353238, 0.00439412154902114],
#               [0.00841807904053251, -0.00437847669953504, 0.999954981430194]])
#
# # 平移向量
# T = np.array([[-120.326603502087], [0.199732192805711], [-0.203594457929446]])
#
# size = (640, 640)
#
# R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
#                                                                   right_camera_matrix, right_distortion, size, R,
#                                                                   T)
#
# left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
# right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)


