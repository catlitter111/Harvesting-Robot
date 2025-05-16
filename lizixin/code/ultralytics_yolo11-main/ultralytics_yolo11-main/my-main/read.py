import cv2 as cv
import numpy as np
import pandas as pd


df1=pd.read_excel('./my-main/out.xls',header=None)

left_camera_matrix=np.array(df1.iloc[0:3,1:4],dtype=np.float64)
left_distortion=np.array(df1.iloc[5,1:6],dtype=np.float64).reshape(1,5)
right_camera_matrix=np.array(df1.iloc[6:9,1:4],dtype=np.float64)
right_distortion=np.array(df1.iloc[11,1:6],dtype=np.float64).reshape(1,5)
T=np.array(df1.iloc[12,1:4],dtype=np.float64)
R=np.array(df1.iloc[13:16,1:4],dtype=np.float64)
# print(cameraMatrix1.head())

# size=(w,h)
size = (640, 480) # 图像尺寸

# 进行立体更正
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(left_camera_matrix, left_distortion,right_camera_matrix, right_distortion, size, R,T)
# 计算更正map
left_map1, left_map2 = cv.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv.CV_16SC2)
right_map1, right_map2 = cv.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv.CV_16SC2)
