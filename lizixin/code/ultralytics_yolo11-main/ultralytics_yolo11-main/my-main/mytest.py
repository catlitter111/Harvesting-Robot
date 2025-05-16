import cv2

# 尝试打开默认摄像头
cap = cv2.VideoCapture(21)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("摄像头未连接或无法打开。")
else:
    print("摄像头已连接并可以正常工作。")

# 持续读取和显示帧，直到用户按下 'q' 键
while True:
    # 读取一帧图像
    ret, frame = cap.read()
    
    # 如果正确读取帧，ret 为 True
    if not ret:
        print("无法读取摄像头帧。")
        break
    
    # 显示结果帧
    cv2.imshow('Camera Frame', frame)
    
    # 按 'q' 键退出循环
    if cv2.waitKey(1) == ord('q'):
        break

# 释放摄像头资源
cap.release()
# 关闭所有 OpenCV 窗口
cv2.destroyAllWindows()