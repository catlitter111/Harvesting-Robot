# author: young
import cv2
import numpy as np
import camera_configs
import time

# 初始化双目摄像头
cap = cv2.VideoCapture(21)
if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

# 设置合适的分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 创建主窗口
cv2.namedWindow('SGBM')

# 创建参数调节窗口
cv2.namedWindow('Parameters')

# 基本参数
cv2.createTrackbar('num', 'Parameters', 2, 10, lambda x: None)  # numDisparities = 16*num
cv2.createTrackbar('blockSize', 'Parameters', 5, 21, lambda x: None)
cv2.createTrackbar('mode', 'Parameters', 0, 1, lambda x: None)  # 模式切换

# 高级参数
cv2.createTrackbar('minDisparity', 'Parameters', 0, 25, lambda x: None)
cv2.createTrackbar('P1', 'Parameters', 8, 200, lambda x: None)
cv2.createTrackbar('P2', 'Parameters', 32, 400, lambda x: None)
cv2.createTrackbar('disp12MaxDiff', 'Parameters', 1, 25, lambda x: None)
cv2.createTrackbar('preFilterCap', 'Parameters', 31, 100, lambda x: None)
cv2.createTrackbar('uniquenessRatio', 'Parameters', 10, 30, lambda x: None)
cv2.createTrackbar('speckleWindowSize', 'Parameters', 100, 200, lambda x: None)
cv2.createTrackbar('speckleRange', 'Parameters', 32, 50, lambda x: None)

# 可视化参数
cv2.createTrackbar('colormap', 'Parameters', 2, 21, lambda x: None)  # OpenCV颜色映射方案

app = 0
print("提示: mode=0 为参数调整模式(慢), mode=1 为实时模式(快)")
print("按 'q' 键退出")

try:
    while True:
        # 捕获帧
        ret, frame = cap.read()
        if not ret:
            print("错误：无法接收帧。等待重试...")
            time.sleep(0.1)
            continue

        # 分割为左右图像
        img_left = frame[0:480, 0:640]
        img_right = frame[0:480, 640:1280]

        # 显示原始左图像
        cv2.imshow("original_left", img_left)
        cv2.imshow("original_right", img_right)

        try:
            # 校正图像
            img_left_rectified = cv2.remap(img_left, camera_configs.left_map1, camera_configs.left_map2,
                                           cv2.INTER_LINEAR)
            img_right_rectified = cv2.remap(img_right, camera_configs.right_map1, camera_configs.right_map2,
                                            cv2.INTER_LINEAR)

            # 显示校正后的图像
            cv2.imshow("rectified_left", img_left_rectified)
            cv2.imshow("rectified_right", img_right_rectified)

            # 转换为灰度图
            imgL = cv2.cvtColor(img_left_rectified, cv2.COLOR_BGR2GRAY)
            imgR = cv2.cvtColor(img_right_rectified, cv2.COLOR_BGR2GRAY)

            # 获取基本参数
            num = cv2.getTrackbarPos('num', 'Parameters')
            blockSize = cv2.getTrackbarPos('blockSize', 'Parameters')
            mode = cv2.getTrackbarPos('mode', 'Parameters')

            # 获取高级参数
            minDisparity = cv2.getTrackbarPos('minDisparity', 'Parameters')
            p1 = cv2.getTrackbarPos('P1', 'Parameters')
            p2 = cv2.getTrackbarPos('P2', 'Parameters')
            disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'Parameters')
            preFilterCap = cv2.getTrackbarPos('preFilterCap', 'Parameters')
            uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio', 'Parameters')
            speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize', 'Parameters')
            speckleRange = cv2.getTrackbarPos('speckleRange', 'Parameters')
            colormap = cv2.getTrackbarPos('colormap', 'Parameters')

            # 确保参数有效
            if blockSize % 2 == 0:
                blockSize += 1
            if blockSize < 5:
                blockSize = 5

            if p2 <= p1:
                p2 = p1 + 1

            numDisparities = 16 * max(1, num)  # 确保至少为16

            # 创建SGBM对象并设置所有参数
            stereo = cv2.StereoSGBM_create(
                minDisparity=minDisparity,
                numDisparities=numDisparities,
                blockSize=blockSize,
                P1=p1 * 3 * blockSize ** 2,  # 根据blockSize动态调整
                P2=p2 * 3 * blockSize ** 2,  # 根据blockSize动态调整
                disp12MaxDiff=disp12MaxDiff if disp12MaxDiff > 0 else -1,
                preFilterCap=preFilterCap,
                uniquenessRatio=uniquenessRatio,
                speckleWindowSize=speckleWindowSize,
                speckleRange=speckleRange,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 使用全范围算法
            )

            # 计算视差图
            dis = stereo.compute(imgL, imgR)

            if app == 0:
                print("视差图维度：" + str(dis.ndim))
                print(type(dis))
                app = 1

            # 归一化视差图用于显示
            disp_normalized = cv2.normalize(dis, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # 应用彩色映射（使用用户选择的颜色映射）
            disp_color = cv2.applyColorMap(disp_normalized, colormap)

            # 添加当前参数信息到视差图
            info_text = f"numDisp:{numDisparities} blk:{blockSize} P1:{p1 * 3 * blockSize ** 2} P2:{p2 * 3 * blockSize ** 2}"
            cv2.putText(disp_color, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 显示视差图
            cv2.imshow('SGBM', disp_color)

            # 创建一个带有距离信息的伪3D视图
            if dis.size > 0:
                # 提取有效视差值（排除无效像素）
                valid_disparities = dis[dis > 0]
                if valid_disparities.size > 0:
                    # 假设基线和焦距已知
                    baseline = 60.81 # 毫米
                    focal_length = 478.93  # 像素

                    # 计算深度图 (深度 = 基线 * 焦距 / 视差)
                    # 注意：这里的深度单位取决于基线的单位
                    depth = np.zeros_like(dis, dtype=np.float32)
                    mask = dis > 0
                    depth[mask] = (baseline * focal_length) / dis[mask]

                    # 为显示目的限制深度范围
                    max_depth = 10000  # 最大深度，毫米
                    depth[depth > max_depth] = max_depth
                    depth[depth < 0] = 0

                    # 归一化深度图用于显示
                    depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

                    cv2.imshow('Depth', depth_color)

        except cv2.error as e:
            print(f"OpenCV错误: {e}")
            continue  # 如果有错误，跳到下一帧
        except Exception as e:
            print(f"其他错误: {e}")
            continue

        # 等待时间取决于模式
        wait_time = 1 if mode == 1 else 500  # 1ms (实时) 或 500ms (参数调整)
        key = cv2.waitKey(wait_time)

        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('s'):
            # 保存当前参数和视差图
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(f"disparity_{timestamp}.png", disp_color)

            # 保存参数到文本文件
            with open(f"params_{timestamp}.txt", "w") as f:
                f.write(f"minDisparity: {minDisparity}\n")
                f.write(f"numDisparities: {numDisparities}\n")
                f.write(f"blockSize: {blockSize}\n")
                f.write(f"P1: {p1 * 3 * blockSize ** 2}\n")
                f.write(f"P2: {p2 * 3 * blockSize ** 2}\n")
                f.write(f"disp12MaxDiff: {disp12MaxDiff}\n")
                f.write(f"preFilterCap: {preFilterCap}\n")
                f.write(f"uniquenessRatio: {uniquenessRatio}\n")
                f.write(f"speckleWindowSize: {speckleWindowSize}\n")
                f.write(f"speckleRange: {speckleRange}\n")

            print(f"已保存视差图和参数到文件 (disparity_{timestamp}.png, params_{timestamp}.txt)")

except Exception as e:
    print(f"发生错误: {e}")

finally:
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()