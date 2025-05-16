from ultralytics import YOLO
import cv2

# 加载训练好的模型
model = YOLO(r"./my-main/train5/weights/best.pt")  # 修改为你的模型路径

# 初始化摄像头
cap = cv2.VideoCapture(21)  # 0 表示默认摄像头

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 定义一个函数来绘制检测结果
def draw_detections(frame, results):
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # 提取边界框坐标
            label = f"{model.names[int(box.cls[0])]} {box.conf[0].item():.2f}"  # 提取类别和置信度
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绘制矩形框
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # 绘制标签

# 实时检测循环
while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头帧")
        break

    # 调整帧大小（可选）
    frame = cv2.resize(frame, (640, 480))

    # 运行推理，设置置信度阈值为 0.5
    results = model.predict(frame, conf=0.7, verbose=False)  # conf=0.5 是置信度阈值
    # 绘制检测结果
    draw_detections(frame, results)

    # 显示结果
    cv2.imshow('YOLO Real-Time Detection', frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源
cap.release()
cv2.destroyAllWindows()