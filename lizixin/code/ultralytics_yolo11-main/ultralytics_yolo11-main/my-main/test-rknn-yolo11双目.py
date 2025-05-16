import cv2
import numpy as np
from rknn.api import RKNN
import time  # 导入time模块用于计算时间
import math
import read as camera_config  # 确保双目校准参数正确
import threading  # 导入 threading 模块用于多线程
import queue
import os
# 初始化参数
CAMERA_ID = 21  # 摄像头设备号
#RKNN_MODEL = "./ultralytics-main/yolo11n.rknn"
RKNN_MODEL =   "./my-main/train11/weights/best2-20.rknn"
MODEL_SIZE = (640, 640)  # 模型输入尺寸

# CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
#            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
#            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
#            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
#            'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
#            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
#            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
#            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
#            'hair drier', 'toothbrush'] # 保持原有类别列表
CLASSES = ['orange','bottle'] # 保持原有类别列表
# 后处理参数
# OBJ_THRESH = 0.03#yu zhi
OBJ_THRESH = 0.4#yu zhi
NMS_THRESH = 0.5

color_palette = np.random.uniform(0, 255, size=(len(CLASSES), 3))
# 添加帧率计算和显示功能
def draw_fps(image, fps):
    """
    在图像上绘制帧率信息。
    Args:
        image: 输入图像。
        fps: 当前帧率。
    """
    cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
# 保留所有原有处理函数（sigmoid、letter_box、filter_boxes、nms_boxes、dfl、box_process、post_process、draw等）
def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def letter_box(im, new_shape, pad_color=(255,255,255), info_need=False):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)  # add border

    if info_need is True:
        return im, ratio, (dw, dh)
    else:
        return im


def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def softmax(x, axis=None):
    x = x - x.max(axis=axis, keepdims=True)
    y = np.exp(x)
    return y / y.sum(axis=axis, keepdims=True)


def dfl(position):
    # Distribution Focal Loss (DFL)
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)
    y = softmax(y, 2)
    acc_metrix = np.array(range(mc), dtype=float).reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y


def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([MODEL_SIZE[1] // grid_h, MODEL_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

    return xyxy


def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    defualt_branch = 3
    pair_per_branch = len(input_data) // defualt_branch
    # Python 忽略 score_sum 输出
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # filter according to threshold
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

   

def draw_detections(img, left, top, right, bottom, score, class_id):
    """
    Draws bounding boxes and labels on the input image based on the detected objects.
    Args:
        img: The input image to draw detections on.
        box: Detected bounding box.
        score: Corresponding detection score.
        class_id: Class ID for the detected object.
    Returns:
        None
    """
    global distance1
    # Retrieve the color for the class ID
    color = color_palette[class_id]

    # Draw the bounding box on the image
    cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), color, 2)

    # Create the label text with class name and score
    if class_id==0:
        #label = f"{CLASSES[class_id]}: {score:.2f} "
        label = f"{CLASSES[class_id]}: {score:.2f} {distance1:.0f}cm"
    else:
        label = f"{CLASSES[class_id]}: {score*4:.2f}"

    # Calculate the dimensions of the label text
    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

    # Calculate the position of the label text
    label_x = left
    label_y = top - 10 if top - 10 > label_height else top + 10

    # Draw a filled rectangle as the background for the label text
    cv2.rectangle(img, (label_x, label_y - label_height), (label_x + label_width, label_y + label_height), color,
                  cv2.FILLED)

    # Draw the label text on the image
    cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def draw(image, boxes, scores, classes,threeD):
    global distance1
    img_h, img_w = image.shape[:2]
    # Calculate scaling factors for bounding box coordinates
    x_factor = img_w / MODEL_SIZE[0]
    y_factor = img_h / MODEL_SIZE[1]

    for box, score, cl in zip(boxes, scores, classes):
        x1, y1, x2, y2 = [int(_b) for _b in box]
        if cl==0:
            #获取中心坐标
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            print(center_x,center_y)
            # 获取3D坐标
            if 0 <= center_y < threeD.shape[0] and 0 <= center_x < threeD.shape[1]:
                point = threeD[center_y, center_x]
                distance = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) / 1000.0

                if distance < 10:
                    distance1=distance*100
            left = int(x1 * x_factor)
            top = int(y1 * y_factor) - 10
            right = int(x2 * x_factor)
            bottom = int(y2 * y_factor) + 10

            print('class: {}, score: {}'.format(CLASSES[cl], score))
            print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(left, top, right, bottom))

            # Retrieve the color for the class ID

            draw_detections(image, left, top, right, bottom, score, cl)
        else:continue
        # cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        # cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
        #             (left, top - 6),
        #             cv2.FONT_HERSHEY_SIMPLEX,
        #             0.6, (0, 0, 255), 2)


# 多线程计算深度图
def get_deepth_frame(left_frame, right_frame, threeD_queue):
    gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

    rect_left = cv2.remap(gray_left, camera_config.left_map1, camera_config.left_map2, cv2.INTER_LINEAR)
    rect_right = cv2.remap(gray_right, camera_config.right_map1, camera_config.right_map2, cv2.INTER_LINEAR)

    stereo = cv2.StereoBM_create(numDisparities=96, blockSize=13)
    stereo.setMinDisparity(4)
    disparity = stereo.compute(rect_left, rect_right)

    threeD = cv2.reprojectImageTo3D(disparity, camera_config.Q, handleMissingValues=True)
    threeD = threeD * 16  # 根据校准参数调整

    threeD_queue.put(threeD)


if __name__ == '__main__':
    # 创建并加载RKNN模型
    rknn = RKNN()
    print('--> Load RKNN model')
    if rknn.load_rknn(RKNN_MODEL) != 0:
        print('Load RKNN model failed')
        exit()

    # 初始化运行时环境
    print('--> Init runtime environment')
    if rknn.init_runtime(target='rk3588', device_id=0) != 0:
        print('Init runtime failed!')
        exit()

    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(3, 1280)  # 设置双目摄像头分辨率
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH,3840)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    try:
        image_count=0
        distance1=0
        # 初始化帧率相关变量
        start_time = time.time()
        frame_count = 0
        # 创建一个队列用于存储深度图
        threeD_queue = queue.Queue()
        while True:
            # 读取帧
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame")
                break

            # 预处理（保持与图片处理一致）

            left_frame = frame[0:480, 0:640]
            right_frame = frame[0:480, 640:1280]
            # 创建一个线程来计算深度图
            depth_thread = threading.Thread(target=get_deepth_frame, args=(left_frame, right_frame, threeD_queue))
            depth_thread.start()
            img = letter_box(left_frame, MODEL_SIZE)
            input_tensor = np.expand_dims(img, axis=0)

            # 推理
            outputs = rknn.inference([input_tensor])
            # print(f'outputs shape:{outputs[0].shape}')
            # 后处理
            boxes, classes, scores = post_process(outputs)
            # 等待深度图计算完成
            threeD = threeD_queue.get()
            # 绘制结果
            if boxes is not None:
                draw(left_frame, boxes, scores, classes,threeD)
            # 计算帧率
            frame_count += 1
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time

            # 在图像上绘制帧率信息
            draw_fps(left_frame, fps)
            # 显示结果
            cv2.imshow('RKNN Camera Detection', left_frame)
            image_path="./my-main"
            image_path = os.path.join(image_path, f"image_{image_count}.jpg")
            # 按Q退出
        # 如果按下 's' 键，保存图像
            if cv2.waitKey(1) & 0xFF == ord('s'):
                cv2.imwrite(image_path, left_frame)
                print(f"图像已保存到 {image_path}")
                image_count += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
        rknn.release()