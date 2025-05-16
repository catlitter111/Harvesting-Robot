import cv2
import numpy as np
from rknn.api import RKNN
import time
import math
import read as camera_config  # 确保双目校准参数正确
import threading
import queue
import serial

# 全局配置参数
CAMERA_ID = 21  # 摄像头设备号
RKNN_MODEL = "./my-main/yolo11n.rknn"
MODEL_SIZE = (640, 640)  # 模型输入尺寸
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
           'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
           'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
           'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
           'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
           'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
           'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
           'hair drier', 'toothbrush'] # 保持原有类别列表

OBJ_THRESH = 0.4  # 置信度阈值
NMS_THRESH = 0.6  # 非极大值抑制阈值

# 串口配置参数
UART_DEVICE = "/dev/ttyS9"  # UART0 设备节点（可能是 /dev/ttyS2）
BAUD_RATE = 115200  # 波特率
TIMEOUT = 0.1  # 读取超时（秒)

# 控制参数
CENTERX = 400
CENTERY = 280
SPEED=7.5
PID_STARTX=0
stop_flag_x = 1
read_flag_x = 1
stop_flag_y = 1
read_flag_y = 1
send_left=1
send_right=1
send_up=1
send_down=1
flag_stop=0

identify_flag=1
catch_process=0
send_catch_flag=1
send_receive_flag=1
catch_flag=1
# 初始化颜色
color_palette = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# 串口初始化
def uart_init():
    try:
        # 创建串口对象
        ser = serial.Serial(
            port=UART_DEVICE,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT,
            xonxoff=False,
            rtscts=False
        )
        if not ser.is_open:
            ser.open()
        return ser
    except Exception as e:
        print(f"无法打开串口: {e}")
        return None

# 发送数据
def send(ser, data):
    try:
        if isinstance(data, str):
            data = data.encode('utf-8')
        written = ser.write(data)
        print(f"已发送 {written} 字节: {data}")
        return written
    except Exception as e:
        print(f"发送失败: {e}")
        return 0


def receive_catch(ser):
    try:
        data = ser.read(256)
        if data:
            # 假设数据是字符串格式类似 "#000P1000!\r\n"
            data_str = data.decode('utf-8').strip()
            print(f"接收到数据: {data_str}")
            
            # 提取舵机编号和角度
            if data_str.startswith("#") and data_str.endswith("!"):
                data_content = data_str.strip("#!").strip().strip("\r\n")
                parts = data_content.split('P')
                if len(parts) >= 2:
                    servo_id = parts[0]
                    angle = int(parts[1].split('!')[0])  # 以防有其他字符
                    # 打印结果
                    print(f"舵机编号: {servo_id}, 角度: {angle}")
                    return int(angle)
            else:
                print("数据格式不正确")
        else:
            return None
    except Exception as e:
        print(f"接收失败: {e}")
        return None
# 跟随控制逻辑
def gensui_x():
    cx=400
    global CENTERX, stop_flag_x, read_flag_x,send_left,send_right,SPEED,PID_STARTX
    #print(cx - CENTERX)
    if abs(CENTERX - cx) <= 30:
        if stop_flag_x == 1:
            command1 = "#{:03d}PDST!".format(0)
            send(uart9, command1)
            stop_flag_x = 0
            read_flag_x = 1
            send_left=1
            send_right=1
    else:
        stop_flag_x = 1
        if read_flag_x == 1:
            command1 = "#{:03d}PRAD!".format(0)
            send(uart9, command1)
            PID_STARTX=receive_catch(uart9)
            read_flag_x=0
        else:
            #print(PID_STARTX)
            if cx - CENTERX > 30:
                
                if PID_STARTX > 2100:
                    command1 = "#{:03d}PDST!".format(0)
                else:
                    temp=int((2167-PID_STARTX)*SPEED)
                    if temp<4000:
                        temp=4000
                    command1 = "#{:03d}P{:04d}T{:04d}!".format(0, 2167, temp)
                if send_left==1:
                    send(uart9, command1)
                    send_left=0
                    send_right=1
            elif CENTERX - cx > 30:
                
                if PID_STARTX < 900:
                    command1 = "#{:03d}PDST!".format(0)
                else:
                    temp=int((PID_STARTX-833)*SPEED)
                    if temp<3000:
                        temp=3000
                    command1 = "#{:03d}P{:04d}T{:04d}!".format(0, 833, temp)
                if send_right==1:
                    send(uart9, command1)
                    send_right=0
                    send_left=1   
        # else:
        #     read_flag_x=1

def gensui_y():
    global CENTERY, stop_flag_y, read_flag_y,send_up,send_down,SPEED,PID_STARTY
    cy=280
    #print(cy - CENTERY)
    if abs(CENTERY - cy) <= 30:
        if stop_flag_y == 1:
            print(1)
            command1 = "#{:03d}PDST!".format(1)
            send(uart9, command1)
            stop_flag_y = 0
            read_flag_y = 1
            send_up=1
            send_down=1
    else:
        stop_flag_y = 1
        if read_flag_y == 1:
            command1 = "#{:03d}PRAD!".format(1)
            send(uart9, command1)
            PID_STARTY=receive_catch(uart9)
            read_flag_y=0
        else:
            if cy - CENTERY > 30:  
                if PID_STARTY > 1480:
                    command1 = "#{:03d}PDST!".format(1)
                else:
                    temp=int((1500 - PID_STARTY)*SPEED)
                    if temp<3000:
                        temp=3000
                    command1 = "#{:03d}P{:04d}T{:04d}!".format(1, 1500, temp)
                if send_up==1:
                    send(uart9, command1)
                    send_up=0
                    send_down=1
            elif CENTERY - cy > 30:
                
                if PID_STARTY < 882:
                    command1 = "#{:03d}PDST!".format(1)
                else:
                    temp=int((PID_STARTY - 882)*SPEED)
                    if temp <3000:
                        temp=3000
                    command1 = "#{:03d}P{:04d}T{:04d}!".format(1, 882, int((PID_STARTY - 882)*SPEED))
                if send_down==1:
                    send(uart9, command1)
                    send_down=0
                    send_up=1
        # else:
        #     read_flag_y=1
        # if read_flag == 1:
        #     send(uart9, command1)
        #     read_flag = 0


def catch_proc():
    global catch_process,send_catch_flag,identify_flag,send_receive_flag,distance1_count,distance1,catch_flag,read_flag_y,read_flag_x
    commands = {
    "read0":"#000PRAD!",
    "read5":"#005PRAD!",
    "get_mod":"#000PMOD!",
    "rt_start": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P2000T1500!#005P1200T2000!",
    "rt_catch1": "#000P1250T2000!#001P0900T2000!#002P1750T2000!#003P1200T2000!#004P1500T2000!#005P1750T2000!",
    "rt_catch2": "#000P2500T2000!#001P1400T2000!#002P1850T2000!#003P1700T2000!#004P1500T2000!#005P1750T2000!",
    "rt_catch3": "#000P2500T1500!#001P1300T1500!#002P2000T1500!#003P1700T1500!#004P1500T1500!#005P1200T1500!",
    "rt_catch4": "#000P1250T2000!#001P0900T2000!#002P2000T2000!#003P0800T2000!#004P1500T2000!#005P1200T2000!",
    "DST":"#001PDST!",
    }
    catch0_pid0=None
    if catch_process==0:
        if catch_flag==1:
            send(uart9,commands["read0"])
            catch0_pid0=receive_catch(uart9)
        if catch0_pid0!=None:
            catch_flag=0
        if catch_flag==0:
            if send_catch_flag==1:
                command1 = "#000P{:04d}T1250!#001P0900T1500!#002P1750T1500!#003P1200T1500!#004P1500T1500!#005P1790T1500!".format(catch0_pid0)
                send(uart9,command1)
                send_catch_flag=0
            else:
                # if send_receive_flag==1:
                send(uart9,commands["read5"])
                catch0_pid5=receive_catch(uart9)
                if catch0_pid5!=None and catch0_pid5>1720:
                    catch_process=1
                    send_catch_flag=1
                    catch_flag=1
    elif catch_process==1:
        if send_catch_flag==1:
            send(uart9,commands["rt_catch2"])
            send_catch_flag=0
        else:
            # if send_receive_flag==1:
            send(uart9,commands["read0"])
            catch1_pid0=receive_catch(uart9)
            if catch1_pid0!=None and catch1_pid0>2450:
                catch_process=2
                send_catch_flag=1
    elif catch_process==2:
        if send_catch_flag==1:
            send(uart9,commands["rt_catch3"])
            send_catch_flag=0
        else:
            send(uart9,commands["read5"])
            catch2_pid5=receive_catch(uart9)
            if catch2_pid5!=None and catch2_pid5<1208:
                catch_process=3
                send_catch_flag=1
    elif catch_process==3:
        if send_catch_flag==1:
            send(uart9,commands["rt_catch4"])
            send_catch_flag=0
        else:
            send(uart9,commands["read0"])
            catch3_pid0=receive_catch(uart9)
            if catch3_pid0!=None and  catch3_pid0<1260 and catch3_pid0>1245:
                catch_process=0
                send_catch_flag=1
                identify_flag=1
                distance1_count=0
                distance1=1000
                read_flag_y=1
                read_flag_x=1
    return

# 绘制帧率
def draw_fps(image, fps):
    cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# 计算目标检测相关函数
def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def letter_box(im, new_shape, pad_color=(255, 255, 255), info_need=False):
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # 计算缩放比例
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # 计算填充尺寸
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

    if info_need:
        return im, ratio, (dw, dh)
    else:
        return im

def filter_boxes(boxes, box_confidences, box_class_probs):
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

    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    if boxes.size == 0:
        return None, None, None

    # 非极大值抑制
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

    if not nclasses:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

# 绘制检测框
def draw_detections(img, left, top, right, bottom, score, class_id,_distance):
    color = color_palette[class_id]

    cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), color, 2)

    if class_id == 39:#39
        label = f"{CLASSES[class_id]}: {score:.2f} {_distance:.0f}cm"
    else:
        label = f"{CLASSES[class_id]}: {score:.2f}"

    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    label_x = left
    label_y = top - 10 if top - 10 > label_height else top + 10

    cv2.rectangle(img, (label_x, label_y - label_height), (label_x + label_width, label_y + label_height), color, cv2.FILLED)
    cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

def draw(image, boxes, scores, classes, threeD):
    global distance1, CENTERX, CENTERY,flag_stop,distance2
    TARGIT=[0,0,0]
    img_h, img_w = image.shape[:2]
    x_factor = img_w / MODEL_SIZE[0]
    y_factor = img_h / MODEL_SIZE[1]
    distance2=0
    i=0
    for box, score, cl in zip(boxes, scores, classes):
        if cl == 39:
            x1, y1, x2, y2 = [int(_b) for _b in box]
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            if 0 <= center_y < threeD.shape[0] and 0 <= center_x < threeD.shape[1]:
                point = threeD[center_y, center_x]
                distance = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) / 1000.0
                if distance < 10:
                    TARGIT.insert(i,distance*100)
                    i+=1
                    if distance2<distance:
                        distance1=distance*100
                        distance2=distance
                        CENTERY = center_y
                        CENTERX = center_x                    
    i=0
    for box, score, cl in zip(boxes, scores, classes):
        if cl == 39:
            x1, y1, x2, y2 = [int(_b) for _b in box]
            left = int(x1 * x_factor)
            top = int(y1 * y_factor) - 10
            right = int(x2 * x_factor)
            bottom = int(y2 * y_factor) + 10
            draw_detections(image, left, top, right, bottom, score, cl,TARGIT[i])
            i+=1
    if TARGIT[0]!=0:

        gensui_x()
        gensui_y()
    # if 39 not in list:
    #     if flag_stop==1:
    #         # stop()
    #         flag_stop=0
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
    # 初始化 RKNN 模型
    rknn = RKNN()
    print('--> Load RKNN model')
    if rknn.load_rknn(RKNN_MODEL) != 0:
        print('Load RKNN model failed')
        exit()

    print('--> Init runtime environment')
    if rknn.init_runtime(target='rk3588', device_id=0) != 0:
        print('Init runtime failed!')
        exit()

    # 初始化串口
    uart9 = uart_init()
    if not uart9:
        print("串口初始化失败")
        exit()
    else:
        print("串口初始化成功")
    send(uart9, "#000P1250T1500!#001P0900T1500!#002P2000T1500!#003P0800T1500!#004P1500T1500!#005P1200T1500!")
    time.sleep(1)
    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(3, 1280)  # 设置双目摄像头分辨率
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    try:
        distance1 = 1000
        distance2=0
        distance1_count=0
        start_time = time.time()
        frame_count = 0
        threeD_queue = queue.Queue()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame")
                break

            # 分割左右图像
            left_frame = frame[0:480, 0:640]
            right_frame = frame[0:480, 640:1280]

            if identify_flag==1:
                    
                # 创建线程计算深度图
                depth_thread = threading.Thread(target=get_deepth_frame, args=(left_frame, right_frame, threeD_queue))
                depth_thread.start()

                # 预处理图像
                img = letter_box(left_frame, MODEL_SIZE)
                input_tensor = np.expand_dims(img, axis=0)

                # RKNN 推理
                outputs = rknn.inference([input_tensor])

                # 后处理
                boxes, classes, scores = post_process(outputs)

                # 获取深度图
                threeD = threeD_queue.get()

                # 绘制结果
                if boxes is not None:
                    draw(left_frame, boxes, scores, classes, threeD)
                
            # 计算并显示帧率
            frame_count += 1
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time
            draw_fps(left_frame, fps)

            cv2.imshow('RKNN Camera Detection', left_frame)


            if distance1 <=35 :
                distance1_count+=1
                if distance1_count>=10:
                    identify_flag=0
                    catch_proc()
            # if cv2.waitKey(1) & 0xFF == ord('r'):
            #     identify_flag=1
            #     distance1_count=0
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        rknn.release()
        uart9.close()