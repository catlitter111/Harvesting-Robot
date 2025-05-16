# robot_client.py
import cv2
import base64
import json
import websocket
import threading
import time
import numpy as np
import psutil
import logging
import os
import serial
import serial.tools.list_ports
from queue import Queue, Empty
import random

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("robot_client")

# 配置项
SERVER_URL = "ws://101.201.150.96:1234/ws/robot/robot_123"
CAMERA_ID = 0  # 摄像头ID
INITIAL_PRESET = "medium"  # 初始质量预设

# 质量预设配置
QUALITY_PRESETS = {
    "high": {
        "resolution": (640, 480),
        "fps": 15,
        "bitrate": 800,  # Kbps
        "quality": 80  # JPEG质量(1-100)
    },
    "medium": {
        "resolution": (480, 360),
        "fps": 10,
        "bitrate": 500,
        "quality": 70
    },
    "low": {
        "resolution": (320, 240),
        "fps": 8,
        "bitrate": 300,
        "quality": 60
    },
    "very_low": {
        "resolution": (240, 180),
        "fps": 5,
        "bitrate": 150,
        "quality": 50
    },
    "minimum": {
        "resolution": (160, 120),
        "fps": 3,
        "bitrate": 80,
        "quality": 40
    }
}

# =====================串口命令相关常量=====================
# 命令类型常量
CMD_SET_DIRECTION = 0x01
CMD_SET_SPEED = 0x02
CMD_SET_MOTOR = 0x03
CMD_REQUEST_STATUS = 0x04
CMD_SET_POSITION = 0x05  # 新增：设置/更新位置信息

# 方向常量
DIR_FORWARD = 0x00
DIR_BACKWARD = 0x01
DIR_LEFT = 0x02
DIR_RIGHT = 0x03
DIR_STOP = 0x04

# 电机ID常量
MOTOR_FRONT_LEFT = 0
MOTOR_FRONT_RIGHT = 1
MOTOR_REAR_LEFT = 2
MOTOR_REAR_RIGHT = 3

# 串口设置
SERIAL_PORT = 'COM16'  # 如"/dev/ttyUSB0"或"COM3"，设为None则自动检测
SERIAL_BAUDRATE = 115200

# 全局变量
running = True
ws = None
connected = False
reconnect_count = 0
max_reconnect_attempts = 5
reconnect_interval = 3  # 重连间隔(秒)

# 视频配置
current_preset = INITIAL_PRESET
current_config = QUALITY_PRESETS[INITIAL_PRESET]
frame_queue = Queue(maxsize=10)  # 帧缓冲队列

# 机器人状态
robot_status = {
    "battery_level": 85,
    "position": {"x": 0, "y": 0, "latitude": 0.0, "longitude": 0.0},
    "harvested_count": 0,
    "cpu_usage": 0,
    "signal_strength": 70,
    "upload_bandwidth": 1000,  # 初始估计值(Kbps)
    "frames_sent": 0,
    "bytes_sent": 0,
    "last_bandwidth_check": time.time(),
    "last_bytes_sent": 0,
    "current_speed": 50,  # 默认速度为50%
    "current_direction": DIR_STOP  # 默认方向为停止
}

# 串口通信
serial_port = None
serial_lock = threading.Lock()  # 用于线程安全的串口访问


# 串口命令生成类
class CommandGenerator:
    @staticmethod
    def generate_packet(cmd, data=None):
        """
        生成数据包
        :param cmd: 命令类型
        :param data: 数据列表
        :return: 完整的命令字节序列
        """
        if data is None:
            data = []

        # 计算校验和
        checksum = cmd + len(data)
        for byte in data:
            checksum += byte
        checksum &= 0xFF  # 取低8位

        # 构建数据包
        packet = [0xAA, 0x55, cmd, len(data)] + data + [checksum]
        return bytes(packet)

    @staticmethod
    def generate_direction_command(direction, speed):
        """
        生成设置方向和速度的命令
        :param direction: 方向
        :param speed: 速度 (0-100)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return CommandGenerator.generate_packet(CMD_SET_DIRECTION, [direction, speed])

    @staticmethod
    def generate_speed_command(speed):
        """
        生成设置速度的命令
        :param speed: 速度 (0-100)
        :return: 命令字节序列
        """
        if speed > 100:
            speed = 100
        if speed < 0:
            speed = 0

        return CommandGenerator.generate_packet(CMD_SET_SPEED, [speed])

    @staticmethod
    def generate_position_command(latitude, longitude):
        """
        生成设置位置的命令
        :param latitude: 纬度
        :param longitude: 经度
        :return: 命令字节序列
        """
        # 将浮点数经纬度转换为整数（乘以10^6）
        lat_int = int(latitude * 1000000)
        lon_int = int(longitude * 1000000)

        # 将32位整数分解为4个字节
        position_data = [
            (lat_int >> 24) & 0xFF,
            (lat_int >> 16) & 0xFF,
            (lat_int >> 8) & 0xFF,
            lat_int & 0xFF,
            (lon_int >> 24) & 0xFF,
            (lon_int >> 16) & 0xFF,
            (lon_int >> 8) & 0xFF,
            lon_int & 0xFF
        ]

        return CommandGenerator.generate_packet(CMD_SET_POSITION, position_data)


# 串口初始化
def init_serial():
    global serial_port

    try:
        # 如果没有指定串口，尝试自动检测
        if SERIAL_PORT is None:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                logger.error("未找到串口设备")
                return False

            # 使用第一个找到的串口
            port = ports[0].device
            logger.info(f"自动选择串口: {port}")
        else:
            port = SERIAL_PORT

        # 打开串口
        serial_port = serial.Serial(port, SERIAL_BAUDRATE, timeout=1)
        logger.info(f"已连接到串口 {port}, 波特率: {SERIAL_BAUDRATE}")
        return True

    except Exception as e:
        logger.error(f"串口初始化失败: {e}")
        return False


# 发送命令到串口
def send_to_serial(command):
    global serial_port

    with serial_lock:
        if serial_port is None or not serial_port.is_open:
            if not init_serial():
                logger.error("串口未初始化，无法发送命令")
                return False

        try:
            serial_port.write(command)
            # 将命令转为十六进制字符串便于日志记录
            hex_command = ' '.join([f"{b:02X}" for b in command])
            logger.info(f"发送命令: {hex_command}")
            return True
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            return False


# 处理收到的控制命令
def handle_command(command_data):
    cmd = command_data.get("command")
    params = command_data.get("params", {})
    speed = params.get("speed", robot_status["current_speed"])

    # 确保速度在有效范围内
    speed = max(0, min(100, speed))

    logger.info(f"收到命令: {cmd}, 参数: {params}")

    # 更新当前速度
    robot_status["current_speed"] = speed

    # 根据不同命令执行不同操作
    if cmd == "forward":
        robot_status["current_direction"] = DIR_FORWARD
        command = CommandGenerator.generate_direction_command(DIR_FORWARD, speed)
        send_to_serial(command)
        logger.info(f"机器人前进, 速度: {speed}%")

    elif cmd == "backward":
        robot_status["current_direction"] = DIR_BACKWARD
        command = CommandGenerator.generate_direction_command(DIR_BACKWARD, speed)
        send_to_serial(command)
        logger.info(f"机器人后退, 速度: {speed}%")

    elif cmd == "left":
        robot_status["current_direction"] = DIR_LEFT
        command = CommandGenerator.generate_direction_command(DIR_LEFT, speed)
        send_to_serial(command)
        logger.info(f"机器人左转, 速度: {speed}%")

    elif cmd == "right":
        robot_status["current_direction"] = DIR_RIGHT
        command = CommandGenerator.generate_direction_command(DIR_RIGHT, speed)
        send_to_serial(command)
        logger.info(f"机器人右转, 速度: {speed}%")

    elif cmd == "stop":
        robot_status["current_direction"] = DIR_STOP
        command = CommandGenerator.generate_direction_command(DIR_STOP, 0)
        send_to_serial(command)
        logger.info("机器人停止")

    elif cmd == "set_motor_speed":
        # 单独设置速度命令
        command = CommandGenerator.generate_speed_command(speed)
        send_to_serial(command)
        logger.info(f"设置电机速度: {speed}%")

        # 如果当前有方向，更新该方向的速度
        if robot_status["current_direction"] != DIR_STOP:
            direction_cmd = CommandGenerator.generate_direction_command(
                robot_status["current_direction"], speed)
            send_to_serial(direction_cmd)

    elif cmd == "startHarvest":
        # 采摘功能可以添加其他串口命令
        logger.info("开始采摘")

    elif cmd == "stopHarvest":
        # 暂停采摘
        logger.info("暂停采摘")

    elif cmd == "emergencyStop":
        # 紧急停止所有功能
        robot_status["current_direction"] = DIR_STOP
        command = CommandGenerator.generate_direction_command(DIR_STOP, 0)
        send_to_serial(command)
        logger.info("紧急停止所有操作")


# 处理位置更新
def handle_position_update(data):
    try:
        position_data = data.get("data", [])
        if len(position_data) >= 8:
            # 从字节数组中解析出经纬度
            lat_int = (position_data[0] << 24) | (position_data[1] << 16) | (position_data[2] << 8) | position_data[3]
            lon_int = (position_data[4] << 24) | (position_data[5] << 16) | (position_data[6] << 8) | position_data[7]

            # 处理有符号整数
            if lat_int & 0x80000000:
                lat_int = lat_int - 0x100000000
            if lon_int & 0x80000000:
                lon_int = lon_int - 0x100000000

            # 转换回浮点数
            latitude = lat_int / 1000000.0
            longitude = lon_int / 1000000.0

            # 更新机器人状态中的位置信息
            robot_status["position"]["latitude"] = latitude
            robot_status["position"]["longitude"] = longitude

            logger.info(f"收到位置更新: 纬度={latitude}, 经度={longitude}")

            # 如果需要，可以在这里添加代码将位置信息发送到服务器
            if ws and connected:
                ws.send(json.dumps({
                    "type": "position_update",
                    "data": {
                        "latitude": latitude,
                        "longitude": longitude,
                        "timestamp": int(time.time() * 1000)
                    }
                }))
        else:
            logger.error("位置数据格式错误")
    except Exception as e:
        logger.error(f"处理位置更新错误: {e}")


# 设置位置并发送到小车
def set_position(latitude, longitude):
    try:
        # 更新本地状态
        robot_status["position"]["latitude"] = latitude
        robot_status["position"]["longitude"] = longitude

        # 生成并发送位置命令到小车
        command = CommandGenerator.generate_position_command(latitude, longitude)
        send_to_serial(command)

        # 发送位置更新到服务器
        if ws and connected:
            ws.send(json.dumps({
                "type": "position_update",
                "data": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "timestamp": int(time.time() * 1000)
                }
            }))

        logger.info(f"位置已设置: 纬度={latitude}, 经度={longitude}")
        return True
    except Exception as e:
        logger.error(f"设置位置失败: {e}")
        return False


# 处理质量调整命令
def handle_quality_adjustment(adjustment_data):
    global current_preset, current_config

    preset = adjustment_data.get("preset")

    if preset in QUALITY_PRESETS:
        logger.info(f"收到质量调整命令: {preset}")

        # 更新当前质量设置
        current_preset = preset
        current_config = QUALITY_PRESETS[preset]

        # 可以在这里调整摄像头参数
        # 注意：有些摄像头可能不支持动态调整分辨率

        # 发送调整结果
        if ws and connected:
            try:
                ws.send(json.dumps({
                    "type": "quality_adjustment_result",
                    "success": True,
                    "preset": preset,
                    "actual_resolution": f"{current_config['resolution'][0]}x{current_config['resolution'][1]}",
                    "actual_fps": current_config["fps"]
                }))
            except Exception as e:
                logger.error(f"发送质量调整结果失败: {e}")

        return True
    else:
        logger.error(f"未知的质量预设: {preset}")
        return False


# WebSocket接收消息处理
def on_message(ws, message):
    try:
        data = json.loads(message)
        message_type = data.get("type")

        if message_type == "command":
            handle_command(data)
        elif message_type == "quality_adjustment":
            handle_quality_adjustment(data)
        elif message_type == "set_position":
            handle_position_update(data)

    except json.JSONDecodeError:
        logger.error(f"收到无效JSON: {message}")
    except Exception as e:
        logger.error(f"处理消息错误: {e}")


def on_error(ws, error):
    logger.error(f"WebSocket错误: {error}")


def on_close(ws, close_status_code, close_msg):
    global connected
    connected = False
    logger.warning(f"WebSocket连接关闭: {close_status_code} {close_msg}")
    schedule_reconnect()


def on_open(ws):
    global connected, reconnect_count
    connected = True
    reconnect_count = 0
    logger.info("WebSocket连接已建立")

    # 发送初始状态
    send_status_update()


def schedule_reconnect():
    global reconnect_count

    if not running:
        return

    if reconnect_count < max_reconnect_attempts:
        reconnect_count += 1
        logger.info(f"计划第 {reconnect_count} 次重连，{reconnect_interval}秒后尝试...")

        time.sleep(reconnect_interval)

        if not connected and running:
            connect_to_server()
    else:
        logger.error(f"达到最大重连次数({max_reconnect_attempts})，停止重连")


def connect_to_server():
    global ws

    # 创建WebSocket连接
    ws = websocket.WebSocketApp(SERVER_URL,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    # 在新线程中运行WebSocket连接
    ws_thread = threading.Thread(target=ws.run_forever)
    ws_thread.daemon = True
    ws_thread.start()


# 状态更新线程
def status_update_thread():
    last_status_time = 0
    status_interval = 5  # 每5秒发送一次状态

    while running:
        try:
            current_time = time.time()

            # 更新CPU使用率
            robot_status["cpu_usage"] = psutil.cpu_percent(interval=0.1)

            # 计算上传带宽
            bytes_sent_diff = robot_status["bytes_sent"] - robot_status["last_bytes_sent"]
            time_diff = current_time - robot_status["last_bandwidth_check"]

            if time_diff > 0:
                # 计算Kbps
                upload_speed = (bytes_sent_diff * 8) / (time_diff * 1000)
                # 平滑带宽估计
                robot_status["upload_bandwidth"] = (robot_status["upload_bandwidth"] * 0.7) + (upload_speed * 0.3)

                # 更新检查点
                robot_status["last_bandwidth_check"] = current_time
                robot_status["last_bytes_sent"] = robot_status["bytes_sent"]

            # 定期发送状态更新
            if current_time - last_status_time >= status_interval:
                send_status_update()
                last_status_time = current_time

        except Exception as e:
            logger.error(f"状态更新错误: {e}")

        time.sleep(1)


def send_status_update():
    if ws and connected:
        try:
            ws.send(json.dumps({
                "type": "status_update",
                "data": {
                    "battery_level": robot_status["battery_level"],
                    "cpu_usage": robot_status["cpu_usage"],
                    "signal_strength": robot_status["signal_strength"],
                    "upload_bandwidth": robot_status["upload_bandwidth"],
                    "frames_sent": robot_status["frames_sent"],
                    "bytes_sent": robot_status["bytes_sent"],
                    "current_preset": current_preset,
                    "current_speed": robot_status["current_speed"],
                    "current_direction": robot_status["current_direction"],
                    "position": robot_status["position"]
                }
            }))
            logger.debug("状态更新已发送")
        except Exception as e:
            logger.error(f"发送状态更新失败: {e}")


# 视频捕获线程
def video_capture_thread():
    global running

    try:
        # 先设置好当前配置
        width, height = current_config["resolution"]

        # 打开摄像头并预先设置好分辨率
        cap = cv2.VideoCapture(CAMERA_ID)
        if not cap.isOpened():
            logger.error(f"无法打开摄像头 {CAMERA_ID}")
            return

        # 一次性设置参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        logger.info(f"摄像头已打开: {CAMERA_ID}, 分辨率: {width}x{height}")

        while running:
            # 读取视频帧
            ret, frame = cap.read()
            if not ret:
                logger.warning("无法获取视频帧")
                time.sleep(0.1)
                continue

            # 如果分辨率设置发生了变化，需要重新初始化摄像头
            current_width, current_height = current_config["resolution"]
            if (width != current_width or height != current_height):
                logger.info(f"分辨率设置已变更，重新初始化摄像头: {current_width}x{current_height}")

                # 关闭旧的摄像头
                cap.release()
                time.sleep(0.5)  # 给摄像头一些恢复时间

                # 更新当前分辨率记录
                width, height = current_width, current_height

                # 重新打开摄像头
                cap = cv2.VideoCapture(CAMERA_ID)
                if not cap.isOpened():
                    logger.error(f"无法重新打开摄像头 {CAMERA_ID}")
                    time.sleep(1)
                    continue

                # 设置新的分辨率
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                continue  # 跳过当前循环，重新读取帧

            # 将帧放入队列
            try:
                # 非阻塞方式，如果队列满了就丢弃帧
                if not frame_queue.full():
                    frame_queue.put_nowait(frame)
                else:
                    # 队列满了，丢弃这一帧
                    pass
            except Exception as e:
                logger.error(f"放入帧队列失败: {e}")

            # 根据目标帧率控制捕获频率
            time.sleep(1.0 / current_config["fps"])

    except Exception as e:
        logger.error(f"视频捕获错误: {e}")

    finally:
        if cap is not None:
            cap.release()
            logger.info("摄像头已释放")


# 视频发送线程
def video_sending_thread():
    global robot_status

    last_frame_time = time.time()
    frame_interval = 1.0 / QUALITY_PRESETS[INITIAL_PRESET]["fps"]

    # 用于计算实际FPS
    fps_counter = 0
    fps_timer = time.time()
    actual_fps = 0

    while running:
        try:
            # 控制发送频率
            current_time = time.time()
            elapsed = current_time - last_frame_time

            # 根据当前预设更新帧间隔
            frame_interval = 1.0 / current_config["fps"]

            if elapsed < frame_interval:
                time.sleep(0.001)  # 短暂休眠，减少CPU使用
                continue

            # 尝试从队列获取一帧
            try:
                frame = frame_queue.get(block=False)
            except Empty:
                time.sleep(0.01)
                continue

            # 处理并发送帧
            if ws and connected:
                try:
                    # 调整大小以匹配当前分辨率设置
                    if frame.shape[1] != current_config["resolution"][0] or frame.shape[0] != \
                            current_config["resolution"][1]:
                        frame = cv2.resize(frame, current_config["resolution"])

                    # 压缩帧为JPEG格式
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), current_config["quality"]]
                    _, buffer = cv2.imencode('.jpg', frame, encode_param)

                    # 转为Base64编码
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')

                    # 构建消息
                    message = {
                        "type": "video_frame",
                        "preset": current_preset,
                        "resolution": f"{current_config['resolution'][0]}x{current_config['resolution'][1]}",
                        "timestamp": int(current_time * 1000),  # 毫秒时间戳
                        "data": jpg_as_text
                    }

                    # 发送视频帧
                    message_json = json.dumps(message)
                    ws.send(message_json)

                    # 更新统计信息
                    robot_status["frames_sent"] += 1
                    robot_status["bytes_sent"] += len(message_json)

                    # 计算实际FPS
                    fps_counter += 1
                    if current_time - fps_timer >= 1.0:  # 每秒计算一次
                        actual_fps = fps_counter
                        fps_counter = 0
                        fps_timer = current_time
                        logger.debug(f"实际FPS: {actual_fps}, 目标FPS: {current_config['fps']}")

                    # 更新最后发送时间
                    last_frame_time = current_time

                except Exception as e:
                    logger.error(f"处理视频帧错误: {e}")

        except Exception as e:
            logger.error(f"视频发送线程错误: {e}")


# 模拟电池消耗
def battery_simulation_thread():
    while running:
        try:
            # 缓慢减少电池电量
            robot_status["battery_level"] = max(0, robot_status["battery_level"] - 0.02)

            # 随机变化信号强度
            if random.random() < 0.1:  # 10%的概率改变信号强度
                signal_delta = random.uniform(-5, 5)
                robot_status["signal_strength"] = max(0, min(100, robot_status["signal_strength"] + signal_delta))

            time.sleep(5)

        except Exception as e:
            logger.error(f"电池模拟错误: {e}")


# 串口监听线程
def serial_listen_thread():
    global serial_port

    while running:
        with serial_lock:
            if serial_port is not None and serial_port.is_open:
                try:
                    if serial_port.in_waiting > 0:
                        data = serial_port.read(serial_port.in_waiting)
                        if data:
                            # 将接收到的数据转换为十六进制字符串
                            hex_data = ' '.join([f"{b:02X}" for b in data])
                            logger.debug(f"收到串口数据: {hex_data}")

                            # 这里可以添加代码解析和处理串口数据
                            parse_serial_data(data)
                except Exception as e:
                    logger.error(f"读取串口数据错误: {e}")

        time.sleep(0.1)  # 短暂休眠，减少CPU使用


# 解析串口数据
def parse_serial_data(data):
    # 寻找帧头
    i = 0
    while i < len(data) - 1:
        if data[i] == 0xAA and data[i + 1] == 0x55:
            # 找到帧头，检查数据包是否完整
            if i + 3 < len(data):
                cmd = data[i + 2]
                length = data[i + 3]

                # 检查数据包长度是否足够
                if i + 4 + length + 1 <= len(data):
                    # 提取数据部分
                    packet_data = data[i + 4:i + 4 + length]

                    # 计算校验和
                    checksum = cmd + length
                    for b in packet_data:
                        checksum += b
                    checksum &= 0xFF  # 取低8位

                    # 获取接收到的校验和
                    received_checksum = data[i + 4 + length]

                    # 校验
                    if checksum == received_checksum:
                        # 校验通过，处理命令
                        process_serial_command(cmd, packet_data)
                    else:
                        logger.warning(f"校验和错误: 计算={checksum:02X}, 接收={received_checksum:02X}")

                    # 跳过已处理的数据
                    i += 4 + length + 1
                    continue

        i += 1


# 处理串口命令
def process_serial_command(cmd, data):
    if cmd == CMD_SET_POSITION and len(data) >= 8:
        # 解析位置数据
        lat_int = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        lon_int = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]

        # 处理有符号整数
        if lat_int & 0x80000000:
            lat_int = lat_int - 0x100000000
        if lon_int & 0x80000000:
            lon_int = lon_int - 0x100000000

        # 转换回浮点数
        latitude = lat_int / 1000000.0
        longitude = lon_int / 1000000.0

        # 更新位置
        robot_status["position"]["latitude"] = latitude
        robot_status["position"]["longitude"] = longitude

        logger.info(f"从小车接收位置更新: 纬度={latitude}, 经度={longitude}")

        # 发送到服务器
        if ws and connected:
            ws.send(json.dumps({
                "type": "position_update",
                "data": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "timestamp": int(time.time() * 1000)
                }
            }))

    elif cmd == CMD_SET_DIRECTION and len(data) >= 2:
        # 更新方向和速度
        direction = data[0]
        speed = data[1]

        robot_status["current_direction"] = direction
        robot_status["current_speed"] = speed

        logger.info(f"从小车接收状态更新: 方向={direction}, 速度={speed}%")

    elif cmd == CMD_REQUEST_STATUS and len(data) >= 6:
        # 解析状态数据
        direction = data[0]
        speed = data[1]
        motor_speeds = data[2:6]

        robot_status["current_direction"] = direction
        robot_status["current_speed"] = speed

        logger.info(
            f"从小车接收状态: 方向={direction}, 速度={speed}%, 电机=[{motor_speeds[0]}, {motor_speeds[1]}, {motor_speeds[2]}, {motor_speeds[3]}]")


# 主函数
def main():
    global running

    try:
        logger.info("启动机器人客户端...")

        # 初始化串口
        init_serial()

        # 连接到服务器
        connect_to_server()

        # 创建并启动状态更新线程
        status_thread = threading.Thread(target=status_update_thread)
        status_thread.daemon = True
        status_thread.start()

        # 创建并启动视频捕获线程
        capture_thread = threading.Thread(target=video_capture_thread)
        capture_thread.daemon = True
        capture_thread.start()

        # 创建并启动视频发送线程
        sending_thread = threading.Thread(target=video_sending_thread)
        sending_thread.daemon = True
        sending_thread.start()

        # 创建并启动电池模拟线程
        battery_thread = threading.Thread(target=battery_simulation_thread)
        battery_thread.daemon = True
        battery_thread.start()

        # 创建并启动串口监听线程
        serial_thread = threading.Thread(target=serial_listen_thread)
        serial_thread.daemon = True
        serial_thread.start()

        # 设置初始位置 (北京天安门广场坐标)
        set_position(39.9042, 116.4074)

        # 主线程保持运行
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logger.info("接收到终止信号，正在关闭...")
    except Exception as e:
        logger.error(f"主线程错误: {e}")
    finally:
        running = False

        # 关闭WebSocket连接
        if ws:
            ws.close()

        # 关闭串口连接
        if serial_port and serial_port.is_open:
            serial_port.close()
            logger.info("串口连接已关闭")

        logger.info("机器人客户端已关闭")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"程序异常退出: {e}")