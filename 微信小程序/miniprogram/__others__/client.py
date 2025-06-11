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
import random
import math
from queue import Queue, Empty

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("robot_client")

# WebSocket配置
SERVER_URL = "ws://101.201.150.96:1234/ws/robot/robot_123"
CAMERA_ID = 0  # 摄像头ID
INITIAL_PRESET = "medium"  # 初始质量预设

# 质量预设配置
QUALITY_PRESETS = {
    "ultra_high": {
        "resolution": (640, 480),
        "fps": 30,
        "bitrate": 1200,
        "quality": 75  # 稍微降低质量补偿帧率提升
    },
    "high_fps": {
        "resolution": (480, 360),
        "fps": 25,
        "bitrate": 800,
        "quality": 70
    },
    "medium": {
        "resolution": (480, 360),
        "fps": 15,
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

# 模拟工作区域路径点（经纬度坐标）
WORK_AREA_WAYPOINTS = [
    {"longitude": 116.3000, "latitude": 39.9000, "name": "苹果园区入口"},
    {"longitude": 116.3010, "latitude": 39.9015, "name": "A区域起点"},
    {"longitude": 116.3030, "latitude": 39.9020, "name": "A区域终点"},
    {"longitude": 116.3040, "latitude": 39.9010, "name": "B区域起点"},
    {"longitude": 116.3050, "latitude": 39.9025, "name": "B区域终点"},
    {"longitude": 116.3020, "latitude": 39.9030, "name": "C区域起点"},
    {"longitude": 116.3000, "latitude": 39.9040, "name": "C区域终点"},
    {"longitude": 116.2990, "latitude": 39.9020, "name": "充电区"}
]

# 模拟采摘效率
HARVEST_EFFICIENCY = {
    "morning": 1.2,  # 早上效率高
    "noon": 1.0,  # 中午正常
    "afternoon": 0.9,  # 下午略低
    "evening": 0.8  # 傍晚更低
}

# 机器人状态
robot_status = {
    "battery_level": 85,
    "battery_charging": False,
    "position": {
        "x": 0,
        "y": 0,
        "longitude": WORK_AREA_WAYPOINTS[0]["longitude"],
        "latitude": WORK_AREA_WAYPOINTS[0]["latitude"],
        "location_name": WORK_AREA_WAYPOINTS[0]["name"]
    },
    "destination_index": 0,  # 当前目标路径点索引
    "moving_to_waypoint": False,  # 是否正在移动到路径点
    "harvested_count": 0,
    "today_harvested": 0,
    "working_area": 0.0,
    "working_hours": 0.0,
    "harvest_accuracy": 95.0,
    "harvest_efficiency": 1.0,  # 当前采摘效率系数
    "total_harvested": 0,
    "cpu_usage": 0,
    "signal_strength": 70,
    "upload_bandwidth": 1000,
    "frames_sent": 0,
    "bytes_sent": 0,
    "last_bandwidth_check": time.time(),
    "last_bytes_sent": 0,
    "last_harvest_update": time.time(),
    "harvesting": False,
    "start_time": time.time(),
    "work_mode": "standby",  # standby, harvesting, moving, charging
    "route_history": [],  # 记录移动历史
    "current_day": time.localtime().tm_mday  # 记录当前日期以便识别新的一天
}

# 模拟特殊事件
SPECIAL_EVENTS = [
    {"name": "丰收", "probability": 0.05, "harvest_multiplier": 2.0, "duration": 60},
    {"name": "采摘困难", "probability": 0.03, "harvest_multiplier": 0.5, "duration": 45},
    {"name": "信号干扰", "probability": 0.02, "signal_drop": 30, "duration": 30}
]

# 当前活跃的特殊事件
active_events = []


# 处理收到的控制命令
def handle_command(command_data):
    global robot_status

    cmd = command_data.get("command")
    params = command_data.get("params", {})

    logger.info(f"收到命令: {cmd}, 参数: {params}")

    # 根据不同命令执行不同操作
    if cmd == "forward":
        logger.info("机器人前进")
        robot_status["position"]["x"] += 1
        move_to_next_waypoint()
        robot_status["work_mode"] = "moving"
    elif cmd == "backward":
        logger.info("机器人后退")
        robot_status["position"]["x"] -= 1
        robot_status["work_mode"] = "moving"
        # 后退时移动到上一个路径点
        if robot_status["destination_index"] > 0:
            robot_status["destination_index"] -= 1
            robot_status["moving_to_waypoint"] = True
    elif cmd == "left":
        logger.info("机器人左转")
        robot_status["position"]["y"] -= 1
        robot_status["work_mode"] = "moving"
    elif cmd == "right":
        logger.info("机器人右转")
        robot_status["position"]["y"] += 1
        robot_status["work_mode"] = "moving"
    elif cmd == "stop":
        logger.info("机器人停止")
        robot_status["moving_to_waypoint"] = False
        robot_status["work_mode"] = "standby"
    elif cmd == "startHarvest" or cmd == "start_harvest":
        logger.info("开始采摘")
        robot_status["harvesting"] = True
        robot_status["work_mode"] = "harvesting"
        # 记录开始工作时间，如果之前不是在工作状态的话
        if robot_status["start_time"] == 0:
            robot_status["start_time"] = time.time()
    elif cmd == "stopHarvest" or cmd == "pause_harvest":
        logger.info("暂停采摘")
        robot_status["harvesting"] = False
        robot_status["work_mode"] = "standby"
    elif cmd == "emergencyStop" or cmd == "emergency_stop":
        logger.info("紧急停止所有操作")
        robot_status["harvesting"] = False
        robot_status["moving_to_waypoint"] = False
        robot_status["work_mode"] = "standby"
    elif cmd == "startCharging":
        logger.info("开始充电")
        # 移动到充电区
        robot_status["destination_index"] = 7  # 充电区索引
        robot_status["moving_to_waypoint"] = True
        robot_status["work_mode"] = "moving"
        # 到达后会自动开始充电
    elif cmd == "stopCharging":
        logger.info("停止充电")
        robot_status["battery_charging"] = False
        robot_status["work_mode"] = "standby"
    elif cmd == "moveToWaypoint":
        # 移动到特定路径点
        waypoint_index = params.get("index", 0)
        if 0 <= waypoint_index < len(WORK_AREA_WAYPOINTS):
            robot_status["destination_index"] = waypoint_index
            robot_status["moving_to_waypoint"] = True
            robot_status["work_mode"] = "moving"
            logger.info(f"移动到路径点: {WORK_AREA_WAYPOINTS[waypoint_index]['name']}")


# 移动到下一个路径点
def move_to_next_waypoint():
    if robot_status["destination_index"] < len(WORK_AREA_WAYPOINTS) - 1:
        robot_status["destination_index"] += 1
        robot_status["moving_to_waypoint"] = True
        logger.info(f"移动到下一个路径点: {WORK_AREA_WAYPOINTS[robot_status['destination_index']]['name']}")
    else:
        # 已经到达最后一个点，循环回到第一个点
        robot_status["destination_index"] = 0
        robot_status["moving_to_waypoint"] = True
        logger.info(f"循环回到第一个路径点: {WORK_AREA_WAYPOINTS[0]['name']}")


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


# 获取当前时段的采摘效率
def get_current_harvest_efficiency():
    current_hour = time.localtime().tm_hour

    if 6 <= current_hour < 11:
        return HARVEST_EFFICIENCY["morning"]
    elif 11 <= current_hour < 14:
        return HARVEST_EFFICIENCY["noon"]
    elif 14 <= current_hour < 18:
        return HARVEST_EFFICIENCY["afternoon"]
    else:
        return HARVEST_EFFICIENCY["evening"]


# 生成特殊事件
def generate_special_events():
    global active_events

    # 清理过期的事件
    current_time = time.time()
    active_events = [event for event in active_events if current_time < event["end_time"]]

    # 检查是否触发新事件
    for event_template in SPECIAL_EVENTS:
        if random.random() < event_template["probability"] / 60:  # 每分钟的概率
            # 创建新事件
            new_event = event_template.copy()
            new_event["start_time"] = current_time
            new_event["end_time"] = current_time + event_template["duration"]
            active_events.append(new_event)

            logger.info(f"特殊事件触发: {new_event['name']}, 持续{new_event['duration']}秒")

            # 立即应用事件效果
            if "signal_drop" in new_event:
                robot_status["signal_strength"] = max(10, robot_status["signal_strength"] - new_event["signal_drop"])


# 检查日期变更，重置每日数据
def check_day_change():
    current_day = time.localtime().tm_mday
    if current_day != robot_status["current_day"]:
        logger.info(f"日期变更，重置每日数据。昨日总采摘量: {robot_status['today_harvested']}")
        # 保存昨日数据到历史记录

        # 重置每日数据
        robot_status["today_harvested"] = 0
        robot_status["working_area"] = 0.0
        robot_status["working_hours"] = 0.0
        robot_status["current_day"] = current_day
        robot_status["start_time"] = time.time()  # 重置开始时间


# 计算距离（经纬度坐标之间）
def calculate_distance(lon1, lat1, lon2, lat2):
    # 简化版计算，实际应用中应使用更精确的算法
    R = 6371000  # 地球半径（米）
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat / 2) * math.sin(dLat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dLon / 2) * math.sin(dLon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance  # 返回米


# 更新机器人位置
def update_robot_position():
    if not robot_status["moving_to_waypoint"]:
        return

    # 获取目标路径点
    destination = WORK_AREA_WAYPOINTS[robot_status["destination_index"]]
    current_lon = robot_status["position"]["longitude"]
    current_lat = robot_status["position"]["latitude"]
    dest_lon = destination["longitude"]
    dest_lat = destination["latitude"]

    # 计算距离
    distance = calculate_distance(current_lon, current_lat, dest_lon, dest_lat)

    # 如果距离很小，认为已到达
    if distance < 10:  # 10米以内认为已到达
        robot_status["position"]["longitude"] = dest_lon
        robot_status["position"]["latitude"] = dest_lat
        robot_status["position"]["location_name"] = destination["name"]
        robot_status["moving_to_waypoint"] = False

        # 到达充电区开始充电
        if robot_status["destination_index"] == 7:  # 充电区索引
            robot_status["battery_charging"] = True
            robot_status["work_mode"] = "charging"
            logger.info("已到达充电区，开始充电")
        else:
            robot_status["work_mode"] = "standby"
            logger.info(f"已到达路径点: {destination['name']}")

        # 添加到路径历史
        timestamp = time.strftime("%H:%M", time.localtime())
        robot_status["route_history"].append({
            "time": timestamp,
            "location": destination["name"]
        })

        return

    # 移动速度（米/秒）
    speed = 1.0 if robot_status["harvesting"] else 2.0

    # 计算本次更新需要移动的比例
    move_ratio = min(1.0, speed / distance if distance > 0 else 1.0)

    # 计算新位置
    new_lon = current_lon + (dest_lon - current_lon) * move_ratio
    new_lat = current_lat + (dest_lat - current_lat) * move_ratio

    # 更新位置
    robot_status["position"]["longitude"] = new_lon
    robot_status["position"]["latitude"] = new_lat

    # 更新工作面积（如果是在工作状态）
    if robot_status["harvesting"]:
        # 移动距离转换为亩（假设每100米对应0.01亩）
        area_increase = speed * move_ratio * 0.0001
        robot_status["working_area"] += area_increase


# 更新采摘数据
def update_harvest_data():
    global robot_status, active_events

    current_time = time.time()
    time_diff = current_time - robot_status["last_harvest_update"]
    robot_status["last_harvest_update"] = current_time

    # 检查日期变更
    check_day_change()

    # 更新特殊事件
    generate_special_events()

    # 更新移动位置
    update_robot_position()

    # 更新采摘效率
    robot_status["harvest_efficiency"] = get_current_harvest_efficiency()

    # 应用特殊事件效果
    harvest_multiplier = 1.0
    for event in active_events:
        if "harvest_multiplier" in event:
            harvest_multiplier *= event["harvest_multiplier"]

    if robot_status["harvesting"]:
        # 基础采摘速率（个/秒）
        base_rate = 0.2

        # 计算实际采摘速率
        actual_rate = base_rate * robot_status["harvest_efficiency"] * harvest_multiplier

        # 计算本次更新采摘的数量
        new_harvested = int(actual_rate * time_diff)

        # 添加随机性
        new_harvested += random.randint(-1, 1)
        new_harvested = max(0, new_harvested)

        if new_harvested > 0:
            robot_status["today_harvested"] += new_harvested
            robot_status["total_harvested"] += new_harvested
            robot_status["harvested_count"] += new_harvested

        # 更新工作时间
        robot_status["working_hours"] = round((current_time - robot_status["start_time"]) / 3600.0, 2)

        # 计算疲劳因素对准确率的影响（工作时间越长，准确率可能越低）
        hours_worked = robot_status["working_hours"]
        fatigue_factor = max(0.9, 1.0 - (hours_worked * 0.01))  # 每小时降低1%，最多降低10%

        # 计算电量因素对准确率的影响（电量低时准确率降低）
        battery_factor = max(0.9, robot_status["battery_level"] / 100)  # 电量对应的系数，最低0.9

        # 加入随机波动
        random_factor = random.uniform(0.99, 1.01)

        # 计算最终准确率
        target_accuracy = 98.0 * fatigue_factor * battery_factor * random_factor

        # 平滑过渡到目标准确率
        current_accuracy = robot_status["harvest_accuracy"]
        robot_status["harvest_accuracy"] = round(current_accuracy * 0.9 + target_accuracy * 0.1, 1)
        robot_status["harvest_accuracy"] = max(90.0, min(99.5, robot_status["harvest_accuracy"]))


# 状态更新线程
def status_update_thread():
    last_status_time = 0
    status_interval = 5  # 每5秒向WebSocket服务器发送一次

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

            # 更新采摘数据
            update_harvest_data()

            # 定期发送状态更新到WebSocket服务器
            if current_time - last_status_time >= status_interval:
                send_status_update()
                last_status_time = current_time

        except Exception as e:
            logger.error(f"状态更新错误: {e}")

        time.sleep(1)


def send_status_update():
    if ws and connected:
        try:
            # 准备地理位置和路径信息
            location_info = {
                "name": robot_status["position"]["location_name"],
                "longitude": robot_status["position"]["longitude"],
                "latitude": robot_status["position"]["latitude"],
            }

            # 准备工作状态信息
            work_status = {
                "mode": robot_status["work_mode"],
                "harvesting": robot_status["harvesting"],
                "moving": robot_status["moving_to_waypoint"],
                "charging": robot_status["battery_charging"],
                "efficiency": robot_status["harvest_efficiency"]
            }

            # 检查活跃的特殊事件
            active_event_names = [event["name"] for event in active_events]

            # 构建状态消息
            status_message = {
                "type": "status_update",
                "data": {
                    "battery_level": robot_status["battery_level"],
                    "battery_charging": robot_status["battery_charging"],
                    "cpu_usage": robot_status["cpu_usage"],
                    "signal_strength": robot_status["signal_strength"],
                    "upload_bandwidth": robot_status["upload_bandwidth"],
                    "frames_sent": robot_status["frames_sent"],
                    "bytes_sent": robot_status["bytes_sent"],
                    "current_preset": current_preset,

                    # 采摘统计数据
                    "today_harvested": robot_status["today_harvested"],
                    "working_area": robot_status["working_area"],
                    "working_hours": robot_status["working_hours"],
                    "harvest_accuracy": robot_status["harvest_accuracy"],
                    "total_harvested": robot_status["total_harvested"],

                    # 位置信息
                    "longitude": robot_status["position"]["longitude"],
                    "latitude": robot_status["position"]["latitude"],
                    "location": location_info,

                    # 工作状态
                    "work_status": work_status,

                    # 路径历史
                    "route_history": robot_status["route_history"],

                    # 特殊事件
                    "active_events": active_event_names
                }
            }

            ws.send(json.dumps(status_message))
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
            # 模拟视频数据
            use_mock_video = True
            logger.info("使用模拟视频数据")
        else:
            use_mock_video = False
            # 设置摄像头参数
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            logger.info(f"摄像头已打开: {CAMERA_ID}, 分辨率: {width}x{height}")

        while running:
            # 读取视频帧或生成模拟帧
            if use_mock_video:
                # 创建模拟视频帧
                frame = np.zeros((height, width, 3), dtype=np.uint8)

                # 根据工作模式设置不同的底色
                if robot_status["work_mode"] == "harvesting":
                    frame[:, :, 1] = 40  # 绿色底色表示采摘
                elif robot_status["work_mode"] == "moving":
                    frame[:, :, 2] = 40  # 红色底色表示移动
                elif robot_status["work_mode"] == "charging":
                    frame[:, :, 0] = 40  # 蓝色底色表示充电

                # 在帧上添加文本
                status_text = f"状态: {robot_status['work_mode']}"
                if robot_status["harvesting"]:
                    status_text += " (采摘中)"

                cv2.putText(frame, status_text,
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.putText(frame, f"采摘量: {robot_status['today_harvested']}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.putText(frame, f"电量: {int(robot_status['battery_level'])}%",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.putText(frame, f"位置: {robot_status['position']['location_name']}",
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # 添加特殊事件显示
                if active_events:
                    event_text = f"事件: {active_events[0]['name']}"
                    cv2.putText(frame, event_text,
                                (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                # 添加时间戳
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, timestamp, (width - 150, height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                ret = True
            else:
                # 读取真实视频帧
                ret, frame = cap.read()
                if not ret:
                    logger.warning("无法获取视频帧")
                    time.sleep(0.1)
                    continue

            # 如果分辨率设置发生了变化，需要重新初始化
            current_width, current_height = current_config["resolution"]
            if (width != current_width or height != current_height):
                logger.info(f"分辨率设置已变更，重新初始化: {current_width}x{current_height}")

                if not use_mock_video:
                    # 关闭旧的摄像头
                    cap.release()
                    time.sleep(0.5)  # 给摄像头一些恢复时间

                # 更新当前分辨率记录
                width, height = current_width, current_height

                if not use_mock_video:
                    # 重新打开摄像头
                    cap = cv2.VideoCapture(CAMERA_ID)
                    if not cap.isOpened():
                        logger.error(f"无法重新打开摄像头 {CAMERA_ID}")
                        use_mock_video = True
                    else:
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
        if 'cap' in locals() and cap is not None:
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

                    # 检查信号强度是否太低
                    signal_too_weak = robot_status["signal_strength"] < 20

                    # 如果信号太弱，模拟画面干扰
                    if signal_too_weak:
                        # 添加噪点
                        noise = np.random.randint(0, 50, frame.shape, dtype=np.uint8)
                        frame = cv2.addWeighted(frame, 0.8, noise, 0.2, 0)

                        # 随机冻结某些帧
                        if random.random() < 0.3:
                            # 跳过这一帧，模拟卡顿
                            continue

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


# 电池和信号模拟线程
def resource_simulation_thread():
    while running:
        try:
            # 电池模拟
            if robot_status["battery_charging"]:
                # 充电模式
                battery_increase = 0.05  # 每秒增加0.05%
                robot_status["battery_level"] = min(100, robot_status["battery_level"] + battery_increase)
            else:
                # 放电模式，根据工作状态消耗不同
                battery_decrease = 0

                if robot_status["work_mode"] == "harvesting":
                    battery_decrease = 0.03  # 采摘模式每秒减少0.03%
                elif robot_status["work_mode"] == "moving":
                    battery_decrease = 0.04  # 移动模式每秒减少0.04%
                else:
                    battery_decrease = 0.01  # 待机模式每秒减少0.01%

                # 电池电量越低，消耗越慢（模拟电量保护）
                if robot_status["battery_level"] < 20:
                    battery_decrease *= 0.7

                robot_status["battery_level"] = max(0, robot_status["battery_level"] - battery_decrease)

            # 当电量过低时自动进入省电模式
            if robot_status["battery_level"] < 10 and not robot_status["battery_charging"]:
                if robot_status["harvesting"]:
                    logger.warning("电量过低，自动停止采摘")
                    robot_status["harvesting"] = False

                # 如果不在充电区，自动前往充电区
                if robot_status["destination_index"] != 7:  # 充电区索引
                    logger.warning("电量过低，自动前往充电区")
                    robot_status["destination_index"] = 7
                    robot_status["moving_to_waypoint"] = True
                    robot_status["work_mode"] = "moving"

            # 信号强度模拟
            # 基础信号强度根据位置变化（某些区域信号更好）
            base_signal = 70

            # 计算与各个路径点的距离，找到最近的点
            min_distance = float('inf')
            for waypoint in WORK_AREA_WAYPOINTS:
                distance = calculate_distance(
                    robot_status["position"]["longitude"],
                    robot_status["position"]["latitude"],
                    waypoint["longitude"],
                    waypoint["latitude"]
                )
                min_distance = min(min_distance, distance)

            # 根据距离调整信号强度（距离越远信号越弱）
            distance_factor = max(0.7, 1.0 - (min_distance / 1000))  # 最多减弱30%

            # 添加随机波动
            random_factor = random.uniform(0.9, 1.1)

            # 计算最终信号强度
            target_signal = base_signal * distance_factor * random_factor

            # 平滑过渡到目标信号强度
            current_signal = robot_status["signal_strength"]
            robot_status["signal_strength"] = round(current_signal * 0.9 + target_signal * 0.1)
            robot_status["signal_strength"] = max(10, min(100, robot_status["signal_strength"]))

            time.sleep(1)

        except Exception as e:
            logger.error(f"资源模拟错误: {e}")


# 主函数
def main():
    global running

    try:
        logger.info("启动机器人客户端...")

        # 连接到WebSocket服务器
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

        # 创建并启动资源模拟线程
        resource_thread = threading.Thread(target=resource_simulation_thread)
        resource_thread.daemon = True
        resource_thread.start()

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

        logger.info("机器人客户端已关闭")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"程序异常退出: {e}")