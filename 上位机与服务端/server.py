# server.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import json
import asyncio
import logging
import datetime
import threading
import time
import random

# 导入华为IoT SDK
from iot_device_sdk_python.client.client_conf import ClientConf
from iot_device_sdk_python.client.connect_auth_info import ConnectAuthInfo
from iot_device_sdk_python.client.request.service_property import ServiceProperty
from iot_device_sdk_python.service.abstract_service import AbstractService
from iot_device_sdk_python.service.property import Property
from iot_device_sdk_python.client.request.command_response import CommandRsp
from iot_device_sdk_python.iot_device import IotDevice
from iot_device_sdk_python.client.listener.command_listener import CommandListener
from iot_device_sdk_python.client.listener.property_listener import PropertyListener

# 导入自适应视频管理器
from adaptive_video_manager import AdaptiveVideoManager

app = FastAPI()

# 允许跨域请求
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 日志配置
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("server")

# 华为IoT平台配置
IOT_SERVER_URI = "78c7a8f557.st1.iotda-device.cn-east-3.myhuaweicloud.com"
IOT_PORT = 1883
IOT_DEVICE_ID = "6804c32cfde7ae37459990d1_my_picking_robot"
IOT_SECRET = "71fa98bcd2e07ddd8a4051f2e8c85304"

# 全局变量
robots = {}  # 存储所有连接的机器人
clients = {}  # 存储所有连接的微信客户端
robot_to_clients = {}  # 机器人ID到客户端ID的映射
client_to_robot = {}  # 客户端ID到机器人ID的映射
iot_device = None  # IoT设备实例
picking_service = None  # 采摘机器人服务实例
lock = asyncio.Lock()  # 异步锁，用于访问共享资源
adaptive_video_manager = None  # 自适应视频管理器实例

# 历史数据存储
daily_statistics = {}  # 格式: {robot_id: {date: {data}}}
monthly_records = {}  # 格式: {robot_id: {year-month: [记录列表]}}

# AI聊天相关 - 存储待处理的AI请求
pending_ai_requests = {}  # 格式: {request_id: {client_id, timestamp, ...}}


# 命令监听器
class ServerCommandListener(CommandListener):
    def on_command(self, request_id, service_id, command_name, paras):
        logger.info(f"收到命令: request_id={request_id}, service_id={service_id}, command={command_name}, 参数={paras}")

        # 查找所有与此设备关联的机器人
        robot_ids = []
        for robot_id in robots:
            robot_ids.append(robot_id)

        if not robot_ids:
            logger.warning("没有连接的机器人可以执行命令")
            return

        # 选择第一个机器人执行命令（实际应用中可能需要更复杂的路由逻辑）
        robot_id = robot_ids[0]

        # 创建命令并发送给机器人
        asyncio.create_task(forward_command_to_robot(robot_id, command_name, paras))

        # 响应命令
        command_rsp = CommandRsp()
        command_rsp.result_code = CommandRsp.success_code()
        command_rsp.response_name = command_name
        command_rsp.paras = {"result": "Command forwarded to robot"}
        iot_device.get_client().respond_command(request_id, command_rsp)


# 属性监听器
class ServerPropertyListener(PropertyListener):
    def on_property_set(self, request_id, services):
        logger.info(f"收到属性设置请求: request_id={request_id}")

        for service_property in services:
            logger.info(f"服务ID: {service_property.service_id}")
            for prop_name, prop_value in service_property.properties.items():
                logger.info(f"属性: {prop_name}={prop_value}")

                # 更新服务属性
                if picking_service and hasattr(picking_service, f"set_{prop_name}"):
                    setter = getattr(picking_service, f"set_{prop_name}")
                    setter(prop_value)

        # 响应属性设置请求
        iot_device.get_client().respond_properties_set(request_id, 0)

    def on_property_get(self, request_id, service_id):
        logger.info(f"收到属性读取请求: request_id={request_id}, service_id={service_id}")

        # 响应属性读取请求
        if picking_service:
            service_property = ServiceProperty()
            service_property.service_id = "pickingRobot"

            # 收集所有属性
            properties = {}
            for prop_name in picking_service._readable_prop2field:
                if hasattr(picking_service, f"get_{picking_service._readable_prop2field[prop_name]}"):
                    getter = getattr(picking_service, f"get_{picking_service._readable_prop2field[prop_name]}")
                    properties[prop_name] = getter()

            service_property.properties = properties
            iot_device.get_client().respond_properties_get(request_id, [service_property])


# 采摘机器人服务类
class PickingRobotService(AbstractService):
    """
    采摘机器人服务，管理设备属性和命令
    """

    def __init__(self):
        super().__init__()

        # 定义设备属性
        self.today_harvested = Property(val=0, field_name="today_harvested", prop_name="todayHarvested", writeable=True)
        self.working_area = Property(val=float(0.0), field_name="working_area", prop_name="workingArea", writeable=True)
        self.working_hours = Property(val=float(0.0), field_name="working_hours", prop_name="workingHours",
                                      writeable=True)
        self.harvest_accuracy = Property(val=float(95.0), field_name="harvest_accuracy", prop_name="harvestAccuracy",
                                         writeable=False)
        self.longitude = Property(val=float(116.3), field_name="longitude", prop_name="longitude", writeable=True)
        self.latitude = Property(val=float(39.9), field_name="latitude", prop_name="latitude", writeable=True)
        self.battery_level = Property(val=85, field_name="battery_level", prop_name="batteryLevel", writeable=True)
        self.total_harvested = Property(val=0, field_name="total_harvested", prop_name="totalHarvested", writeable=True)

        # 定义命令名称与方法名称的映射关系
        self.command2method = {
            "startHarvest": "start_harvest",
            "stopHarvest": "stop_harvest",
            "emergencyStop": "emergency_stop"
        }

        # 设置可读写属性和命令映射
        self._set_properties()
        self._command2method = self.command2method

        # 运行状态
        self.harvesting = False

    def _set_properties(self):
        """设置属性可读写状态"""
        props = [self.today_harvested, self.working_area, self.working_hours,
                 self.harvest_accuracy, self.longitude, self.latitude,
                 self.battery_level, self.total_harvested]

        for prop in props:
            self._readable_prop2field[prop.prop_name] = prop.field_name
            if prop.writeable:
                self._writeable_prop2field[prop.prop_name] = prop.field_name

    # 命令处理方法
    def start_harvest(self, paras: dict):
        logger.info("执行开始采摘命令")
        self.harvesting = True
        command_rsp = CommandRsp()
        command_rsp.result_code = CommandRsp.success_code()
        return command_rsp

    def stop_harvest(self, paras: dict):
        logger.info("执行停止采摘命令")
        self.harvesting = False
        command_rsp = CommandRsp()
        command_rsp.result_code = CommandRsp.success_code()
        return command_rsp

    def emergency_stop(self, paras: dict):
        logger.info("执行紧急停止命令")
        self.harvesting = False
        command_rsp = CommandRsp()
        command_rsp.result_code = CommandRsp.success_code()
        return command_rsp

    # 属性getter/setter方法
    def get_today_harvested(self):
        return self.today_harvested.val

    def set_today_harvested(self, value):
        self.today_harvested.val = value

    def get_working_area(self):
        return self.working_area.val

    def set_working_area(self, value):
        self.working_area.val = value

    def get_working_hours(self):
        return self.working_hours.val

    def set_working_hours(self, value):
        self.working_hours.val = value

    def get_harvest_accuracy(self):
        return self.harvest_accuracy.val

    def set_harvest_accuracy(self, value):
        # 只读属性，实际不会被调用
        pass

    def get_longitude(self):
        return self.longitude.val

    def set_longitude(self, value):
        self.longitude.val = value

    def get_latitude(self):
        return self.latitude.val

    def set_latitude(self, value):
        self.latitude.val = value

    def get_battery_level(self):
        return self.battery_level.val

    def set_battery_level(self, value):
        self.battery_level.val = value

    def get_total_harvested(self):
        return self.total_harvested.val

    def set_total_harvested(self, value):
        self.total_harvested.val = value


# 连接到IoT平台
def connect_to_iot_platform():
    global iot_device, picking_service

    logger.info("正在连接到华为IoT平台...")

    # 创建连接认证信息
    connect_auth_info = ConnectAuthInfo()
    connect_auth_info.server_uri = IOT_SERVER_URI
    connect_auth_info.port = IOT_PORT
    connect_auth_info.id = IOT_DEVICE_ID
    connect_auth_info.secret = IOT_SECRET
    connect_auth_info.bs_mode = ConnectAuthInfo.BS_MODE_DIRECT_CONNECT

    # 创建客户端配置和设备实例
    client_conf = ClientConf(connect_auth_info)
    iot_device = IotDevice(client_conf)

    # 添加命令监听器
    iot_device.get_client().set_command_listener(ServerCommandListener())

    # 添加属性监听器
    iot_device.get_client().set_properties_listener(ServerPropertyListener())

    # 添加机器人服务
    picking_service = PickingRobotService()
    iot_device.add_service("pickingRobot", picking_service)

    # 连接到平台
    result = iot_device.connect()
    if result != 0:
        logger.error("连接华为IoT平台失败")
        return False

    logger.info("已成功连接到华为IoT平台")
    return True


# 上报数据到IoT平台
def report_status_to_iot(robot_data):
    global iot_device, picking_service

    if iot_device is None or picking_service is None:
        logger.warning("IoT设备或服务未初始化，无法上报数据")
        return

    try:
        # 更新服务中的属性
        picking_service.set_today_harvested(robot_data.get("today_harvested", 0))
        picking_service.set_working_area(robot_data.get("working_area", 0.0))
        picking_service.set_working_hours(robot_data.get("working_hours", 0.0))

        # 设置经纬度
        picking_service.set_longitude(robot_data.get("longitude", 116.3))
        picking_service.set_latitude(robot_data.get("latitude", 39.9))

        # 设置电池和采摘总量
        picking_service.set_battery_level(robot_data.get("battery_level", 85))
        picking_service.set_total_harvested(robot_data.get("total_harvested", 0))

        # 创建服务属性对象并上报
        service_property = ServiceProperty()
        service_property.service_id = "pickingRobot"
        service_property.properties = {
            "todayHarvested": picking_service.get_today_harvested(),
            "workingArea": picking_service.get_working_area(),
            "workingHours": picking_service.get_working_hours(),
            "harvestAccuracy": picking_service.get_harvest_accuracy(),
            "longitude": picking_service.get_longitude(),
            "latitude": picking_service.get_latitude(),
            "batteryLevel": picking_service.get_battery_level(),
            "totalHarvested": picking_service.get_total_harvested()
        }

        # 上报属性
        iot_device.get_client().report_properties([service_property])
        logger.debug("已上报设备状态到IoT平台")
    except Exception as e:
        logger.error(f"上报状态到IoT平台出错: {e}")


# 微信客户端WebSocket连接处理
@app.websocket("/ws/wechat/{client_id}")
async def wechat_websocket_endpoint(websocket: WebSocket, client_id: str):
    await websocket.accept()
    logger.info(f"微信客户端 {client_id} 已连接")

    # 注册客户端
    async with lock:
        clients[client_id] = {
            "websocket": websocket,
            "last_active": datetime.datetime.now(),
            "connection_id": f"{client_id}_{datetime.datetime.now().timestamp()}"  # 唯一连接ID
        }

    # 注册到自适应视频管理器
    adaptive_video_manager.register_client(client_id, websocket)

    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                message_type = message.get("type", "")

                # 更新客户端活跃时间
                async with lock:
                    if client_id in clients:
                        clients[client_id]["last_active"] = datetime.datetime.now()

                if message_type == "ping":
                    # 处理ping消息
                    timestamp = message.get("timestamp", 0)
                    robot_id = None
                    robot_online = False

                    # 查找连接的机器人
                    async with lock:
                        if client_id in client_to_robot:
                            robot_id = client_to_robot[client_id]
                            if robot_id in robots:
                                robot_online = True

                    # 发送pong响应
                    await websocket.send_json({
                        "type": "pong",
                        "timestamp": timestamp,
                        "robot_id": robot_id,
                        "robot_online": robot_online
                    })

                elif message_type == "init":
                    # 客户端初始化
                    robot_id = message.get("robot_id")
                    if robot_id:
                        # 关联客户端和机器人
                        success = await connect_client_to_robot(client_id, robot_id)

                        # 在自适应视频管理器中连接客户端和机器人
                        if success:
                            adaptive_video_manager.connect_client_to_robot(client_id, robot_id)

                        # 获取机器人在线状态
                        robot_online = robot_id in robots

                        # 如果机器人在线，发送初始状态
                        if robot_online:
                            robot_data = robots[robot_id].get("data", {})
                            await websocket.send_json({
                                "type": "statistics_update",
                                "data": robot_data
                            })

                        # 立即告知客户端机器人连接状态
                        await websocket.send_json({
                            "type": "robot_connection_status",
                            "robot_id": robot_id,
                            "connected": robot_online,
                            "timestamp": int(time.time() * 1000)
                        })

                        await websocket.send_json({
                            "type": "init_response",
                            "success": robot_online,
                            "message": "连接成功" if robot_online else "机器人不在线",
                            "robot_online": robot_online
                        })

                elif message_type == "command":
                    # 转发控制命令
                    robot_id = message.get("robot_id")
                    command = message.get("command")
                    params = message.get("params", {})

                    # 检查是否是模式切换命令
                    if command in ["switch_to_auto", "switch_to_manual"]:
                        # 调用专门处理模式切换的函数
                        asyncio.create_task(forward_mode_command(robot_id, command))
                        # 向客户端发送确认
                        await websocket.send_json({
                            "type": "command_result",
                            "result": "success",
                            "message": "模式切换命令已发送"
                        })
                    elif robot_id in robots:
                        # 转发其他命令
                        await forward_command_to_robot(robot_id, command, params)
                        # 向客户端发送确认
                        await websocket.send_json({
                            "type": "command_result",
                            "result": "success",
                            "message": "命令已发送"
                        })
                    else:
                        await websocket.send_json({
                            "type": "command_result",
                            "result": "error",
                            "message": "机器人不在线"
                        })

                elif message_type == "get_statistics":
                    # 处理统计数据请求
                    robot_id = message.get("robot_id")
                    if robot_id in robots and "data" in robots[robot_id]:
                        await websocket.send_json({
                            "type": "statistics_update",
                            "data": robots[robot_id]["data"]
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "未找到机器人数据"
                        })

                elif message_type == "client_network_status":
                    # 处理客户端网络状态更新
                    status_data = message.get("status", {})

                    # 更新自适应视频管理器中的客户端状态
                    adaptive_video_manager.update_client_status(client_id, status_data)

                    logger.debug(f"已更新客户端 {client_id} 的网络状态: {status_data}")

                elif message_type == "client_quality_request":
                    # 处理客户端质量调整请求
                    robot_id = message.get("robot_id")
                    preset = message.get("preset")

                    if robot_id in robots:
                        # 在自适应视频管理器中更新客户端偏好
                        clients[client_id]["preferred_preset"] = preset

                        # 通知客户端请求已收到
                        await websocket.send_json({
                            "type": "quality_request_received",
                            "preset": preset
                        })

                        logger.info(f"客户端 {client_id} 请求调整质量为 {preset}")
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "机器人不在线"
                        })

                elif message_type == "get_statistics_by_date":
                    # 处理特定日期的统计数据请求
                    robot_id = message.get("robot_id")
                    date = message.get("date")

                    if robot_id and date:
                        # 查找该日期的数据
                        robot_daily_stats = daily_statistics.get(robot_id, {})
                        date_stats = robot_daily_stats.get(date)

                        if date_stats:
                            # 发送历史数据
                            await websocket.send_json({
                                "type": "statistics_update",
                                "data": date_stats
                            })
                        else:
                            # 没有找到历史数据
                            await websocket.send_json({
                                "type": "error",
                                "message": f"未找到日期 {date} 的统计数据"
                            })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "请求参数不完整"
                        })

                elif message_type == "get_history_records":
                    # 处理历史记录请求
                    robot_id = message.get("robot_id")
                    month = message.get("month")
                    filter_type = message.get("filter", "day")

                    if robot_id and month:
                        # 获取该月的历史记录
                        robot_monthly_records = monthly_records.get(robot_id, {})
                        month_records = robot_monthly_records.get(month, [])

                        # 根据过滤类型处理数据
                        filtered_records = filter_history_records(month_records, filter_type)

                        # 发送历史记录
                        await websocket.send_json({
                            "type": "history_data",
                            "data": {
                                "records": filtered_records
                            }
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "请求参数不完整"
                        })

                elif message_type == "get_robot_status":
                    # 处理获取机器人状态请求
                    robot_id = message.get("robot_id")
                    if robot_id in robots:
                        # 向客户端发送机器人连接状态
                        await websocket.send_json({
                            "type": "robot_connection_status",
                            "robot_id": robot_id,
                            "connected": True,
                            "timestamp": int(time.time() * 1000)
                        })
                    else:
                        await websocket.send_json({
                            "type": "robot_connection_status",
                            "robot_id": robot_id,
                            "connected": False,
                            "timestamp": int(time.time() * 1000)
                        })

                elif message_type == "request_video_stream":
                    # 处理请求重新开始视频流
                    robot_id = message.get("robot_id")
                    if robot_id in robots:
                        # 转发请求到机器人
                        await forward_command_to_robot(robot_id, "request_video_stream", {})
                        await websocket.send_json({
                            "type": "video_stream_request_sent",
                            "robot_id": robot_id,
                            "timestamp": int(time.time() * 1000)
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "机器人不在线，无法请求视频流"
                        })

                elif message_type == "get_position":
                    # 处理位置数据请求
                    robot_id = message.get("robot_id")
                    if robot_id in robots and "data" in robots[robot_id]:
                        robot_data = robots[robot_id]["data"]

                        # 构建位置数据
                        position_data = {
                            "longitude": robot_data.get("longitude", 108.2415),
                            "latitude": robot_data.get("latitude", 34.9385),
                            "location_name": robot_data.get("position", {}).get("location_name", "未知位置"),
                            "work_status": robot_data.get("work_status", {}),
                            "speed": robot_data.get("speed", 0),
                            "route_history": robot_data.get("route_history", [])
                        }

                        await websocket.send_json({
                            "type": "position_update",
                            "data": position_data
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "未找到机器人位置数据"
                        })

                elif message_type == "get_position_history":
                    # 处理位置历史请求
                    robot_id = message.get("robot_id")
                    date = message.get("date")

                    if robot_id in robots and "data" in robots[robot_id]:
                        # 获取当前的采摘点历史
                        robot_data = robots[robot_id]["data"]
                        harvest_points = robot_data.get("harvest_points", [])

                        # 转换为位置历史格式
                        history_positions = []
                        for point in harvest_points:
                            if "position" in point:
                                history_positions.append({
                                    "longitude": point["position"].get("longitude", 108.2415),
                                    "latitude": point["position"].get("latitude", 34.9385),
                                    "time": point.get("time", "")
                                })

                        # 发送位置历史
                        await websocket.send_json({
                            "type": "position_history",
                            "positions": history_positions
                        })
                    else:
                        # 如果没有数据，发送空历史
                        await websocket.send_json({
                            "type": "position_history",
                            "positions": []
                        })

                elif message_type == "get_route_history":
                    # 处理路线历史请求
                    robot_id = message.get("robot_id")
                    date = message.get("date")

                    if robot_id in robots and "data" in robots[robot_id]:
                        # 获取路线历史
                        robot_data = robots[robot_id]["data"]
                        route_history = robot_data.get("route_history", [])

                        # 确保格式正确
                        formatted_routes = []
                        for route in route_history:
                            formatted_routes.append({
                                "time": route.get("time", "--:--"),
                                "location": route.get("location", "未知位置")
                            })

                        # 如果没有路线历史，创建默认的
                        if not formatted_routes:
                            formatted_routes = [{
                                "time": time.strftime("%H:%M"),
                                "location": "等待开始作业"
                            }]

                        # 发送路线历史
                        await websocket.send_json({
                            "type": "route_history",
                            "routes": formatted_routes
                        })
                    else:
                        # 发送默认路线
                        await websocket.send_json({
                            "type": "route_history",
                            "routes": [{
                                "time": time.strftime("%H:%M"),
                                "location": "等待开始作业"
                            }]
                        })

                elif message_type == "ai_chat_request":
                    # 处理AI聊天请求 - 转发给ROS2节点
                    user_message = message.get("message", "").strip()
                    timestamp = message.get("timestamp", int(time.time() * 1000))
                    robot_id = message.get("robot_id", "robot_123")

                    logger.info(f"收到AI聊天请求 - 客户端: {client_id}, 消息: {user_message[:50]}...")

                    try:
                        # 验证消息内容
                        if not user_message:
                            await websocket.send_json({
                                "type": "error",
                                "message": "消息内容不能为空",
                                "context": "ai_chat",
                                "timestamp": timestamp
                            })
                            continue

                        # 生成唯一的请求ID
                        request_id = f"{client_id}_{timestamp}_{random.randint(1000, 9999)}"

                        # 存储待处理的请求
                        async with lock:
                            pending_ai_requests[request_id] = {
                                "client_id": client_id,
                                "timestamp": timestamp,
                                "websocket": websocket,
                                "robot_id": robot_id,
                                "user_message": user_message
                            }

                        # 转发AI请求到机器人节点
                        if robot_id in robots and "websocket" in robots[robot_id]:
                            ai_request_message = {
                                "type": "ai_chat_request",
                                "message": user_message,
                                "client_id": client_id,
                                "timestamp": timestamp,
                                "robot_id": robot_id,
                                "request_id": request_id
                            }

                            await robots[robot_id]["websocket"].send_json(ai_request_message)
                            logger.info(f"AI请求已转发给机器人 {robot_id}")
                        else:
                            # 机器人不在线，发送错误响应
                            async with lock:
                                if request_id in pending_ai_requests:
                                    del pending_ai_requests[request_id]

                            await websocket.send_json({
                                "type": "error",
                                "message": "机器人不在线，AI服务不可用",
                                "context": "ai_chat",
                                "timestamp": timestamp
                            })

                    except Exception as e:
                        logger.error(f"处理AI聊天请求出错: {e}")
                        # 清理待处理请求
                        async with lock:
                            if 'request_id' in locals() and request_id in pending_ai_requests:
                                del pending_ai_requests[request_id]

                        await websocket.send_json({
                            "type": "error",
                            "message": "AI服务暂时不可用，请稍后重试",
                            "context": "ai_chat",
                            "timestamp": timestamp
                        })

            except json.JSONDecodeError:
                logger.error(f"收到无效JSON: {data}")
            except Exception as e:
                logger.error(f"处理微信客户端消息出错: {e}")

    except WebSocketDisconnect:
        logger.info(f"微信客户端 {client_id} 已断开连接")
        # 清理客户端连接
        await handle_client_disconnect(client_id)


# 将模式切换命令转换为机器人可以理解的格式并转发
async def forward_mode_command(robot_id, mode_command):
    """
    将模式切换命令转换为机器人可以理解的格式并转发
    :param robot_id: 机器人ID
    :param mode_command: 模式命令(switch_to_auto 或 switch_to_manual)
    :return: 成功返回True，失败返回False
    """
    # 根据命令类型决定要发送的模式
    if mode_command == "switch_to_auto":
        mode = "auto"
        harvest = True
    elif mode_command == "switch_to_manual":
        mode = "manual"
        harvest = False
    else:
        logger.warning(f"未知的模式命令: {mode_command}")
        return False

    # 创建模式控制消息
    mode_message = {
        "type": "mode_control",
        "mode": mode,
        "harvest": harvest
    }

    # 发送到机器人
    async with lock:
        if robot_id in robots and "websocket" in robots[robot_id]:
            try:
                await robots[robot_id]["websocket"].send_json(mode_message)
                logger.info(f"向机器人 {robot_id} 转发模式切换命令: {mode}")
                return True
            except Exception as e:
                logger.error(f"向机器人 {robot_id} 转发模式切换命令失败: {e}")
                return False
        else:
            logger.warning(f"找不到机器人 {robot_id}")
            return False


# 新增：客户端断开连接处理函数
async def handle_client_disconnect(client_id):
    async with lock:
        if client_id in clients:
            del clients[client_id]
        if client_id in client_to_robot:
            robot_id = client_to_robot[client_id]
            if robot_id in robot_to_clients and client_id in robot_to_clients[robot_id]:
                robot_to_clients[robot_id].remove(client_id)
            del client_to_robot[client_id]

        # 清理该客户端的待处理AI请求
        requests_to_remove = []
        for request_id, request_info in pending_ai_requests.items():
            if request_info.get("client_id") == client_id:
                requests_to_remove.append(request_id)

        for request_id in requests_to_remove:
            del pending_ai_requests[request_id]

    # 从自适应视频管理器中断开客户端
    adaptive_video_manager.disconnect_client(client_id)


# 机器人WebSocket连接处理
@app.websocket("/ws/robot/{robot_id}")
async def robot_websocket_endpoint(websocket: WebSocket, robot_id: str):
    await websocket.accept()
    logger.info(f"机器人 {robot_id} 已连接")

    # 注册机器人
    async with lock:
        # 首先检查是否存在旧连接，如果存在则关闭
        if robot_id in robots and "websocket" in robots[robot_id]:
            try:
                old_ws = robots[robot_id]["websocket"]
                # 发送关闭消息
                await old_ws.close(code=1000, reason="新连接替代")
            except Exception as e:
                logger.error(f"关闭旧连接失败: {e}")

        robots[robot_id] = {
            "websocket": websocket,
            "last_active": datetime.datetime.now(),
            "data": {},  # 存储机器人数据
            "connection_id": f"{robot_id}_{datetime.datetime.now().timestamp()}"  # 唯一连接ID
        }
        # 初始化机器人到客户端的映射
        if robot_id not in robot_to_clients:
            robot_to_clients[robot_id] = []

    # 注册到自适应视频管理器
    adaptive_video_manager.register_robot(robot_id, websocket)

    # 立即通知所有关联的客户端机器人已连接
    await broadcast_robot_status(robot_id, True)

    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                message_type = message.get("type", "")

                # 更新机器人活跃时间
                async with lock:
                    if robot_id in robots:
                        robots[robot_id]["last_active"] = datetime.datetime.now()

                if message_type == "heartbeat":
                    # 处理心跳消息，回复确认
                    await websocket.send_json({
                        "type": "heartbeat_ack",
                        "timestamp": message.get("timestamp", int(time.time() * 1000))
                    })

                elif message_type == "video_frame":
                    # 转发视频帧
                    # 添加服务器时间戳，用于客户端计算网络延迟
                    message["server_timestamp"] = int(time.time() * 1000)
                    await forward_video_frame_optimized(robot_id, message)

                elif message_type == "status_update":
                    # 更新机器人状态
                    status_data = message.get("data", {})

                    # 更新本地存储的机器人数据
                    async with lock:
                        if robot_id in robots:
                            robots[robot_id]["data"].update(status_data)

                    # 更新自适应视频管理器中的机器人状态
                    adaptive_video_manager.update_robot_status(robot_id, status_data)

                    # 将数据上报到IoT平台
                    if robot_id in robots and "data" in robots[robot_id]:
                        report_status_to_iot(robots[robot_id]["data"])

                    # 保存每日统计数据
                    today_date = datetime.datetime.now().strftime("%Y-%m-%d")
                    if status_data:
                        save_daily_statistics(robot_id, today_date, status_data)

                    # 广播数据更新给关联的微信客户端
                    await broadcast_statistics_update(robot_id)

                elif message_type == "quality_adjustment_result":
                    # 处理质量调整结果
                    success = message.get("success", False)
                    preset = message.get("preset")
                    actual_resolution = message.get("actual_resolution")
                    actual_fps = message.get("actual_fps")

                    logger.info(f"机器人 {robot_id} 质量调整结果: preset={preset}, "
                                f"success={success}, resolution={actual_resolution}, fps={actual_fps}")

                    # 通知关联的客户端质量已调整
                    if success:
                        async with lock:
                            if robot_id in robot_to_clients:
                                for client_id in robot_to_clients[robot_id]:
                                    if client_id in clients and "websocket" in clients[client_id]:
                                        try:
                                            await clients[client_id]["websocket"].send_json({
                                                "type": "video_quality_update",
                                                "preset": preset,
                                                "resolution": actual_resolution,
                                                "fps": actual_fps
                                            })
                                        except Exception as e:
                                            logger.error(f"通知客户端 {client_id} 质量更新失败: {e}")

                elif message_type == "ai_chat_response":
                    # 处理来自机器人节点的AI回复
                    await handle_ai_response_from_robot(message)

            except json.JSONDecodeError:
                logger.error(f"收到无效JSON: {data}")
            except Exception as e:
                logger.error(f"处理机器人消息出错: {e}")

    except WebSocketDisconnect:
        logger.info(f"机器人 {robot_id} 已断开连接")
        # 清理机器人连接
        await handle_robot_disconnect(robot_id)


async def handle_ai_response_from_robot(message):
    """处理来自机器人节点的AI回复"""
    try:
        client_id = message.get("client_id", "")
        success = message.get("success", False)
        ai_message = message.get("message", "")
        timestamp = message.get("timestamp", int(time.time() * 1000))
        robot_id = message.get("robot_id", "")
        error = message.get("error", "")

        logger.info(f"收到机器人AI回复 - 客户端: {client_id}, 成功: {success}")

        # 查找对应的客户端WebSocket连接
        async with lock:
            if client_id in clients and "websocket" in clients[client_id]:
                client_ws = clients[client_id]["websocket"]

                # 发送AI回复给客户端
                if success:
                    response = {
                        "type": "ai_chat_response",
                        "message": ai_message,
                        "timestamp": timestamp,
                        "status": "success",
                        "robot_id": robot_id
                    }
                else:
                    response = {
                        "type": "error",
                        "message": ai_message or "AI服务暂时不可用",
                        "context": "ai_chat",
                        "timestamp": timestamp,
                        "error": error
                    }

                await client_ws.send_json(response)
                logger.info(f"AI回复已转发给客户端 {client_id}")
            else:
                logger.warning(f"找不到客户端 {client_id} 的WebSocket连接")

    except Exception as e:
        logger.error(f"处理机器人AI回复出错: {e}")


# 新增：机器人断开连接处理函数
async def handle_robot_disconnect(robot_id):
    async with lock:
        if robot_id in robots:
            del robots[robot_id]

    # 通知所有关联的客户端机器人已断开
    await broadcast_robot_status(robot_id, False)

    # 从自适应视频管理器中断开机器人
    adaptive_video_manager.disconnect_robot(robot_id)


# 新增：向所有关联客户端广播机器人状态
async def broadcast_robot_status(robot_id, is_connected):
    async with lock:
        if robot_id in robot_to_clients:
            for client_id in robot_to_clients[robot_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "robot_connection_status",
                            "robot_id": robot_id,
                            "connected": is_connected,
                            "timestamp": int(time.time() * 1000)
                        })
                    except Exception as e:
                        logger.error(f"通知客户端 {client_id} 机器人状态变更失败: {e}")


# 优化后的视频帧转发函数
async def forward_video_frame_optimized(robot_id, message):
    async with lock:
        if robot_id in robot_to_clients:
            for client_id in robot_to_clients[robot_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        # 直接发送视频帧，不使用gather以避免一个客户端阻塞其他客户端
                        await clients[client_id]["websocket"].send_json(message)
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送视频帧失败: {e}")


# 转发控制命令到机器人
async def forward_command_to_robot(robot_id, command, params):
    async with lock:
        if robot_id in robots and "websocket" in robots[robot_id]:
            try:
                await robots[robot_id]["websocket"].send_json({
                    "type": "command",
                    "command": command,
                    "params": params
                })
                logger.info(f"向机器人 {robot_id} 转发命令: {command}")
                return True
            except Exception as e:
                logger.error(f"向机器人 {robot_id} 转发命令失败: {e}")
                return False
        else:
            logger.warning(f"找不到机器人 {robot_id}")
            return False


# 连接客户端到机器人
async def connect_client_to_robot(client_id, robot_id):
    async with lock:
        # 如果客户端已关联其他机器人，先取消关联
        if client_id in client_to_robot:
            old_robot_id = client_to_robot[client_id]
            if old_robot_id in robot_to_clients and client_id in robot_to_clients[old_robot_id]:
                robot_to_clients[old_robot_id].remove(client_id)

        # 建立新的关联
        client_to_robot[client_id] = robot_id
        if robot_id not in robot_to_clients:
            robot_to_clients[robot_id] = []
        if client_id not in robot_to_clients[robot_id]:
            robot_to_clients[robot_id].append(client_id)

        logger.info(f"客户端 {client_id} 已连接到机器人 {robot_id}")

        # 返回机器人是否在线
        return robot_id in robots


# 广播统计数据更新
async def broadcast_statistics_update(robot_id):
    async with lock:
        if robot_id in robot_to_clients and robot_id in robots and "data" in robots[robot_id]:
            robot_data = robots[robot_id]["data"]

            # 确保数据包含所有必需字段
            if "route_history" not in robot_data:
                robot_data["route_history"] = []

            # 如果没有路线历史，创建一个基本的
            if not robot_data["route_history"] and "position" in robot_data:
                position = robot_data["position"]
                robot_data["route_history"] = [{
                    "time": time.strftime("%H:%M"),
                    "location": position.get("location_name", "当前位置")
                }]

            # 向所有关联的客户端发送数据
            for client_id in robot_to_clients[robot_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "statistics_update",
                            "data": robot_data
                        })
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送统计数据更新失败: {e}")


async def generate_test_data(robot_id):
    """生成测试统计数据"""
    import random
    import math

    # 基础数据
    base_time = time.time()
    working_hours = random.uniform(1.0, 8.0)

    # 生成采摘点历史
    harvest_points = []
    for i in range(10):
        point_time = time.strftime("%H:%M", time.localtime(base_time - (9 - i) * 600))
        harvest_points.append({
            "time": point_time,
            "location": f"N34.{938500 + i * 100}, E108.{241500 + i * 100}",
            "position": {
                "longitude": 108.2415 + i * 0.001,
                "latitude": 34.9385 + i * 0.001
            }
        })

    # 生成路线历史
    route_history = []
    for i in range(5):
        route_time = time.strftime("%H:%M", time.localtime(base_time - (4 - i) * 1800))
        route_history.append({
            "time": route_time,
            "location": f"采摘区 {chr(65 + i)}-{random.randint(1, 20)}"
        })

    # 生成统计数据
    test_data = {
        "today_harvested": random.randint(100, 500),
        "working_area": round(random.uniform(1.0, 5.0), 2),
        "working_hours": round(working_hours, 2),
        "total_harvested": random.randint(1000, 5000),
        "total_area": round(random.uniform(10.0, 50.0), 1),
        "total_hours": round(working_hours * 10, 1),
        "harvest_accuracy": round(random.uniform(92.0, 98.0), 1),
        "battery_level": random.randint(20, 100),
        "longitude": 108.2415 + random.uniform(-0.01, 0.01),
        "latitude": 34.9385 + random.uniform(-0.01, 0.01),
        "speed": round(random.uniform(0.0, 0.5), 2),
        "position": {
            "longitude": 108.2415 + random.uniform(-0.01, 0.01),
            "latitude": 34.9385 + random.uniform(-0.01, 0.01),
            "location_name": f"苹果园区3号地块 {chr(random.randint(65, 68))}-{random.randint(1, 20)} 区域"
        },
        "work_status": {
            "mode": random.choice(["harvesting", "moving", "idle"]),
            "status": "active"
        },
        "route_history": route_history,
        "harvest_points": harvest_points,
        "cpu_usage": random.randint(10, 80),
        "timestamp": int(time.time() * 1000)
    }

    return test_data


async def test_data_generator_task():
    """定期生成测试数据"""
    test_robot_id = "robot_123"

    # 模拟机器人连接
    async with lock:
        robots[test_robot_id] = {
            "websocket": None,  # 测试模式下没有实际WebSocket
            "last_active": datetime.datetime.now(),
            "data": {},
            "connection_id": f"{test_robot_id}_test"
        }

    # 通知客户端机器人已连接
    await broadcast_robot_status(test_robot_id, True)

    while True:
        try:
            # 生成测试数据
            test_data = await generate_test_data(test_robot_id)

            # 更新机器人数据
            async with lock:
                if test_robot_id in robots:
                    robots[test_robot_id]["data"] = test_data
                    robots[test_robot_id]["last_active"] = datetime.datetime.now()

            # 广播数据更新
            await broadcast_statistics_update(test_robot_id)

            # 每10秒更新一次
            await asyncio.sleep(10)

        except Exception as e:
            logger.error(f"测试数据生成错误: {e}")
            await asyncio.sleep(10)


# 保存每日统计数据
def save_daily_statistics(robot_id, date, data):
    """保存每日统计数据"""
    if robot_id not in daily_statistics:
        daily_statistics[robot_id] = {}

    # 保存日期数据
    daily_statistics[robot_id][date] = data

    # 处理月度记录
    save_monthly_record(robot_id, date, data)

    logger.debug(f"已保存机器人 {robot_id} 在 {date} 的统计数据")


def save_monthly_record(robot_id, date, data):
    """保存月度记录"""
    # 解析日期
    try:
        year, month, day = date.split('-')
        month_key = f"{year}-{month}"
        day_num = int(day)
    except:
        logger.error(f"日期格式错误: {date}")
        return

    # 初始化记录
    if robot_id not in monthly_records:
        monthly_records[robot_id] = {}

    if month_key not in monthly_records[robot_id]:
        monthly_records[robot_id][month_key] = []

    # 生成记录
    record = {
        "date": date,
        "day": day_num,
        "month": int(month),
        "title": "农业采摘作业",
        "harvested": data.get("today_harvested", 0),
        "area": round(data.get("working_area", 0), 1),
        "hours": round(data.get("working_hours", 0), 1)
    }

    # 检查是否已存在该日期的记录
    exists = False
    for i, rec in enumerate(monthly_records[robot_id][month_key]):
        if rec.get("date") == date:
            # 更新现有记录
            monthly_records[robot_id][month_key][i] = record
            exists = True
            break

    # 如果不存在就添加新记录
    if not exists:
        monthly_records[robot_id][month_key].append(record)

    # 按日期排序
    monthly_records[robot_id][month_key].sort(key=lambda x: x.get("day", 0), reverse=True)


def filter_history_records(records, filter_type):
    """根据过滤类型处理历史记录"""
    if filter_type == "day":
        # 每日记录，直接返回
        return records
    elif filter_type == "week":
        # 按周汇总
        weekly_records = {}
        for record in records:
            # 根据日期计算周数
            try:
                date_parts = record["date"].split('-')
                from datetime import datetime
                record_date = datetime(int(date_parts[0]), int(date_parts[1]), int(date_parts[2]))
                week_num = record_date.isocalendar()[1]  # 获取一年中的第几周
                week_key = f"{date_parts[0]}-{date_parts[1]}-W{week_num}"
            except:
                continue

            if week_key not in weekly_records:
                # 初始化该周的汇总数据
                weekly_records[week_key] = {
                    "date": f"{date_parts[0]}-{date_parts[1]}-W{week_num}",
                    "day": week_num,  # 使用周数作为"天"
                    "month": int(date_parts[1]),
                    "title": f"第{week_num}周采摘汇总",
                    "harvested": 0,
                    "area": 0.0,
                    "hours": 0.0
                }

            # 累加数据
            weekly_records[week_key]["harvested"] += record["harvested"]
            weekly_records[week_key]["area"] += record["area"]
            weekly_records[week_key]["hours"] += record["hours"]

        # 转换为列表并按周排序
        result = list(weekly_records.values())
        result.sort(key=lambda x: x["day"], reverse=True)
        return result
    elif filter_type == "month":
        # 月度汇总，只需要一条记录
        if not records:
            return []

        monthly_sum = {
            "date": records[0]["date"][:7] if records else "",
            "day": 1,
            "month": records[0]["month"] if records else 0,
            "title": "本月采摘汇总",
            "harvested": sum(r["harvested"] for r in records),
            "area": round(sum(r["area"] for r in records), 1),
            "hours": round(sum(r["hours"] for r in records), 1)
        }
        return [monthly_sum] if records else []

    return records


# 定期检查连接状态，清理过期连接
async def connection_monitor():
    while True:
        try:
            current_time = datetime.datetime.now()

            # 检查机器人连接状态
            async with lock:
                expired_robots = []
                for robot_id, robot in robots.items():
                    # 如果超过30秒未活动，认为连接已过期
                    if (current_time - robot["last_active"]).total_seconds() > 30:
                        expired_robots.append(robot_id)

                # 清理过期的机器人连接
                for robot_id in expired_robots:
                    logger.warning(f"机器人 {robot_id} 连接超时，清理连接")
                    await handle_robot_disconnect(robot_id)

                # 检查客户端连接状态
                expired_clients = []
                for client_id, client in clients.items():
                    # 如果超过60秒未活动，认为连接已过期
                    if (current_time - client["last_active"]).total_seconds() > 60:
                        expired_clients.append(client_id)

                # 清理过期的客户端连接
                for client_id in expired_clients:
                    logger.warning(f"客户端 {client_id} 连接超时，清理连接")
                    await handle_client_disconnect(client_id)

        except Exception as e:
            logger.error(f"连接监控错误: {e}")

        # 每10秒检查一次
        await asyncio.sleep(10)


# 启动事件
@app.on_event("startup")
async def startup_event():
    """服务器启动时的初始化工作"""
    global adaptive_video_manager

    # 连接到IoT平台
    threading.Thread(target=connect_to_iot_platform, daemon=True).start()

    # 初始化自适应视频管理器
    adaptive_video_manager = AdaptiveVideoManager()
    logger.info("自适应视频管理器已初始化")

    # 启动连接监控
    asyncio.create_task(connection_monitor())

    # 启动测试数据生成器（可选，如果需要在没有真实机器人时测试）
    # 取消下面一行的注释来启用测试数据生成
    # asyncio.create_task(test_data_generator_task())


# 关闭事件
@app.on_event("shutdown")
def shutdown_event():
    """服务器关闭时的清理工作"""
    # 断开IoT连接
    if iot_device:
        iot_device.disconnect()

    # 关闭自适应视频管理器
    if adaptive_video_manager:
        adaptive_video_manager.shutdown()

    logger.info("服务器已关闭")


# 健康检查路由
@app.get("/health")
def health_check():
    return {"status": "healthy"}


# 启动服务器
if __name__ == "__main__":
    uvicorn.run(app, host="172.20.39.181", port=1234)