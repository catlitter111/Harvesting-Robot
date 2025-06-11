# adaptive_video_manager.py
import time
import threading
import json
import logging
from collections import defaultdict

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("AdaptiveVideoManager")


class AdaptiveVideoManager:
    """管理多个设备的视频质量自适应"""

    # 预设质量配置
    QUALITY_PRESETS = {
        "high": {
            "resolution": (640, 480),
            "fps": 15,
            "bitrate": 800,  # Kbps
            "quality": 80,  # JPEG质量(1-100)
            "min_bandwidth": 1000  # 需要的最小带宽(Kbps)
        },
        "medium": {
            "resolution": (480, 360),
            "fps": 10,
            "bitrate": 500,
            "quality": 70,
            "min_bandwidth": 600
        },
        "low": {
            "resolution": (320, 240),
            "fps": 8,
            "bitrate": 300,
            "quality": 60,
            "min_bandwidth": 350
        },
        "very_low": {
            "resolution": (240, 180),
            "fps": 5,
            "bitrate": 150,
            "quality": 50,
            "min_bandwidth": 200
        },
        "minimum": {
            "resolution": (160, 120),
            "fps": 3,
            "bitrate": 80,
            "quality": 40,
            "min_bandwidth": 100
        }
    }

    def __init__(self):
        self.clients = {}  # 存储各个客户端的连接信息
        self.robots = {}  # 存储各个机器人的状态信息
        self.robot_to_clients = defaultdict(set)  # 映射机器人到观看它的客户端
        self.lock = threading.RLock()  # 用于线程安全访问共享数据

        # 启动监控线程
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def register_client(self, client_id, websocket):
        """注册一个微信客户端"""
        with self.lock:
            self.clients[client_id] = {
                "websocket": websocket,
                "connected_at": time.time(),
                "last_seen": time.time(),
                "buffer_health": 100,  # 初始缓冲健康度(0-100)
                "network_quality": "unknown",  # 网络质量评估
                "current_preset": "medium",  # 默认使用中等质量
                "robot_id": None,  # 尚未连接到任何机器人
                "latency": 0,  # 客户端报告的延迟(ms)
                "jitter": 0  # 客户端报告的抖动(ms)
            }
            logger.info(f"客户端 {client_id} 已注册")

    def register_robot(self, robot_id, websocket):
        """注册一个机器人"""
        with self.lock:
            self.robots[robot_id] = {
                "websocket": websocket,
                "connected_at": time.time(),
                "last_seen": time.time(),
                "upload_bandwidth": 1000,  # 初始假设上传带宽1000Kbps
                "current_preset": "medium",  # 默认使用中等质量
                "cpu_usage": 0,  # CPU使用率(0-100)
                "battery_level": 100,  # 电池电量(0-100)
                "signal_strength": 70,  # 信号强度(0-100)
                "frames_sent": 0,  # 已发送的帧数
                "bytes_sent": 0  # 已发送的字节数
            }
            logger.info(f"机器人 {robot_id} 已注册")

    def connect_client_to_robot(self, client_id, robot_id):
        """将客户端连接到特定机器人"""
        with self.lock:
            if client_id in self.clients and robot_id in self.robots:
                self.clients[client_id]["robot_id"] = robot_id
                self.robot_to_clients[robot_id].add(client_id)
                # 立即发送当前质量设置
                current_preset = self.robots[robot_id]["current_preset"]
                self._send_quality_info_to_client(client_id, current_preset)
                logger.info(f"客户端 {client_id} 已连接到机器人 {robot_id}")
                return True
            return False

    def disconnect_client(self, client_id):
        """断开客户端连接"""
        with self.lock:
            if client_id in self.clients:
                robot_id = self.clients[client_id].get("robot_id")
                if robot_id and robot_id in self.robot_to_clients:
                    self.robot_to_clients[robot_id].discard(client_id)
                del self.clients[client_id]
                logger.info(f"客户端 {client_id} 已断开连接")

    def disconnect_robot(self, robot_id):
        """断开机器人连接"""
        with self.lock:
            if robot_id in self.robots:
                # 通知所有连接到此机器人的客户端
                for client_id in list(self.robot_to_clients[robot_id]):
                    if client_id in self.clients:
                        self._send_to_client(client_id, {
                            "type": "robot_disconnected",
                            "robot_id": robot_id
                        })
                        self.clients[client_id]["robot_id"] = None

                del self.robots[robot_id]
                self.robot_to_clients[robot_id].clear()
                logger.info(f"机器人 {robot_id} 已断开连接")

    def update_client_status(self, client_id, status_data):
        """更新客户端状态"""
        with self.lock:
            if client_id in self.clients:
                client = self.clients[client_id]
                client["last_seen"] = time.time()

                # 更新客户端报告的状态
                if "buffer_health" in status_data:
                    client["buffer_health"] = status_data["buffer_health"]
                if "network_quality" in status_data:
                    client["network_quality"] = status_data["network_quality"]
                if "latency" in status_data:
                    client["latency"] = status_data["latency"]
                if "jitter" in status_data:
                    client["jitter"] = status_data["jitter"]

                # 根据客户端状态可能需要调整质量
                self._check_client_quality_adjustment(client_id)

    def update_robot_status(self, robot_id, status_data):
        """更新机器人状态"""
        with self.lock:
            if robot_id in self.robots:
                robot = self.robots[robot_id]
                robot["last_seen"] = time.time()

                # 更新机器人报告的状态
                if "upload_bandwidth" in status_data:
                    robot["upload_bandwidth"] = status_data["upload_bandwidth"]
                if "cpu_usage" in status_data:
                    robot["cpu_usage"] = status_data["cpu_usage"]
                if "battery_level" in status_data:
                    robot["battery_level"] = status_data["battery_level"]
                if "signal_strength" in status_data:
                    robot["signal_strength"] = status_data["signal_strength"]
                if "frames_sent" in status_data:
                    robot["frames_sent"] = status_data["frames_sent"]
                if "bytes_sent" in status_data:
                    robot["bytes_sent"] = status_data["bytes_sent"]

                # 根据机器人状态可能需要调整质量
                self._check_robot_quality_adjustment(robot_id)

    def _check_client_quality_adjustment(self, client_id):
        """检查是否需要根据客户端状态调整质量"""
        client = self.clients[client_id]
        robot_id = client.get("robot_id")

        if not robot_id or robot_id not in self.robots:
            return

        current_preset = client["current_preset"]
        buffer_health = client["buffer_health"]

        # 根据缓冲健康度调整质量
        if buffer_health < 20:  # 严重缓冲不足
            new_preset = self._get_lower_preset(current_preset, 2)
        elif buffer_health < 40:  # 轻微缓冲不足
            new_preset = self._get_lower_preset(current_preset, 1)
        elif buffer_health > 90 and client["latency"] < 500:  # 缓冲充足且延迟低
            new_preset = self._get_higher_preset(current_preset, 1)
        else:
            return  # 保持当前质量

        if new_preset != current_preset:
            logger.info(f"根据客户端 {client_id} 的缓冲状态将质量从 {current_preset} 调整为 {new_preset}")
            client["current_preset"] = new_preset
            self._send_quality_info_to_client(client_id, new_preset)

    def _check_robot_quality_adjustment(self, robot_id):
        """检查是否需要根据机器人状态调整质量"""
        robot = self.robots[robot_id]
        current_preset = robot["current_preset"]
        upload_bandwidth = robot["upload_bandwidth"]

        # 查找最适合当前带宽的预设
        client_count = len(self.robot_to_clients[robot_id])
        if client_count == 0:
            return

        # 计算每个客户端可用的带宽
        available_bandwidth_per_client = upload_bandwidth * 0.8 / client_count

        # 找到适合当前带宽的最高质量预设
        best_preset = "minimum"  # 最低质量作为默认值
        for preset, config in sorted(self.QUALITY_PRESETS.items(),
                                     key=lambda x: x[1]["min_bandwidth"],
                                     reverse=True):
            if config["min_bandwidth"] <= available_bandwidth_per_client:
                best_preset = preset
                break

        if best_preset != current_preset:
            logger.info(f"根据带宽 {upload_bandwidth}Kbps 和 {client_count} 个客户端, "
                        f"将机器人 {robot_id} 的质量从 {current_preset} 调整为 {best_preset}")
            robot["current_preset"] = best_preset

            # 通知所有连接的客户端
            for client_id in self.robot_to_clients[robot_id]:
                if client_id in self.clients:
                    self.clients[client_id]["current_preset"] = best_preset
                    self._send_quality_info_to_client(client_id, best_preset)

            # 通知机器人调整质量
            self._send_quality_command_to_robot(robot_id, best_preset)

    def _send_quality_command_to_robot(self, robot_id, preset):
        """向机器人发送质量调整命令"""
        if robot_id in self.robots:
            robot = self.robots[robot_id]
            ws = robot["websocket"]

            quality_config = self.QUALITY_PRESETS[preset]
            command = {
                "type": "quality_adjustment",
                "preset": preset,
                "resolution": {
                    "width": quality_config["resolution"][0],
                    "height": quality_config["resolution"][1]
                },
                "fps": quality_config["fps"],
                "bitrate": quality_config["bitrate"],
                "quality": quality_config["quality"]
            }

            try:
                ws.send(json.dumps(command))
                logger.info(f"已向机器人 {robot_id} 发送质量调整命令: {preset}")
            except Exception as e:
                logger.error(f"向机器人 {robot_id} 发送质量命令失败: {e}")

    def _send_quality_info_to_client(self, client_id, preset):
        """向客户端发送质量信息"""
        if client_id in self.clients:
            client = self.clients[client_id]
            ws = client["websocket"]

            quality_config = self.QUALITY_PRESETS[preset]
            info = {
                "type": "video_quality_update",
                "preset": preset,
                "resolution": f"{quality_config['resolution'][0]}x{quality_config['resolution'][1]}",
                "fps": quality_config["fps"]
            }

            try:
                ws.send(json.dumps(info))
                logger.info(f"已向客户端 {client_id} 发送质量信息: {preset}")
            except Exception as e:
                logger.error(f"向客户端 {client_id} 发送质量信息失败: {e}")

    def _send_to_client(self, client_id, message):
        """向客户端发送通用消息"""
        if client_id in self.clients:
            client = self.clients[client_id]
            ws = client["websocket"]

            try:
                ws.send(json.dumps(message))
            except Exception as e:
                logger.error(f"向客户端 {client_id} 发送消息失败: {e}")

    def _get_lower_preset(self, current_preset, steps=1):
        """获取比当前更低的质量预设"""
        presets = list(self.QUALITY_PRESETS.keys())
        try:
            current_index = presets.index(current_preset)
            target_index = min(len(presets) - 1, current_index + steps)
            return presets[target_index]
        except ValueError:
            return "medium"  # 默认

    def _get_higher_preset(self, current_preset, steps=1):
        """获取比当前更高的质量预设"""
        presets = list(self.QUALITY_PRESETS.keys())
        try:
            current_index = presets.index(current_preset)
            target_index = max(0, current_index - steps)
            return presets[target_index]
        except ValueError:
            return "medium"  # 默认

    def _monitor_loop(self):
        """监控循环，定期检查所有连接的状态"""
        while self.running:
            try:
                with self.lock:
                    current_time = time.time()

                    # 检查离线客户端
                    offline_clients = []
                    for client_id, client in self.clients.items():
                        if current_time - client["last_seen"] > 30:  # 30秒无响应视为离线
                            offline_clients.append(client_id)

                    # 移除离线客户端
                    for client_id in offline_clients:
                        self.disconnect_client(client_id)

                    # 检查离线机器人
                    offline_robots = []
                    for robot_id, robot in self.robots.items():
                        if current_time - robot["last_seen"] > 30:  # 30秒无响应视为离线
                            offline_robots.append(robot_id)

                    # 移除离线机器人
                    for robot_id in offline_robots:
                        self.disconnect_robot(robot_id)

                    # 每10秒重新评估所有机器人的质量设置
                    for robot_id in self.robots:
                        self._check_robot_quality_adjustment(robot_id)

            except Exception as e:
                logger.error(f"监控循环出错: {e}")

            time.sleep(10)  # 每10秒检查一次

    def shutdown(self):
        """关闭管理器"""
        self.running = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        logger.info("自适应视频管理器已关闭")