# 本地控制界面使用指南

## 📋 概述

本地控制界面是一个基于 tkinter 的图形化控制系统，专为智慧农业采摘机器人设计。它提供了实时监控、系统控制和状态显示功能，可以替代微信小程序进行本地控制。

## 🎨 界面设计

界面完全按照设计图实现，包含以下主要区域：

### 1. 顶部标题栏
- 🌱 系统标题：智慧农业采摘系统
- ⏰ 实时时间显示
- 🟢 系统运行状态指示器

### 2. 左侧控制中心
- **系统运行控制**：启动/停止系统
- **工作模式选择**：手动/自动模式切换
- **采摘控制**：开始/停止采摘
- **紧急停止**：立即停止所有操作
- **今日成果**：显示当日采摘数量

### 3. 中间实时监控
- **视频显示**：实时显示处理后的摄像头画面
- **FPS显示**：显示当前帧率
- **检测信息**：目标数量、最近距离、准确率

### 4. 右侧系统状态
- **电池电量**：显示电池电量百分比和进度条
- **CPU使用率**：显示CPU使用情况
- **系统温度**：显示系统温度状态
- **位置信息**：GPS坐标和区域信息
- **工作状态**：工作时长、移动速度等

### 5. 底部农场数据总览
- **累计采摘**：总采摘数量
- **作业精度**：采摘准确率
- **信号强度**：网络信号强度
- **运行天数**：系统运行天数
- **健康状态**：系统健康评估

## 🚀 启动方式

### 方式一：使用便捷脚本（推荐）

```bash
# 进入项目目录
cd Harvesting-Robot/robot_ROS2-6_9

# 运行启动脚本
./src/bottle_detection_ros2/scripts/start_local_control.sh
```

### 方式二：直接使用launch文件

```bash
# 确保环境已设置
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动本地控制系统
ros2 launch bottle_detection_ros2 local_control_system.launch.py
```

### 方式三：与集成系统一起启动

```bash
# 启动完整集成系统（包含GUI）
ros2 launch bottle_detection_ros2 integrated_system.launch.py
```

## 🔧 配置参数

### 启动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `camera_id` | 21 | 双目相机设备ID |
| `model_path` | yolo11n.rknn路径 | RKNN模型文件路径 |
| `robot_serial_port` | /dev/ttyS3 | 机器人控制串口 |
| `servo_serial_port` | /dev/ttyS9 | 舵机控制串口 |
| `show_display` | true | 是否显示图像窗口 |

### 自定义启动示例

```bash
ros2 launch bottle_detection_ros2 local_control_system.launch.py \
    camera_id:=0 \
    robot_serial_port:="/dev/ttyUSB0" \
    servo_serial_port:="/dev/ttyUSB1"
```

## 📡 ROS2 话题接口

### 订阅的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/camera/processed_image/compressed` | `CompressedImage` | 处理后的摄像头图像 |
| `/bottle_detection` | `BottleDetection` | 瓶子检测结果 |
| `/robot_status` | `RobotStatus` | 机器人状态信息 |
| `/camera/fps` | `Float32` | 摄像头FPS信息 |

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/robot_command` | `RobotCommand` | 机器人控制命令 |
| `/robot/harvest_command` | `HarvestCommand` | 采摘控制命令 |
| `/work_mode` | `String` | 工作模式切换 |

## 🎮 控制功能

### 系统控制
- **启动系统**：启动所有必要的节点
- **停止系统**：停止所有运行的节点
- **紧急停止**：立即停止所有运动和操作

### 工作模式
- **手动模式**：手动控制机器人运动和采摘
- **自动模式**：启用自动采摘算法

### 采摘控制
- **开始采摘**：启动采摘任务
- **停止采摘**：停止当前采摘任务

## 📊 数据显示

### 实时数据
- **检测结果**：实时显示检测到的目标数量和距离
- **系统状态**：电池、CPU、温度等硬件状态
- **位置信息**：GPS坐标和区域位置
- **工作状态**：当前工作模式和运行时间

### 统计数据
- **今日成果**：当日采摘数量
- **累计采摘**：总采摘数量
- **作业精度**：采摘准确率统计
- **运行天数**：系统运行时间统计

## 🛠️ 依赖要求

### 系统依赖
```bash
# Python GUI库
sudo apt install python3-tk python3-pil python3-pil.imagetk

# OpenCV
pip install opencv-python

# ROS2 Humble
# 确保已安装ROS2 Humble
```

### Python依赖
```python
# 主要依赖
rclpy
tkinter
PIL (Pillow)
cv2 (opencv-python)
numpy
threading
traceback
subprocess
```

## 🔍 故障排除

### 常见问题

1. **界面无法启动**
   - 检查是否安装了 tkinter：`python3 -c "import tkinter"`
   - 确保X11转发已启用（SSH连接时）

2. **摄像头画面不显示**
   - 检查摄像头设备是否存在：`ls /dev/video*`
   - 确认camera_id参数正确
   - 检查瓶子检测节点是否正常运行

3. **串口连接失败**
   - 检查串口设备权限：`ls -l /dev/ttyS*`
   - 添加用户到dialout组：`sudo usermod -a -G dialout $USER`

4. **ROS2节点通信问题**
   - 检查节点状态：`ros2 node list`
   - 检查话题状态：`ros2 topic list`
   - 查看节点日志：`ros2 log`

### 调试模式

启用详细日志输出：
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch bottle_detection_ros2 local_control_system.launch.py
```

## 📝 开发说明

### 代码结构
```
local_control_gui.py
├── LocalControlGUI类 - 主GUI控制器
│   ├── 界面创建方法
│   ├── 控制功能方法
│   ├── 数据更新方法
│   └── ROS2接口方法
└── main函数 - 程序入口
```

### 扩展功能
- 所有方法都包含完整的错误处理和traceback
- 支持自定义配置参数
- 模块化设计，易于扩展新功能
- 完全兼容现有ROS2系统架构

### 错误处理
```python
try:
    # 业务逻辑
    pass
except Exception as e:
    self.get_logger().error(f'操作失败: {e}')
    traceback.print_exc()
```

## 📄 许可证

本项目采用 Apache-2.0 许可证。

## 👥 贡献

欢迎提交问题和功能请求！

---

**注意**：本界面设计完全按照用户提供的设计图实现，包含所有功能模块和视觉元素。 