# 本地控制GUI使用说明

## 概述

本地控制GUI是一个基于Tkinter的图形界面程序，用于监控和控制智慧农业采摘机器人系统。界面提供了实时视频流、系统状态监控、手动/自动模式切换、采摘控制等功能。

## 功能特性

### 🎮 控制中心
- **系统运行开关**: 启动/停止整个机器人系统
- **工作模式选择**: 手动模式和自动模式切换
- **采摘控制**: 开始/停止采摘作业
- **紧急停止**: 紧急情况下立即停止所有操作
- **今日成果**: 实时显示今日采摘数量

### 📹 实时监控
- **视频流显示**: 实时显示摄像头画面
- **FPS显示**: 显示视频帧率
- **检测信息**: 显示检测到的目标数量、最近距离、准确率

### 🔧 系统状态
- **电池电量**: 实时显示电池状态和电量百分比
- **CPU使用率**: 系统CPU使用情况
- **系统温度**: 设备温度监控
- **位置信息**: GPS坐标和当前位置名称
- **工作状态**: 工作时长、移动速度等信息

### 🌾 农场数据总览
- **累计采摘**: 历史总采摘数量
- **作业精度**: 系统作业准确率
- **信号强度**: 网络信号强度
- **运行天数**: 系统运行天数统计
- **健康状态**: 整体系统健康评估

## 安装依赖

### 系统依赖
```bash
# 安装tkinter
sudo apt install python3-tk

# 安装其他Python包
pip3 install pillow opencv-python numpy
```

### ROS2依赖
确保已安装ROS2 Humble并正确配置环境：
```bash
source /opt/ros/humble/setup.bash
```

## 编译和安装

1. 编译工作空间：
```bash
cd robot_ROS2-6_9
colcon build --packages-select bottle_detection_ros2
```

2. 加载环境：
```bash
source install/setup.bash
```

## 启动方式

### 方式1：使用启动脚本（推荐）
```bash
cd robot_ROS2-6_9/src/bottle_detection_ros2/scripts
./start_local_control_gui.sh
```

脚本提供三种启动选项：
1. 仅启动GUI界面
2. 启动GUI + 基础系统
3. 使用launch文件启动

### 方式2：直接运行
```bash
ros2 run bottle_detection_ros2 local_control_gui
```

### 方式3：使用launch文件
```bash
ros2 launch bottle_detection_ros2 local_control_gui.launch.py
```

## 使用方法

### 启动系统
1. 打开GUI界面
2. 点击"启动系统"按钮
3. 等待系统初始化完成
4. 状态指示器变为"运行中"

### 模式切换
- **手动模式**: 需要手动控制机器人的移动和采摘动作
- **自动模式**: 机器人自动执行采摘任务

### 采摘控制
1. 确保系统已启动
2. 选择适当的工作模式
3. 点击"开始采摘"按钮
4. 系统开始执行采摘任务
5. 可随时点击"停止采摘"暂停任务

### 紧急停止
在任何情况下都可以点击红色的"紧急停止"按钮立即停止所有操作。

## 消息接口

### 订阅的话题
- `/camera/processed_image/compressed`: 处理后的图像流
- `/bottle_detection`: 检测结果
- `/robot_status`: 机器人状态信息
- `/camera/fps`: 视频帧率信息

### 发布的话题
- `/robot_command`: 机器人控制命令
- `/harvest_command`: 采摘控制命令
- `/work_mode`: 工作模式切换

## 故障排除

### 常见问题

1. **GUI无法启动**
   - 检查DISPLAY环境变量是否设置
   - 确认tkinter已正确安装
   - 检查ROS2环境是否正确配置

2. **视频流无显示**
   - 检查摄像头节点是否运行
   - 确认视频流话题是否正确发布
   - 检查OpenCV是否正确安装

3. **控制命令无响应**
   - 检查机器人控制节点是否运行
   - 确认ROS2话题连接正常
   - 检查系统是否处于正确状态

4. **状态信息不更新**
   - 检查机器人状态发布节点
   - 确认消息类型匹配
   - 检查网络连接

### 调试方法

1. **查看日志**
```bash
ros2 run bottle_detection_ros2 local_control_gui --ros-args --log-level debug
```

2. **检查话题**
```bash
ros2 topic list
ros2 topic echo /robot_status
```

3. **查看节点状态**
```bash
ros2 node list
ros2 node info /local_control_gui_node
```

## 开发说明

### 代码结构
- `LocalControlGUI`: 主界面类
- `ROSControlNode`: ROS2节点类
- 界面采用模块化设计，便于扩展和维护

### 自定义配置
可以通过修改代码中的初始化参数来调整界面布局、颜色主题等。

### 扩展功能
- 添加新的状态显示组件
- 集成更多控制功能
- 添加数据记录和导出功能

## 注意事项

1. 确保在图形界面环境中运行
2. 建议使用分辨率至少1200x800的显示器
3. 运行时保持网络连接稳定
4. 定期检查系统资源使用情况

## 版本信息

- 版本：1.0.0
- 兼容ROS2版本：Humble
- Python版本：3.8+
- 支持系统：Ubuntu 22.04

## 许可证

Apache-2.0 