# 舵机调试系统使用指南

## 概述

舵机调试系统提供了一个交互式的机械臂调试工具，可以手动控制舵机运动、测试采摘动作序列，并监控系统状态。该系统包含两个主要节点：

- **servo_control_node**: 舵机控制节点，负责实际的硬件控制
- **servo_debug_node**: 调试节点，提供交互式的键盘控制界面

## 系统架构

```
┌─────────────────┐    ServoCommand     ┌─────────────────┐
│                 │ ──────────────────→ │                 │
│ servo_debug_    │                     │ servo_control_  │
│ node            │ ←────────────────── │ node            │
│ (键盘控制)      │    ServoStatus      │ (硬件控制)      │
└─────────────────┘                     └─────────────────┘
         │                                       │
         │ HarvestCommand                        │ 串口通信
         │                                       │
         └─────────────────────────────────────┘ ┌─────────────┐
                                                 │   舵机硬件   │
                                                 │   (6个舵机)  │
                                                 └─────────────┘
```

## 安装和构建

### 1. 确保依赖包已安装

```bash
# 检查消息包是否存在
ls ~/AgriSage3/src/bottle_detection_msgs

# 如果不存在，请先构建消息包
cd ~/AgriSage3
colcon build --packages-select bottle_detection_msgs
```

### 2. 构建舵机调试系统

```bash
cd ~/AgriSage3
colcon build --packages-select bottle_detection_ros2
source install/setup.bash
```

### 3. 检查串口权限

```bash
# 检查串口设备
ls -l /dev/ttyS*

# 添加用户到dialout组（如果需要）
sudo usermod -a -G dialout $USER

# 重新登录或使用以下命令立即生效
newgrp dialout
```

## 启动系统

### 基础启动

```bash
# 启动舵机调试系统（使用默认参数）
ros2 launch bottle_detection_ros2 servo_debug.launch.py

# 使用自定义串口
ros2 launch bottle_detection_ros2 servo_debug.launch.py serial_port:=/dev/ttyUSB0

# 使用自定义参数
ros2 launch bottle_detection_ros2 servo_debug.launch.py \
    serial_port:=/dev/ttyS9 \
    baudrate:=115200 \
    step_size:=30 \
    enable_tracking:=false
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyS9` | 舵机串口设备路径 |
| `baudrate` | `115200` | 串口波特率 |
| `step_size` | `50` | 调试节点的步长大小（PWM） |
| `time_ms` | `200` | 舵机移动时间（毫秒） |
| `enable_tracking` | `false` | 是否启用自动跟踪功能 |

## 操作指南

### 键盘控制

启动系统后，servo_debug_node会显示控制说明：

```
==================================================
机械臂调试控制
==================================================
使用方向键控制舵机:
  ↑/↓ : 控制垂直舵机 (上/下)
  ←/→ : 控制水平舵机 (左/右)

其他控制键:
  Space : 回到中心位置
  +/-   : 增加/减少步长
  q     : 退出程序

快捷位置:
  1 : 左上角     2 : 正上方     3 : 右上角
  4 : 左边       5 : 中心       6 : 右边
  7 : 左下角     8 : 正下方     9 : 右下角

高级功能:
  h : 显示/隐藏帮助
  i : 显示当前舵机信息
  r : 读取所有舵机位置
  t : 测试采摘动作序列
  c : 回到中心并重置采摘系统
==================================================
```

### 基本操作流程

1. **初始化检查**
   ```bash
   # 启动后检查节点状态
   ros2 node list
   # 应该看到:
   # /servo_control_node
   # /servo_debug_node
   ```

2. **舵机范围测试**
   - 按数字键1-9测试各个预设位置
   - 使用方向键进行精细调整
   - 按空格键回到中心位置

3. **采摘动作测试**
   - 按 't' 键启动完整的采摘动作序列
   - 观察机械臂执行5个步骤的采摘动作
   - 按 'c' 键重置系统到初始状态

4. **参数调整**
   - 按 '+' 或 '-' 键调整移动步长
   - 按 'i' 键查看当前舵机参数信息

### 故障排除

#### 常见问题

1. **串口连接失败**
   ```bash
   # 检查串口是否存在
   ls -l /dev/ttyS9
   
   # 检查权限
   groups $USER  # 应该包含 dialout
   
   # 检查串口是否被占用
   sudo lsof /dev/ttyS9
   ```

2. **舵机无响应**
   ```bash
   # 检查消息发布
   ros2 topic echo /servo/command
   
   # 检查串口数据
   ros2 topic echo /servo/status
   
   # 手动测试串口通信
   sudo minicom -D /dev/ttyS9 -b 115200
   ```

3. **节点启动失败**
   ```bash
   # 检查包路径
   ros2 pkg list | grep bottle_detection
   
   # 重新构建
   cd ~/AgriSage3
   colcon build --packages-select bottle_detection_ros2 --symlink-install
   source install/setup.bash
   ```

#### 调试命令

```bash
# 查看节点详细信息
ros2 node info /servo_control_node
ros2 node info /servo_debug_node

# 查看话题列表
ros2 topic list

# 监控消息流
ros2 topic hz /servo/command
ros2 topic hz /servo/status

# 查看参数
ros2 param list /servo_control_node
ros2 param get /servo_control_node serial_port
```

## 高级配置

### 修改舵机参数

编辑配置文件：
```bash
nano ~/AgriSage3/src/bottle_detection_ros2/config/servo_debug_params.yaml
```

重要参数说明：
- `horizontal_servo_center`: 水平舵机中心位置（PWM值）
- `vertical_servo_center`: 垂直舵机中心位置（PWM值）
- `step_size`: 默认移动步长
- `tracking_deadzone`: 跟踪死区大小

### 自定义采摘动作

在配置文件中修改 `harvest_sequence` 部分：
```yaml
harvest_sequence:
  commands:
    rt_catch1: "#002P1650T2000!#003P1300T2000!#005P1700T2000!"
    # ... 其他动作命令
  step_delays:
    step1_delay: 2.0  # 调整动作间隔时间
```

### 集成到现有系统

```bash
# 在现有launch文件中包含舵机调试
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py \
    include_servo_debug:=true
```

## 安全注意事项

1. **运动范围限制**
   - 水平舵机：500-2500 PWM
   - 垂直舵机：500-1500 PWM
   - 超出范围可能损坏硬件

2. **紧急停止**
   - 按 'q' 键立即退出程序
   - 按 'c' 键重置到安全位置
   - 如果硬件失控，立即断开电源

3. **调试环境**
   - 确保机械臂周围有足够空间
   - 调试时建议关闭自动跟踪功能
   - 首次使用请在小步长下测试

## 开发和扩展

### 添加新功能

1. **在servo_debug_node.py中添加新的键盘处理**
2. **在servo_control_node.py中添加新的控制逻辑**
3. **更新配置文件以包含新参数**
4. **重新构建和测试**

### 消息接口

主要消息类型：
- `ServoCommand`: 舵机控制命令
- `ServoStatus`: 舵机状态反馈
- `HarvestCommand`: 采摘动作命令

### 话题列表

```bash
/servo/command          # 舵机控制命令
/servo/status           # 舵机状态反馈  
/servo/tracking_target  # 跟踪目标位置
/robot/harvest_command  # 采摘控制命令
/harvest/status         # 采摘状态反馈
```

## 贡献和反馈

如有问题或建议，请：
1. 查看日志输出进行初步诊断
2. 检查硬件连接和配置
3. 提供详细的错误信息和环境配置
4. 建议改进功能或优化方案

---

**最后更新**: 2024年12月
**版本**: 1.0.0
**兼容性**: ROS2 Humble 