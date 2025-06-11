# AgriSage3 激光雷达避障功能集成指南

## 概述

本文档介绍了如何将激光雷达避障功能集成到AgriSage3农业采摘机器人系统中。通过整合三个程序包的功能，实现了智能避障与自动采摘的协同工作。

## 系统架构

### 整合的程序包

1. **AgriSage3** - 您的主程序
   - 瓶子检测与追踪
   - 机械臂控制
   - 自动采摘逻辑
   - WebSocket通信

2. **order-humble-main** - 激光雷达驱动
   - MS200激光雷达数据获取
   - 发布`sensor_msgs/LaserScan`消息

3. **yahboomcar_laser** - 官方避障算法（已适配）
   - 原始避障逻辑参考
   - 已适配为ROS2 Humble版本

### 新增组件

#### 1. 激光雷达避障控制器 (`laser_obstacle_avoidance.py`)

**功能特性：**
- 智能障碍物检测和分析
- 多级避障策略（减速、转向、紧急停止）
- 与自动采摘控制器协同工作
- 实时参数调整
- 完整的状态监控和日志

**关键参数：**
```yaml
response_distance: 1.2    # 响应距离
danger_distance: 0.6      # 危险距离
emergency_distance: 0.3   # 紧急距离
front_angle: 30.0         # 前方扇区角度
side_angle: 60.0          # 侧面扇区角度
```

#### 2. 系统集成策略

**话题重定向：**
- 原始: `auto_harvest_controller` → `cmd_vel`
- 新架构: `auto_harvest_controller` → `cmd_vel_raw` → `laser_obstacle_avoidance` → `cmd_vel`

**优势：**
- 无缝集成，无需修改核心业务逻辑
- 避障功能可独立开关
- 保持原有控制精度

## 系统拓扑图

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   激光雷达      │    │   双目相机      │    │   串口设备      │
│   MS200         │    │   瓶子检测      │    │   机械臂/底盘   │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          │ /scan                │ 检测结果              │ 控制指令
          ▼                      ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ 激光雷达避障    │    │ 瓶子检测节点    │    │ 舵机控制节点    │
│ 控制器          │    │                 │    │                 │
└─────────┬───────┘    └─────────┬───────┘    └─────────────────┘
          │                      │
          │ cmd_vel              │ 目标信息
          ▼                      ▼
┌─────────────────┐    ┌─────────────────┐
│ 机器人控制      │◄───┤ 自动采摘控制器  │
│ 节点            │    │ cmd_vel_raw     │
└─────────────────┘    └─────────────────┘
          │                      ▲
          │                      │ 模式控制
          ▼                      │
┌─────────────────┐    ┌─────────────────┐
│ WebSocket       │    │ 用户界面        │
│ 桥接节点        │    │ (微信小程序)    │
└─────────────────┘    └─────────────────┘
```

## 安装和配置

### 1. 依赖安装

确保您的系统已安装以下依赖：

```bash
# ROS2 Humble 基础包
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros

# 激光雷达驱动依赖
sudo apt install ros-humble-launch-ros
sudo apt install ros-humble-launch

# Python依赖
pip3 install numpy
```

### 2. 编译工作空间

```bash
cd AgriSage3
colcon build --symlink-install
source install/setup.bash
```

### 3. 设备配置

#### 激光雷达设备权限

```bash
# 添加udev规则
sudo cp oradar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 检查设备
ls -l /dev/oradar
```

#### 串口设备权限

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启生效
```

## 使用方法

### 1. 快速启动

```bash
cd AgriSage3
./src/bottle_detection_ros2/scripts/start_agrisage_with_lidar.sh
```

### 2. 自定义启动

```bash
# 禁用避障功能
./start_agrisage_with_lidar.sh --no-avoidance

# 禁用显示窗口
./start_agrisage_with_lidar.sh --no-display

# 查看帮助
./start_agrisage_with_lidar.sh --help
```

### 3. 手动启动

```bash
# 启动完整系统
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py

# 自定义参数
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py \
    enable_avoidance:=true \
    enable_display:=false
```

## 参数配置

### 避障参数调整

编辑配置文件：`config/laser_avoidance_params.yaml`

```yaml
laser_obstacle_avoidance:
  ros__parameters:
    # 距离阈值调整
    response_distance: 1.2      # 增大值可提前响应
    danger_distance: 0.6        # 减小值可更保守
    emergency_distance: 0.3     # 紧急制动距离
    
    # 速度限制
    max_linear_speed: 0.5       # 最大前进速度
    max_angular_speed: 1.0      # 最大转向速度
    slow_down_factor: 0.4       # 减速比例
    
    # 检测敏感度
    obstacle_count_threshold: 8  # 障碍物点数阈值
```

### 运行时参数调整

```bash
# 启用/禁用避障
ros2 param set /laser_obstacle_avoidance avoidance_enabled true

# 调整响应距离
ros2 param set /laser_obstacle_avoidance response_distance 1.5

# 调整速度限制
ros2 param set /laser_obstacle_avoidance max_linear_speed 0.3
```

## 监控和调试

### 1. 话题监控

```bash
# 查看激光雷达数据
ros2 topic echo /scan

# 查看避障状态
ros2 topic echo /avoidance/status

# 查看障碍物信息
ros2 topic echo /obstacle/info

# 查看速度指令
ros2 topic echo /cmd_vel_raw      # 原始指令
ros2 topic echo /cmd_vel          # 避障后指令
```

### 2. 节点状态检查

```bash
# 查看所有节点
ros2 node list

# 检查节点信息
ros2 node info /laser_obstacle_avoidance

# 查看参数
ros2 param list /laser_obstacle_avoidance
```

### 3. 性能监控

```bash
# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# 查看计算延迟
ros2 topic delay /scan
```

## 工作模式

### 1. 手动模式

- **特征**: 用户通过小程序或遥控器直接控制
- **避障行为**: 仅提供安全保护，不干扰正常操作
- **适用场景**: 精确定位、调试、紧急操作

### 2. 自动模式

- **特征**: 系统自主进行目标检测和采摘
- **避障行为**: 智能规避障碍物，保持采摘路径
- **适用场景**: 常规作业、批量采摘

### 3. 协同工作机制

1. **远距离**: 电机驱动 + 避障导航
2. **中距离**: 精细控制 + 路径规划
3. **近距离**: 舵机精确定位 + 安全监控
4. **采摘时**: 机械臂动作 + 碰撞保护

## 故障排查

### 常见问题

#### 1. 激光雷达无数据

**症状**: `/scan`话题无消息
**排查步骤**:
```bash
# 检查设备连接
ls -l /dev/oradar

# 检查权限
sudo chmod 666 /dev/oradar

# 重启驱动节点
ros2 lifecycle set /laser_driver shutdown
ros2 lifecycle set /laser_driver configure
ros2 lifecycle set /laser_driver activate
```

#### 2. 避障过于敏感

**症状**: 机器人频繁停止或转向
**解决方案**:
```bash
# 调整阈值
ros2 param set /laser_obstacle_avoidance response_distance 1.5
ros2 param set /laser_obstacle_avoidance obstacle_count_threshold 10
```

#### 3. 采摘时误触发避障

**症状**: 接近目标时被误判为障碍物
**解决方案**:
```bash
# 调整前方扇区角度
ros2 param set /laser_obstacle_avoidance front_angle 20.0
# 或临时关闭避障
ros2 param set /laser_obstacle_avoidance avoidance_enabled false
```

### 日志分析

查看详细日志：
```bash
# 实时日志
ros2 run bottle_detection_ros2 laser_obstacle_avoidance --ros-args --log-level DEBUG

# 查看系统日志
journalctl -u ros2-*
```

## 性能优化

### 1. 参数调优

根据实际使用环境调整：
- **果园环境**: 增大`response_distance`，减小`obstacle_count_threshold`
- **大棚环境**: 减小`response_distance`，增大`max_angular_speed`
- **密集种植**: 调整`front_angle`和`side_angle`

### 2. 硬件优化

- **激光雷达频率**: 建议10-15Hz，平衡性能和响应速度
- **控制频率**: 避障控制器20Hz，采摘控制器10Hz
- **通信优化**: 使用QoS配置优化网络传输

## 扩展功能

### 1. 地图构建

可集成SLAM功能：
```bash
# 安装SLAM工具包
sudo apt install ros-humble-slam-toolbox

# 启动建图
ros2 launch slam_toolbox online_async_launch.py
```

### 2. 导航规划

集成导航功能：
```bash
# 安装Navigation2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# 启动导航
ros2 launch nav2_bringup navigation_launch.py
```

### 3. 多机协作

支持多机器人协同作业：
- 命名空间隔离
- 分布式任务分配
- 碰撞避免协议

## 技术支持

### 联系方式

如有问题，请通过以下方式联系：
- GitHub Issues: [项目链接]
- 邮箱: [技术支持邮箱]
- 文档更新: [文档链接]

### 版本信息

- **AgriSage3 版本**: v1.0.0
- **ROS2 版本**: Humble
- **激光雷达支持**: MS200
- **更新日期**: 2024年

---

**注意**: 请在真实环境中测试前，先在安全的环境下验证所有功能。确保所有安全机制正常工作后再投入生产使用。 