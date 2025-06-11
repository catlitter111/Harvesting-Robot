# AgriSage3 SLAM建图功能说明

## 概述

为AgriSage3四轮小车添加了专用的SLAM建图功能，基于ROS2 Humble适配，整合了order-humble的激光雷达驱动和slam_toolbox建图算法。

## 主要特性

### 1. 适配ROS2 Humble
- 完全兼容ROS2 Humble API
- 修复了order-humble中Foxy时代的API调用
- 使用现代ROS2启动文件格式

### 2. 独立建图节点
- `slam_mapping_node` - 自定义SLAM监控节点
- 实时状态监控和地图保存
- 与避障系统协同工作
- 支持手动和自动建图模式

### 3. 完整的系统集成
- 激光雷达驱动（MS200）
- SLAM Toolbox建图算法
- RViz2实时可视化
- 自动地图保存功能

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   MS200激光雷达  │    │  SLAM Toolbox   │    │   自定义监控    │
│   order-humble   │────│  建图算法       │────│   slam_mapping  │
│   (适配Humble)   │    │  (Humble版)     │    │   _node         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
         ┌─────────────────┐     │     ┌─────────────────┐
         │   RViz2可视化   │─────┼─────│   地图保存      │
         │   实时显示      │     │     │   ~/agrisage3   │
         └─────────────────┘     │     │   _maps/        │
                                 │     └─────────────────┘
         ┌─────────────────┐     │     
         │   TF变换树      │─────┘
         │   坐标管理      │
         └─────────────────┘
```

## 安装和配置

### 1. 依赖检查

确保已安装必要的ROS2包：

```bash
# SLAM工具箱
sudo apt install ros-humble-slam-toolbox

# TF和可视化
sudo apt install ros-humble-tf2-ros ros-humble-rviz2

# 其他依赖
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
```

### 2. 激光雷达设备配置

确保order-humble包已正确安装并配置：

```bash
# 检查设备
ls -l /dev/oradar

# 如果设备不存在，复制udev规则
sudo cp order-humble/oradar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 编译工作空间

```bash
cd robot_ROS2
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 1. 快速启动

```bash
cd robot_ROS2
./src/bottle_detection_ros2/scripts/start_slam_mapping.sh
```

### 2. 启动选项

```bash
# 不显示RViz（节省资源）
./start_slam_mapping.sh --no-rviz

# 自动开始建图
./start_slam_mapping.sh --auto-mapping

# 仿真模式
./start_slam_mapping.sh --sim-time

# 查看帮助
./start_slam_mapping.sh --help
```

### 3. 手动启动

```bash
# 启动SLAM系统
ros2 launch bottle_detection_ros2 slam_mapping.launch.py

# 自定义参数
ros2 launch bottle_detection_ros2 slam_mapping.launch.py \
    enable_rviz:=true \
    auto_start_mapping:=false
```

## 建图操作

### 1. 开始建图

```bash
# 使用服务调用
ros2 service call /slam/start_mapping std_srvs/srv/SetBool "{data: true}"

# 或发布话题消息
ros2 topic pub /slam/control std_msgs/msg/String \
    'data: "{\"command\": \"start\"}"' --once
```

### 2. 停止建图

```bash
# 停止建图
ros2 service call /slam/start_mapping std_srvs/srv/SetBool "{data: false}"

# 或使用话题
ros2 topic pub /slam/control std_msgs/msg/String \
    'data: "{\"command\": \"stop\"}"' --once
```

### 3. 保存地图

```bash
# 手动保存地图
ros2 service call /slam/save_map std_srvs/srv/Empty

# 或使用话题
ros2 topic pub /slam/control std_msgs/msg/String \
    'data: "{\"command\": \"save\"}"' --once
```

## 监控和调试

### 1. 话题监控

```bash
# 查看激光雷达数据
ros2 topic echo /scan

# 查看地图数据
ros2 topic echo /map

# 查看SLAM状态
ros2 topic echo /slam/status

# 查看地图信息
ros2 topic echo /slam/map_info

# 查看机器人位姿
ros2 topic echo /slam/robot_pose
```

### 2. 节点状态

```bash
# 查看所有节点
ros2 node list

# 检查SLAM节点
ros2 node info /slam_toolbox
ros2 node info /slam_mapping_node

# 查看参数
ros2 param list /slam_toolbox
ros2 param list /slam_mapping_node
```

### 3. TF树检查

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查TF变换
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar
```

## 参数配置

### 1. SLAM参数调整

编辑 `config/slam_mapping_params.yaml`：

```yaml
slam_toolbox:
  ros__parameters:
    # 建图精度
    resolution: 0.05              # 地图分辨率（米/像素）
    
    # 运动阈值
    minimum_travel_distance: 0.3  # 最小移动距离（米）
    minimum_travel_heading: 0.3   # 最小转向角度（弧度）
    
    # 激光参数
    max_laser_range: 20.0         # 最大激光距离（米）
    
    # 回环检测
    do_loop_closing: true         # 是否进行回环检测
    loop_search_maximum_distance: 3.0  # 回环搜索距离
```

### 2. 建图节点参数

```yaml
slam_mapping_node:
  ros__parameters:
    # 自动保存
    auto_save_map: true
    save_interval: 30.0           # 自动保存间隔（秒）
    
    # 地图命名
    map_name_prefix: "agrisage3_map"
```

### 3. 运行时参数调整

```bash
# 修改地图分辨率
ros2 param set /slam_toolbox resolution 0.03

# 调整运动阈值
ros2 param set /slam_toolbox minimum_travel_distance 0.2

# 启用/禁用自动保存
ros2 param set /slam_mapping_node auto_save_map false
```

## 地图文件

### 1. 地图保存位置

地图文件自动保存在：`~/agrisage3_maps/`

### 2. 地图文件格式

- `.yaml` - 地图元数据文件
- `.pgm` - 地图图像文件（PGM格式）

### 3. 地图命名规则

```
agrisage3_map_YYYYMMDD_HHMMSS.yaml
agrisage3_map_YYYYMMDD_HHMMSS.pgm
```

例如：`agrisage3_map_20241201_143022.yaml`

## 与现有系统集成

### 1. 与避障系统协同

SLAM建图可以与现有的激光雷达避障系统同时运行：

```bash
# 启动完整系统（建图+避障+瓶子检测）
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py

# 在另一个终端启动建图
./start_slam_mapping.sh --no-rviz
```

### 2. 话题共享

两个系统共享激光雷达数据：
- `/scan` - 激光雷达原始数据
- 避障系统：实时避障决策
- 建图系统：长期地图构建

## 故障排查

### 1. 激光雷达无数据

```bash
# 检查设备
ls -l /dev/oradar

# 检查驱动节点
ros2 node list | grep laser_driver

# 检查话题
ros2 topic hz /scan
```

### 2. 建图不工作

```bash
# 检查SLAM节点
ros2 node info /slam_toolbox

# 查看错误信息
ros2 node list
ros2 topic echo /rosout

# 检查TF变换
ros2 run tf2_ros tf2_echo map odom
```

### 3. RViz显示问题

```bash
# 重新启动RViz
ros2 run rviz2 rviz2 -d src/bottle_detection_ros2/rviz/slam_mapping.rviz

# 检查话题连接
ros2 topic list | grep map
```

## 性能优化

### 1. 建图性能

- 适当调整`throttle_scans`参数减少计算负载
- 增大`minimum_travel_distance`减少频繁更新
- 在低配置系统上可以降低地图分辨率

### 2. 系统资源

- 使用`--no-rviz`选项节省GPU资源
- 调整`map_update_interval`控制地图更新频率
- 设置合理的`save_interval`避免频繁文件操作

## 扩展功能

### 1. 导航集成

建图完成后，可以集成nav2导航功能：

```bash
# 安装nav2
sudo apt install ros-humble-navigation2

# 使用保存的地图进行导航
ros2 launch nav2_bringup navigation_launch.py \
    map:=~/agrisage3_maps/latest_map.yaml
```

### 2. 多机器人建图

可以扩展为多机器人协同建图：

```bash
# 设置不同的命名空间
ros2 launch bottle_detection_ros2 slam_mapping.launch.py \
    namespace:=robot1
```

## 技术规格

- **ROS2版本**: Humble
- **SLAM算法**: slam_toolbox (Karto SLAM)
- **激光雷达**: MS200 (order-humble驱动)
- **地图格式**: OccupancyGrid (.yaml + .pgm)
- **坐标系**: map -> odom -> base_link -> lidar
- **更新频率**: 20Hz (激光), 5Hz (地图)

## 更新日志

- **v1.0.0** - 初始版本，适配ROS2 Humble
- 基于order-humble激光雷达驱动
- 集成slam_toolbox建图算法
- 添加自定义监控和地图保存功能
- 创建完整的启动脚本和配置文件 