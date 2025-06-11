# 瓶子检测ROS2项目文件结构说明

## 项目概述
这是一个基于ROS2 Humble的智能农业机器人瓶子检测系统，整合了计算机视觉、机器人控制、远程通信等多个功能模块。

## 重新组织后的文件结构

```
bottle_detection_ros2/
├── __init__.py                    # 包初始化文件
├── README_STRUCTURE.md            # 本说明文档
│
├── nodes/                         # ROS2节点层
│   ├── __init__.py
│   ├── detection/                 # 检测相关节点
│   │   ├── __init__.py
│   │   ├── integrated_bottle_detection_node.py    # 主要的集成检测节点（1061行）
│   │   ├── bottle_detection_node.py               # 基础检测节点（412行）
│   │   └── bottle_detection_node_async.py         # 异步检测节点（507行）
│   │
│   ├── control/                   # 控制相关节点  
│   │   ├── __init__.py
│   │   ├── robot_control_node.py                  # 机器人底盘控制（523行）
│   │   ├── servo_control_node.py                  # 舵机控制（698行）
│   │   └── auto_harvest_controller.py             # 自动采摘控制器（428行）
│   │
│   └── communication/             # 通信相关节点
│       ├── __init__.py
│       └── websocket_bridge_node.py               # WebSocket桥接（644行）
│
├── core/                          # 核心功能模块层
│   ├── __init__.py
│   ├── vision/                    # 视觉处理模块
│   │   ├── __init__.py
│   │   ├── bottle_detector.py                     # RKNN瓶子检测器（386行）
│   │   ├── bottle_detector_async.py               # 异步瓶子检测器（474行）
│   │   └── stereo_camera.py                       # 双目相机处理（438行）
│   │
│   ├── hardware/                  # 硬件接口模块
│   │   ├── __init__.py
│   │   └── laser_obstacle_avoidance.py            # 激光避障（413行）
│   │
│   └── processing/                # 数据处理模块
│       ├── __init__.py
│       └── bottle_rknn_pool.py                    # RKNN模型池管理（181行）
│
├── utils/                         # 工具类
│   ├── __init__.py
│   └── utils.py                                   # 通用工具函数（290行）
│
└── gui/                          # 图形界面
    ├── __init__.py
    └── debug_visualizer_gui.py                    # 调试可视化界面（1002行）
```

## 各模块功能说明

### 🎯 nodes/detection/ - 检测节点
- **integrated_bottle_detection_node.py**: 核心检测节点，整合了异步检测、双目深度估计、视频质量控制等功能
- **bottle_detection_node.py**: 基础的瓶子检测节点，提供基本的检测功能
- **bottle_detection_node_async.py**: 异步版本的检测节点，支持高并发处理

### 🤖 nodes/control/ - 控制节点
- **robot_control_node.py**: 机器人底盘控制，负责移动、转向等基础运动控制
- **servo_control_node.py**: 舵机控制节点，处理机械臂和云台的精确控制
- **auto_harvest_controller.py**: 自动采摘控制器，实现智能采摘逻辑

### 📡 nodes/communication/ - 通信节点
- **websocket_bridge_node.py**: WebSocket桥接节点，提供远程监控和控制接口

### 👁️ core/vision/ - 视觉处理
- **bottle_detector.py**: 基于RKNN的瓶子检测器，使用YOLO模型进行目标检测
- **bottle_detector_async.py**: 异步版本的检测器，支持高效的并行处理
- **stereo_camera.py**: 双目相机处理，提供深度估计和三维定位

### 🔧 core/hardware/ - 硬件接口
- **laser_obstacle_avoidance.py**: 激光雷达避障系统，确保机器人安全导航

### ⚡ core/processing/ - 数据处理
- **bottle_rknn_pool.py**: RKNN模型池管理器，优化AI推理性能

### 🛠️ utils/ - 工具类
- **utils.py**: 通用工具函数，包含滤波、数学计算等辅助功能

### 🖥️ gui/ - 图形界面
- **debug_visualizer_gui.py**: 调试可视化界面，提供实时监控和参数调试

## 架构优势

### 1. 分层架构
- **节点层(nodes/)**: 处理ROS2通信和业务逻辑
- **核心层(core/)**: 封装具体功能实现
- **工具层(utils/)**: 提供通用辅助功能

### 2. 模块化设计
- 检测、控制、通信功能独立
- 便于维护和扩展
- 支持单独测试各个模块

### 3. 清晰的职责分离
- 视觉处理与控制逻辑分离
- 硬件接口与业务逻辑分离
- 通信协议与功能实现分离

## 使用建议

### 启动顺序
1. 首先启动核心检测节点：`integrated_bottle_detection_node.py`
2. 启动控制节点：`robot_control_node.py` 和 `servo_control_node.py`
3. 启动通信节点：`websocket_bridge_node.py`
4. 可选启动调试界面：`debug_visualizer_gui.py`

### 开发建议
- 新功能优先在对应的core模块中实现
- 节点文件主要负责ROS2消息处理和业务流程
- 保持各模块间的低耦合性
- 使用统一的错误处理和日志记录

## 注意事项
1. 所有Python文件都遵循ROS2 Humble标准
2. 导入路径需要根据新结构进行调整
3. launch文件和配置文件可能需要更新路径
4. 建议在修改后进行完整的功能测试

---
*该文档记录了项目重构的详细信息，便于团队成员理解和维护代码结构。* 