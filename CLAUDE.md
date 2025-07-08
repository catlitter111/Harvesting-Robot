# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AgriSage is a comprehensive intelligent agricultural harvesting robot system built on ROS2 Humble. It integrates computer vision, autonomous navigation, robotic control, cloud communication, and AI assistance. The system consists of multiple interconnected components operating across different platforms and technologies.

## System Architecture

The project follows a **five-layer modular architecture**:

1. **Perception Layer**: Stereo cameras (ID: 21), MS200 LiDAR, YOLO11n AI model (4.1MB RKNN)
2. **Decision Layer**: ROS2 Humble framework, SLAM mapping, laser obstacle avoidance, AI dual-model architecture
3. **Communication Layer**: WebSocket bridges, FastAPI server, Huawei IoT platform integration
4. **Execution Layer**: STM32F103C8T6 controller, servo systems, serial communication protocols
5. **Interface Layer**: WeChat miniprogram, RViz2 visualization, cloud management console

## Build and Development Commands

### ROS2 System Build

```bash
# Build the workspace (from robot_ROS2-6_9/)
colcon build --packages-select bottle_detection_msgs bottle_detection_ros2
source install/setup.bash

# Quick rebuild after changes
colcon build --packages-select bottle_detection_ros2 --symlink-install
```

### System Startup

```bash
# Main production system (includes everything)
ros2 launch bottle_detection_ros2 integrated_system.launch.py

# Alternative: System with LiDAR navigation
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py

# Local control only (no WebSocket)
ros2 launch bottle_detection_ros2 local_control_system.launch.py

# SLAM mapping mode
ros2 launch bottle_detection_ros2 slam_mapping.launch.py

# Servo debugging
ros2 launch bottle_detection_ros2 servo_debug.launch.py
```

### Server Components

```bash
# Start FastAPI server (from 上位机与服务端/)
python server.py
# Server runs on: http://172.20.39.181:1234

# Test client connection
python client.py
```

### Development and Testing

```bash
# Monitor system status
ros2 node list
ros2 topic list
ros2 topic echo /bottle_detection/info

# Debug auto mode issues
python3 src/bottle_detection_ros2/scripts/debug_auto_mode.py

# Check communication topics
ros2 topic echo /cmd_vel_raw
ros2 topic echo /robot/status

# Run servo debugging
ros2 run bottle_detection_ros2 servo_debug_node

# View detection images
python3 nodes/detection/view_debug_images.py
```

### Configuration Management

```bash
# View/modify system configuration
cat src/bottle_detection_ros2/config/system_config.yaml

# Check parameter values
ros2 param list
ros2 param get auto_harvest_controller search_timeout
```

## Key Architecture Patterns

### ROS2 Node Communication

The system uses a **layered node architecture** with specialized communication patterns:

- **Detection Nodes**: `integrated_bottle_detection_node` (main), `bottle_detection_node_async` (async processing)
- **Control Nodes**: `auto_harvest_controller` (autonomous), `robot_control_node` (movement), `servo_control_node` (arm)
- **Communication Nodes**: `websocket_bridge_node` (external interface)
- **Navigation Nodes**: `slam_mapping_node` (SLAM), `laser_obstacle_avoidance` (safety)

### Message Flow Architecture

```
Vision Pipeline: Camera → RKNN Detection → Stereo Depth → BottleDetection msg
Control Pipeline: WebSocket Commands → RobotCommand msg → Serial Protocol
Status Pipeline: Hardware Status → RobotStatus msg → WebSocket/IoT Cloud
```

### Core Message Types

- **BottleDetection.msg**: Detection results with 3D position data, confidence, fruit classification
- **RobotCommand.msg**: Movement commands (forward, backward, turn_left, turn_right, stop)
- **ServoCommand.msg**: Mechanical arm control with servo_id, angle, speed parameters
- **HarvestCommand.msg**: High-level harvesting actions (approach, harvest, retreat)

### Communication Architecture

**Dual Communication System**:
1. **Internal ROS2**: High-frequency real-time communication between nodes
2. **External WebSocket**: Remote control via WeChat miniprogram and cloud services

**Protocol Stack**:
- **ROS2 Layer**: QoS-optimized topic/service communication
- **WebSocket Layer**: JSON message protocol for remote control
- **Serial Layer**: STM32 communication with packet checksum verification
- **Cloud Layer**: Huawei IoT platform integration with device management

### Hardware Integration

**STM32 Embedded Controller**:
- **Communication**: UART protocol with packet structure `[0xAA][0x55][CMD][LEN][DATA][CHECKSUM]`
- **Control System**: 4-wheel PID control with encoder feedback (400 PPR encoders)
- **Real-time Performance**: 50Hz control loop, <100ms response time

**AI Model Integration**:
- **RKNN Inference**: Hardware-accelerated inference on NPU
- **Model Files**: `data/yolo11n.rknn` (4.1MB), `data/RDK_yolo11s.bin` (updated model)
- **Performance**: ~30FPS detection, mAP@0.5 > 0.85

### Server Architecture

**FastAPI Multi-Service Server**:
- **WebSocket Endpoints**: `/ws/robot/{robot_id}` (robot connections), `/ws/wechat/{client_id}` (client connections)
- **AI Integration**: Dual-model architecture (doubao-vision for images, doubao-lite for text)
- **IoT Integration**: Huawei Cloud IoT platform with device property reporting
- **Video Management**: Adaptive quality adjustment based on network conditions

### WeChat Miniprogram Integration

**5-Page Application Structure**:
- **Control Center**: Real-time video, movement controls, mode switching
- **Data Statistics**: Work statistics, location tracking, route history  
- **Smart Detection**: Fruit recognition, quality analysis, detection history
- **AI Assistant**: Chat interface with technical support and diagnostics
- **Settings**: System configuration and preferences

**Real-time Communication**:
- **WebSocket Protocol**: Persistent connection with heartbeat mechanism
- **Message Types**: Commands, status updates, video frames, AI responses
- **Network Optimization**: Adaptive video quality, connection recovery

## Development Workflows

### Auto Mode Debugging

The v3.2 system includes comprehensive auto mode fixes:

```bash
# Monitor auto mode operation
python3 scripts/debug_auto_mode.py

# Key parameters to check:
# - Detection range: 15m (expanded from 5m)
# - Confidence threshold: 0.05 (lowered from 0.3)
# - Topic consistency: cmd_vel vs cmd_vel_raw
```

### Vision System Development

**Detection Pipeline**:
1. Stereo camera capture (Camera ID: 21)
2. RKNN YOLO inference with thread pool
3. 3D position calculation using stereo vision
4. Results published to `/bottle_detection` topic

**Key Configuration**:
- **Model Path**: Update `model_path` in launch files for new RKNN models
- **Camera Params**: Modify `config/camera_params.yaml` for calibration
- **Detection Thresholds**: Adjust confidence and distance limits in system config

### Servo Control Development

**High-Frequency Tracking System**:
- **Control Rate**: 50Hz for smooth arm movement
- **Position Range**: Configurable servo limits in system config
- **Debug Mode**: Use `servo_debug_node` for individual servo testing

### Network and Cloud Integration

**Multi-Layer Communication**:
- **Local ROS2**: Internal robot communication
- **WebSocket Bridge**: External interface for remote control
- **Cloud IoT**: Device status reporting and remote management

**Development Tips**:
- Monitor WebSocket connections with browser dev tools
- Check IoT platform logs in Huawei Cloud console
- Use adaptive video manager for bandwidth optimization

## Troubleshooting Common Issues

### Auto Mode Robot Not Moving
```bash
# Check topic publication
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel

# Verify obstacle avoidance
ros2 node info laser_obstacle_avoidance

# Monitor control state
python3 scripts/debug_auto_mode.py
```

### LiDAR Connection Issues
```bash
# Check device permissions
ls -l /dev/oradar
sudo chmod 666 /dev/oradar

# Restart LiDAR node
ros2 lifecycle set /laser_driver configure
ros2 lifecycle set /laser_driver activate
```

### Camera Detection Problems
```bash
# Test camera devices
v4l2-ctl --list-devices
ros2 topic echo /bottle_detection/debug_image

# Verify model file
ls -la data/yolo11n.rknn
ls -la data/RDK_yolo11s.bin
```

### Serial Communication Issues
```bash
# Check STM32 connection
ls -l /dev/ttyS*
sudo chmod 666 /dev/ttyS7

# Monitor serial data
ros2 topic echo /robot/status
```

## Important File Locations

- **Main Launch**: `src/bottle_detection_ros2/launch/integrated_system.launch.py`
- **System Config**: `src/bottle_detection_ros2/config/system_config.yaml`
- **AI Models**: `src/bottle_detection_ros2/data/` (RKNN files)
- **Server**: `上位机与服务端/server.py`
- **WeChat App**: `微信小程序/miniprogram/`
- **STM32 Code**: `stm32/mytest_ABlun_/`
- **Documentation**: Various README files in each subsystem

## System Integration Notes

This is a production agricultural robot system with real hardware dependencies. When developing:

- Always test in safe environments due to physical robot movement
- Monitor battery levels and hardware status during operation
- Use simulation/debug modes when possible for software development
- Coordinate changes across the multi-platform architecture (ROS2, STM32, WeChat, Server)
- Consider network reliability for remote operations
- Maintain proper error handling and recovery mechanisms across all layers