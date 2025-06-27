#!/bin/bash
# -*- coding: utf-8 -*-
"""
本地控制系统启动脚本
用于启动完整的本地控制界面系统
"""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info "=========================================="
print_info "      智慧农业采摘机器人本地控制系统      "
print_info "=========================================="

# 检查ROS2环境
print_info "检查ROS2环境..."
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2环境未设置，请先source ROS2环境"
    print_info "运行: source /opt/ros/humble/setup.bash"
    exit 1
fi
print_success "ROS2环境已设置: $ROS_DISTRO"

# 进入工作空间
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
print_info "进入工作空间: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# 检查是否已编译
if [ ! -d "install" ]; then
    print_warning "工作空间未编译，正在编译..."
    colcon build --packages-select bottle_detection_ros2 bottle_detection_msgs
    if [ $? -ne 0 ]; then
        print_error "编译失败"
        exit 1
    fi
    print_success "编译完成"
fi

# Source工作空间
print_info "加载工作空间环境..."
source install/setup.bash
print_success "工作空间环境已加载"

# 检查设备权限
print_info "检查设备权限..."
if [ ! -c "/dev/ttyS3" ]; then
    print_warning "机器人控制串口 /dev/ttyS3 不存在或无权限"
fi

if [ ! -c "/dev/ttyS9" ]; then
    print_warning "舵机控制串口 /dev/ttyS9 不存在或无权限"
fi

# 检查相机设备
print_info "检查相机设备..."
if [ ! -c "/dev/video21" ]; then
    print_warning "相机设备 /dev/video21 不存在，可能需要调整camera_id参数"
fi

# 启动系统
print_info "启动本地控制系统..."
print_info "界面将在几秒钟后显示..."

# 启动launch文件
ros2 launch bottle_detection_ros2 local_control_system.launch.py \
    camera_id:=21 \
    model_path:="/home/elf/Downloads/Harvesting-Robot/robot_ROS2-6_9/src/bottle_detection_ros2/data/yolo11n.rknn" \
    robot_serial_port:="/dev/ttyS3" \
    servo_serial_port:="/dev/ttyS9" \
    show_display:=true

print_info "系统已停止" 