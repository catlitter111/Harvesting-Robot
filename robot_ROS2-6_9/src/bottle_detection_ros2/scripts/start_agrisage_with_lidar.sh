#!/bin/bash
# AgriSage3 系统启动脚本（包含激光雷达避障）
# 使用方法：
#   1. 启动完整系统：./start_agrisage_with_lidar.sh
#   2. 禁用避障：./start_agrisage_with_lidar.sh --no-avoidance
#   3. 禁用显示：./start_agrisage_with_lidar.sh --no-display

set -e

# 默认参数
ENABLE_AVOIDANCE="true"
ENABLE_DISPLAY="true"
USE_SIM_TIME="false"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-avoidance)
            ENABLE_AVOIDANCE="false"
            shift
            ;;
        --no-display)
            ENABLE_DISPLAY="false"
            shift
            ;;
        --sim-time)
            USE_SIM_TIME="true"
            shift
            ;;
        -h|--help)
            echo "AgriSage3 系统启动脚本（包含激光雷达避障）"
            echo ""
            echo "使用方法: $0 [选项]"
            echo ""
            echo "选项："
            echo "  --no-avoidance    禁用激光雷达避障功能"
            echo "  --no-display      禁用摄像头显示窗口"
            echo "  --sim-time        使用仿真时间"
            echo "  -h, --help        显示此帮助信息"
            echo ""
            echo "示例："
            echo "  $0                         # 启动完整系统"
            echo "  $0 --no-avoidance         # 启动系统但禁用避障"
            echo "  $0 --no-display          # 启动系统但不显示摄像头窗口"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误：未检测到ROS2环境，请先source ROS2 setup脚本"
    exit 1
fi

# 检查AgriSage3工作空间
if [ ! -d "$(pwd)/src/bottle_detection_ros2" ]; then
    echo "错误：请在AgriSage3工作空间根目录下运行此脚本"
    exit 1
fi

echo "=========================================="
echo "启动 AgriSage3 系统（包含激光雷达避障）"
echo "=========================================="
echo "ROS2发行版: $ROS_DISTRO"
echo "避障功能: $ENABLE_AVOIDANCE"
echo "显示窗口: $ENABLE_DISPLAY"
echo "仿真时间: $USE_SIM_TIME"
echo "=========================================="

# 构建工作空间（如果需要）
echo "检查编译状态..."
if [ ! -d "build" ] || [ ! -d "install" ]; then
    echo "首次运行，开始编译工作空间..."
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo "编译失败，请检查代码"
        exit 1
    fi
fi

# Source工作空间
echo "加载工作空间环境..."
source install/setup.bash

# 检查激光雷达设备
if [ "$ENABLE_AVOIDANCE" = "true" ]; then
    if [ ! -e "/dev/oradar" ]; then
        echo "警告：未找到激光雷达设备 /dev/oradar，避障功能可能无法正常工作"
        echo "请检查："
        echo "  1. 激光雷达是否正确连接"
        echo "  2. 设备权限是否正确"
        echo "  3. 设备驱动是否安装"
        echo ""
        read -p "是否继续启动？(y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# 检查串口设备
echo "检查串口设备..."
for device in "/dev/ttyS8" "/dev/ttyS9"; do
    if [ ! -e "$device" ]; then
        echo "警告：未找到串口设备 $device"
    fi
done

echo ""
echo "正在启动系统节点..."
echo "使用 Ctrl+C 停止系统"
echo ""

# 启动ROS2 launch文件
ros2 launch bottle_detection_ros2 agrisage_with_lidar.launch.py \
    enable_avoidance:=$ENABLE_AVOIDANCE \
    enable_display:=$ENABLE_DISPLAY \
    use_sim_time:=$USE_SIM_TIME

echo ""
echo "AgriSage3 系统已退出" 