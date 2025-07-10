#!/bin/bash
# -*- coding: utf-8 -*-
"""
启动本地控制GUI的脚本
使用方法: ./start_local_control_gui.sh
"""

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    智慧农业采摘系统本地控制界面    ${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未配置！${NC}"
    echo -e "${YELLOW}请先运行: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}ROS2环境已配置: $ROS_DISTRO${NC}"

# 检查当前目录
CURRENT_DIR=$(pwd)
WORKSPACE_DIR="${CURRENT_DIR}/../../.."

# 检查工作空间
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo -e "${YELLOW}警告: 工作空间未编译或路径不正确${NC}"
    echo -e "${YELLOW}请先编译工作空间: colcon build${NC}"
    echo -e "${YELLOW}当前路径: $CURRENT_DIR${NC}"
    echo -e "${YELLOW}期望工作空间路径: $WORKSPACE_DIR${NC}"
    read -p "是否继续启动? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}加载工作空间环境...${NC}"
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 检查显示环境
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}警告: DISPLAY环境变量未设置${NC}"
    echo -e "${YELLOW}GUI可能无法正常显示${NC}"
fi

# 检查所需的Python包
echo -e "${BLUE}检查依赖包...${NC}"
python3 -c "import tkinter, PIL, cv2, numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}错误: 缺少必要的Python包${NC}"
    echo -e "${YELLOW}请安装以下包:${NC}"
    echo -e "${YELLOW}  sudo apt install python3-tk${NC}"
    echo -e "${YELLOW}  pip3 install pillow opencv-python numpy${NC}"
    exit 1
fi

echo -e "${GREEN}依赖包检查完成${NC}"

# 启动选项
echo -e "${BLUE}选择启动方式:${NC}"
echo -e "${YELLOW}1) 仅启动GUI界面${NC}"
echo -e "${YELLOW}2) 启动GUI + 基础系统${NC}"
echo -e "${YELLOW}3) 使用launch文件启动${NC}"

read -p "请选择 (1-3, 默认1): " CHOICE
CHOICE=${CHOICE:-1}

case $CHOICE in
    1)
        echo -e "${GREEN}启动本地控制GUI界面...${NC}"
        ros2 run bottle_detection_ros2 local_control_gui
        ;;
    2)
        echo -e "${GREEN}启动GUI和基础系统...${NC}"
        # 在后台启动基础系统
        echo -e "${BLUE}启动检测系统...${NC}"
        ros2 launch bottle_detection_ros2 bottle_detection.launch.py &
        DETECTION_PID=$!
        
        # 等待一下让系统启动
        sleep 3
        
        # 启动GUI
        echo -e "${BLUE}启动控制界面...${NC}"
        ros2 run bottle_detection_ros2 local_control_gui
        
        # 清理后台进程
        echo -e "${YELLOW}清理后台进程...${NC}"
        kill $DETECTION_PID 2>/dev/null
        ;;
    3)
        echo -e "${GREEN}使用launch文件启动...${NC}"
        ros2 launch bottle_detection_ros2 local_control_gui.launch.py
        ;;
    *)
        echo -e "${RED}无效选择，退出${NC}"
        exit 1
        ;;
esac

echo -e "${BLUE}程序已退出${NC}" 