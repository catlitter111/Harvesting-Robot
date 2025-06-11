#!/bin/bash
# SLAM建图系统启动脚本 - 适配ROS2 Humble
# 使用方法：
#   1. 启动完整系统：./start_slam_mapping.sh
#   2. 禁用RViz：./start_slam_mapping.sh --no-rviz
#   3. 自动开始建图：./start_slam_mapping.sh --auto-mapping

set -e

# 默认参数
ENABLE_RVIZ="true"
AUTO_START_MAPPING="false"
USE_SIM_TIME="false"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --auto-mapping)
            AUTO_START_MAPPING="true"
            shift
            ;;
        --sim-time)
            USE_SIM_TIME="true"
            shift
            ;;
        -h|--help)
            echo "SLAM建图系统启动脚本"
            echo ""
            echo "使用方法: $0 [选项]"
            echo ""
            echo "选项："
            echo "  --no-rviz        禁用RViz2可视化"
            echo "  --auto-mapping   自动开始建图"
            echo "  --sim-time       使用仿真时间"
            echo "  -h, --help       显示此帮助信息"
            echo ""
            echo "示例："
            echo "  $0                         # 启动SLAM系统"
            echo "  $0 --auto-mapping         # 启动并自动开始建图"
            echo "  $0 --no-rviz              # 启动但不显示RViz"
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

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "警告：检测到ROS2发行版为 $ROS_DISTRO，建议使用 humble"
fi

# 检查工作空间
if [ ! -d "$(pwd)/src/bottle_detection_ros2" ]; then
    echo "错误：请在robot_ROS2工作空间根目录下运行此脚本"
    exit 1
fi

echo "=========================================="
echo "启动 SLAM建图系统"
echo "=========================================="
echo "ROS2发行版: $ROS_DISTRO"
echo "RViz可视化: $ENABLE_RVIZ"
echo "自动建图: $AUTO_START_MAPPING"
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

# 检查依赖包
echo "检查ROS2依赖包..."
required_packages=("slam_toolbox" "tf2_ros" "rviz2")
for pkg in "${required_packages[@]}"; do
    if ! ros2 pkg list | grep -q "^$pkg$"; then
        echo "错误：缺少必需的ROS2包: $pkg"
        echo "请安装: sudo apt install ros-humble-$pkg"
        exit 1
    fi
done

# 检查激光雷达设备
if [ ! -e "/dev/oradar" ]; then
    echo "警告：未找到激光雷达设备 /dev/oradar"
    echo "请检查："
    echo "  1. 激光雷达是否正确连接"
    echo "  2. 设备权限是否正确"
    echo "  3. order-humble包是否正确安装"
    echo ""
    read -p "是否继续启动？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "正在启动SLAM建图系统..."
echo "使用 Ctrl+C 停止系统"
echo ""
echo "建图控制命令："
echo "  # 开始建图"
echo "  ros2 service call /slam/start_mapping std_srvs/srv/SetBool \"{data: true}\""
echo ""
echo "  # 停止建图"
echo "  ros2 service call /slam/start_mapping std_srvs/srv/SetBool \"{data: false}\""
echo ""
echo "  # 保存地图"
echo "  ros2 service call /slam/save_map std_srvs/srv/Empty"
echo ""

# 启动ROS2 launch文件
ros2 launch bottle_detection_ros2 slam_mapping.launch.py \
    enable_rviz:=$ENABLE_RVIZ \
    auto_start_mapping:=$AUTO_START_MAPPING \
    use_sim_time:=$USE_SIM_TIME

echo ""
echo "SLAM建图系统已退出" 