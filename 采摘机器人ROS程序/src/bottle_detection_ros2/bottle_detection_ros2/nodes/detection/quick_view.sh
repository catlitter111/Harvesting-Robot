#!/bin/bash

# 瓶子检测调试图像快速查看脚本

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIEWER_SCRIPT="$SCRIPT_DIR/view_debug_images.py"

echo "瓶子检测调试图像快速查看工具"
echo "================================"

# 检查Python脚本是否存在
if [ ! -f "$VIEWER_SCRIPT" ]; then
    echo "错误: 查看器脚本不存在: $VIEWER_SCRIPT"
    exit 1
fi

# 检查调试目录是否存在
DEBUG_DIR="$HOME/bottle_detection_debug_images"
if [ ! -d "$DEBUG_DIR" ]; then
    echo "调试目录不存在: $DEBUG_DIR"
    echo "请先运行瓶子检测节点并进行一次采摘操作以生成调试图像"
    exit 1
fi

# 显示目录信息
echo "调试目录: $DEBUG_DIR"
echo "目录内容:"
ls -la "$DEBUG_DIR"

echo ""
echo "可用选项:"
echo "1. 查看最新的图像集"
echo "2. 查看最近10个图像集"
echo "3. 查看所有图像集"
echo "4. 列出所有图像集（不显示图像）"
echo "5. 自定义参数"
echo "6. 退出"

read -p "请选择 (1-6): " choice

case $choice in
    1)
        echo "查看最新的图像集..."
        python3 "$VIEWER_SCRIPT" --latest
        ;;
    2)
        echo "查看最近10个图像集..."
        python3 "$VIEWER_SCRIPT" --count 10
        ;;
    3)
        echo "查看所有图像集..."
        python3 "$VIEWER_SCRIPT" --count 100
        ;;
    4)
        echo "列出所有图像集..."
        python3 -c "
import sys
sys.path.append('$SCRIPT_DIR')
from view_debug_images import list_command
list_command()
"
        ;;
    5)
        echo "可用参数:"
        echo "  --latest          只显示最新的图像集"
        echo "  --count N         显示N个图像集"
        echo "  --no-details      不显示详细信息"
        echo "  --dir PATH        指定调试目录"
        echo ""
        read -p "请输入参数: " params
        python3 "$VIEWER_SCRIPT" $params
        ;;
    6)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo ""
echo "查看完成" 