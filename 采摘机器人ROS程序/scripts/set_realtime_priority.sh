#!/bin/bash
# 设置舵机控制节点的实时调度优先级
# 使用前需要以root权限运行或配置相应权限

echo "设置ROS2舵机控制节点的实时优先级..."

# 找到舵机控制节点进程
SERVO_PID=$(pgrep -f "servo_control_node")

if [ -n "$SERVO_PID" ]; then
    echo "找到舵机控制节点进程 PID: $SERVO_PID"
    
    # 设置FIFO调度策略，优先级50（较高）
    chrt -f -p 50 $SERVO_PID
    if [ $? -eq 0 ]; then
        echo "成功设置舵机控制节点为实时调度，优先级50"
    else
        echo "设置实时调度失败，可能需要root权限"
    fi
    
    # 设置CPU亲和性到核心1（避免与其他进程竞争）
    taskset -cp 1 $SERVO_PID
    if [ $? -eq 0 ]; then
        echo "成功绑定舵机控制节点到CPU核心1"
    fi
else
    echo "未找到舵机控制节点进程"
fi

# 调整系统网络缓冲区大小以降低延迟
echo 'net.core.rmem_default = 262144' >> /etc/sysctl.conf
echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.core.wmem_default = 262144' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' >> /etc/sysctl.conf
sysctl -p

echo "实时优化设置完成" 