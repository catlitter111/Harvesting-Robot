# 机器人控制命令消息
# 用于发送移动控制命令

# 消息头
std_msgs/Header header

# 命令类型 (forward, backward, left, right, stop, emergencyStop)
string command

# 速度 (0.0-1.0)
float32 speed

# 紧急停止标志
bool emergency_stop

# 附加参数（JSON格式）
string params