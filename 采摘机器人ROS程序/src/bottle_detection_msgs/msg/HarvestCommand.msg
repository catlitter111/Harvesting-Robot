# 采摘控制命令消息
# 用于控制机械臂执行采摘动作

# 消息头
std_msgs/Header header

# 开始采摘
bool start_harvest

# 停止采摘
bool stop_harvest

# 采摘位置（可选）
geometry_msgs/Point target_position

# 采摘参数（预留）
float32 approach_speed
float32 grasp_force