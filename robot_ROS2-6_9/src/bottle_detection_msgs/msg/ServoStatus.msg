# 舵机状态消息
# 发布舵机的当前状态

# 消息头
std_msgs/Header header

# 舵机位置数组（最多6个舵机）
int32[] servo_positions

# 舵机温度（可选）
float32[] servo_temperatures

# 舵机负载（可选）
float32[] servo_loads

# 采摘状态
int32 harvest_state  # 0=空闲, 1=开始, 2-5=步骤1-4, 6=完成
int32 harvested_count

# 跟踪状态
bool tracking_active
float32 tracking_error_x
float32 tracking_error_y