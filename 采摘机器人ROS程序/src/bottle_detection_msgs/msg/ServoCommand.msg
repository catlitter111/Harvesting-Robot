# 舵机控制命令消息
# 用于控制单个或多个舵机

# 舵机ID (0-254)
int32 servo_id

# 目标位置 (500-2500)
int32 position

# 移动时间 (毫秒)
int32 time_ms

# 停止舵机
bool stop

# 设置模式
bool set_mode
int32 mode

# 批量控制（可选）
int32[] servo_ids
int32[] positions
int32[] times_ms