# 机器人状态消息
# 定期发布机器人的各种状态信息

# 消息头
std_msgs/Header header

# 电池电量 (0-100)
float32 battery_level

# CPU使用率 (0-100)
float32 cpu_usage

# 当前速度 (m/s) - 改为float32以支持更精确的速度值
float32 current_speed

# 当前方向 (弧度) - 改为float32以支持连续的方向值
float32 current_direction

# 位置信息
float64 position_x
float64 position_y
float64 latitude
float64 longitude

# 采摘统计
int32 harvested_count      # 本次运行的采摘数量
int32 today_harvested      # 今日采摘总数
int32 total_harvested      # 历史总采摘数

# 工作统计
float32 working_hours      # 工作时长（小时）
float32 working_area       # 作业面积（亩）

# 状态标志
bool emergency_stop
bool is_moving
bool is_harvesting

# 网络信息
int32 signal_strength
float32 upload_bandwidth

# 状态描述
string status_text

# 额外添加的字段（可选）
float32 harvest_accuracy   # 采摘准确率 (0-100)
float32 temperature        # 设备温度
string location_name       # 当前位置名称