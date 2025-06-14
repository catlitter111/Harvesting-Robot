# 采摘图像消息
# 用于传输采摘时截取的水果图像

# 消息头
std_msgs/Header header

# 采摘会话ID（用于关联采摘动作）
string harvest_session_id

# 检测到的物品信息
string item_type              # 物品类型（如：apple, bottle等）
float32 confidence           # 检测置信度
geometry_msgs/Point position # 物品在图像中的中心位置
float32 distance            # 物品距离（米）

# 截取的图像数据
sensor_msgs/CompressedImage cropped_image  # 截取的物品图像
sensor_msgs/CompressedImage full_image     # 完整的原始图像（可选，用于调试）

# 截取区域信息
int32 crop_x                # 截取区域左上角X坐标
int32 crop_y                # 截取区域左上角Y坐标
int32 crop_width            # 截取区域宽度
int32 crop_height           # 截取区域高度

# 采摘状态
string harvest_status       # 采摘状态：approaching, gripping, confirming, completed
int64 timestamp             # 时间戳（毫秒）

# 额外信息
string robot_id            # 机器人ID
string location_info       # 位置信息 