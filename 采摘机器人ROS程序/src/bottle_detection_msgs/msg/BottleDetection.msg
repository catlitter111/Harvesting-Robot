# 瓶子检测消息定义
# 包含检测到的瓶子信息，特别是最近瓶子的距离和位置

# 消息头，包含时间戳和坐标系信息
std_msgs/Header header

# 是否检测到瓶子
bool bottle_detected

# 检测到的瓶子总数
int32 bottle_count

# 最近瓶子的信息
# 像素坐标（图像坐标系）
int32 nearest_bottle_x      # 瓶子中心x坐标（像素）
int32 nearest_bottle_y      # 瓶子中心y坐标（像素）

# 边界框信息
int32 bbox_left            # 边界框左边界
int32 bbox_top             # 边界框上边界  
int32 bbox_right           # 边界框右边界
int32 bbox_bottom          # 边界框下边界

# 检测置信度
float32 confidence         # 检测置信度（0-1）

# 3D信息
float32 distance           # 瓶子距离相机的距离（米）
float32 position_x         # 瓶子在相机坐标系中的x位置（米）
float32 position_y         # 瓶子在相机坐标系中的y位置（米）
float32 position_z         # 瓶子在相机坐标系中的z位置（米）

# 图像信息
int32 image_width          # 原始图像宽度
int32 image_height         # 原始图像高度

# 状态信息
string status              # 状态描述（如："正常"，"距离过近"，"距离过远"等）