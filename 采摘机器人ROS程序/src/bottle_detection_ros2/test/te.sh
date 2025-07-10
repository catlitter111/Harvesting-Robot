#!/bin/bash
# 更新所有Python文件中的消息导入

cd /home/monster/download/AgriSage3/src/bottle_detection_ros2/

# 更新所有Python文件
echo "更新Python文件中的导入语句..."

# 替换消息导入
find bottle_detection_ros2 -name "*.py" -type f -exec sed -i \
    's/from bottle_detection_ros2.msg import/from bottle_detection_msgs.msg import/g' {} +

# 如果有其他形式的导入
find bottle_detection_ros2 -name "*.py" -type f -exec sed -i \
    's/import bottle_detection_ros2.msg/import bottle_detection_msgs.msg/g' {} +

echo "更新完成！"
