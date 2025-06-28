# 瓶子检测调试图像保存功能

## 功能概述

在 `integrated_bottle_detection_node.py` 中新增了调试图像保存功能，当采摘系统截取瓶子图像时，会自动将相关图像保存到本地文件夹，方便调试和分析。

## 保存目录结构

默认保存位置：`~/bottle_detection_debug_images/`

```
bottle_detection_debug_images/
├── cropped_bottles/        # 截取的瓶子图像
│   ├── bottle_crop_20241216_143022_123_harvest_1734336622123.jpg
│   ├── crop_info_20241216_143022_123_harvest_1734336622123.txt
│   └── ...
├── full_frames/           # 完整的原始图像
│   ├── full_frame_20241216_143022_123_harvest_1734336622123.jpg
│   └── ...
└── annotated_frames/      # 带标注的图像（显示边界框）
    ├── annotated_20241216_143022_123_harvest_1734336622123.jpg
    └── ...
```

## 保存的内容

### 1. 截取的瓶子图像 (`cropped_bottles/`)
- **文件格式**: `bottle_crop_{时间戳}_{请求ID}.jpg`
- **内容**: 从原始图像中截取的瓶子区域
- **用途**: 查看AI识别系统接收到的具体图像

### 2. 完整原始图像 (`full_frames/`)
- **文件格式**: `full_frame_{时间戳}_{请求ID}.jpg`
- **内容**: 截取时刻的完整相机图像
- **用途**: 对比截取前后的效果，分析截取位置是否正确

### 3. 标注图像 (`annotated_frames/`)
- **文件格式**: `annotated_{时间戳}_{请求ID}.jpg`
- **内容**: 在完整图像上标注了边界框和截取信息
- **标注说明**:
  - 红色边界框：原始检测边界框
  - 绿色边界框：实际截取区域（包含边距）
  - 白色文字：详细信息（请求ID、时间戳、尺寸等）

### 4. 截取信息文本文件 (`cropped_bottles/`)
- **文件格式**: `crop_info_{时间戳}_{请求ID}.txt`
- **内容**: 详细的截取参数和坐标信息
- **包含信息**:
  - 请求ID和时间戳
  - 原始图像尺寸
  - 边界框坐标（原始、约束后、最终截取）
  - 边距设置
  - 截取图像尺寸
  - 相关文件列表

## 文件命名规则

- **时间戳格式**: `YYYYMMDD_HHMMSS_mmm` (年月日_时分秒_毫秒)
- **请求ID**: 通常为 `harvest_{时间戳毫秒}`
- **示例**: `bottle_crop_20241216_143022_123_harvest_1734336622123.jpg`

## 触发条件

图像保存功能在以下情况下触发：
1. 机器人处于自动采摘模式
2. 检测到瓶子并到达采摘距离
3. 自动采摘控制器发送瓶子截取请求
4. 检测节点成功处理截取请求

## 配置选项

在 `integrated_bottle_detection_node.py` 中可以配置：

```python
# 是否启用图像保存功能
self.debug_save_images = True  # 设为 False 可禁用保存

# 保存目录路径
self.save_directory = os.path.expanduser("~/bottle_detection_debug_images")

# 截取边距（像素）
margin = 20  # 在 _crop_and_publish_bottle_image 方法中
```

## 调试使用建议

### 1. 验证截取精度
- 查看 `annotated_frames/` 中的标注图像
- 检查绿色截取区域是否正确包含了瓶子
- 确认截取区域不会过大或过小

### 2. 分析AI识别效果
- 查看 `cropped_bottles/` 中的截取图像
- 这些图像就是发送给AI识别系统的内容
- 如果AI识别效果不好，可能需要调整截取参数

### 3. 对比完整场景
- 查看 `full_frames/` 中的完整图像
- 了解截取时的整体环境和瓶子位置
- 分析是否存在干扰物或光照问题

### 4. 查看详细参数
- 查看 `crop_info_*.txt` 文件
- 了解具体的截取坐标和参数
- 用于调试边界框计算逻辑

## 日志信息

启用图像保存后，ROS2日志中会显示：

```
[INFO] [integrated_bottle_detection_node]: 创建调试图像保存目录: /home/user/bottle_detection_debug_images
[INFO] [integrated_bottle_detection_node]: 调试图像保存目录结构创建完成
[INFO] [integrated_bottle_detection_node]: 收到瓶子截取请求，开始处理...
[INFO] [integrated_bottle_detection_node]: 保存完整原始图像: /home/user/bottle_detection_debug_images/full_frames/full_frame_20241216_143022_123_harvest_1734336622123.jpg
[INFO] [integrated_bottle_detection_node]: 保存截取瓶子图像: /home/user/bottle_detection_debug_images/cropped_bottles/bottle_crop_20241216_143022_123_harvest_1734336622123.jpg
[INFO] [integrated_bottle_detection_node]: 保存标注图像: /home/user/bottle_detection_debug_images/annotated_frames/annotated_20241216_143022_123_harvest_1734336622123.jpg
[INFO] [integrated_bottle_detection_node]: 保存截取信息: /home/user/bottle_detection_debug_images/cropped_bottles/crop_info_20241216_143022_123_harvest_1734336622123.txt
[INFO] [integrated_bottle_detection_node]: 调试图像已保存到目录: /home/user/bottle_detection_debug_images
```

## 注意事项

1. **磁盘空间**: 图像文件会占用磁盘空间，定期清理旧文件
2. **性能影响**: 保存操作会轻微影响性能，生产环境可考虑禁用
3. **权限要求**: 确保程序对保存目录有写权限
4. **文件格式**: 所有图像均保存为JPEG格式，质量为80%

## 禁用功能

如需禁用图像保存功能，可以：

1. 修改代码中的 `self.debug_save_images = False`
2. 或者注释掉保存相关的代码块

这样可以在不影响核心功能的情况下停止图像保存。 