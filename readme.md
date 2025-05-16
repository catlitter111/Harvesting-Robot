# 双目视觉目标跟随抓取系统

基于RK3588开发板和RKNN神经网络加速器的双目视觉目标跟随与自动抓取系统。

## 系统功能

- 实时目标检测与跟踪
- 基于双目视觉的深度估计
- 机械臂控制与自动抓取
- 支持多种目标识别（默认跟踪瓶子类别）

## 系统架构

系统采用模块化设计，主要分为以下几个部分：

1. **配置管理**：加载和管理系统配置
2. **相机校准**：处理双目相机的校准参数
3. **串口控制**：负责与机械臂舵机的通信
4. **深度处理**：计算双目图像的深度信息
5. **目标检测**：使用RKNN模型检测目标
6. **跟随控制**：管理目标跟随和抓取逻辑
7. **主程序**：整合各模块并运行主循环

## 目录结构

```
robotarm_follow/
├── config/
│   └── config.yaml       # 配置文件
├── modules/
│   ├── __init__.py       # 包初始化
│   ├── config.py         # 配置管理类
│   ├── uart_controller.py # 串口控制类
│   ├── object_detector.py # 目标检测类
│   ├── depth_processor.py # 深度处理类
│   ├── follow_controller.py # 跟随控制类
│   └── camera_calibration.py # 相机校准类
├── data/
│   └── out.xls           # 双目相机校准数据
├── models/
│   └── yolo11n.rknn      # RKNN模型
├── main.py               # 主程序入口
└── README.md             # 项目说明
```

## 安装依赖

```bash
pip install opencv-python numpy pandas pyyaml pyserial
```

需要额外安装RKNN-Toolkit2，请参考官方文档：
[RKNN-Toolkit2 安装指南](https://github.com/rockchip-linux/rknn-toolkit2)

## 使用方法

1. 连接双目摄像头和机械臂
2. 配置`config.yaml`文件
3. 运行主程序：

```bash
python main.py
```

### 命令行参数

- `--config` - 指定配置文件路径，默认为`config/config.yaml`
- `--show-depth` - 显示深度图

### 键盘控制

- `q` - 退出程序
- `r` - 重置为识别模式（如果抓取过程中出现问题）
- `d` - 切换深度图显示

## 配置文件说明

配置文件采用YAML格式，主要包含以下几个部分：

```yaml
# 摄像头配置
camera:
  id: 21                # 摄像头设备号
  width: 1280           # 摄像头宽度
  height: 480           # 摄像头高度
  fps: 30               # 帧率

# 模型配置
model:
  path: "./models/yolo11n.rknn"  # 模型路径
  size: [640, 640]      # 模型输入尺寸
  obj_thresh: 0.4       # 目标检测置信度阈值
  nms_thresh: 0.6       # 非极大值抑制阈值

# 串口配置
uart:
  device: "/dev/ttyS9"  # 串口设备
  baudrate: 115200      # 波特率
  timeout: 0.1          # 读取超时（秒）

# 控制参数
control:
  center_x: 400         # 屏幕中心X坐标
  center_y: 280         # 屏幕中心Y坐标
  speed: 7.5            # 舵机转动速度
  catch_distance: 35    # 触发抓取的距离（厘米）
```

## 自定义目标跟踪

默认跟踪物体为瓶子（bottle，COCO数据集类别ID为39）。如果需要跟踪其他类别的物体，可以修改`config.py`中的`TARGET_CLASS_ID`变量：

```python
# 在Config类中
self.TARGET_CLASS_ID = 39  # bottle的类别ID
```

COCO数据集类别ID对照表可在`config.py`的`CLASSES`列表中查看。

## 机械臂控制协议

本系统使用的机械臂通过串口通信控制，指令格式为：

```
#<ID>P<角度>T<时间>!
```

- `<ID>` - 舵机ID（000-999）
- `<角度>` - 舵机角度（0000-9999）
- `<时间>` - 运动时间（0000-9999，单位ms）

例如：`#000P1500T2000!` 表示ID为0的舵机在2000ms内运动到1500的位置。

## 日志

系统日志保存在`robot_arm.log`文件中，包含系统运行过程中的重要信息。

## 贡献

欢迎提交问题和改进建议！

## 许可证

MIT License