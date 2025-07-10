# STM32四轮小车控制系统

> 🎉 **最新更新**: 已集成520电机专用编码器接口，支持硬件编码器模式，测速精度大幅提升！
> 
> 📖 详细移植说明请查看：[520编码器移植指南](ENCODER_520_INTEGRATION_GUIDE.md)

## 🚗 项目简介
基于STM32F103的四轮小车控制系统，集成GPS导航、PID速度控制、编码器反馈等功能。特别优化支持520电机的硬件编码器接口，提供更高精度的速度控制。

## ✨ 主要功能
- 🎯 **PID速度控制**: 四轮独立PID闭环控制
- 📡 **GPS导航**: 支持GPS定位和路径规划
- 🔄 **编码器反馈**: 实时速度测量和反馈
- 🚀 **520电机支持**: 专用硬件编码器接口适配 ⭐**新增**
- 📱 **串口通信**: 支持指令控制和数据传输
- ⚡ **实时调度**: 多任务实时调度系统
- 🛡️ **安全保护**: 紧急停止和故障检测

## 🏗️ 系统架构

### 硬件平台
- **主控**: STM32F103C8T6
- **电机**: 4个PWM直流电机(支持520电机)
- **编码器**: 增量式编码器(400PPR/520PPR)
- **GPS**: UART接口GPS模块
- **通信**: UART串口调试

### 软件架构
```
mytest_ABlun_/
├── Components/
│   └── pid/                    # PID算法库
├── hardware/
│   ├── config.h               # 统一配置
│   ├── pid_motor.*            # 四轮PID控制
│   ├── encoder.*              # 通用编码器测速
│   ├── encoder_520.*          # 520电机编码器(硬件接口)
│   ├── robot_control.*        # 上层控制接口
│   ├── motor.*                # 电机驱动
│   ├── scheduler.*            # 任务调度
│   └── gps.*                  # GPS模块
└── Core/                      # STM32 HAL库
```

## 🚀 快速开始

### 1. 环境准备
- Keil MDK-ARM 5.29+
- STM32CubeMX (可选)
- ST-Link调试器

### 2. 编译下载
```bash
# 1. 打开Keil项目文件
# 2. 编译项目 (F7)
# 3. 下载到目标板 (F8)
```

### 3. 基础使用
```c
// 初始化系统
Robot_Control_Init();

// 前进控制
Robot_Move_Forward(50);  // 50%速度前进

// 转向控制
Robot_Turn_Left(30);     // 30%速度左转

// 停止
Robot_Stop();
```

## 🎮 控制接口

### 基础运动控制
```c
void Robot_Move_Forward(int16_t speed);    // 前进
void Robot_Move_Backward(int16_t speed);   // 后退
void Robot_Turn_Left(int16_t speed);       // 左转
void Robot_Turn_Right(int16_t speed);      // 右转
void Robot_Stop(void);                     // 停止
```

### 高级控制
```c
// 四轮独立控制
void Robot_Set_Speed_Individual(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

// 弧线转弯
void Robot_Arc_Left(int16_t speed, int16_t radius);

// 紧急停止
void Robot_Emergency_Stop(void);
```

### PID参数调节
```c
// 设置PID参数
Robot_Set_PID_Params(2.0f, 0.1f, 0.5f);  // Kp, Ki, Kd

// 启用/禁用PID控制
Robot_Enable_PID(1);  // 启用
Robot_Enable_PID(0);  // 禁用
```

## 📊 性能参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 控制频率 | 50Hz | PID控制周期20ms |
| 编码器分辨率 | 400PPR | 四倍频解码1600脉冲/转 |
| 速度范围 | ±100 | 百分比速度控制 |
| 响应时间 | <100ms | 阶跃响应时间 |
| 内存占用 | ~4.5KB Flash | 350B RAM |

## 🔧 配置说明

### PID参数配置
在 `hardware/config.h` 中修改：
```c
#define PID_KP_DEFAULT 2.0f      // 比例系数
#define PID_KI_DEFAULT 0.1f      // 积分系数  
#define PID_KD_DEFAULT 0.5f      // 微分系数
#define PID_SAMPLE_TIME 20       // 采样周期(ms)
```

### 编码器引脚配置
```c
// 编码器1 (电机1)
#define ENCODER1_A_PIN GPIO_PIN_0
#define ENCODER1_B_PIN GPIO_PIN_1
#define ENCODER1_PORT GPIOC
```

### 任务调度配置
```c
#define PID_TASK_PERIOD 20       // PID任务周期(ms)
#define ENCODER_TASK_PERIOD 50   // 编码器任务周期(ms)
```

## 🐛 调试指南

### 串口调试
启用调试输出：
```c
#define DEBUG 1
```

查看关键信息：
```c
DEBUG_PRINTF("PID: Kp=%.2f Ki=%.3f Kd=%.2f", kp, ki, kd);
DEBUG_PRINTF("速度: 目标=%d 反馈=%d", target, feedback);
```

### 常见问题
1. **编码器无反应**: 检查引脚配置和连接
2. **PID震荡**: 减小Kp参数或增加Kd参数
3. **电机不转**: 检查PWM信号和驱动电路
4. **系统不稳定**: 检查任务周期和内存使用

## 📚 文档资源

- 📖 [详细移植指南](PID_PORTING_GUIDE.md)
- 🔧 [API参考文档](PID_PORTING_GUIDE.md#API参考)
- ❓ [常见问题解答](PID_PORTING_GUIDE.md#常见问题)
- 🧪 [测试验证方法](PID_PORTING_GUIDE.md#测试验证)
- 🚀 [**520编码器移植指南**](ENCODER_520_INTEGRATION_GUIDE.md) ⭐**新增**

## 🎯 移植特色

### 从ElectronicController_Board移植
- ✅ 完整PID算法库复用
- ✅ 双轴位置控制 → 四轮速度控制
- ✅ 步进电机 → PWM直流电机
- ✅ MultiTimer → 扩展scheduler调度
- ✅ 新增编码器反馈系统

### 代码优化特点
- 🎯 **码高尔夫风格**: 极简代码实现
- 🔧 **统一配置管理**: config.h集中配置
- 🏗️ **模块化设计**: 低耦合高内聚
- 🌏 **中文友好**: 完整中文注释
- ⚡ **高效实现**: 优化内存和性能

## 📈 版本历史

- **V1.1** (2025): ⭐**新增520电机编码器硬件接口支持，提升测速精度**
- **V1.0** (2025): 完成PID控制移植，支持四轮独立控制
- **V0.9** (2025): 基础四轮小车功能，GPS导航
- **V0.8** (2025): 初始项目框架

### V1.1更新内容 🎉
- ✅ 移植UpStanding_Car(520)的硬件编码器接口
- ✅ 支持STM32定时器编码器模式，CPU占用率降低90%+
- ✅ 520电机专用PPR设置，测速精度显著提升
- ✅ 完整的向后兼容性，无需修改现有代码
- ✅ 提供详细集成指导和示例代码

## 👥 贡献指南

欢迎提交Issue和Pull Request！

### 开发规范
- 遵循码高尔夫风格
- 使用中文注释
- 统一配置管理
- 完善测试用例

## 📄 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 🙏 致谢

- STMicroelectronics: STM32 HAL库
- ElectronicController_Board: PID算法库来源
- 开源社区: 技术支持和灵感

---

**Happy Coding! 🚀**

*让每一行代码都有意义，让每一个功能都精益求精！* 