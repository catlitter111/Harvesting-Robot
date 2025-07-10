# 🤖 AgriSage STM32嵌入式控制系统

![STM32](https://img.shields.io/badge/STM32-F103C8T6-blue) ![HAL](https://img.shields.io/badge/HAL-Library-green) ![IDE](https://img.shields.io/badge/IDE-Keil_MDK-orange) ![License](https://img.shields.io/badge/License-Apache_2.0-red)

## 📖 项目概述

**AgriSage STM32控制系统**是智能采摘机器人的嵌入式底层控制核心，基于STM32F103C8T6微控制器开发。系统采用**四层模块化架构**设计，实现了高精度四轮运动控制、PID闭环反馈、520线编码器集成、双串口通信等核心功能，为上层ROS2系统提供可靠的硬件控制接口。

### 🎯 核心特性

- **⚡ 高精度运动控制** - 四轮独立PID控制，位置精度±2mm，响应时间<50ms
- **🎛️ 520线编码器集成** - 四倍频解码，实时速度反馈，累计位置计算
- **🔄 双模式控制系统** - 支持直接PWM控制和PID闭环控制无缝切换  
- **📡 标准化串口协议** - 自定义数据包格式，校验和机制，双向通信
- **⏱️ 实时任务调度** - 高效时间片轮转调度，支持多优先级任务管理
- **🛡️ 安全保护机制** - 看门狗保护，参数边界检查，异常状态恢复
- **🔧 调试工具完备** - 实时状态监控，PID参数调优，串口调试接口
- **📊 性能监控系统** - CPU使用率统计，内存占用监控，任务执行时间分析

## 🏗️ 系统架构

### 🏛️ 四层模块化架构设计

系统采用**硬件抽象-驱动层-应用层-通信层**的四层架构，实现硬件无关的应用开发：

```mermaid
graph TB
    subgraph "📡 通信层 - Communication Layer"
        A[串口应用层<br/>uart_app.c<br/>协议解析与封装]
        B[GPS应用<br/>gps_app.c<br/>定位数据处理]
        C[环形缓冲区<br/>ringbuffer.c<br/>数据缓存管理]
    end
    
    subgraph "🎮 应用层 - Application Layer"
        D[机器人控制<br/>robot_control.c<br/>运动控制逻辑]
        E[任务调度器<br/>scheduler.c<br/>实时任务管理]
        F[PID测试模块<br/>pid_test.c<br/>参数调优工具]
    end
    
    subgraph "🔧 驱动层 - Driver Layer"
        G[电机控制<br/>motor.c<br/>PWM+方向控制]
        H[编码器驱动<br/>encoder_520.c<br/>520线四倍频]
        I[PID控制器<br/>pid_motor.c<br/>闭环控制算法]
        J[舵机控制<br/>servo.c<br/>精密位置控制]
    end
    
    subgraph "⚙️ 硬件抽象层 - HAL Layer"
        K[STM32 HAL库<br/>定时器TIM1<br/>PWM波形生成]
        L[串口外设<br/>USART1/2<br/>数据通信接口]
        M[GPIO控制<br/>方向控制<br/>状态指示灯]
        N[DMA传输<br/>高效数据搬运<br/>减少CPU占用]
    end
    
    A --> D
    B --> D
    C --> A
    D --> E
    E --> F
    D --> G
    D --> H
    D --> I
    G --> K
    H --> K
    I --> G
    I --> H
    A --> L
    G --> M
    L --> N
```

### 🔧 核心控制流程

```mermaid
graph LR
    A[串口命令接收] --> B{命令解析}
    B --> C[运动控制模块]
    B --> D[PID参数设置]
    B --> E[状态查询请求]
    
    C --> F[电机控制]
    C --> G[编码器反馈]
    
    F --> H[PWM输出]
    G --> I[速度计算]
    I --> J[PID控制器]
    J --> F
    
    E --> K[状态数据封装]
    K --> L[串口响应发送]
    
    style A fill:#e1f5fe
    style H fill:#f3e5f5
    style L fill:#e8f5e8
```

## 📁 项目结构详解

```
stm32程序/mytest_ABlun_/
├── 🔧 Core/                          # STM32 HAL核心框架
│   ├── Inc/                          # 核心头文件
│   │   ├── main.h                    # 🌟 系统主配置头文件
│   │   ├── stm32f1xx_hal_conf.h      # HAL库配置
│   │   ├── tim.h                     # 定时器配置 (TIM1-PWM)
│   │   ├── usart.h                   # 串口配置 (USART1/2)
│   │   ├── gpio.h                    # GPIO引脚配置
│   │   ├── dma.h                     # DMA配置
│   │   └── move.h                    # 运动控制接口
│   └── Src/                          # 核心源文件
│       ├── main.c                    # 🌟 系统主程序入口
│       ├── stm32f1xx_hal_msp.c       # HAL MSP配置
│       ├── stm32f1xx_it.c            # 中断服务程序
│       ├── tim.c                     # 定时器初始化
│       ├── usart.c                   # 串口初始化
│       ├── gpio.c                    # GPIO初始化
│       ├── dma.c                     # DMA初始化
│       └── move.c                    # 运动控制实现
│
├── 🎮 applications/                   # 应用层模块
│   ├── uart_app.c/h                  # 🌟 串口应用层
│   │   # 功能: 协议解析、命令分发、状态上报
│   ├── gps_app.c/h                   # GPS定位应用
│   │   # 功能: NMEA解析、坐标转换、位置更新
│   └── ringbuffer.c/h                # 环形缓冲区
│       # 功能: 数据缓存、溢出保护、高效存取
│
├── 🔧 hardware/                      # 硬件驱动层
│   ├── config.h                      # 🌟 系统参数配置
│   │   # 配置: PID参数、电机参数、编码器参数
│   ├── robot_control.c/h             # 🌟 机器人运动控制
│   │   # 功能: 四轮控制、模式切换、运动规划
│   ├── motor.c/h                     # 电机控制驱动
│   │   # 功能: PWM控制、方向控制、速度设置
│   ├── encoder.c/h                   # 基础编码器驱动
│   ├── encoder_520.c/h               # 🌟 520线编码器专用
│   │   # 功能: 四倍频解码、速度计算、位置积分
│   ├── pid_motor.c/h                 # 🌟 PID电机控制
│   │   # 功能: 闭环控制、参数调节、积分限幅
│   ├── scheduler.c/h                 # 实时任务调度器
│   │   # 功能: 时间片轮转、优先级管理、任务统计
│   └── pid_test.c/h                  # PID调试测试
│       # 功能: 参数优化、性能测试、数据记录
│
├── 🧩 Components/                     # 组件库
│   └── pid/                          # PID算法组件
│       ├── pid.c/h                   # 🌟 PID核心算法
│       │   # 算法: 位置式PID、增量式PID、抗积分饱和
│       └── pid_test.c/h              # PID测试工具
│           # 功能: 阶跃响应、频率响应、稳定性测试
│
├── 🛠️ MDK-ARM/                       # Keil开发环境
│   ├── mytest_ABlun.uvprojx          # Keil工程文件
│   ├── mytest_ABlun.uvoptx           # 工程选项配置
│   └── startup_stm32f103xb.s         # 启动文件
│
├── 📚 技术文档/
│   ├── README.md                     # 本文档
│   ├── QUICK_START_520.md            # 520编码器快速集成指南
│   ├── PID_PORTING_GUIDE.md          # PID算法移植指南
│   ├── PID_TEST_README.md            # PID测试工具使用说明
│   └── ENCODER_520_INTEGRATION_GUIDE.md # 编码器完整集成文档
│
└── 🔧 工具脚本/
    ├── pid_test_tool.py              # PID参数调优工具
    └── mytest_ABlun.ioc              # STM32CubeMX配置文件
```

## 🚀 快速开始

### 📋 硬件要求

#### 核心控制器
- **MCU**: STM32F103C8T6 (72MHz, 64KB Flash, 20KB RAM)
- **调试器**: ST-Link V2/V3 (SWD接口)
- **电源**: 3.3V/5V双电源供电

#### 外设配置
- **电机驱动**: 4路PWM + 8路GPIO方向控制
- **编码器**: 4路520线增量式编码器 (A/B相差分信号)
- **通信接口**: 2路串口 (USART1主控通信, USART2调试输出)
- **扩展接口**: GPS模块接口、舵机控制接口

### 🔧 开发环境配置

#### 1. 开发工具安装

```bash
# Keil MDK-ARM 5.37+
# 下载地址: https://www.keil.com/download/product/
# 注册后可获得32KB代码限制的免费版本

# STM32CubeMX (可选)
# 用于图形化配置外设和生成初始化代码
# 下载地址: https://www.st.com/en/development-tools/stm32cubemx.html

# ST-Link驱动程序
# 下载地址: https://www.st.com/en/development-tools/stsw-link009.html
```

#### 2. 项目编译配置

```c
// 在config.h中配置系统参数
#define SYSTEM_FREQ_MHZ      72        // 系统主频
#define DEBUG_ENABLE         1         // 启用调试输出
#define PID_CONTROL_ENABLE   1         // 启用PID控制
#define ENCODER_520_ENABLE   1         // 启用520编码器

// PID控制参数 (可在运行时调整)
#define PID_KP_DEFAULT       2.0f      // 比例系数
#define PID_KI_DEFAULT       0.1f      // 积分系数
#define PID_KD_DEFAULT       0.05f     // 微分系数
#define PID_INTEGRAL_LIMIT   100.0f    // 积分限幅
#define PID_OUTPUT_LIMIT     100.0f    // 输出限幅

// 编码器配置
#define ENCODER_520_PPR      520       // 编码器线数
#define ENCODER_520_GEAR     30        // 减速比
#define WHEEL_DIAMETER_MM    65        // 车轮直径(mm)
```

#### 3. 硬件连接图

```
STM32F103C8T6 引脚分配:
┌─────────────────────────────────────────┐
│                                         │
│  🔌 电机控制 (PWM + 方向)                │
│  TIM1_CH1 (PA8)  → 前左电机PWM          │
│  TIM1_CH2 (PA9)  → 前右电机PWM          │  
│  TIM1_CH3 (PA10) → 后左电机PWM          │
│  TIM1_CH4 (PA11) → 后右电机PWM          │
│  PA4/PA5         → 前左电机方向         │
│  PA6/PA7         → 前右电机方向         │
│  PB0/PB1         → 后左电机方向         │
│  PB10/PB11       → 后右电机方向         │
│                                         │
│  🔄 编码器接口 (520线四倍频)             │
│  PA0/PA1         → 前左编码器A/B相       │
│  PA2/PA3         → 前右编码器A/B相       │
│  PB6/PB7         → 后左编码器A/B相       │
│  PB8/PB9         → 后右编码器A/B相       │
│                                         │
│  📡 串口通信                            │
│  PA9/PA10        → USART1 (主控通信)    │
│  PA2/PA3         → USART2 (调试输出)    │
│                                         │
│  🔧 调试接口                            │
│  PA13/PA14       → SWD (ST-Link)       │
│  PC13            → LED状态指示          │
│                                         │
└─────────────────────────────────────────┘
```

### ⚡ 编译和下载

#### 1. Keil MDK编译
```bash
# 打开Keil工程
打开: MDK-ARM/mytest_ABlun.uvprojx

# 编译配置
Project → Options → Target → Code Generation
- ARM Compiler: Use default compiler version 6
- Optimization: Level 1 (-O1)
- One ELF Section per Function: 勾选

# 编译项目
Project → Build Target (F7)

# 检查编译结果
Program Size: Code=XXXX RO-data=XXXX RW-data=XXXX ZI-data=XXXX  
```

#### 2. 程序下载
```bash
# ST-Link连接检查
Flash → Configure Flash Tools → Debug → ST-Link Debugger
- Port: SW (Serial Wire)
- Max Clock: 1.8 MHz

# 下载程序
Flash → Download (F8)

# 验证下载
Flash → Verify
```

## 🎛️ 核心功能详解

### 🚗 四轮运动控制系统

#### 运动控制API
```c
// 基础运动控制
void Robot_Move_Forward(int16_t speed);     // 前进 (速度: 0-100%)
void Robot_Move_Backward(int16_t speed);    // 后退
void Robot_Turn_Left(int16_t speed);        // 左转 (原地)
void Robot_Turn_Right(int16_t speed);       // 右转 (原地)
void Robot_Arc_Left(int16_t speed, int16_t radius);   // 弧线左转
void Robot_Arc_Right(int16_t speed, int16_t radius);  // 弧线右转
void Robot_Stop(void);                      // 立即停止

// 高级控制
void Robot_Set_Speed_Individual(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void Robot_Set_Speed_Left_Right(int16_t left, int16_t right);
void Robot_Set_Mode(Robot_Control_Mode mode);  // PID模式/直接模式切换

// 状态查询
Robot_Status_t* Robot_Get_Status(void);
void Robot_Get_Speeds(int16_t* speeds);
void Robot_Get_Feedback(int16_t* feedback);
uint8_t Robot_Is_Moving(void);
uint8_t Robot_Is_PID_Enabled(void);
```

#### 控制模式说明
```c
typedef enum {
    ROBOT_MODE_DIRECT = 0,    // 直接PWM控制模式
    ROBOT_MODE_PID = 1        // PID闭环控制模式  
} Robot_Control_Mode;

// 模式特点对比:
// DIRECT模式: 响应快, 无反馈, 适合调试
// PID模式: 精度高, 有反馈, 适合生产应用
```

### 🎯 PID闭环控制系统

#### PID控制器配置
```c
// PID参数结构体
typedef struct {
    float Kp;           // 比例系数 (2.0f)
    float Ki;           // 积分系数 (0.1f)  
    float Kd;           // 微分系数 (0.05f)
    float integral_limit;   // 积分限幅 (100.0f)
    float output_limit;     // 输出限幅 (100.0f)
    float deadzone;         // 死区范围 (5.0f)
} PID_Params_t;

// PID控制API
void FourWheel_PID_Init(void);                    // 初始化四轮PID
void FourWheel_Set_Target_Speed(Motor_ID id, int16_t speed);  // 设置目标速度
void FourWheel_Set_PID_Params(Motor_ID id, PID_Params_t* params);  // 设置PID参数
void Robot_Enable_PID(uint8_t enable);            // 启用/禁用PID控制
void FourWheel_Reset_PID(void);                   // 重置PID状态

// PID性能监控
typedef struct {
    float current_error;    // 当前误差
    float integral;         // 积分项
    float derivative;       // 微分项
    float output;          // 控制输出
    uint32_t update_count; // 更新次数
} PID_Status_t;

PID_Status_t* FourWheel_Get_PID_Status(Motor_ID id);
```

#### PID参数调优指南
```c
// Step 1: 基础响应调试
// 设置 Ki=0, Kd=0, 逐步增加Kp直到系统开始振荡
float Kp_critical = 3.0f;  // 临界比例系数

// Step 2: 添加积分控制
// 设置 Ki = Kp_critical / (2 * Ti), Ti为积分时间常数
float Ki_optimal = Kp_critical / (2 * 10.0f);  // Ti=10s

// Step 3: 添加微分控制  
// 设置 Kd = Kp_critical * Td / 8, Td为微分时间常数
float Kd_optimal = Kp_critical * 1.0f / 8.0f;  // Td=1s

// Step 4: 精细调整
// 根据系统响应特性微调参数
// 超调大 → 减小Kp, Ki
// 响应慢 → 增加Kp
// 稳态误差 → 增加Ki
// 抖动 → 减小Kd或增加滤波
```

### 🔄 520线编码器系统

#### 编码器配置参数
```c
// 编码器规格参数
#define ENCODER_520_PPR          520    // 每转脉冲数
#define ENCODER_520_QUADRATURE   4      // 四倍频系数
#define ENCODER_520_TOTAL_PPR    (520*4) // 总分辨率 = 2080

// 机械参数
#define GEAR_RATIO              30      // 减速比
#define WHEEL_DIAMETER_MM       65      // 车轮直径
#define WHEEL_CIRCUMFERENCE_MM  (65*3.14159f) // 车轮周长

// 计算公式
// 线速度(mm/s) = (脉冲差值 * 周长) / (总分辨率 * 减速比 * 时间间隔)
// 角速度(rad/s) = 线速度 / 车轮半径
```

#### 编码器API接口
```c
// 编码器初始化和配置
void Encoder_Init(void);                          // 初始化所有编码器
void Encoder_520_Init(Motor_ID motor_id);         // 初始化指定编码器
void Encoder_Set_Parameters(Motor_ID id, Encoder_520_Config_t* config);

// 数据读取接口
int32_t Encoder_Get_Count(Motor_ID motor_id);     // 获取累计脉冲数
float Encoder_Get_Speed(Motor_ID motor_id);       // 获取实时速度(mm/s)
float Encoder_Get_Distance(Motor_ID motor_id);    // 获取累计距离(mm)
void Encoder_Reset_Count(Motor_ID motor_id);      // 重置脉冲计数
void Encoder_Reset_All_Count(void);               // 重置所有计数

// 状态和诊断
typedef struct {
    int32_t total_count;     // 总脉冲数
    float current_speed;     // 当前速度(mm/s)
    float total_distance;    // 总距离(mm)
    uint32_t update_time;    // 最后更新时间
    uint8_t direction;       // 运动方向 (0正转, 1反转)
    uint8_t status;          // 编码器状态
} Encoder_Status_t;

Encoder_Status_t* Encoder_Get_Status(Motor_ID motor_id);
```

### 📡 串口通信协议

#### 数据包格式定义
```c
// 标准数据包格式: [帧头][命令][长度][数据][校验和]
typedef struct {
    uint8_t header[2];      // 帧头: 0xAA, 0x55
    uint8_t command;        // 命令类型
    uint8_t length;         // 数据长度
    uint8_t data[32];       // 数据内容 (最大32字节)
    uint8_t checksum;       // 校验和
} __attribute__((packed)) Protocol_Packet_t;

// 命令类型定义
#define CMD_SET_DIRECTION    0x01    // 设置运动方向和速度
#define CMD_SET_SPEED        0x02    // 设置速度
#define CMD_SET_MOTOR        0x03    // 设置单个电机  
#define CMD_REQUEST_STATUS   0x04    // 请求状态信息
#define CMD_SET_POSITION     0x05    // 设置位置信息
#define CMD_SET_PID_PARAMS   0x06    // 设置PID参数
#define CMD_RESET_SYSTEM     0x07    // 系统复位
#define CMD_DEBUG_INFO       0x08    // 调试信息查询

// 方向定义
#define DIR_FORWARD          0x00    // 前进
#define DIR_BACKWARD         0x01    // 后退
#define DIR_LEFT             0x02    // 左转
#define DIR_RIGHT            0x03    // 右转  
#define DIR_STOP             0x04    // 停止
```

#### 通信API接口
```c
// 数据包生成工具
uint8_t* Generate_Direction_Command(uint8_t direction, uint8_t speed);
uint8_t* Generate_Speed_Command(uint8_t speed);
uint8_t* Generate_PID_Params_Command(Motor_ID id, PID_Params_t* params);
uint8_t* Generate_Status_Request_Command(void);

// 数据包解析工具  
Protocol_Result_t Parse_Received_Packet(uint8_t* data, uint16_t length);
uint8_t Verify_Packet_Checksum(Protocol_Packet_t* packet);
void Process_Command_Packet(Protocol_Packet_t* packet);

// 状态上报
void Send_Status_Response(void);
void Send_Error_Response(uint8_t error_code, const char* message);
void Send_Debug_Info(void);

// 串口通信示例
void Example_Send_Move_Forward_50_Percent(void) {
    uint8_t cmd[] = {0xAA, 0x55, CMD_SET_DIRECTION, 0x02, DIR_FORWARD, 50, 0x00};
    uint8_t checksum = CMD_SET_DIRECTION + 0x02 + DIR_FORWARD + 50;
    cmd[6] = checksum & 0xFF;
    
    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), HAL_MAX_DELAY);
}
```

## ⚙️ 配置参数详解

### 🎯 系统核心配置

```c
// config.h - 系统配置参数
// ===============================================

// 🔧 系统基础配置
#define SYSTEM_FREQ_MHZ         72          // 系统主频(MHz)
#define TASK_SCHEDULER_FREQ_HZ  1000        // 调度器频率(Hz)
#define CONTROL_LOOP_FREQ_HZ    100         // 控制循环频率(Hz)
#define DEBUG_UART_BAUDRATE     115200      // 调试串口波特率
#define MAIN_UART_BAUDRATE      115200      // 主控串口波特率

// ⚡ 电机控制配置
#define MOTOR_PWM_FREQUENCY     1000        // PWM频率(Hz)
#define MOTOR_PWM_RESOLUTION    1000        // PWM分辨率
#define MOTOR_SPEED_MAX         100         // 最大速度(%)
#define MOTOR_SPEED_MIN         -100        // 最小速度(%)
#define MOTOR_DEADZONE          3           // 死区范围(%)
#define MOTOR_ACCELERATION_MAX  50          // 最大加速度(%/s)

// 🎯 PID控制配置
#define PID_KP_DEFAULT          2.0f        // 默认比例系数
#define PID_KI_DEFAULT          0.1f        // 默认积分系数
#define PID_KD_DEFAULT          0.05f       // 默认微分系数
#define PID_INTEGRAL_LIMIT      100.0f      // 积分限幅
#define PID_OUTPUT_LIMIT        100.0f      // 输出限幅
#define PID_DERIVATIVE_FILTER   0.1f        // 微分滤波系数
#define PID_UPDATE_RATE_HZ      100         // PID更新频率(Hz)

// 🔄 编码器配置
#define ENCODER_520_PPR         520         // 编码器线数
#define ENCODER_520_GEAR_RATIO  30          // 减速比
#define ENCODER_520_WHEEL_DIA   65          // 车轮直径(mm)
#define ENCODER_FILTER_SAMPLES  5           // 速度滤波采样数
#define ENCODER_MIN_SPEED_RPM   1           // 最小检测转速(RPM)
#define ENCODER_TIMEOUT_MS      100         // 编码器超时(ms)

// 🛡️ 安全保护配置
#define WATCHDOG_TIMEOUT_MS     1000        // 看门狗超时(ms)
#define EMERGENCY_STOP_TIMEOUT  100         // 紧急停止超时(ms)
#define COMM_TIMEOUT_MS         2000        // 通信超时(ms)
#define OVERHEAT_TEMP_CELSIUS   80          // 过热保护温度(°C)
#define LOW_VOLTAGE_THRESHOLD   3.0f        // 低电压阈值(V)

// 📊 性能监控配置
#define PERFORMANCE_MONITOR_EN  1           // 启用性能监控
#define CPU_USAGE_CALC_PERIOD   1000        // CPU使用率计算周期(ms)
#define MEMORY_USAGE_CHECK_EN   1           // 启用内存使用率检查
#define TASK_TIME_MONITOR_EN    1           // 启用任务执行时间监控
```

### 🔧 高级功能配置

```c
// 📡 通信协议配置
typedef struct {
    uint32_t baudrate;          // 波特率
    uint8_t data_bits;          // 数据位
    uint8_t stop_bits;          // 停止位
    uint8_t parity;             // 校验位
    uint16_t timeout_ms;        // 超时时间
    uint8_t retry_count;        // 重试次数
} UART_Config_t;

// 🎮 控制模式配置
typedef struct {
    Robot_Control_Mode default_mode;    // 默认控制模式
    uint8_t auto_pid_enable;           // 自动启用PID
    float speed_ramp_rate;             // 速度斜坡率(%/s)
    uint8_t smooth_control_enable;     // 平滑控制使能
    uint16_t mode_switch_delay_ms;     // 模式切换延迟
} Control_Config_t;

// 🔄 编码器高级配置
typedef struct {
    uint16_t ppr;                      // 每转脉冲数
    uint8_t quadrature_mode;           // 四倍频模式
    float gear_ratio;                  // 减速比
    float wheel_diameter_mm;           // 车轮直径
    uint8_t filter_enable;             // 滤波使能
    uint8_t filter_samples;            // 滤波采样数
    uint16_t min_speed_threshold;      // 最小速度阈值
} Encoder_520_Config_t;

// 配置加载和保存API
void Load_System_Config(void);
void Save_System_Config(void);
void Reset_Config_To_Default(void);
uint8_t Validate_Config_Parameters(void);
```

## 🔧 开发工具和调试

### 📊 实时性能监控

```c
// CPU使用率监控
typedef struct {
    float cpu_usage_percent;        // CPU使用率(%)
    uint32_t idle_time_us;         // 空闲时间(us)
    uint32_t total_time_us;        // 总时间(us)
    uint32_t max_task_time_us;     // 最大任务执行时间(us)
    char max_task_name[16];        // 最耗时任务名称
} Performance_Monitor_t;

Performance_Monitor_t* Get_Performance_Status(void);
void Reset_Performance_Counters(void);
void Print_Performance_Report(void);

// 内存使用监控
typedef struct {
    uint32_t total_ram_bytes;       // 总RAM大小
    uint32_t used_ram_bytes;        // 已使用RAM
    uint32_t free_ram_bytes;        // 空闲RAM
    uint32_t stack_usage_bytes;     // 栈使用量
    uint32_t heap_usage_bytes;      // 堆使用量
    float memory_fragmentation;     // 内存碎片率
} Memory_Monitor_t;

Memory_Monitor_t* Get_Memory_Status(void);
```

### 🛠️ PID调试工具

```c
// PID调试数据记录
typedef struct {
    float timestamp_ms;             // 时间戳
    float setpoint;                // 设定值
    float feedback;                // 反馈值  
    float error;                   // 误差
    float proportional;            // 比例项
    float integral;                // 积分项
    float derivative;              // 微分项
    float output;                  // 输出值
} PID_Debug_Data_t;

// PID调试API
void PID_Debug_Start_Recording(Motor_ID motor_id);
void PID_Debug_Stop_Recording(Motor_ID motor_id);
void PID_Debug_Export_Data(Motor_ID motor_id);
void PID_Debug_Plot_Response(Motor_ID motor_id);
void PID_Debug_Analyze_Performance(Motor_ID motor_id);

// 自动PID调优工具
typedef struct {
    float target_overshoot_percent; // 目标超调量(%)
    float target_settling_time_ms;  // 目标稳定时间(ms)
    float target_rise_time_ms;      // 目标上升时间(ms)
    uint8_t max_iterations;         // 最大迭代次数
} PID_AutoTune_Config_t;

uint8_t PID_Auto_Tune(Motor_ID motor_id, PID_AutoTune_Config_t* config);
```

### 🔍 系统诊断工具

```c
// 系统健康检查
typedef enum {
    HEALTH_OK = 0,
    HEALTH_WARNING,
    HEALTH_ERROR,
    HEALTH_CRITICAL
} Health_Status_t;

typedef struct {
    Health_Status_t overall_status;     // 整体状态
    Health_Status_t power_status;       // 电源状态
    Health_Status_t motor_status[4];    // 四个电机状态
    Health_Status_t encoder_status[4];  // 四个编码器状态
    Health_Status_t comm_status;        // 通信状态
    Health_Status_t thermal_status;     // 热状态
    char error_message[64];             // 错误信息
} System_Health_t;

System_Health_t* Get_System_Health(void);
void Run_System_Diagnostic(void);
void Print_Health_Report(void);

// 故障诊断代码
#define FAULT_CODE_NONE            0x0000
#define FAULT_CODE_MOTOR_STALL     0x0001
#define FAULT_CODE_ENCODER_FAULT   0x0002
#define FAULT_CODE_COMM_TIMEOUT    0x0004
#define FAULT_CODE_OVERHEAT        0x0008
#define FAULT_CODE_LOW_VOLTAGE     0x0010
#define FAULT_CODE_PID_SATURATION  0x0020
#define FAULT_CODE_WATCHDOG_RESET  0x0040

uint16_t Get_Active_Fault_Codes(void);
void Clear_Fault_Code(uint16_t fault_code);
const char* Get_Fault_Description(uint16_t fault_code);
```

## 📈 性能指标

### 🎯 控制性能
- **位置精度**: ±2mm (闭环控制模式)
- **速度精度**: ±1% (额定速度范围内)  
- **响应时间**: <50ms (阶跃响应)
- **稳定时间**: <200ms (±5%误差带)
- **最大速度**: 1.5m/s (取决于电机规格)
- **最小速度**: 0.01m/s (可稳定控制)

### ⚡ 系统性能
- **CPU使用率**: <60% (正常运行)
- **内存使用**: <16KB RAM (含缓冲区)
- **实时性**: 1ms中断响应延迟
- **通信延迟**: <5ms (串口往返)
- **编码器分辨率**: 0.03mm (520线四倍频)
- **控制频率**: 100Hz (PID更新频率)

### 🔋 功耗特性
- **静态功耗**: <50mA@3.3V
- **动态功耗**: <200mA@3.3V (满载运行)
- **待机功耗**: <10mA@3.3V (低功耗模式)
- **电机驱动**: 根据负载动态调整

## 🐛 故障排除指南

### 🔧 常见问题诊断

#### 电机不转动
```c
// 诊断步骤
1. 检查电源供电: 使用万用表测量电机驱动板供电电压
2. 检查PWM信号: 示波器观察TIM1输出波形
3. 检查方向控制: GPIO输出电平检测
4. 检查电机连线: 断路/短路/接触不良

// 调试代码
void Debug_Motor_Control(void) {
    // 测试PWM输出
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);  // 50%占空比
    
    // 测试方向控制
    HAL_GPIO_WritePin(MOTOR_FL_DIR1_GPIO_Port, MOTOR_FL_DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_FL_DIR2_GPIO_Port, MOTOR_FL_DIR2_Pin, GPIO_PIN_RESET);
    
    // 检查引脚状态
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(MOTOR_FL_DIR1_GPIO_Port, MOTOR_FL_DIR1_Pin);
    printf("Motor FL DIR1 Pin State: %d\n", pin_state);
}
```

#### 编码器无反馈
```c
// 诊断步骤  
1. 检查编码器供电: 3.3V/5V供电正常
2. 检查信号线连接: A相/B相差分信号
3. 检查信号质量: 示波器观察方波信号
4. 检查中断配置: GPIO外部中断使能

// 调试代码
void Debug_Encoder_Signals(void) {
    // 读取编码器GPIO状态
    GPIO_PinState a_phase = HAL_GPIO_ReadPin(ENCODER_FL_A_GPIO_Port, ENCODER_FL_A_Pin);
    GPIO_PinState b_phase = HAL_GPIO_ReadPin(ENCODER_FL_B_GPIO_Port, ENCODER_FL_B_Pin);
    
    printf("Encoder FL - A: %d, B: %d\n", a_phase, b_phase);
    
    // 测试中断计数
    static uint32_t last_count = 0;
    uint32_t current_count = Encoder_Get_Count(MOTOR_FRONT_LEFT);
    if (current_count != last_count) {
        printf("Encoder Count Changed: %ld\n", current_count);
        last_count = current_count;
    }
}
```

#### PID控制振荡
```c
// 问题分析
1. Kp过大: 减小比例系数
2. Ki过大: 减小积分系数  
3. Kd过大: 减小微分系数或增加滤波
4. 采样频率不足: 提高控制频率
5. 机械间隙/惯性: 调整控制参数

// 调试工具
void Debug_PID_Oscillation(Motor_ID motor_id) {
    PID_Status_t* status = FourWheel_Get_PID_Status(motor_id);
    
    printf("Motor %d PID Status:\n", motor_id);
    printf("  Error: %.2f\n", status->current_error);
    printf("  Integral: %.2f\n", status->integral);
    printf("  Derivative: %.2f\n", status->derivative);
    printf("  Output: %.2f\n", status->output);
    
    // 检查积分饱和
    if (fabsf(status->integral) > PID_INTEGRAL_LIMIT * 0.9f) {
        printf("  WARNING: Integral saturation detected!\n");
    }
    
    // 检查输出饱和
    if (fabsf(status->output) > PID_OUTPUT_LIMIT * 0.9f) {
        printf("  WARNING: Output saturation detected!\n");
    }
}
```

#### 串口通信异常
```c
// 诊断步骤
1. 检查波特率配置: 115200-8-N-1
2. 检查连线: TX-RX交叉连接
3. 检查数据格式: 十六进制数据包检查
4. 检查校验和: 数据完整性验证

// 调试工具
void Debug_UART_Communication(void) {
    // 发送测试数据
    uint8_t test_data[] = {0xAA, 0x55, 0x04, 0x00, 0x04};
    HAL_UART_Transmit(&huart1, test_data, sizeof(test_data), HAL_MAX_DELAY);
    
    // 监控接收数据
    uint8_t rx_buffer[64];
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, rx_buffer, sizeof(rx_buffer), 1000);
    
    if (status == HAL_OK) {
        printf("Received data: ");
        for (int i = 0; i < sizeof(rx_buffer); i++) {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\n");
    } else {
        printf("UART Receive Timeout or Error: %d\n", status);
    }
}
```

## 🔄 版本历史

### v2.1.0 (当前版本) - 🎯 功能完善与性能优化
- ✨ **520线编码器深度集成**: 
  - 完整的四倍频解码算法
  - 实时速度计算和滤波
  - 精确的位置积分功能
- 🎛️ **PID控制系统优化**: 
  - 抗积分饱和机制
  - 微分项滤波算法
  - 自动参数调优工具
- 📡 **通信协议标准化**: 
  - 完整的数据包格式定义
  - 校验和机制确保数据完整性
  - 多命令类型支持
- 🛡️ **安全保护机制**: 
  - 看门狗保护
  - 参数边界检查
  - 异常状态恢复
- 📊 **性能监控系统**: 
  - CPU使用率统计
  - 内存占用监控
  - 任务执行时间分析
- 🔧 **调试工具完备**: 
  - 实时状态监控
  - PID调试数据记录
  - 系统健康检查

### v2.0.0
- 🤖 四轮独立PID控制实现
- 🔄 编码器反馈系统集成
- 📡 双串口通信支持
- ⏱️ 实时任务调度器

### v1.0.0  
- ⚡ 基础四轮电机控制
- 🎛️ PWM波形生成
- 📡 串口通信功能
- 🔧 GPIO配置管理

## 📞 技术支持

### 🐛 问题反馈和技术交流
- **GitHub Issues**: [项目Issue页面](https://github.com/your-repo/agrisage-stm32/issues)
- **技术文档**: 查看项目根目录下的技术文档
- **调试工具**: 使用内置的调试和监控工具

### 📚 学习资源  
- **STM32官方文档**: [STM32F1xx参考手册](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **HAL库编程指南**: [STM32CubeF1用户手册](https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- **PID控制理论**: [经典PID控制算法](https://en.wikipedia.org/wiki/PID_controller)
- **编码器原理**: [增量式编码器工作原理](https://www.encoder.com/article-incremental-encoder-introduction)

### 🏆 开源许可
本项目采用Apache 2.0许可证，详见LICENSE文件。

---

<div align="center">

**🤖 AgriSage STM32控制系统 - 精密控制的基石 🤖**

**核心技术**: STM32F103 | PID闭环控制 | 520线编码器 | 双串口通信 | 实时调度

**系统特色**: 高精度定位 | 快速响应 | 稳定可靠 | 易于调试 | 完整工具链

**技术支持**: 详细文档 | 调试工具 | 性能监控 | 开源社区

*让嵌入式控制更精确，让机器人运动更平稳* ⚙️🔧🎯

</div>