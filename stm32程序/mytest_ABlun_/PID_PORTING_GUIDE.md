# STM32四轮小车PID控制移植指南

## 📋 目录
1. [移植概述](#移植概述)
2. [代码结构对比](#代码结构对比)
3. [PID算法详解](#PID算法详解)
4. [硬件接口说明](#硬件接口说明)
5. [编译部署指南](#编译部署指南)
6. [调试技巧](#调试技巧)
7. [测试验证](#测试验证)
8. [常见问题](#常见问题)
9. [API参考](#API参考)

---

## 🎯 移植概述

### 移植目标
将 `ElectronicController_Board` 项目中的双轴PID位置控制系统移植到 `mytest_ABlun_` 四轮小车项目中，实现精确的四轮PID速度控制。

### 核心变化
- **电机类型**: 步进电机 → PWM直流电机
- **控制方式**: 位置控制 → 速度控制  
- **控制轴数**: 双轴(X/Y) → 四轮独立控制
- **反馈方式**: 无反馈 → 编码器速度反馈
- **调度系统**: MultiTimer → 扩展scheduler

### 新增功能
- ✅ 编码器测速模块
- ✅ 四轮独立PID控制器
- ✅ 统一配置管理(config.h)
- ✅ 上层控制接口
- ✅ 实时任务调度

---

## 📁 代码结构对比

### 移植前 (ElectronicController_Board)
```
ElectronicController_Board/
├── Components/
│   └── pid/                    # PID算法库
│       ├── pid.h
│       └── pid.c
├── App/
│   └── app_pid.c              # 双轴PID控制应用
└── MultiTimer/                # 定时器调度系统
```

### 移植后 (mytest_ABlun_)
```
mytest_ABlun_/
├── Components/
│   └── pid/                    # 复用PID算法库(已优化)
│       ├── pid.h              # 新API设计
│       ├── pid.c              # 位置式+增量式PID
│       ├── pid_test.h         # 测试框架
│       └── pid_test.c
├── hardware/
│   ├── config.h               # 统一配置文件 ⭐
│   ├── pid_motor.h            # 四轮PID控制器 ⭐
│   ├── pid_motor.c
│   ├── encoder.h              # 编码器测速模块 ⭐
│   ├── encoder.c
│   ├── robot_control.h        # 上层控制接口 ⭐
│   ├── robot_control.c
│   └── scheduler.c            # 扩展调度系统
└── Core/Src/
    └── main.c                 # 集成初始化
```

---

## 🧮 PID算法详解

### PID控制原理
PID控制器通过比例(P)、积分(I)、微分(D)三个环节的组合来实现精确控制：

```
输出 = Kp×误差 + Ki×误差积分 + Kd×误差微分
```

### 速度PID vs 位置PID
| 特性 | 位置PID | 速度PID |
|------|---------|---------|
| 控制目标 | 位置精度 | 速度响应 |
| 积分作用 | 消除位置偏差 | 消除速度偏差 |
| 适用场景 | 定位控制 | 运动控制 |

### 默认PID参数
```c
#define PID_KP_DEFAULT 2.0f     // 比例系数：响应速度
#define PID_KI_DEFAULT 0.1f     // 积分系数：消除稳态误差  
#define PID_KD_DEFAULT 0.5f     // 微分系数：减少超调
#define PID_SAMPLE_TIME 20      // 采样周期：20ms
```

### PID参数调优步骤
1. **第一步：调节Kp**
   - 从小值开始(0.5)逐步增大
   - 观察系统响应速度
   - 过大会导致震荡

2. **第二步：调节Ki**  
   - 从0.01开始逐步增大
   - 消除稳态误差
   - 过大会导致超调

3. **第三步：调节Kd**
   - 从0开始逐步增大
   - 改善系统稳定性
   - 过大会放大噪声

### 调优实例
```c
// 保守参数(稳定优先)
Robot_Set_PID_Params(1.5f, 0.05f, 0.3f);

// 激进参数(响应优先)  
Robot_Set_PID_Params(3.0f, 0.2f, 0.8f);

// 平衡参数(推荐)
Robot_Set_PID_Params(2.0f, 0.1f, 0.5f);
```

---

## 🔌 硬件接口说明

### 编码器连接(临时分配)
> ⚠️ **注意**: 以下为临时引脚分配，请根据实际硬件调整 `config.h` 中的定义

| 电机 | A相引脚 | B相引脚 | GPIO端口 |
|------|---------|---------|----------|
| 电机1 | PC0 | PC1 | GPIOC |
| 电机2 | PC2 | PC3 | GPIOC |  
| 电机3 | PC4 | PC5 | GPIOC |
| 电机4 | PC6 | PC7 | GPIOC |

### 编码器类型支持
- **增量式编码器**: 支持A/B相正交信号
- **分辨率**: 400PPR(可配置)
- **解码方式**: 四倍频解码
- **方向检测**: 自动识别正反转

### PWM电机控制
复用现有TIM1四通道PWM配置：
- **PWM频率**: 1kHz
- **分辨率**: 1000级(0.1%精度)
- **方向控制**: GPIO数字信号

### 引脚配置修改
如需修改编码器引脚，请编辑 `hardware/config.h`:
```c
// 修改编码器1引脚
#define ENCODER1_A_PIN GPIO_PIN_0    // 改为你的A相引脚
#define ENCODER1_B_PIN GPIO_PIN_1    // 改为你的B相引脚  
#define ENCODER1_PORT GPIOC          // 改为你的GPIO端口
```

---

## 🔧 编译部署指南

### 环境要求
- **IDE**: Keil MDK-ARM 5.29+
- **芯片**: STM32F103系列
- **编译器**: ARM Compiler 6

### 编译步骤
1. **添加包含路径**
   ```
   Project → Options → C/C++ → Include Paths
   添加: .\Components\pid
   ```

2. **添加源文件**
   - 将 `Components/pid/` 下所有.c文件添加到项目
   - 将 `hardware/` 下新增.c文件添加到项目

3. **编译验证**
   ```
   Project → Build Target (F7)
   确保无编译错误和警告
   ```

4. **下载程序**
   ```
   Flash → Download (F8)
   ```

### 内存占用评估
| 模块 | Flash占用 | RAM占用 |
|------|-----------|---------|
| PID算法库 | ~2KB | ~200B |
| 四轮PID控制 | ~1.5KB | ~100B |
| 编码器模块 | ~1KB | ~50B |
| **总计** | **~4.5KB** | **~350B** |

---

## 🐛 调试技巧

### 串口调试输出
启用调试模式，在 `config.h` 中定义：
```c
#define DEBUG 1
```

### 关键调试信息
```c
// 查看PID参数
DEBUG_PRINTF("PID: Kp=%.2f Ki=%.3f Kd=%.2f", pid.Kp, pid.Ki, pid.Kd);

// 查看速度反馈
DEBUG_PRINTF("速度: 目标=%d 反馈=%d 输出=%d", target, feedback, output);

// 查看编码器计数
DEBUG_PRINTF("编码器: 计数=%ld 速度=%d", count, speed);
```

### 逐步调试方法
1. **第一步：验证编码器**
   ```c
   // 手动转动电机，观察计数变化
   int32_t count = Encoder_Get_Count(MOTOR_FRONT_LEFT);
   DEBUG_PRINTF("编码器计数: %ld", count);
   ```

2. **第二步：测试单轮PID**
   ```c
   // 单轮测试，其他轮停止
   FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, 50);
   FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, 0);
   // ...
   ```

3. **第三步：四轮协调测试**
   ```c
   // 四轮同速测试
   Robot_Move_Forward(30);
   HAL_Delay(2000);
   Robot_Stop();
   ```

### 常用调试命令
```c
// 紧急停止
Robot_Emergency_Stop();

// 重置系统
Robot_Reset_Control();

// 切换控制模式
Robot_Set_Mode(ROBOT_MODE_DIRECT);  // 直接控制
Robot_Set_Mode(ROBOT_MODE_PID);     // PID控制
```

---

## ✅ 测试验证

### 测试清单
- [ ] 编码器测速准确性
- [ ] PID响应特性  
- [ ] 四轮协调控制
- [ ] 转向精度
- [ ] 紧急停止功能

### 编码器测速测试
```c
void Test_Encoder_Speed(void) {
    // 设置固定PWM，测量编码器反馈
    Motor_SetSpeed(MOTOR_FRONT_LEFT, 50);
    HAL_Delay(1000);
    
    int16_t speed = Encoder_Get_Speed(MOTOR_FRONT_LEFT);
    DEBUG_PRINTF("PWM=50, 测得速度=%d", speed);
    
    Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
}
```

### PID响应测试
```c
void Test_PID_Response(void) {
    // 阶跃响应测试
    FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, 60);
    
    for(int i = 0; i < 100; i++) {
        int16_t feedback = Encoder_Get_Speed(MOTOR_FRONT_LEFT);
        DEBUG_PRINTF("时间=%dms, 反馈=%d", i*20, feedback);
        HAL_Delay(20);
    }
}
```

### 四轮协调测试
```c
void Test_Four_Wheel_Coordination(void) {
    // 前进测试
    Robot_Move_Forward(40);
    HAL_Delay(2000);
    
    // 转向测试  
    Robot_Turn_Left(30);
    HAL_Delay(1000);
    
    Robot_Stop();
}
```

---

## ❓ 常见问题

### Q1: 编码器计数不变化
**原因**: 引脚配置错误或编码器未连接
**解决**: 
1. 检查 `config.h` 中引脚定义
2. 用万用表测量编码器信号
3. 确认GPIO配置为输入模式

### Q2: PID输出震荡
**原因**: PID参数过大或采样频率过高
**解决**:
1. 减小Kp参数
2. 增加Kd参数抑制震荡
3. 检查采样周期设置

### Q3: 电机不转或转速不对
**原因**: PWM配置问题或电机驱动故障
**解决**:
1. 用示波器检查PWM信号
2. 检查电机驱动电路
3. 验证方向控制信号

### Q4: 系统运行不稳定
**原因**: 任务调度冲突或内存不足
**解决**:
1. 检查任务周期配置
2. 增加堆栈大小
3. 优化代码减少内存占用

### Q5: 编译错误
**原因**: 包含路径或文件缺失
**解决**:
1. 检查所有.h文件路径
2. 确认所有.c文件已添加到项目
3. 检查宏定义冲突

---

## 📚 API参考

### PID控制器API
```c
// 初始化PID控制器
void PID_Init(PID_T* pid, float kp, float ki, float kd);

// 设置目标值和反馈值
void PID_SetTarget(PID_T* pid, float target);
void PID_SetFeedback(PID_T* pid, float feedback);

// 计算PID输出
float PID_Compute(PID_T* pid);

// 设置输出限制
void PID_SetOutputLimits(PID_T* pid, float min, float max);
```

### 四轮控制API
```c
// 初始化四轮PID系统
void FourWheel_PID_Init(void);

// 设置目标速度
void FourWheel_Set_Target_Speed(Motor_ID motor, int16_t speed);
void FourWheel_Set_All_Target_Speed(int16_t speed);

// PID计算和应用
void FourWheel_PID_Calculate(void);
void FourWheel_Apply_Control(void);

// PID控制开关
void Robot_Enable_PID(uint8_t enable);
```

### 编码器API
```c
// 初始化编码器
void Encoder_Init(void);

// 更新速度
void Encoder_Update_All_Speed(void);

// 获取数据
int16_t Encoder_Get_Speed(Motor_ID motor);
int32_t Encoder_Get_Count(Motor_ID motor);

// 重置计数
void Encoder_Reset_Count(Motor_ID motor);
```

### 上层控制API
```c
// 基础运动控制
void Robot_Move_Forward(int16_t speed);
void Robot_Move_Backward(int16_t speed);
void Robot_Turn_Left(int16_t speed);
void Robot_Turn_Right(int16_t speed);
void Robot_Stop(void);

// 高级控制
void Robot_Set_Speed_Individual(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void Robot_Arc_Left(int16_t speed, int16_t radius);
void Robot_Emergency_Stop(void);

// 状态查询
Robot_Status_t* Robot_Get_Status(void);
uint8_t Robot_Is_Moving(void);
```

---

## 📝 版本信息

- **文档版本**: V1.0
- **移植版本**: 基于ElectronicController_Board V2.0
- **目标平台**: STM32F103 + mytest_ABlun_
- **作者**: STM32四轮小车PID控制移植项目
- **更新日期**: 2025年

---

## 🎉 结语

本移植指南详细介绍了从双轴位置PID到四轮速度PID的完整移植过程。通过模块化设计和统一配置管理，实现了代码的高度复用和易维护性。

如有问题，请参考常见问题章节或查看源码注释。祝您移植成功！

**Happy Coding! 🚀** 