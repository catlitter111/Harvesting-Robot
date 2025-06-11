#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"
#include "motor.h"

// ============================================================================
// 编码器数据结构定义
// ============================================================================
typedef struct {
    int32_t pulse_count;             // 脉冲累计计数
    int32_t last_count;              // 上次计数值
    int16_t speed_rpm;               // 转速(RPM)
    int16_t speed_percent;           // 速度百分比(-100~+100)
    uint32_t last_time;              // 上次更新时间(ms)
    GPIO_TypeDef* port_a;            // A相GPIO端口
    uint16_t pin_a;                  // A相GPIO引脚
    GPIO_TypeDef* port_b;            // B相GPIO端口
    uint16_t pin_b;                  // B相GPIO引脚
    uint8_t last_state_a;            // A相上次状态
    uint8_t last_state_b;            // B相上次状态
} Encoder_t;

// ============================================================================
// 全局变量声明
// ============================================================================
extern Encoder_t encoders[MOTOR_COUNT];

// ============================================================================
// 编码器功能函数声明
// ============================================================================
void Encoder_Init(void);                                    // 编码器初始化
void Encoder_Update_All_Speed(void);                        // 更新四轮速度
void Encoder_Read_All_Pulses(void);                         // 读取四轮脉冲
int16_t Encoder_Get_Speed(Motor_ID motor);                  // 获取单轮速度百分比
int16_t Encoder_Get_RPM(Motor_ID motor);                    // 获取单轮转速RPM
int32_t Encoder_Get_Count(Motor_ID motor);                  // 获取单轮脉冲计数
void Encoder_Reset_Count(Motor_ID motor);                   // 重置编码器计数
void Encoder_Reset_All_Count(void);                         // 重置所有编码器计数
void Encoder_Filter_Speed(Motor_ID motor);                  // 速度滤波处理

// ============================================================================
// 内部辅助函数声明
// ============================================================================
void Encoder_Process_Single(Motor_ID motor);                // 处理单个编码器
uint8_t Encoder_Read_Pin(GPIO_TypeDef* port, uint16_t pin); // 读取编码器引脚状态

#endif /* ENCODER_H */



