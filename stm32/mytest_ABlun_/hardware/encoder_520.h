#ifndef ENCODER_520_H
#define ENCODER_520_H

#include "config.h"
#include "motor.h"
#include "tim.h"
#include <stdint.h>

// ============================================================================
// 520电机编码器配置参数
// ============================================================================
#define ENCODER_520_PPR 520              // 520电机编码器每转脉冲数
#define ENCODER_520_WHEEL_DIAMETER 65    // 轮径(mm)
#define ENCODER_520_GEAR_RATIO 1.0f      // 减速比
#define ENCODER_520_SAMPLE_PERIOD 10     // 采样周期(ms)
#define ENCODER_520_SPEED_FILTER 0.7f    // 速度滤波系数

// ============================================================================
// 编码器数据结构定义
// ============================================================================
typedef struct {
    TIM_HandleTypeDef* htim;             // 定时器句柄
    int16_t count_current;               // 当前计数值
    int16_t count_last;                  // 上次计数值
    int16_t speed_raw;                   // 原始速度值
    int16_t speed_filtered;              // 滤波后速度值
    float speed_rpm;                     // 转速(RPM)
    float speed_ms;                      // 线速度(m/s)
    uint32_t last_time;                  // 上次更新时间
    uint8_t direction;                   // 旋转方向(0:正转,1:反转)
} Encoder520_t;

// ============================================================================
// 定时器配置表
// ============================================================================
typedef struct {
    TIM_HandleTypeDef* htim;             // 定时器句柄
    uint32_t channel;                    // 定时器通道
    int8_t direction_factor;             // 方向系数(1或-1)
} EncoderTim_Config_t;

// ============================================================================
// 全局变量声明
// ============================================================================
extern Encoder520_t encoders_520[MOTOR_COUNT];
extern int Encoder_Left, Encoder_Right;  // 兼容原有接口

// ============================================================================
// 520编码器功能函数声明
// ============================================================================
void Encoder520_Init(void);                                 // 520编码器初始化
void Encoder520_Start_All(void);                           // 启动所有编码器
void Encoder520_Stop_All(void);                           // 停止所有编码器
void Encoder520_Update_Speed(Motor_ID motor);              // 更新单个编码器速度
void Encoder520_Update_All_Speed(void);                   // 更新所有编码器速度
int16_t Encoder520_Read_Speed(Motor_ID motor);             // 读取编码器速度
int16_t Encoder520_Read_Count(Motor_ID motor);             // 读取编码器计数
float Encoder520_Get_RPM(Motor_ID motor);                  // 获取转速RPM
float Encoder520_Get_Speed_MS(Motor_ID motor);             // 获取线速度m/s
void Encoder520_Reset_Count(Motor_ID motor);               // 重置编码器计数
void Encoder520_Reset_All_Count(void);                     // 重置所有编码器

// ============================================================================
// 兼容性接口函数(与原编码器接口保持一致)
// ============================================================================
int Read_Speed_Left(void);                                 // 读取左轮速度
int Read_Speed_Right(void);                                // 读取右轮速度
void Encoder_Compatibility_Update(void);                   // 兼容性更新函数

// ============================================================================
// 内部辅助函数声明
// ============================================================================
void Encoder520_Filter_Speed(Motor_ID motor);              // 速度滤波处理
int16_t Encoder520_Read_Timer_Count(TIM_HandleTypeDef* htim); // 读取定时器计数

#endif /* ENCODER_520_H */ 