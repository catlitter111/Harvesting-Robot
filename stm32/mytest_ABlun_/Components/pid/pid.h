#ifndef PID_H
#define PID_H

#include "../../hardware/config.h"

// ============================================================================
// PID控制器结构体定义
// ============================================================================
typedef struct {
    float Kp, Ki, Kd;                    // PID系数
    float target, feedback, output;      // 目标值、反馈值、输出值
    float error, lastError, sumError;    // 误差、上次误差、误差积分
    float prevError;                     // 前两次误差(增量式用)
    float outputMax, outputMin;          // 输出限制
    uint32_t lastTime;                   // 上次计算时间
    uint8_t mode;                        // 控制模式: 0=位置式, 1=增量式
} PID_T;

// ============================================================================
// PID控制器函数声明
// ============================================================================
void PID_Init(PID_T* pid, float kp, float ki, float kd);                    // 初始化PID控制器
void PID_SetTarget(PID_T* pid, float target);                               // 设置目标值
void PID_SetFeedback(PID_T* pid, float feedback);                          // 设置反馈值  
void PID_SetOutputLimits(PID_T* pid, float min, float max);                // 设置输出限制
void PID_SetMode(PID_T* pid, uint8_t mode);                                // 设置控制模式
void PID_Reset(PID_T* pid);                                                 // 重置PID控制器
float PID_Compute(PID_T* pid);                                              // 计算PID输出
void PID_SetSampleTime(PID_T* pid, uint32_t sampleTime);                   // 设置采样时间

// ============================================================================
// 默认初始化宏定义
// ============================================================================
#define PID_DEFAULT_INIT() { \
    .Kp = PID_KP_DEFAULT, \
    .Ki = PID_KI_DEFAULT, \
    .Kd = PID_KD_DEFAULT, \
    .outputMax = PID_OUTPUT_LIMIT, \
    .outputMin = -PID_OUTPUT_LIMIT, \
    .mode = 0 \
}

#endif /* PID_H */ 

