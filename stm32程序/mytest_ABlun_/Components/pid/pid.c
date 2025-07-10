#include "pid.h"
#include <stdint.h>

// ============================================================================
// 内部辅助函数声明
// ============================================================================
static uint32_t get_system_time_ms(void) { return HAL_GetTick(); }

// ============================================================================
// PID控制器初始化
// ============================================================================
void PID_Init(PID_T* pid, float kp, float ki, float kd) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->target = pid->feedback = pid->output = 0.0f;
    pid->error = pid->lastError = pid->sumError = pid->prevError = 0.0f;
    pid->outputMax = PID_OUTPUT_LIMIT; pid->outputMin = -PID_OUTPUT_LIMIT;
    pid->lastTime = get_system_time_ms(); pid->mode = 0;
}

// ============================================================================
// 设置目标值
// ============================================================================
void PID_SetTarget(PID_T* pid, float target) {
    pid->target = target;
}

// ============================================================================
// 设置反馈值
// ============================================================================
void PID_SetFeedback(PID_T* pid, float feedback) {
    pid->feedback = feedback;
}

// ============================================================================
// 设置输出限制
// ============================================================================
void PID_SetOutputLimits(PID_T* pid, float min, float max) {
    if (min >= max) return;
    pid->outputMin = min; pid->outputMax = max;
    if (pid->output > max) pid->output = max;
    else if (pid->output < min) pid->output = min;
    if (pid->sumError > max) pid->sumError = max;
    else if (pid->sumError < min) pid->sumError = min;
}

// ============================================================================
// 设置控制模式
// ============================================================================
void PID_SetMode(PID_T* pid, uint8_t mode) {
    if (mode != pid->mode) { PID_Reset(pid); pid->mode = mode; }
}

// ============================================================================
// 重置PID控制器
// ============================================================================
void PID_Reset(PID_T* pid) {
    pid->error = pid->lastError = pid->sumError = pid->prevError = 0.0f;
    pid->output = 0.0f; pid->lastTime = get_system_time_ms();
}

// ============================================================================
// 设置采样时间(暂未使用，保留接口)
// ============================================================================
void PID_SetSampleTime(PID_T* pid, uint32_t sampleTime) {
    // 预留功能，当前使用固定采样时间
}

// ============================================================================
// 计算PID输出(主要计算函数)
// ============================================================================
float PID_Compute(PID_T* pid) {
    uint32_t now = get_system_time_ms();
    uint32_t timeChange = now - pid->lastTime;
    
    // 时间间隔检查
    if (timeChange < PID_SAMPLE_TIME) return pid->output;
    
    // 计算误差
    pid->error = pid->target - pid->feedback;
    
    if (pid->mode == 0) {  // 位置式PID
        pid->sumError += pid->error;
        // 积分限幅
        if (pid->sumError > PID_INTEGRAL_LIMIT) pid->sumError = PID_INTEGRAL_LIMIT;
        else if (pid->sumError < -PID_INTEGRAL_LIMIT) pid->sumError = -PID_INTEGRAL_LIMIT;
        
        pid->output = pid->Kp * pid->error + pid->Ki * pid->sumError + pid->Kd * (pid->error - pid->lastError);
    } else {  // 增量式PID
        float deltaOutput = pid->Kp * (pid->error - pid->lastError) + pid->Ki * pid->error + pid->Kd * (pid->error - 2*pid->lastError + pid->prevError);
        pid->output += deltaOutput;
        pid->prevError = pid->lastError;  // 保存前两次误差
    }
    
    // 输出限幅
    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    else if (pid->output < pid->outputMin) pid->output = pid->outputMin;
    
    // 更新历史数据
    pid->lastError = pid->error; pid->lastTime = now;
    return pid->output;
} 

