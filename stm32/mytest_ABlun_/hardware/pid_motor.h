#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "config.h"
#include "../Components/pid/pid.h"
#include "motor.h"

// ============================================================================
// 四轮PID控制器结构体定义
// ============================================================================
typedef struct {
    PID_T pid_controller;            // PID控制器实例
    int16_t target_speed;            // 目标速度(-100~+100)
    int16_t current_speed;           // 当前速度(编码器反馈)
    int8_t pwm_output;               // PWM输出值
    uint8_t enabled;                 // PID使能标志
} MotorPID_t;

// ============================================================================
// 全局变量声明
// ============================================================================
extern MotorPID_t motor_pids[MOTOR_COUNT];

// ============================================================================
// 四轮PID控制器函数声明
// ============================================================================
void FourWheel_PID_Init(void);                                      // 初始化四个PID控制器
void FourWheel_Set_Target_Speed(Motor_ID motor, int16_t speed);     // 设置单轮目标速度
void FourWheel_Set_All_Target_Speed(int16_t speed);                 // 设置所有轮目标速度
void FourWheel_Update_Speed(Motor_ID motor, int16_t speed);         // 更新单轮当前速度
void FourWheel_PID_Calculate(void);                                 // 计算四路PID输出
void FourWheel_Apply_Control(void);                                 // 应用PID输出到电机
void Robot_Enable_PID(uint8_t enable);                              // PID控制总开关
void Robot_Set_PID_Params(float kp, float ki, float kd);           // 设置所有轮PID参数
void FourWheel_Reset_PID(void);                                     // 重置所有PID控制器
uint8_t Robot_Get_PID_Status(void);                                 // 获取PID使能状态

// ============================================================================
// 高级运动控制接口
// ============================================================================
void Robot_Move_PID(Robot_Direction direction, int16_t speed);      // PID运动控制
void Robot_Differential_Drive(int16_t left_speed, int16_t right_speed); // 差分驱动控制
void Robot_Four_Wheel_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr); // 四轮独立控制

#endif /* PID_MOTOR_H */ 

