#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "config.h"
#include "motor.h"
#include "pid_motor.h"
#include "encoder.h"

// ============================================================================
// 机器人运动控制模式定义
// ============================================================================
typedef enum {
    ROBOT_MODE_DIRECT = 0,       // 直接电机控制模式
    ROBOT_MODE_PID,              // PID速度控制模式
    ROBOT_MODE_AUTO              // 自动控制模式(预留)
} Robot_Control_Mode;

// ============================================================================
// 机器人状态结构体
// ============================================================================
typedef struct {
    int16_t wheel_speeds[MOTOR_COUNT];   // 四轮目标速度
    int16_t wheel_feedback[MOTOR_COUNT]; // 四轮反馈速度
    Robot_Control_Mode mode;             // 控制模式
    uint8_t pid_enabled;                 // PID使能状态
    uint32_t last_cmd_time;              // 最后命令时间
} Robot_Status_t;

// ============================================================================
// 全局变量声明
// ============================================================================
extern Robot_Status_t robot_status;

// ============================================================================
// 基础控制接口
// ============================================================================
void Robot_Control_Init(void);                              // 机器人控制初始化
void Robot_Set_Mode(Robot_Control_Mode mode);               // 设置控制模式
Robot_Control_Mode Robot_Get_Mode(void);                    // 获取控制模式

// ============================================================================
// 速度控制接口
// ============================================================================
void Robot_Set_Speed_All(int16_t speed);                    // 四轮同速设置
void Robot_Set_Speed_Individual(int16_t fl, int16_t fr, int16_t rl, int16_t rr); // 四轮独立设置
void Robot_Set_Speed_Left_Right(int16_t left, int16_t right); // 左右差速设置

// ============================================================================
// 运动控制接口
// ============================================================================
void Robot_Move_Forward(int16_t speed);                     // 前进
void Robot_Move_Backward(int16_t speed);                    // 后退
void Robot_Turn_Left(int16_t speed);                        // 左转(原地转向)
void Robot_Turn_Right(int16_t speed);                       // 右转(原地转向)
void Robot_Arc_Left(int16_t speed, int16_t radius);         // 左转弯(弧线)
void Robot_Arc_Right(int16_t speed, int16_t radius);        // 右转弯(弧线)
void Robot_Stop(void);                                      // 立即停止
void Robot_Brake(uint16_t brake_time_ms);                   // 制动停止

// ============================================================================
// 状态查询接口
// ============================================================================
void Robot_Get_Speeds(int16_t* speeds);                     // 获取四轮目标速度
void Robot_Get_Feedback(int16_t* feedback);                 // 获取四轮反馈速度
Robot_Status_t* Robot_Get_Status(void);                     // 获取完整状态
uint8_t Robot_Is_Moving(void);                              // 判断是否运动中
uint8_t Robot_Is_PID_Enabled(void);                         // 判断PID是否使能

// ============================================================================
// 高级控制接口
// ============================================================================
void Robot_Follow_Path(int16_t* speeds, uint8_t count, uint16_t interval); // 路径跟踪(预留)
void Robot_Emergency_Stop(void);                            // 紧急停止
void Robot_Reset_Control(void);                             // 重置控制系统

#endif /* ROBOT_CONTROL_H */ 


