#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include "gpio.h"

// 电机标识符枚举
// typedef enum {
//     MOTOR_FRONT_LEFT = 0,
//     MOTOR_FRONT_RIGHT,
//     MOTOR_REAR_LEFT,
//     MOTOR_REAR_RIGHT,
//     MOTOR_COUNT
// } Motor_ID;

typedef enum {
    MOTOR_REAR_RIGHT = 0,    // 后右电机
    MOTOR_FRONT_RIGHT,       // 前右电机
    MOTOR_REAR_LEFT,         // 后左电机
    MOTOR_FRONT_LEFT,        // 前左电机
    MOTOR_COUNT_NUM              // 电机总数
} Motor_ID;

// 电机方向枚举
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_STOP
} Motor_Direction;

// 电机配置结构体
typedef struct {
    TIM_HandleTypeDef* htim;      // 定时器句柄
    uint32_t channel;             // 定时器通道
    GPIO_TypeDef* pwm_port;       // PWM端口
    uint16_t pwm_pin;             // PWM引脚
    GPIO_TypeDef* in1_port;       // 输入1端口
    uint16_t in1_pin;             // 输入1引脚
    GPIO_TypeDef* in2_port;       // 输入2端口
    uint16_t in2_pin;             // 输入2引脚
} Motor_Config;

// 机器人移动方向枚举
typedef enum {
    ROBOT_DIR_FORWARD = 0,
    ROBOT_DIR_BACKWARD,
    ROBOT_DIR_LEFT,
    ROBOT_DIR_RIGHT,
    ROBOT_DIR_STOP
} Robot_Direction;

// 函数原型声明
HAL_StatusTypeDef Motor_Init(void);
HAL_StatusTypeDef Motor_SetPWM(Motor_ID motor, uint8_t duty_cycle);
void Motor_SetDirection(Motor_ID motor, Motor_Direction direction);
void Motor_SetSpeed(Motor_ID motor, int8_t speed); // -100到+100，负数表示反转
void Robot_Move(Robot_Direction direction);
void Robot_SetSpeed(uint8_t speed);
void Motor_Test(void);

#endif // MOTOR_H

