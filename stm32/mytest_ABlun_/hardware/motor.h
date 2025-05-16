#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include "gpio.h"

// �����ʶö��
// typedef enum {
//     MOTOR_FRONT_LEFT = 0,
//     MOTOR_FRONT_RIGHT,
//     MOTOR_REAR_LEFT,
//     MOTOR_REAR_RIGHT,
//     MOTOR_COUNT
// } Motor_ID;

typedef enum {
    MOTOR_REAR_RIGHT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_FRONT_LEFT,
    MOTOR_COUNT
} Motor_ID;

// �������ö��
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_STOP
} Motor_Direction;

// ������ýṹ��
typedef struct {
    TIM_HandleTypeDef* htim;      // ��ʱ�����
    uint32_t channel;             // ��ʱ��ͨ��
    GPIO_TypeDef* pwm_port;       // PWM�˿�
    uint16_t pwm_pin;             // PWM����
    GPIO_TypeDef* in1_port;       // ����1�˿�
    uint16_t in1_pin;             // ����1����
    GPIO_TypeDef* in2_port;       // ����2�˿�
    uint16_t in2_pin;             // ����2����
} Motor_Config;

// �������ƶ�����ö��
typedef enum {
    ROBOT_DIR_FORWARD = 0,
    ROBOT_DIR_BACKWARD,
    ROBOT_DIR_LEFT,
    ROBOT_DIR_RIGHT,
    ROBOT_DIR_STOP
} Robot_Direction;

// ����ԭ������
HAL_StatusTypeDef Motor_Init(void);
HAL_StatusTypeDef Motor_SetPWM(Motor_ID motor, uint8_t duty_cycle);
void Motor_SetDirection(Motor_ID motor, Motor_Direction direction);
void Motor_SetSpeed(Motor_ID motor, int8_t speed); // -100��+100��������ʾ����
void Robot_Move(Robot_Direction direction);
void Robot_SetSpeed(uint8_t speed);
void Motor_Test(void);

#endif // MOTOR_H

