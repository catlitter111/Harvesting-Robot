#include "pid_motor.h"

// ============================================================================
// 全局变量定义
// ============================================================================
MotorPID_t motor_pids[MOTOR_COUNT];
static uint8_t pid_enabled = 0;

// ============================================================================
// 初始化四个PID控制器
// ============================================================================
void FourWheel_PID_Init(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        PID_Init(&motor_pids[i].pid_controller, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
        PID_SetOutputLimits(&motor_pids[i].pid_controller, -SPEED_MAX, SPEED_MAX);
        motor_pids[i].target_speed = 0;
        motor_pids[i].current_speed = 0;
        motor_pids[i].pwm_output = 0;
        motor_pids[i].enabled = 1;
    }
    pid_enabled = 1;
}

// ============================================================================
// 设置单轮目标速度
// ============================================================================
void FourWheel_Set_Target_Speed(Motor_ID motor, int16_t speed) {
    if (motor >= MOTOR_COUNT) return;
    speed = CONSTRAIN(speed, SPEED_MIN, SPEED_MAX);
    motor_pids[motor].target_speed = speed;
    PID_SetTarget(&motor_pids[motor].pid_controller, (float)speed);
}

// ============================================================================
// 设置所有轮目标速度
// ============================================================================
void FourWheel_Set_All_Target_Speed(int16_t speed) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        FourWheel_Set_Target_Speed((Motor_ID)i, speed);
    }
}

// ============================================================================
// 更新单轮当前速度(编码器反馈)
// ============================================================================
void FourWheel_Update_Speed(Motor_ID motor, int16_t speed) {
    if (motor >= MOTOR_COUNT) return;
    motor_pids[motor].current_speed = speed;
    PID_SetFeedback(&motor_pids[motor].pid_controller, (float)speed);
}

// ============================================================================
// 计算四路PID输出
// ============================================================================
void FourWheel_PID_Calculate(void) {
    if (!pid_enabled) return;
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (motor_pids[i].enabled) {
            float output = PID_Compute(&motor_pids[i].pid_controller);
            motor_pids[i].pwm_output = (int8_t)CONSTRAIN(output, SPEED_MIN, SPEED_MAX);
        } else {
            motor_pids[i].pwm_output = 0;
        }
    }
}

// ============================================================================
// 应用PID输出到电机
// ============================================================================
void FourWheel_Apply_Control(void) {
    if (!pid_enabled) return;
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetSpeed((Motor_ID)i, motor_pids[i].pwm_output);
    }
}

// ============================================================================
// PID控制总开关
// ============================================================================
void Robot_Enable_PID(uint8_t enable) {
    pid_enabled = enable;
    if (!enable) {
        // 禁用PID时停止所有电机
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            Motor_SetSpeed((Motor_ID)i, 0);
            motor_pids[i].pwm_output = 0;
        }
    }
}

// ============================================================================
// 设置所有轮PID参数
// ============================================================================
void Robot_Set_PID_Params(float kp, float ki, float kd) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_pids[i].pid_controller.Kp = kp;
        motor_pids[i].pid_controller.Ki = ki;
        motor_pids[i].pid_controller.Kd = kd;
    }
}

// ============================================================================
// 重置所有PID控制器
// ============================================================================
void FourWheel_Reset_PID(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        PID_Reset(&motor_pids[i].pid_controller);
        motor_pids[i].pwm_output = 0;
    }
}

// ============================================================================
// 获取PID使能状态
// ============================================================================
uint8_t Robot_Get_PID_Status(void) {
    return pid_enabled;
}

// ============================================================================
// PID运动控制
// ============================================================================
void Robot_Move_PID(Robot_Direction direction, int16_t speed) {
    if (!pid_enabled) return;
    
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    
    switch (direction) {
        case ROBOT_DIR_FORWARD:
            FourWheel_Set_All_Target_Speed(speed);
            break;
        case ROBOT_DIR_BACKWARD:
            FourWheel_Set_All_Target_Speed(-speed);
            break;
        case ROBOT_DIR_LEFT:
            FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, -speed);
            FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, speed);
            FourWheel_Set_Target_Speed(MOTOR_REAR_LEFT, -speed);
            FourWheel_Set_Target_Speed(MOTOR_REAR_RIGHT, speed);
            break;
        case ROBOT_DIR_RIGHT:
            FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, speed);
            FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, -speed);
            FourWheel_Set_Target_Speed(MOTOR_REAR_LEFT, speed);
            FourWheel_Set_Target_Speed(MOTOR_REAR_RIGHT, -speed);
            break;
        case ROBOT_DIR_STOP:
        default:
            FourWheel_Set_All_Target_Speed(0);
            break;
    }
}

// ============================================================================
// 差分驱动控制
// ============================================================================
void Robot_Differential_Drive(int16_t left_speed, int16_t right_speed) {
    if (!pid_enabled) return;
    
    left_speed = CONSTRAIN(left_speed, SPEED_MIN, SPEED_MAX);
    right_speed = CONSTRAIN(right_speed, SPEED_MIN, SPEED_MAX);
    
    FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, left_speed);
    FourWheel_Set_Target_Speed(MOTOR_REAR_LEFT, left_speed);
    FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, right_speed);
    FourWheel_Set_Target_Speed(MOTOR_REAR_RIGHT, right_speed);
}

// ============================================================================
// 四轮独立控制
// ============================================================================
void Robot_Four_Wheel_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
    if (!pid_enabled) return;
    
    FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, CONSTRAIN(fl, SPEED_MIN, SPEED_MAX));
    FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, CONSTRAIN(fr, SPEED_MIN, SPEED_MAX));
    FourWheel_Set_Target_Speed(MOTOR_REAR_LEFT, CONSTRAIN(rl, SPEED_MIN, SPEED_MAX));
    FourWheel_Set_Target_Speed(MOTOR_REAR_RIGHT, CONSTRAIN(rr, SPEED_MIN, SPEED_MAX));
} 

