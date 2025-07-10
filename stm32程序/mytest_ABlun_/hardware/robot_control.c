#include "robot_control.h"

// ============================================================================
// 全局变量定义
// ============================================================================
Robot_Status_t robot_status = {
    .wheel_speeds = {0, 0, 0, 0},
    .wheel_feedback = {0, 0, 0, 0},
    .mode = ROBOT_MODE_PID,
    .pid_enabled = 0,
    .last_cmd_time = 0
};

// ============================================================================
// 机器人控制初始化
// ============================================================================
void Robot_Control_Init(void) {
    robot_status.mode = ROBOT_MODE_PID;
    robot_status.pid_enabled = 1;
    robot_status.last_cmd_time = HAL_GetTick();
    Robot_Stop();
}

// ============================================================================
// 设置控制模式
// ============================================================================
void Robot_Set_Mode(Robot_Control_Mode mode) {
    if (mode != robot_status.mode) {
        Robot_Stop();  // 切换模式前先停止
        robot_status.mode = mode;
        
        if (mode == ROBOT_MODE_PID) {
            Robot_Enable_PID(1);
            robot_status.pid_enabled = 1;
        } else {
            Robot_Enable_PID(0);
            robot_status.pid_enabled = 0;
        }
    }
}

// ============================================================================
// 获取控制模式
// ============================================================================
Robot_Control_Mode Robot_Get_Mode(void) {
    return robot_status.mode;
}

// ============================================================================
// 四轮同速设置
// ============================================================================
void Robot_Set_Speed_All(int16_t speed) {
    speed = CONSTRAIN(speed, SPEED_MIN, SPEED_MAX);
    Robot_Set_Speed_Individual(speed, speed, speed, speed);
}

// ============================================================================
// 四轮独立设置
// ============================================================================
void Robot_Set_Speed_Individual(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
    // 参数约束
    fl = CONSTRAIN(fl, SPEED_MIN, SPEED_MAX);
    fr = CONSTRAIN(fr, SPEED_MIN, SPEED_MAX);
    rl = CONSTRAIN(rl, SPEED_MIN, SPEED_MAX);
    rr = CONSTRAIN(rr, SPEED_MIN, SPEED_MAX);
    
    // 保存目标速度
    robot_status.wheel_speeds[MOTOR_FRONT_LEFT] = fl;
    robot_status.wheel_speeds[MOTOR_FRONT_RIGHT] = fr;
    robot_status.wheel_speeds[MOTOR_REAR_LEFT] = rl;
    robot_status.wheel_speeds[MOTOR_REAR_RIGHT] = rr;
    
    // 应用控制
    if (robot_status.mode == ROBOT_MODE_PID && robot_status.pid_enabled) {
        FourWheel_Set_Target_Speed(MOTOR_FRONT_LEFT, fl);
        FourWheel_Set_Target_Speed(MOTOR_FRONT_RIGHT, fr);
        FourWheel_Set_Target_Speed(MOTOR_REAR_LEFT, rl);
        FourWheel_Set_Target_Speed(MOTOR_REAR_RIGHT, rr);
    } else {
        Motor_SetSpeed(MOTOR_FRONT_LEFT, (int8_t)fl);
        Motor_SetSpeed(MOTOR_FRONT_RIGHT, (int8_t)fr);
        Motor_SetSpeed(MOTOR_REAR_LEFT, (int8_t)rl);
        Motor_SetSpeed(MOTOR_REAR_RIGHT, (int8_t)rr);
    }
    
    robot_status.last_cmd_time = HAL_GetTick();
}

// ============================================================================
// 左右差速设置
// ============================================================================
void Robot_Set_Speed_Left_Right(int16_t left, int16_t right) {
    Robot_Set_Speed_Individual(left, right, left, right);
}

// ============================================================================
// 前进
// ============================================================================
void Robot_Move_Forward(int16_t speed) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    Robot_Set_Speed_All(speed);
}

// ============================================================================
// 后退
// ============================================================================
void Robot_Move_Backward(int16_t speed) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    Robot_Set_Speed_All(-speed);
}

// ============================================================================
// 左转(原地转向)
// ============================================================================
void Robot_Turn_Left(int16_t speed) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    Robot_Set_Speed_Individual(-speed, speed, -speed, speed);
}

// ============================================================================
// 右转(原地转向)
// ============================================================================
void Robot_Turn_Right(int16_t speed) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    Robot_Set_Speed_Individual(speed, -speed, speed, -speed);
}

// ============================================================================
// 左转弯(弧线)
// ============================================================================
void Robot_Arc_Left(int16_t speed, int16_t radius) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    radius = CONSTRAIN(radius, 10, 100);  // 10%-100%差速比
    
    int16_t outer_speed = speed;
    int16_t inner_speed = (speed * radius) / 100;
    Robot_Set_Speed_Individual(inner_speed, outer_speed, inner_speed, outer_speed);
}

// ============================================================================
// 右转弯(弧线)
// ============================================================================
void Robot_Arc_Right(int16_t speed, int16_t radius) {
    speed = CONSTRAIN(speed, 0, SPEED_MAX);
    radius = CONSTRAIN(radius, 10, 100);  // 10%-100%差速比
    
    int16_t outer_speed = speed;
    int16_t inner_speed = (speed * radius) / 100;
    Robot_Set_Speed_Individual(outer_speed, inner_speed, outer_speed, inner_speed);
}

// ============================================================================
// 立即停止
// ============================================================================
void Robot_Stop(void) {
    Robot_Set_Speed_All(0);
}

// ============================================================================
// 制动停止
// ============================================================================
void Robot_Brake(uint16_t brake_time_ms) {
    Robot_Stop();
    HAL_Delay(brake_time_ms);
}

// ============================================================================
// 获取四轮目标速度
// ============================================================================
void Robot_Get_Speeds(int16_t* speeds) {
    if (speeds) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            speeds[i] = robot_status.wheel_speeds[i];
        }
    }
}

// ============================================================================
// 获取四轮反馈速度
// ============================================================================
void Robot_Get_Feedback(int16_t* feedback) {
    if (feedback) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            robot_status.wheel_feedback[i] = Encoder_Get_Speed((Motor_ID)i);
            feedback[i] = robot_status.wheel_feedback[i];
        }
    }
}

// ============================================================================
// 获取完整状态
// ============================================================================
Robot_Status_t* Robot_Get_Status(void) {
    // 更新反馈速度
    Robot_Get_Feedback(robot_status.wheel_feedback);
    robot_status.pid_enabled = Robot_Get_PID_Status();
    return &robot_status;
}

// ============================================================================
// 判断是否运动中
// ============================================================================
uint8_t Robot_Is_Moving(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (ABS(robot_status.wheel_speeds[i]) > SPEED_DEADZONE) {
            return 1;
        }
    }
    return 0;
}

// ============================================================================
// 判断PID是否使能
// ============================================================================
uint8_t Robot_Is_PID_Enabled(void) {
    return robot_status.pid_enabled;
}

// ============================================================================
// 路径跟踪(预留功能)
// ============================================================================
void Robot_Follow_Path(int16_t* speeds, uint8_t count, uint16_t interval) {
    // 预留功能，后续可扩展为路径跟踪控制
    for (uint8_t i = 0; i < count && i < MOTOR_COUNT; i++) {
        Robot_Set_Speed_Individual(speeds[0], speeds[1], speeds[2], speeds[3]);
        HAL_Delay(interval);
    }
}

// ============================================================================
// 紧急停止
// ============================================================================
void Robot_Emergency_Stop(void) {
    Robot_Enable_PID(0);  // 先禁用PID
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetSpeed((Motor_ID)i, 0);  // 直接停止所有电机
    }
    robot_status.pid_enabled = 0;
    Robot_Stop();
}

// ============================================================================
// 重置控制系统
// ============================================================================
void Robot_Reset_Control(void) {
    Robot_Emergency_Stop();
    FourWheel_Reset_PID();
    Encoder_Reset_All_Count();
    HAL_Delay(100);  // 等待系统稳定
    Robot_Control_Init();  // 重新初始化
} 


