#include "motor.h"

// PWM配置
#define PWM_FREQUENCY 1000 // Hz
#define PWM_DEFAULT_DUTY 50 // %

// 当前电机速度
static int8_t motor_speeds[MOTOR_COUNT_NUM] = {0};
static uint8_t robot_speed = PWM_DEFAULT_DUTY;

// 电机配置数组
static const Motor_Config motor_configs[MOTOR_COUNT_NUM] = {
    // MOTOR_REAR_RIGHT (后右电机)
    {
        .htim = &htim1,
        .channel = TIM_CHANNEL_3,
        .pwm_port = GPIOA,
        .pwm_pin = GPIO_PIN_10,
        .in1_port = GPIOB,
        .in1_pin = GPIO_PIN_1,
        .in2_port = GPIOB,
        .in2_pin = GPIO_PIN_0
    },
    // MOTOR_FRONT_RIGHT (前右电机)
    {
        .htim = &htim1,
        .channel = TIM_CHANNEL_4,
        .pwm_port = GPIOA,
        .pwm_pin = GPIO_PIN_11,
        .in1_port = GPIOB,
        .in1_pin = GPIO_PIN_10,
        .in2_port = GPIOB,
        .in2_pin = GPIO_PIN_11
    },
    // MOTOR_REAR_LEFT (后左电机)
    {
        .htim = &htim1,
        .channel = TIM_CHANNEL_1,
        .pwm_port = GPIOA,
        .pwm_pin = GPIO_PIN_8,
        .in1_port = GPIOB,
        .in1_pin = GPIO_PIN_4,
        .in2_port = GPIOB,
        .in2_pin = GPIO_PIN_3
    },
    // MOTOR_FRONT_LEFT (前左电机)
    {
        .htim = &htim1,
        .channel = TIM_CHANNEL_2,
        .pwm_port = GPIOA,
        .pwm_pin = GPIO_PIN_9,
        .in1_port = GPIOB,
        .in1_pin = GPIO_PIN_8,
        .in2_port = GPIOB,
        .in2_pin = GPIO_PIN_9
    }
};

/**
 * @brief 初始化所有电机的PWM
 * @return HAL状态
 */
HAL_StatusTypeDef Motor_Init(void)
{
    // 启动所有电机PWM
    for (int i = 0; i < MOTOR_COUNT_NUM; i++) {
        // 启动PWM
        if (HAL_TIM_PWM_Start(motor_configs[i].htim, motor_configs[i].channel) != HAL_OK) {
            return HAL_ERROR;
        }
        
        // 根据设定频率的定时器配置
        uint32_t timer_clock = 72000000;
        uint32_t pwm_arr = (timer_clock / PWM_FREQUENCY) - 1;
        
        // 设置定时器周期
        __HAL_TIM_SET_AUTORELOAD(motor_configs[i].htim, pwm_arr);
        HAL_TIM_GenerateEvent(motor_configs[i].htim, TIM_EVENTSOURCE_UPDATE);
        
        // 设置默认占空比
        uint32_t pwm_ccr = ((pwm_arr + 1) * PWM_DEFAULT_DUTY) / 100;
        __HAL_TIM_SET_COMPARE(motor_configs[i].htim, motor_configs[i].channel, pwm_ccr);
    }
    
    // 初始状态设为停止
    Robot_Move(ROBOT_DIR_STOP);
    
    return HAL_OK;
}

/**
 * @brief 设置指定电机的PWM占空比
 * @param motor 电机标识
 * @param duty_cycle PWM占空比(0-100%)
 * @return HAL状态
 */
HAL_StatusTypeDef Motor_SetPWM(Motor_ID motor, uint8_t duty_cycle)
{
    if (motor >= MOTOR_COUNT_NUM || duty_cycle > 100) {
        return HAL_ERROR;  // 参数无效
    }
    
    // 获取当前定时器周期
    uint32_t pwm_arr = __HAL_TIM_GET_AUTORELOAD(motor_configs[motor].htim);
    
    // 根据占空比计算新的比较值
    uint32_t pwm_ccr = ((pwm_arr + 1) * duty_cycle) / 100;
    
    // 设置新的比较值
    __HAL_TIM_SET_COMPARE(motor_configs[motor].htim, 
                          motor_configs[motor].channel, 
                          pwm_ccr);
    
    return HAL_OK;
}

/**
 * @brief 设置指定电机的方向
 * @param motor 电机标识
 * @param direction 运动方向
 */
void Motor_SetDirection(Motor_ID motor, Motor_Direction direction)
{
    if (motor >= MOTOR_COUNT_NUM) {
        return;  // 无效的电机ID
    }
    
    const Motor_Config* config = &motor_configs[motor];
    
    // 启用PWM输出
    HAL_GPIO_WritePin(config->pwm_port, config->pwm_pin, GPIO_PIN_SET);
    
    switch (direction) {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_SET);
            break;
        
        case MOTOR_DIR_BACKWARD:
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
            break;
        
        case MOTOR_DIR_STOP:
        default:
            HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
            break;
    }
}

/**
 * @brief 设置电机速度和方向
 * @param motor 电机标识
 * @param speed 速度值(-100到+100)，负数表示反转
 */
void Motor_SetSpeed(Motor_ID motor, int8_t speed)
{
    if (motor >= MOTOR_COUNT_NUM) {
        return;  // 无效的电机ID
    }
    
    // 存储当前速度
    motor_speeds[motor] = speed;
    
    // 确定方向和绝对速度
    Motor_Direction direction;
    uint8_t abs_speed;
    
    if (speed > 0) {
        direction = MOTOR_DIR_FORWARD;
        abs_speed = (uint8_t)speed;
    } else if (speed < 0) {
        direction = MOTOR_DIR_BACKWARD;
        abs_speed = (uint8_t)(-speed);
    } else {
        direction = MOTOR_DIR_STOP;
        abs_speed = 0;
    }
    
    // 设置方向
    Motor_SetDirection(motor, direction);
    
    // 如果非停止，设置PWM占空比
    if (direction != MOTOR_DIR_STOP) {
        Motor_SetPWM(motor, abs_speed);
    }
}

/**
 * @brief 设置机器人移动方向
 * @param direction 机器人移动方向
 */
void Robot_Move(Robot_Direction direction)
{
    switch (direction) {
        case ROBOT_DIR_FORWARD:
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_FORWARD);
            // 应用当前速度
            Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_BACKWARD:
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_BACKWARD);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_BACKWARD);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_BACKWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_BACKWARD);
            // 应用当前速度
            Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_LEFT:
            // 左转：前轮停止，后轮差动实现坦克式转弯
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_BACKWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_FORWARD);
            // 应用当前速度
            // Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            // Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_RIGHT:
            // 右转：前轮停止，后轮差动实现坦克式转弯
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_BACKWARD);
            // 应用当前速度
            // Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            // Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_STOP:
        default:
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_STOP);
            break;
    }
}

/**
 * @brief 设置全局机器人速度
 * @param speed 速度值(0-100%)
 */
void Robot_SetSpeed(uint8_t speed)
{
    if (speed > 100) {
        speed = 100;
    }
    
    robot_speed = speed;
    
    // 更新所有运行中的电机速度
    for (int i = 0; i < MOTOR_COUNT_NUM; i++) {
        if (motor_speeds[i] != 0) {
            // 只更新正在运行的电机
            uint8_t abs_speed = motor_speeds[i] > 0 ? motor_speeds[i] : -motor_speeds[i];
            uint8_t scaled_speed = (uint8_t)(((uint16_t)abs_speed * speed) / 100);
            Motor_SetPWM((Motor_ID)i, scaled_speed);
        }
    }
}

/**
 * @brief 电机测试函数
 */
void Motor_Test(void)
{
    Robot_SetSpeed(30);

    // 前进
    Robot_Move(ROBOT_DIR_FORWARD);
    HAL_Delay(2000); // 前进2秒
    
    // 后退
    Robot_Move(ROBOT_DIR_BACKWARD);
    HAL_Delay(2000); // 后退2秒
    
    // 左转
    Robot_Move(ROBOT_DIR_LEFT);
    HAL_Delay(1000); // 左转1秒
    
    // 右转
    Robot_Move(ROBOT_DIR_RIGHT);
    HAL_Delay(1000); // 右转1秒
    
    // 停止
    // Robot_Move(ROBOT_DIR_STOP);
    // HAL_Delay(1000); // 暂停1秒
    
    // /* 示例2：速度控制 */
    // // 设置全局速度为70%
    // Robot_SetSpeed(60);
    
    // // 前进
    // Robot_Move(ROBOT_DIR_FORWARD);
    // HAL_Delay(2000);
    
    // // 提升为最高速度
    // Robot_SetSpeed(100);
    // HAL_Delay(2000);
    
    // // 停止
    // Robot_Move(ROBOT_DIR_STOP);
    
    // /* 示例3：独立控制每个电机 */
    // // 前左电机80%的速度前进
    // Motor_SetSpeed(MOTOR_FRONT_LEFT, 70);
    
    // // 前右电机50%的速度后退
    // Motor_SetSpeed(MOTOR_FRONT_RIGHT, -50);
    
    // // 后左电机停止
    // Motor_SetSpeed(MOTOR_REAR_LEFT, 0);
    
    // // 后右电机60%的速度前进
    // Motor_SetSpeed(MOTOR_REAR_RIGHT, 60);
    
    // HAL_Delay(3000);
    
    // 所有电机停止
    Robot_Move(ROBOT_DIR_STOP);
}


