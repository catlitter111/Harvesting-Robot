#include "motor.h"

// PWM����
#define PWM_FREQUENCY 1000 // Hz
#define PWM_DEFAULT_DUTY 50 // %

// ��ǰ����ٶ�
static int8_t motor_speeds[MOTOR_COUNT] = {0};
static uint8_t robot_speed = PWM_DEFAULT_DUTY;

// �������
static const Motor_Config motor_configs[MOTOR_COUNT] = {
    // MOTOR_FRONT_LEFT (�Һ���)
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
    // MOTOR_FRONT_RIGHT (��ǰ���)
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
    // MOTOR_REAR_LEFT (�����)
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
    // MOTOR_REAR_RIGHT (��ǰ���)
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
 * @brief ��ʼ�����е����PWM
 * @return HAL״̬
 */
HAL_StatusTypeDef Motor_Init(void)
{
    // �������е����PWM
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // ����PWM
        if (HAL_TIM_PWM_Start(motor_configs[i].htim, motor_configs[i].channel) != HAL_OK) {
            return HAL_ERROR;
        }
        
        // ��������Ƶ�ʵĶ�ʱ������
        uint32_t timer_clock = 72000000;
        uint32_t pwm_arr = (timer_clock / PWM_FREQUENCY) - 1;
        
        // ���ö�ʱ������
        __HAL_TIM_SET_AUTORELOAD(motor_configs[i].htim, pwm_arr);
        HAL_TIM_GenerateEvent(motor_configs[i].htim, TIM_EVENTSOURCE_UPDATE);
        
        // ����Ĭ��ռ�ձ�
        uint32_t pwm_ccr = ((pwm_arr + 1) * PWM_DEFAULT_DUTY) / 100;
        __HAL_TIM_SET_COMPARE(motor_configs[i].htim, motor_configs[i].channel, pwm_ccr);
    }
    
    // ��ʼ״̬��Ϊֹͣ
    Robot_Move(ROBOT_DIR_STOP);
    
    return HAL_OK;
}

/**
 * @brief �����ض������PWMռ�ձ�
 * @param motor �����ʶ
 * @param duty_cycle PWMռ�ձ�(0-100%)
 * @return HAL״̬
 */
HAL_StatusTypeDef Motor_SetPWM(Motor_ID motor, uint8_t duty_cycle)
{
    if (motor >= MOTOR_COUNT || duty_cycle > 100) {
        return HAL_ERROR;  // ������Ч
    }
    
    // ��ȡ��ǰ��ʱ������
    uint32_t pwm_arr = __HAL_TIM_GET_AUTORELOAD(motor_configs[motor].htim);
    
    // ����ռ�ձȼ����µıȽ�ֵ
    uint32_t pwm_ccr = ((pwm_arr + 1) * duty_cycle) / 100;
    
    // �����µıȽ�ֵ
    __HAL_TIM_SET_COMPARE(motor_configs[motor].htim, 
                          motor_configs[motor].channel, 
                          pwm_ccr);
    
    return HAL_OK;
}

/**
 * @brief �����ض�����ķ���
 * @param motor �����ʶ
 * @param direction �������
 */
void Motor_SetDirection(Motor_ID motor, Motor_Direction direction)
{
    if (motor >= MOTOR_COUNT) {
        return;  // ��Ч�ĵ��ID
    }
    
    const Motor_Config* config = &motor_configs[motor];
    
    // ����PWM���
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
 * @brief ���õ�����ٶȺͷ���
 * @param motor �����ʶ
 * @param speed �ٶ�ֵ(-100��+100)��������ʾ����
 */
void Motor_SetSpeed(Motor_ID motor, int8_t speed)
{
    if (motor >= MOTOR_COUNT) {
        return;  // ��Ч�ĵ��ID
    }
    
    // �洢��ǰ�ٶ�
    motor_speeds[motor] = speed;
    
    // ȷ������;����ٶ�
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
    
    // ���÷���
    Motor_SetDirection(motor, direction);
    
    // �������ֹͣ������PWMռ�ձ�
    if (direction != MOTOR_DIR_STOP) {
        Motor_SetPWM(motor, abs_speed);
    }
}

/**
 * @brief ���û������ƶ�����
 * @param direction �������ƶ�����
 */
void Robot_Move(Robot_Direction direction)
{
    switch (direction) {
        case ROBOT_DIR_FORWARD:
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_FORWARD);
            // Ӧ�õ�ǰ�ٶ�
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
            // Ӧ�õ�ǰ�ٶ�
            Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_LEFT:
            // ����������Ҳ�������ʵ��̹��ʽת��
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_BACKWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_FORWARD);
            // Ӧ�õ�ǰ�ٶ�
            // Motor_SetPWM(MOTOR_FRONT_LEFT, robot_speed);
            // Motor_SetPWM(MOTOR_FRONT_RIGHT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_LEFT, robot_speed);
            Motor_SetPWM(MOTOR_REAR_RIGHT, robot_speed);
            break;
        
        case ROBOT_DIR_RIGHT:
            // ����������Ҳ�������ʵ��̹��ʽת��
            Motor_SetDirection(MOTOR_FRONT_LEFT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_FRONT_RIGHT, MOTOR_DIR_STOP);
            Motor_SetDirection(MOTOR_REAR_LEFT, MOTOR_DIR_FORWARD);
            Motor_SetDirection(MOTOR_REAR_RIGHT, MOTOR_DIR_BACKWARD);
            // Ӧ�õ�ǰ�ٶ�
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
 * @brief ����ȫ�ֻ������ٶ�
 * @param speed �ٶ�ֵ(0-100%)
 */
void Robot_SetSpeed(uint8_t speed)
{
    if (speed > 100) {
        speed = 100;
    }
    
    robot_speed = speed;
    
    // �������������е�����ٶ�
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motor_speeds[i] != 0) {
            // ֻ�����������еĵ��
            uint8_t abs_speed = motor_speeds[i] > 0 ? motor_speeds[i] : -motor_speeds[i];
            uint8_t scaled_speed = (uint8_t)(((uint16_t)abs_speed * speed) / 100);
            Motor_SetPWM((Motor_ID)i, scaled_speed);
        }
    }
}

void Motor_Test(){

    Robot_SetSpeed(30);

    Robot_Move(ROBOT_DIR_FORWARD);
    HAL_Delay(2000); // ǰ��2��
    
    // ����
    Robot_Move(ROBOT_DIR_BACKWARD);
    HAL_Delay(2000); // ����2��
    
    // ��ת
    Robot_Move(ROBOT_DIR_LEFT);
    HAL_Delay(1000); // ��ת1��
    
    // ��ת
    Robot_Move(ROBOT_DIR_RIGHT);
    HAL_Delay(1000); // ��ת1��
    
    // // ֹͣ
    // Robot_Move(ROBOT_DIR_STOP);
    // HAL_Delay(1000); // ��ͣ1��
    
    // /* ʾ��2���ٶȿ��� */
    // // ����ȫ���ٶ�Ϊ70%
    // Robot_SetSpeed(60);
    
    // // ǰ��
    // Robot_Move(ROBOT_DIR_FORWARD);
    // HAL_Delay(2000);
    
    // // ����Ϊ����ٶ�
    // Robot_SetSpeed(100);
    // HAL_Delay(2000);
    
    // // ֹͣ
    // Robot_Move(ROBOT_DIR_STOP);
    
    // /* ʾ��3����������ÿ����� */
    // // ǰ������80%���ٶ�ǰ��
    // Motor_SetSpeed(MOTOR_FRONT_LEFT, 70);
    
    // // ǰ�ҵ����50%���ٶȺ���
    // Motor_SetSpeed(MOTOR_FRONT_RIGHT, -50);
    
    // // ������ֹͣ
    // Motor_SetSpeed(MOTOR_REAR_LEFT, 0);
    
    // // ���ҵ����60%���ٶ�ǰ��
    // Motor_SetSpeed(MOTOR_REAR_RIGHT, 60);
    
    // HAL_Delay(3000);
    
    // ���е��ֹͣ
    Robot_Move(ROBOT_DIR_STOP);

}

