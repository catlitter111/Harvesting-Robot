#include "encoder.h"

// ============================================================================
// 全局变量定义
// ============================================================================
Encoder_t encoders[MOTOR_COUNT];

// 编码器配置表(使用config.h中的引脚定义)
static const struct {
    GPIO_TypeDef* port_a; uint16_t pin_a;
    GPIO_TypeDef* port_b; uint16_t pin_b;
} encoder_pins[MOTOR_COUNT] = {
    {ENCODER1_PORT, ENCODER1_A_PIN, ENCODER1_PORT, ENCODER1_B_PIN}, // 电机1
    {ENCODER2_PORT, ENCODER2_A_PIN, ENCODER2_PORT, ENCODER2_B_PIN}, // 电机2
    {ENCODER3_PORT, ENCODER3_A_PIN, ENCODER3_PORT, ENCODER3_B_PIN}, // 电机3
    {ENCODER4_PORT, ENCODER4_A_PIN, ENCODER4_PORT, ENCODER4_B_PIN}  // 电机4
};

// ============================================================================
// 编码器初始化
// ============================================================================
void Encoder_Init(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoders[i].pulse_count = 0;
        encoders[i].last_count = 0;
        encoders[i].speed_rpm = 0;
        encoders[i].speed_percent = 0;
        encoders[i].last_time = HAL_GetTick();
        encoders[i].port_a = encoder_pins[i].port_a;
        encoders[i].pin_a = encoder_pins[i].pin_a;
        encoders[i].port_b = encoder_pins[i].port_b;
        encoders[i].pin_b = encoder_pins[i].pin_b;
        encoders[i].last_state_a = Encoder_Read_Pin(encoders[i].port_a, encoders[i].pin_a);
        encoders[i].last_state_b = Encoder_Read_Pin(encoders[i].port_b, encoders[i].pin_b);
    }
}

// ============================================================================
// 读取编码器引脚状态
// ============================================================================
uint8_t Encoder_Read_Pin(GPIO_TypeDef* port, uint16_t pin) {
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

// ============================================================================
// 处理单个编码器
// ============================================================================
void Encoder_Process_Single(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    
    uint8_t state_a = Encoder_Read_Pin(encoders[motor].port_a, encoders[motor].pin_a);
    uint8_t state_b = Encoder_Read_Pin(encoders[motor].port_b, encoders[motor].pin_b);
    
    // 四倍频编码器解码逻辑
    if (encoders[motor].last_state_a != state_a) {
        if (state_a == state_b) {
            encoders[motor].pulse_count++;  // 正转
        } else {
            encoders[motor].pulse_count--;  // 反转
        }
    }
    
    if (encoders[motor].last_state_b != state_b) {
        if (state_a != state_b) {
            encoders[motor].pulse_count++;  // 正转
        } else {
            encoders[motor].pulse_count--;  // 反转
        }
    }
    
    encoders[motor].last_state_a = state_a;
    encoders[motor].last_state_b = state_b;
}

// ============================================================================
// 读取四轮脉冲
// ============================================================================
void Encoder_Read_All_Pulses(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Encoder_Process_Single((Motor_ID)i);
    }
}

// ============================================================================
// 更新四轮速度
// ============================================================================
void Encoder_Update_All_Speed(void) {
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        uint32_t delta_time = current_time - encoders[i].last_time;
        
        // 避免时间间隔过小导致的计算错误
        if (delta_time < ENCODER_TASK_PERIOD) continue;
        
        int32_t delta_count = encoders[i].pulse_count - encoders[i].last_count;
        
        if (delta_time > 0) {
            // 计算转速RPM
            encoders[i].speed_rpm = (delta_count * 60000) / (ENCODER_PPR * delta_time);
            
            // 转换为速度百分比(-100~+100)
            encoders[i].speed_percent = CONSTRAIN(encoders[i].speed_rpm, SPEED_MIN, SPEED_MAX);
            
            // 速度滤波处理
            Encoder_Filter_Speed((Motor_ID)i);
        }
        
        encoders[i].last_count = encoders[i].pulse_count;
        encoders[i].last_time = current_time;
    }
}

// ============================================================================
// 速度滤波处理
// ============================================================================
void Encoder_Filter_Speed(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    
    // 简单低通滤波器
    static int16_t last_speed[MOTOR_COUNT] = {0};
    
    encoders[motor].speed_percent = (encoders[motor].speed_percent + last_speed[motor]) / 2;
    last_speed[motor] = encoders[motor].speed_percent;
    
    // 死区处理，消除低速噪声
    if (ABS(encoders[motor].speed_percent) < SPEED_DEADZONE) {
        encoders[motor].speed_percent = 0;
    }
}

// ============================================================================
// 获取单轮速度百分比
// ============================================================================
int16_t Encoder_Get_Speed(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return encoders[motor].speed_percent;
}

// ============================================================================
// 获取单轮转速RPM
// ============================================================================
int16_t Encoder_Get_RPM(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return encoders[motor].speed_rpm;
}

// ============================================================================
// 获取单轮脉冲计数
// ============================================================================
int32_t Encoder_Get_Count(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return encoders[motor].pulse_count;
}

// ============================================================================
// 重置编码器计数
// ============================================================================
void Encoder_Reset_Count(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    encoders[motor].pulse_count = 0;
    encoders[motor].last_count = 0;
    encoders[motor].speed_rpm = 0;
    encoders[motor].speed_percent = 0;
}

// ============================================================================
// 重置所有编码器计数
// ============================================================================
void Encoder_Reset_All_Count(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Encoder_Reset_Count((Motor_ID)i);
    }
} 

