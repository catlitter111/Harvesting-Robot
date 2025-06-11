#include "encoder_520.h"
#include "encoder_520_config.h"

// ============================================================================
// 全局变量定义
// ============================================================================
Encoder520_t encoders_520[MOTOR_COUNT];
int Encoder_Left = 0, Encoder_Right = 0;  // 兼容原有接口

// 定时器配置表 - 根据实际硬件配置修改
static const EncoderTim_Config_t encoder_tim_configs[MOTOR_COUNT] = {
    {&htim2, TIM_CHANNEL_ALL, 1},     // 电机1: TIM2, 正方向
    {&htim4, TIM_CHANNEL_ALL, -1},    // 电机2: TIM4, 反方向
    {&htim3, TIM_CHANNEL_ALL, 1},     // 电机3: TIM3, 正方向(需要配置为编码器模式)
    {&htim1, TIM_CHANNEL_ALL, -1}     // 电机4: TIM1, 反方向(需要配置为编码器模式)
};

// ============================================================================
// 520编码器初始化
// ============================================================================
void Encoder520_Init(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoders_520[i].htim = encoder_tim_configs[i].htim;
        encoders_520[i].count_current = 0;
        encoders_520[i].count_last = 0;
        encoders_520[i].speed_raw = 0;
        encoders_520[i].speed_filtered = 0;
        encoders_520[i].speed_rpm = 0.0f;
        encoders_520[i].speed_ms = 0.0f;
        encoders_520[i].last_time = HAL_GetTick();
        encoders_520[i].direction = 0;
    }
}

// ============================================================================
// 启动所有编码器
// ============================================================================
void Encoder520_Start_All(void) {
    // 只启动已配置为编码器模式的定时器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 电机1
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 电机2
    // 注意：htim3和htim1需要在CubeMX中配置为编码器模式才能使用
}

// ============================================================================
// 停止所有编码器
// ============================================================================
void Encoder520_Stop_All(void) {
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
}

// ============================================================================
// 读取定时器计数值
// ============================================================================
int16_t Encoder520_Read_Timer_Count(TIM_HandleTypeDef* htim) {
    int16_t temp = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);  // 清零计数器
    return temp;
}

// ============================================================================
// 更新单个编码器速度
// ============================================================================
void Encoder520_Update_Speed(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t delta_time = current_time - encoders_520[motor].last_time;
    
    // 避免时间间隔过小
    if (delta_time < ENCODER_520_SAMPLE_PERIOD) return;
    
    // 读取编码器计数
    encoders_520[motor].count_current = Encoder520_Read_Timer_Count(encoders_520[motor].htim);
    
    // 应用方向系数
    encoders_520[motor].count_current *= encoder_tim_configs[motor].direction_factor;
    
    // 计算原始速度值
    encoders_520[motor].speed_raw = encoders_520[motor].count_current;
    
    // 速度滤波
    Encoder520_Filter_Speed(motor);
    
    // 计算RPM
    if (delta_time > 0) {
        encoders_520[motor].speed_rpm = (encoders_520[motor].speed_filtered * 60000.0f) / 
                                       (ENCODER_520_PPR * delta_time);
    }
    
    // 计算线速度(m/s)
    encoders_520[motor].speed_ms = (encoders_520[motor].speed_rpm * PI * 
                                   ENCODER_520_WHEEL_DIAMETER / 1000.0f) / 60.0f;
    
    // 更新方向
    encoders_520[motor].direction = (encoders_520[motor].speed_filtered >= 0) ? 0 : 1;
    
    // 更新时间
    encoders_520[motor].last_time = current_time;
}

// ============================================================================
// 更新所有编码器速度
// ============================================================================
void Encoder520_Update_All_Speed(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Encoder520_Update_Speed((Motor_ID)i);
    }
    
    // 更新兼容性变量
    Encoder_Compatibility_Update();
}

// ============================================================================
// 速度滤波处理
// ============================================================================
void Encoder520_Filter_Speed(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    
    // 一阶低通滤波器
    encoders_520[motor].speed_filtered = (int16_t)(
        ENCODER_520_SPEED_FILTER * encoders_520[motor].speed_raw + 
        (1.0f - ENCODER_520_SPEED_FILTER) * encoders_520[motor].speed_filtered
    );
}

// ============================================================================
// 读取编码器速度
// ============================================================================
int16_t Encoder520_Read_Speed(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return encoders_520[motor].speed_filtered;
}

// ============================================================================
// 读取编码器计数
// ============================================================================
int16_t Encoder520_Read_Count(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0;
    return encoders_520[motor].count_current;
}

// ============================================================================
// 获取转速RPM
// ============================================================================
float Encoder520_Get_RPM(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0.0f;
    return encoders_520[motor].speed_rpm;
}

// ============================================================================
// 获取线速度m/s
// ============================================================================
float Encoder520_Get_Speed_MS(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return 0.0f;
    return encoders_520[motor].speed_ms;
}

// ============================================================================
// 重置编码器计数
// ============================================================================
void Encoder520_Reset_Count(Motor_ID motor) {
    if (motor >= MOTOR_COUNT) return;
    
    encoders_520[motor].count_current = 0;
    encoders_520[motor].count_last = 0;
    encoders_520[motor].speed_raw = 0;
    encoders_520[motor].speed_filtered = 0;
    encoders_520[motor].speed_rpm = 0.0f;
    encoders_520[motor].speed_ms = 0.0f;
    
    // 清零硬件计数器
    if (encoders_520[motor].htim != NULL) {
        __HAL_TIM_SET_COUNTER(encoders_520[motor].htim, 0);
    }
}

// ============================================================================
// 重置所有编码器计数
// ============================================================================
void Encoder520_Reset_All_Count(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Encoder520_Reset_Count((Motor_ID)i);
    }
}

// ============================================================================
// 兼容性接口函数实现
// ============================================================================

// 读取左轮速度(兼容UpStanding_Car接口)
int Read_Speed_Left(void) {
    return Encoder520_Read_Speed((Motor_ID)0);  // 左轮对应电机0
}

// 读取右轮速度(兼容UpStanding_Car接口)
int Read_Speed_Right(void) {
    return Encoder520_Read_Speed((Motor_ID)1);  // 右轮对应电机1
}

// 兼容性更新函数
void Encoder_Compatibility_Update(void) {
    Encoder_Left = Read_Speed_Left();
    Encoder_Right = Read_Speed_Right();
}

// ============================================================================
// 原UpStanding_Car兼容函数(直接使用定时器句柄)
// ============================================================================
int Read_Speed(TIM_HandleTypeDef *htim) {
    int temp = (short)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return temp;
} 