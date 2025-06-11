#include "encoder_520_example.h"
#include "encoder_520.h"
#include <stdio.h>

// ============================================================================
// 520编码器使用示例代码
// ============================================================================

// ============================================================================
// 示例1: 基本编码器测试
// ============================================================================
void Encoder520_Basic_Test(void) {
    printf("=== 520编码器基本测试开始 ===\r\n");
    
    // 初始化编码器
    Encoder520_Init();
    Encoder520_Start_All();
    
    uint32_t test_count = 0;
    while(test_count < 100) {  // 测试100次
        // 更新编码器数据
        Encoder520_Update_All_Speed();
        
        // 打印所有编码器数据
        printf("测试%ld: ", test_count);
        for(uint8_t i = 0; i < MOTOR_COUNT; i++) {
            printf("电机%d[速度:%d RPM:%.1f] ", 
                   i, 
                   Encoder520_Read_Speed((Motor_ID)i),
                   Encoder520_Get_RPM((Motor_ID)i));
        }
        printf("\r\n");
        
        HAL_Delay(100);
        test_count++;
    }
    
    printf("=== 520编码器基本测试结束 ===\r\n");
}

// ============================================================================
// 示例2: 兼容性接口测试
// ============================================================================
void Encoder520_Compatibility_Test(void) {
    printf("=== 兼容性接口测试开始 ===\r\n");
    
    // 初始化
    Encoder520_Init();
    Encoder520_Start_All();
    
    for(uint8_t i = 0; i < 50; i++) {
        // 更新编码器数据
        Encoder520_Update_All_Speed();
        
        // 使用兼容性接口
        int left_speed = Read_Speed_Left();
        int right_speed = Read_Speed_Right();
        
        // 检查全局变量是否更新
        printf("测试%d: 左轮=%d 右轮=%d (全局变量: L=%d R=%d)\r\n", 
               i, left_speed, right_speed, Encoder_Left, Encoder_Right);
        
        HAL_Delay(100);
    }
    
    printf("=== 兼容性接口测试结束 ===\r\n");
}

// ============================================================================
// 示例3: 替换原有PID控制中的编码器调用
// ============================================================================
void Encoder520_PID_Integration_Example(void) {
    // 假设的PID参数
    float target_speed = 50.0f;  // 目标速度
    static float pid_integral_left = 0.0f;
    static float pid_integral_right = 0.0f;
    static float last_error_left = 0.0f;
    static float last_error_right = 0.0f;
    
    // PID参数
    const float Kp = 2.0f;
    const float Ki = 0.1f;
    const float Kd = 0.5f;
    
    printf("=== PID控制集成示例开始 ===\r\n");
    
    // 初始化编码器
    Encoder520_Init();
    Encoder520_Start_All();
    
    for(uint16_t i = 0; i < 200; i++) {
        // 更新编码器数据
        Encoder520_Update_All_Speed();
        
        // 获取当前速度反馈
        float current_speed_left = (float)Encoder520_Read_Speed((Motor_ID)0);
        float current_speed_right = (float)Encoder520_Read_Speed((Motor_ID)1);
        
        // 计算误差
        float error_left = target_speed - current_speed_left;
        float error_right = target_speed - current_speed_right;
        
        // PID计算 - 左轮
        pid_integral_left += error_left;
        pid_integral_left = CONSTRAIN(pid_integral_left, -100, 100);  // 积分限幅
        float derivative_left = error_left - last_error_left;
        float pid_output_left = Kp * error_left + Ki * pid_integral_left + Kd * derivative_left;
        last_error_left = error_left;
        
        // PID计算 - 右轮
        pid_integral_right += error_right;
        pid_integral_right = CONSTRAIN(pid_integral_right, -100, 100);  // 积分限幅
        float derivative_right = error_right - last_error_right;
        float pid_output_right = Kp * error_right + Ki * pid_integral_right + Kd * derivative_right;
        last_error_right = error_right;
        
        // 输出限制
        pid_output_left = CONSTRAIN(pid_output_left, -100, 100);
        pid_output_right = CONSTRAIN(pid_output_right, -100, 100);
        
        // 打印调试信息
        if(i % 10 == 0) {  // 每10次打印一次
            printf("步骤%d: 左轮[当前:%.1f 目标:%.1f 输出:%.1f] 右轮[当前:%.1f 目标:%.1f 输出:%.1f]\r\n",
                   i, current_speed_left, target_speed, pid_output_left,
                   current_speed_right, target_speed, pid_output_right);
        }
        
        // 这里应该将pid_output_left和pid_output_right输出到电机
        // Motor_Set_Speed((Motor_ID)0, (int16_t)pid_output_left);
        // Motor_Set_Speed((Motor_ID)1, (int16_t)pid_output_right);
        
        HAL_Delay(20);  // 20ms控制周期
    }
    
    printf("=== PID控制集成示例结束 ===\r\n");
}

// ============================================================================
// 示例4: 编码器校准和调试
// ============================================================================
void Encoder520_Calibration_Debug(void) {
    printf("=== 编码器校准和调试开始 ===\r\n");
    
    // 初始化
    Encoder520_Init();
    Encoder520_Start_All();
    
    // 重置所有计数器
    Encoder520_Reset_All_Count();
    
    printf("请手动转动各个编码器，观察计数和方向是否正确...\r\n");
    printf("按格式显示: [编码器ID] 计数值 | 原始速度 | 滤波速度 | RPM | 方向\r\n");
    
    for(uint16_t i = 0; i < 300; i++) {  // 运行30秒
        Encoder520_Update_All_Speed();
        
        // 每隔1秒打印一次详细信息
        if(i % 10 == 0) {
            printf("\r\n--- 第%d秒调试信息 ---\r\n", i/10);
            
            for(uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
                printf("[电机%d] 计数:%6d | 原始:%6d | 滤波:%6d | RPM:%7.1f | 方向:%s\r\n",
                       motor,
                       Encoder520_Read_Count((Motor_ID)motor),
                       encoders_520[motor].speed_raw,
                       encoders_520[motor].speed_filtered,
                       Encoder520_Get_RPM((Motor_ID)motor),
                       encoders_520[motor].direction == 0 ? "正转" : "反转");
            }
            
            // 检查兼容性变量
            printf("兼容性变量: Encoder_Left=%d, Encoder_Right=%d\r\n", 
                   Encoder_Left, Encoder_Right);
        }
        
        HAL_Delay(100);
    }
    
    printf("=== 编码器校准和调试结束 ===\r\n");
}

// ============================================================================
// 示例5: 性能对比测试
// ============================================================================
void Encoder520_Performance_Test(void) {
    printf("=== 性能对比测试开始 ===\r\n");
    
    uint32_t start_time, end_time, duration;
    
    // 初始化
    Encoder520_Init();
    Encoder520_Start_All();
    
    // 测试520编码器性能
    printf("测试520编码器更新速度...\r\n");
    start_time = HAL_GetTick();
    
    for(uint16_t i = 0; i < 1000; i++) {
        Encoder520_Update_All_Speed();
    }
    
    end_time = HAL_GetTick();
    duration = end_time - start_time;
    
    printf("520编码器: 1000次更新耗时 %ldms, 平均每次 %.2fms\r\n", 
           duration, (float)duration / 1000.0f);
    
    // 测试兼容性接口性能
    printf("测试兼容性接口速度...\r\n");
    start_time = HAL_GetTick();
    
    for(uint16_t i = 0; i < 1000; i++) {
        Read_Speed_Left();
        Read_Speed_Right();
        Encoder_Compatibility_Update();
    }
    
    end_time = HAL_GetTick();
    duration = end_time - start_time;
    
    printf("兼容性接口: 1000次调用耗时 %ldms, 平均每次 %.2fms\r\n", 
           duration, (float)duration / 1000.0f);
    
    printf("=== 性能对比测试结束 ===\r\n");
}

// ============================================================================
// 示例6: 故障诊断
// ============================================================================
void Encoder520_Fault_Diagnosis(void) {
    printf("=== 故障诊断开始 ===\r\n");
    
    // 初始化
    Encoder520_Init();
    Encoder520_Start_All();
    
    uint16_t fault_count[MOTOR_COUNT] = {0};
    uint16_t no_signal_count[MOTOR_COUNT] = {0};
    
    for(uint16_t i = 0; i < 100; i++) {
        Encoder520_Update_All_Speed();
        
        for(uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
            int16_t current_count = Encoder520_Read_Count((Motor_ID)motor);
            float current_rpm = Encoder520_Get_RPM((Motor_ID)motor);
            
            // 检查异常情况
            if(current_count == 0 && i > 10) {  // 10次采样后仍无信号
                no_signal_count[motor]++;
            }
            
            if(ABS(current_rpm) > 10000) {  // RPM异常高
                fault_count[motor]++;
            }
        }
        
        HAL_Delay(100);
    }
    
    // 输出诊断结果
    printf("\r\n--- 故障诊断结果 ---\r\n");
    for(uint8_t motor = 0; motor < MOTOR_COUNT; motor++) {
        printf("电机%d: ", motor);
        
        if(no_signal_count[motor] > 50) {
            printf("警告 - 无编码器信号! ");
        }
        
        if(fault_count[motor] > 10) {
            printf("警告 - RPM数值异常! ");
        }
        
        if(no_signal_count[motor] <= 50 && fault_count[motor] <= 10) {
            printf("正常");
        }
        
        printf("\r\n");
    }
    
    printf("=== 故障诊断结束 ===\r\n");
}

// ============================================================================
// 主测试函数 - 运行所有示例
// ============================================================================
void Encoder520_Run_All_Examples(void) {
    printf("\r\n");
    printf("*************************************************\r\n");
    printf("*         520编码器移植示例测试程序            *\r\n");
    printf("*************************************************\r\n");
    
    // 运行各个示例
    Encoder520_Basic_Test();
    HAL_Delay(1000);
    
    Encoder520_Compatibility_Test();
    HAL_Delay(1000);
    
    Encoder520_Performance_Test();
    HAL_Delay(1000);
    
    Encoder520_Calibration_Debug();
    HAL_Delay(1000);
    
    Encoder520_PID_Integration_Example();
    HAL_Delay(1000);
    
    Encoder520_Fault_Diagnosis();
    
    printf("\r\n");
    printf("*************************************************\r\n");
    printf("*           所有示例测试完成!                  *\r\n");
    printf("*************************************************\r\n");
} 