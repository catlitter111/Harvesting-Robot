#include "pid_test.h"
#include <stdio.h>

// ============================================================================
// 测试用静态变量
// ============================================================================
static PID_T test_pid1, test_pid2;
static float test_target = 50.0f, test_feedback = 0.0f;

// ============================================================================
// PID基础功能测试
// ============================================================================
void PID_BasicTest(void) {
    printf("=== PID基础功能测试 ===\r\n");
    
    // 初始化测试
    PID_Init(&test_pid1, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    printf("PID初始化: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", test_pid1.Kp, test_pid1.Ki, test_pid1.Kd);
    
    // 设置目标值测试
    PID_SetTarget(&test_pid1, test_target);
    printf("设置目标值: %.2f\r\n", test_pid1.target);
    
    // 反馈与输出测试
    for (int i = 0; i < 10; i++) {
        test_feedback += 5.0f;  // 模拟反馈增长
        PID_SetFeedback(&test_pid1, test_feedback);
        float output = PID_Compute(&test_pid1);
        printf("步骤%d: 反馈=%.1f, 输出=%.2f\r\n", i+1, test_feedback, output);
    }
    printf("基础测试完成\r\n\r\n");
}

// ============================================================================
// PID位置式与增量式对比测试
// ============================================================================
void PID_ModeCompareTest(void) {
    printf("=== PID模式对比测试 ===\r\n");
    
    // 初始化两个PID控制器
    PID_Init(&test_pid1, 2.0f, 0.1f, 0.5f);  // 位置式
    PID_Init(&test_pid2, 2.0f, 0.1f, 0.5f);  // 增量式
    PID_SetMode(&test_pid2, 1);               // 设置为增量式
    
    PID_SetTarget(&test_pid1, 100.0f);
    PID_SetTarget(&test_pid2, 100.0f);
    
    printf("目标值: 100, 测试10步响应\r\n");
    printf("步骤\t反馈\t位置式\t增量式\r\n");
    
    float feedback = 0.0f;
    for (int i = 0; i < 10; i++) {
        feedback += 10.0f + (i % 3) * 2.0f;  // 模拟带扰动的反馈
        
        PID_SetFeedback(&test_pid1, feedback);
        PID_SetFeedback(&test_pid2, feedback);
        
        float out1 = PID_Compute(&test_pid1);
        float out2 = PID_Compute(&test_pid2);
        
        printf("%d\t%.1f\t%.2f\t%.2f\r\n", i+1, feedback, out1, out2);
        HAL_Delay(PID_SAMPLE_TIME);  // 等待采样周期
    }
    printf("模式对比测试完成\r\n\r\n");
}

// ============================================================================
// PID限幅功能测试
// ============================================================================
void PID_LimitTest(void) {
    printf("=== PID限幅功能测试 ===\r\n");
    
    PID_Init(&test_pid1, 5.0f, 1.0f, 0.2f);  // 较大增益
    PID_SetTarget(&test_pid1, 1000.0f);       // 很大目标值
    PID_SetOutputLimits(&test_pid1, -50.0f, 50.0f);  // 设置限幅
    
    printf("目标值: 1000, 输出限制: [-50, 50]\r\n");
    
    for (int i = 0; i < 5; i++) {
        PID_SetFeedback(&test_pid1, i * 10.0f);
        float output = PID_Compute(&test_pid1);
        printf("反馈=%.1f, 输出=%.2f (限幅生效)\r\n", i * 10.0f, output);
        HAL_Delay(PID_SAMPLE_TIME);
    }
    printf("限幅测试完成\r\n\r\n");
}

// ============================================================================
// PID参数调节测试
// ============================================================================
void PID_TuningTest(void) {
    printf("=== PID参数调节测试 ===\r\n");
    
    float kp_values[] = {1.0f, 2.0f, 5.0f};
    float ki_values[] = {0.05f, 0.1f, 0.2f};
    
    for (int i = 0; i < 3; i++) {
        PID_Init(&test_pid1, kp_values[i], ki_values[i], 0.1f);
        PID_SetTarget(&test_pid1, 80.0f);
        
        printf("参数组%d: Kp=%.1f, Ki=%.2f\r\n", i+1, kp_values[i], ki_values[i]);
        
        for (int j = 0; j < 5; j++) {
            PID_SetFeedback(&test_pid1, j * 15.0f);
            float output = PID_Compute(&test_pid1);
            printf("  反馈=%.1f, 输出=%.2f\r\n", j * 15.0f, output);
            HAL_Delay(PID_SAMPLE_TIME);
        }
        PID_Reset(&test_pid1);  // 重置准备下一组测试
    }
    printf("参数调节测试完成\r\n\r\n");
}

// ============================================================================
// 完整PID测试套件
// ============================================================================
void PID_RunAllTests(void) {
    printf("\r\n========== PID算法库测试开始 ==========\r\n");
    printf("配置信息: 采样周期=%dms, 默认Kp=%.1f, Ki=%.2f, Kd=%.1f\r\n", 
           PID_SAMPLE_TIME, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    printf("==========================================\r\n\r\n");
    
    PID_BasicTest();        // 基础功能测试
    PID_ModeCompareTest();  // 模式对比测试  
    PID_LimitTest();        // 限幅功能测试
    PID_TuningTest();       // 参数调节测试
    
    printf("========== 所有PID测试完成 ==========\r\n\r\n");
} 