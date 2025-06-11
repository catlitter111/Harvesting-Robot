#include "scheduler.h"
#include "pid_motor.h"
#include "encoder.h"
#include "pid_test.h"
#include "../hardware/config.h"

// ============================================================================
// 调度器配置
// ============================================================================
uint8_t task_num;
scheduler schedulers[] = {
    {uart_task, 10, 0},                         // 串口任务，10ms周期
    {gps_task, 100, 0},                         // GPS任务，100ms周期
    {encoder_task, ENCODER_TASK_PERIOD, 0},     // 编码器任务，50ms周期
    {pid_motor_task, PID_TASK_PERIOD, 0},       // PID任务，20ms周期
    {pid_test_task, 100, 0},                    // PID测试任务，100ms周期
    {system_led_task, DEBUG_TASK_PERIOD, 0}     // 系统LED任务，1000ms周期
};

// ============================================================================
// 调度器初始化
// ============================================================================
void scheduler_init() {
    task_num = sizeof(schedulers) / sizeof(scheduler);
}

// ============================================================================
// 调度器运行
// ============================================================================
void scheduler_run() {
    for (uint8_t i = 0; i < task_num; i++) {
        uint32_t now_time = HAL_GetTick();
        if (now_time - schedulers[i].last_run >= schedulers[i].run_ms) {
            schedulers[i].last_run = now_time;
            schedulers[i].task_func();
        }
    }
}

// ============================================================================
// PID控制任务
// ============================================================================
void pid_motor_task(void) {
    if (Robot_Get_PID_Status()) {
        FourWheel_PID_Calculate();          // 计算四轮PID输出
        FourWheel_Apply_Control();          // 应用PID输出到电机
    }
}

// ============================================================================
// 编码器任务
// ============================================================================
void encoder_task(void) {
    Encoder_Read_All_Pulses();              // 读取所有编码器脉冲
    Encoder_Update_All_Speed();             // 更新所有编码器速度
    
    // 将编码器速度反馈给PID控制器
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int16_t speed = Encoder_Get_Speed((Motor_ID)i);
        FourWheel_Update_Speed((Motor_ID)i, speed);
    }
}

// ============================================================================
// 系统状态LED任务
// ============================================================================
void system_led_task(void) {
    static uint8_t led_state = 0;
    led_state = !led_state;
    HAL_GPIO_WritePin(SYSTEM_LED_PORT, SYSTEM_LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ============================================================================
// PID测试任务
// ============================================================================
void pid_test_task(void) {
    PID_Test_Task();
}

// ============================================================================
// 任务状态监控函数(调试用)
// ============================================================================
void scheduler_debug_info(void) {
    #ifdef DEBUG
    static uint32_t last_debug = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_debug > 5000) {  // 每5秒输出一次
        DEBUG_PRINTF("调度器状态: 任务数=%d, 运行时间=%ldms", task_num, now);
        for (uint8_t i = 0; i < task_num; i++) {
            DEBUG_PRINTF("任务%d: 周期=%ldms, 上次运行=%ldms", i, schedulers[i].run_ms, schedulers[i].last_run);
        }
        last_debug = now;
    }
    #endif
}

