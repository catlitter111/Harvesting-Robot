#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "bsp_system.h"

typedef struct scheduler{
    void (*task_func)(void);
    uint32_t run_ms;
    uint32_t last_run;
}scheduler;

// ============================================================================
// 调度器函数声明
// ============================================================================
void scheduler_init(void);
void scheduler_run(void);

// ============================================================================
// 系统任务函数声明
// ============================================================================
void uart_task(void);           // 串口任务
void gps_task(void);            // GPS任务
void pid_motor_task(void);      // PID控制任务
void encoder_task(void);        // 编码器更新任务
void system_led_task(void);     // 系统状态LED任务
void pid_test_task(void);       // PID测试任务

#endif

