#include "pid_test.h"
#include "usart.h"
#include <stdio.h>

// ============================================================================
// 全局变量定义
// ============================================================================
PID_Test_t pid_test = {PID_TEST_STOP, 0, 0, 0};    // PID测试控制结构体
static char uart_buf[200];                          // 串口发送缓冲区

// ============================================================================
// PID测试模块初始化
// ============================================================================
void PID_Test_Init(void)
{
    // 初始化测试参数
    pid_test.mode = PID_TEST_STOP;
    pid_test.start_time = 0;
    pid_test.target_speed = 0;
    pid_test.test_active = 0;
    
    // 初始化四轮PID控制器
    FourWheel_PID_Init();
    
    // 启用PID控制功能
    Robot_Enable_PID(1);
    
    printf("PID Test Module Init Complete\r\n");
}

// ============================================================================
// PID测试任务处理函数
// ============================================================================
void PID_Test_Task(void)
{
    // 如果测试未激活，直接返回
    if (!pid_test.test_active) {
        return;
    }
    
    // 计算测试运行时间
    uint32_t elapsed = HAL_GetTick() - pid_test.start_time;
    
    // 根据测试模式执行相应动作
    switch (pid_test.mode) {
        case PID_TEST_FORWARD:
            // 前进测试：持续3秒
            if (elapsed < 3000) {
                Robot_Move_PID(ROBOT_DIR_FORWARD, pid_test.target_speed);
            } else {
                PID_Test_Stop();
                printf("Forward Test Complete\r\n");
            }
            break;
            
        case PID_TEST_BACKWARD:
            // 后退测试：持续3秒
            if (elapsed < 3000) {
                Robot_Move_PID(ROBOT_DIR_BACKWARD, pid_test.target_speed);
            } else {
                PID_Test_Stop();
                printf("Backward Test Complete\r\n");
            }
            break;
            
        case PID_TEST_LEFT_TURN:
            // 左转测试：持续2秒，使用差速驱动
            if (elapsed < 2000) {
                Robot_Differential_Drive(-pid_test.target_speed, pid_test.target_speed);
            } else {
                PID_Test_Stop();
                printf("Left Turn Test Complete\r\n");
            }
            break;
            
        case PID_TEST_CIRCLE:
            // 转圈测试：持续5秒，四轮差速
            if (elapsed < 5000) {
                Robot_Four_Wheel_Drive(-pid_test.target_speed, pid_test.target_speed, 
                                     -pid_test.target_speed, pid_test.target_speed);
            } else {
                PID_Test_Stop();
                printf("Circle Test Complete\r\n");
            }
            break;
            
        case PID_TEST_ACCEL_BRAKE:
            // 加速急停测试
            if (elapsed < 1000) {
                // 第一阶段：1秒内逐渐加速
                uint16_t spd = (pid_test.target_speed * elapsed) / 1000;
                Robot_Move_PID(ROBOT_DIR_FORWARD, spd);
            } else if (elapsed < 1100) {
                // 第二阶段：急停100ms
                Robot_Move_PID(ROBOT_DIR_STOP, 0);
            } else {
                // 测试完成
                PID_Test_Stop();
                printf("Accel Brake Test Complete\r\n");
            }
            break;
            
        default:
            // 未知模式，停止测试
            PID_Test_Stop();
            break;
    }
    
    // 每200ms发送一次数据
    if (elapsed % 200 == 0) {
        PID_Test_Send_Data();
    }
}

// ============================================================================
// 开始PID测试
// ============================================================================
void PID_Test_Start(PID_Test_Mode mode, uint16_t speed)
{
    // 设置测试参数
    pid_test.mode = mode;
    pid_test.target_speed = CONSTRAIN(speed, 10, SPEED_MAX);  // 限制速度范围
    pid_test.start_time = HAL_GetTick();
    pid_test.test_active = 1;
    
    // 测试模式描述字符串
    const char* mode_str[] = {
        "STOP", "FORWARD", "BACKWARD", "LEFT_TURN", "CIRCLE", "ACCEL_BRAKE"
    };
    
    printf("Start %s Test, Target Speed: %d\r\n", mode_str[mode], speed);
}

// ============================================================================
// 停止PID测试
// ============================================================================
void PID_Test_Stop(void)
{
    // 清除测试状态
    pid_test.test_active = 0;
    
    // 停止机器人运动
    Robot_Move_PID(ROBOT_DIR_STOP, 0);
    
    printf("PID Test Stopped\r\n");
}

// ============================================================================
// 发送PID测试数据
// ============================================================================
void PID_Test_Send_Data(void)
{
    // 格式化数据字符串
    int len = snprintf(uart_buf, sizeof(uart_buf),
        "[PID_DATA]Target:%d,%d,%d,%d|Actual:%d,%d,%d,%d|PWM:%d,%d,%d,%d\r\n",
        // 设定速度
        motor_pids[0].target_speed, motor_pids[1].target_speed, 
        motor_pids[2].target_speed, motor_pids[3].target_speed,
        // 实际速度 - 修复类型转换问题
        Encoder_Get_Speed((Motor_ID)0), Encoder_Get_Speed((Motor_ID)1), 
        Encoder_Get_Speed((Motor_ID)2), Encoder_Get_Speed((Motor_ID)3),
        // PWM输出
        motor_pids[0].pwm_output, motor_pids[1].pwm_output, 
        motor_pids[2].pwm_output, motor_pids[3].pwm_output
    );
    
    // 通过调试串口发送数据
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)uart_buf, len, 100);
}

// ============================================================================
// 手动控制四个轮子
// ============================================================================
void PID_Test_Manual_Control(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    // 直接控制四个轮子的速度
    Robot_Four_Wheel_Drive(fl, fr, rl, rr);
    
    printf("Manual Control: FL=%d, FR=%d, RL=%d, RR=%d\r\n", fl, fr, rl, rr);
}

// ============================================================================
// 自动测试序列
// ============================================================================
void PID_Test_Sequence(void)
{
    static uint8_t seq_step = 0;           // 序列步骤
    static uint32_t seq_timer = 0;         // 序列定时器
    
    // 如果有测试正在进行，等待完成
    if (pid_test.test_active) {
        return;
    }
    
    uint32_t now = HAL_GetTick();
    
    // 每个测试间隔1秒
    if (now - seq_timer < 1000) {
        return;
    }
    
    seq_timer = now;
    
    // 执行测试序列
    switch (seq_step) {
        case 0:
            // 步骤1：前进测试
            PID_Test_Start(PID_TEST_FORWARD, 30);
            seq_step++;
            break;
            
        case 1:
            // 步骤2：后退测试
            if (!pid_test.test_active) {
                PID_Test_Start(PID_TEST_BACKWARD, 25);
                seq_step++;
            }
            break;
            
        case 2:
            // 步骤3：左转测试
            if (!pid_test.test_active) {
                PID_Test_Start(PID_TEST_LEFT_TURN, 35);
                seq_step++;
            }
            break;
            
        case 3:
            // 步骤4：转圈测试
            if (!pid_test.test_active) {
                PID_Test_Start(PID_TEST_CIRCLE, 40);
                seq_step++;
            }
            break;
            
        case 4:
            // 步骤5：加速急停测试
            if (!pid_test.test_active) {
                PID_Test_Start(PID_TEST_ACCEL_BRAKE, 50);
                seq_step++;
            }
            break;
            
        case 5:
            // 序列完成，重置步骤
            if (!pid_test.test_active) {
                seq_step = 0;
                printf("Test Sequence Complete\r\n");
            }
            break;
            
        default:
            // 异常情况，重置序列
            seq_step = 0;
            break;
    }
}

