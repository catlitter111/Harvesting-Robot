#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"


// ============================================================================
// 系统基础配置
// ============================================================================
#define MOTOR_COUNT 4                    // 电机数量
#define SYSTEM_VERSION "V1.0"            // 系统版本号

// ============================================================================
// PID控制器配置
// ============================================================================
#define PID_SAMPLE_TIME 20               // PID采样周期(ms)
#define PID_KP_DEFAULT 2.0f              // 默认比例系数
#define PID_KI_DEFAULT 0.1f              // 默认积分系数
#define PID_KD_DEFAULT 0.5f              // 默认微分系数
#define PID_OUTPUT_LIMIT 100.0f          // PID输出限制值
#define PID_INTEGRAL_LIMIT 80.0f         // PID积分限制值

// 速度控制范围
#define SPEED_MAX 100                    // 最大速度限制(%)
#define SPEED_MIN -100                   // 最小速度限制(%)
#define SPEED_DEADZONE 5                 // 速度死区(%)

// ============================================================================
// 编码器配置(临时引脚分配，用户可根据实际硬件修改)
// ============================================================================
// 编码器硬件参数
#define ENCODER_PPR 400                  // 编码器每转脉冲数
#define WHEEL_DIAMETER_MM 65             // 轮径(mm)
#define GEAR_RATIO 1.0f                  // 减速比
#define ENCODER_FILTER_TIME 10           // 编码器滤波时间(ms)

// 编码器引脚定义(GPIO引脚)
#define ENCODER1_A_PIN GPIO_PIN_0        // 电机1编码器A相
#define ENCODER1_B_PIN GPIO_PIN_1        // 电机1编码器B相
#define ENCODER1_PORT GPIOC              // 电机1编码器端口

#define ENCODER2_A_PIN GPIO_PIN_2        // 电机2编码器A相
#define ENCODER2_B_PIN GPIO_PIN_3        // 电机2编码器B相
#define ENCODER2_PORT GPIOC              // 电机2编码器端口

#define ENCODER3_A_PIN GPIO_PIN_4        // 电机3编码器A相
#define ENCODER3_B_PIN GPIO_PIN_5        // 电机3编码器B相
#define ENCODER3_PORT GPIOC              // 电机3编码器端口

#define ENCODER4_A_PIN GPIO_PIN_6        // 电机4编码器A相
#define ENCODER4_B_PIN GPIO_PIN_7        // 电机4编码器B相
#define ENCODER4_PORT GPIOC              // 电机4编码器端口

// ============================================================================
// 任务调度配置
// ============================================================================
#define PID_TASK_PERIOD 20               // PID任务周期(ms)
#define ENCODER_TASK_PERIOD 50           // 编码器任务周期(ms)
#define MOTOR_TASK_PERIOD 10             // 电机任务周期(ms)
#define DEBUG_TASK_PERIOD 1000           // 调试任务周期(ms)

// ============================================================================
// 电机控制配置
// ============================================================================
#define PWM_FREQUENCY 1000               // PWM频率(Hz)
#define PWM_RESOLUTION 1000              // PWM分辨率
#define MOTOR_ACCELERATION_TIME 500      // 电机加速时间(ms)
#define MOTOR_BRAKE_TIME 200             // 电机制动时间(ms)

// ============================================================================
// 通信配置
// ============================================================================
#define DEBUG_UART_ENABLE 1              // 调试串口使能
#define DEBUG_UART huart2                // 调试串口句柄
#define DEBUG_BAUDRATE 115200            // 调试串口波特率

// ============================================================================
// 系统状态配置
// ============================================================================
#define SYSTEM_LED_PIN GPIO_PIN_13       // 系统状态LED
#define SYSTEM_LED_PORT GPIOC            // 系统状态LED端口

// ============================================================================
// 安全保护配置
// ============================================================================
#define WATCHDOG_TIMEOUT 1000            // 看门狗超时时间(ms)
#define EMERGENCY_STOP_ENABLE 1          // 急停功能使能
#define MAX_ERROR_COUNT 10               // 最大错误计数

// ============================================================================
// 数学常量定义
// ============================================================================
#define PI 3.14159265359f                // 圆周率
#define DEG_TO_RAD (PI/180.0f)          // 角度转弧度
#define RAD_TO_DEG (180.0f/PI)          // 弧度转角度

// ============================================================================
// 实用宏定义
// ============================================================================
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))  // 数值限制
#define ABS(x) ((x) > 0 ? (x) : -(x))                                                // 绝对值
#define MAX(a, b) ((a) > (b) ? (a) : (b))                                           // 最大值
#define MIN(a, b) ((a) < (b) ? (a) : (b))                                           // 最小值
#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while(0)            // 交换值

// ============================================================================
// 调试配置
// ============================================================================
#ifdef DEBUG
    #define DEBUG_PRINTF(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_ASSERT(expr) if(!(expr)) { printf("ASSERT FAILED: %s:%d\r\n", __FILE__, __LINE__); }
#else
    #define DEBUG_PRINTF(fmt, ...)
    #define DEBUG_ASSERT(expr)
#endif

// ============================================================================
// 版权信息
// ============================================================================
#define COPYRIGHT_INFO "STM32四轮小车PID控制系统 - 配置文件 V1.0"
#define AUTHOR_INFO "基于ElectronicController_Board项目移植"

#endif /* CONFIG_H */ 
	


