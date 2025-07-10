#ifndef ENCODER_520_EXAMPLE_H
#define ENCODER_520_EXAMPLE_H

#include "config.h"
#include <stdint.h>

// ============================================================================
// 520编码器使用示例函数声明
// ============================================================================

// 基本功能测试
void Encoder520_Basic_Test(void);                    // 基本编码器测试
void Encoder520_Compatibility_Test(void);            // 兼容性接口测试
void Encoder520_Performance_Test(void);              // 性能对比测试

// 高级功能示例
void Encoder520_PID_Integration_Example(void);       // PID控制集成示例
void Encoder520_Calibration_Debug(void);             // 编码器校准和调试
void Encoder520_Fault_Diagnosis(void);               // 故障诊断

// 综合测试
void Encoder520_Run_All_Examples(void);              // 运行所有示例

#endif /* ENCODER_520_EXAMPLE_H */ 