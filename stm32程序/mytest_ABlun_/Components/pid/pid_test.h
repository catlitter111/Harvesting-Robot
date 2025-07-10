#ifndef PID_TEST_H
#define PID_TEST_H

#include "pid.h"

// ============================================================================
// PID测试函数声明
// ============================================================================
void PID_BasicTest(void);        // PID基础功能测试
void PID_ModeCompareTest(void);   // PID位置式与增量式对比测试
void PID_LimitTest(void);         // PID限幅功能测试
void PID_TuningTest(void);        // PID参数调节测试
void PID_RunAllTests(void);       // 完整PID测试套件

#endif /* PID_TEST_H */ 