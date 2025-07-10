# 520编码器快速开始指南

## 🚀 5分钟快速集成520编码器

### 步骤1: 硬件连接
按照以下引脚连接520电机编码器：

```
编码器1 (左轮):
- 编码器A相 → PA0 (TIM2_CH1)
- 编码器B相 → PA1 (TIM2_CH2)

编码器2 (右轮):  
- 编码器A相 → PB6 (TIM4_CH1)
- 编码器B相 → PB7 (TIM4_CH2)
```

### 步骤2: STM32CubeMX配置
1. 打开STM32CubeMX
2. 配置TIM2为编码器模式：
   - Timers → TIM2 → Combined Channels → Encoder Mode
   - Encoder Mode: Encoder Mode TI1 and TI2
   - Counter Period: 65535
3. 配置TIM4为编码器模式（同TIM2设置）
4. 生成代码

### 步骤3: 添加520编码器文件
将以下文件添加到工程：
- `hardware/encoder_520.h`
- `hardware/encoder_520.c` 
- `hardware/encoder_520_config.h`

### 步骤4: 代码集成
在main.c中添加：

```c
#include "encoder_520.h"

int main(void) {
    // 系统初始化...
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();  // 编码器1
    MX_TIM4_Init();  // 编码器2
    
    // 初始化520编码器
    Encoder520_Init();
    Encoder520_Start_All();
    
    while(1) {
        // 更新编码器数据
        Encoder520_Update_All_Speed();
        
        // 读取速度
        int left_speed = Read_Speed_Left();
        int right_speed = Read_Speed_Right();
        
        // 你的控制逻辑...
        
        HAL_Delay(20);
    }
}
```

### 步骤5: 验证测试
运行以下测试代码验证集成：

```c
#include "encoder_520_example.h"

// 在main函数中调用
Encoder520_Basic_Test();
```

## ✅ 完成！
现在您的四轮小车已经成功集成520电机编码器，享受更高精度的速度控制吧！

> 💡 **提示**: 如需更详细的配置和高级功能，请查看完整的[520编码器移植指南](ENCODER_520_INTEGRATION_GUIDE.md)。 