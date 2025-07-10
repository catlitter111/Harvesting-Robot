#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H


#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "usart.h"
#include "math.h"
#include <stdlib.h>

#include "motor.h"
#include "scheduler.h"
#include "ringbuffer.h"
#include "uart_app.h"
#include "gps_app.h"


extern uint8_t uart_dma_buff[256];
extern uint8_t ringbuff[256];
extern uint8_t read_buff[256];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern struct rt_ringbuffer rb;


#endif

