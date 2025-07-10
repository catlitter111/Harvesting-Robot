#ifndef UART_APP_H
#define UART_APP_H

#include "bsp_system.h"
#include "motor.h"

// 命令类型定义
#define CMD_SET_DIRECTION   0x01  // 设置小车方向
#define CMD_SET_SPEED       0x02  // 设置全局速度
#define CMD_SET_MOTOR       0x03  // 设置单个电机
#define CMD_REQUEST_STATUS  0x04  // 请求小车状态
#define CMD_SET_POSITION    0x05  // 设置/更新位置信息
#define CMD_PID_TEST        0x06  // PID测试命令
#define CMD_PID_PARAMS      0x07  // PID参数设置

// 方向定义
#define DIR_FORWARD    0x00  // 前进
#define DIR_BACKWARD   0x01  // 后退
#define DIR_LEFT       0x02  // 左转
#define DIR_RIGHT      0x03  // 右转
#define DIR_STOP       0x04  // 停止

// 帧头定义
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55

// 数据包状态
typedef enum {
    WAIT_HEADER1,  // 等待第一个帧头字节
    WAIT_HEADER2,  // 等待第二个帧头字节
    WAIT_CMD,      // 等待命令字节
    WAIT_LEN,      // 等待数据长度字节
    WAIT_DATA,     // 等待数据
    WAIT_CHECK     // 等待校验和
} PacketState;

// 数据包结构体
typedef struct {
    uint8_t cmd;        // 命令类型
    uint8_t length;     // 数据长度
    uint8_t data[20];   // 数据缓冲区
    uint8_t checksum;   // 校验和
    uint8_t dataIndex;  // 数据索引
    PacketState state;  // 当前状态
} UartPacket;

// 函数声明
void uart_init(void);
void uart_task(void);
void uart_send_packet(uint8_t cmd, uint8_t *data, uint8_t length);
void uart_send_status(void);
void uart_send_ack(uint8_t cmd, uint8_t status);
void uart_send_position(float latitude, float longitude);
void uart_set_position(float latitude, float longitude);

#endif /* UART_APP_H */

