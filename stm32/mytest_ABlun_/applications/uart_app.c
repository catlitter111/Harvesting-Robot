#include "uart_app.h"
#include "ringbuffer.h"
#include "motor.h"
#include "gps_app.h"  // 添加GPS头文件

// DMA接收缓冲区
uint8_t uart_dma_buff[256] = {0};
// 环形缓冲区
uint8_t ringbuff[256] = {0};
// 临时读取缓冲区
uint8_t read_buff[256] = {0};
// 环形缓冲区控制结构
struct rt_ringbuffer rb;
// 数据包解析状态
static UartPacket packet = {0};
// 小车当前状态
static uint8_t current_direction = DIR_STOP;
static uint8_t current_speed = 50;
static int8_t motor_speeds[MOTOR_COUNT] = {0, 0, 0, 0};
// 经纬度信息
float current_latitude = 0.0f;
float current_longitude = 0.0f;

// 重定向printf函数
int fputc(int ch, FILE* file) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
    return ch;
}

// UART接收中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart == &huart2) {
        // 将接收到的数据放入环形缓冲区
        rt_ringbuffer_put(&rb, uart_dma_buff, Size);
        // 清空DMA缓冲区，准备下一次接收
        memset(uart_dma_buff, 0, sizeof(uart_dma_buff));
    }
    else if(huart == &huart1){
        // 将GPS数据放入环形缓冲区
        rt_ringbuffer_put(&rb_gps, uart_dma_buff_gps, Size);
        
        // 打印接收到的原始数据（调试信息）
        printf("GPS RAW DATA [%d bytes]: ", Size);
        for(uint16_t i = 0; i < Size && i < 50; i++) { // 只打印前50个字符，避免信息过多
            printf("%c", uart_dma_buff_gps[i]);
        }
        if(Size > 50) printf("..."); // 如果数据超过50个字符，显示省略号
        printf("\r\n");
        
        // 清空DMA缓冲区，准备下一次接收
        memset(uart_dma_buff_gps, 0, sizeof(uart_dma_buff_gps));
    }
    
    // 重新启动DMA接收
    if(huart == &huart2) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_dma_buff, sizeof(uart_dma_buff));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    } 
    else if(huart == &huart1) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_buff_gps, sizeof(uart_dma_buff_gps));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

// 初始化串口功能
void uart_init(void) {
    // 初始化环形缓冲区
    rt_ringbuffer_init(&rb, ringbuff, sizeof(ringbuff));
    
    // 初始化数据包状态
    packet.state = WAIT_HEADER1;
    
    // 启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_dma_buff, sizeof(uart_dma_buff));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
    
    // 初始状态消息
    printf("Car Control System Ready\r\n");
}

// 执行命令
void execute_command(UartPacket *pkt) {
    uint8_t response[5] = {0};
    
    switch(pkt->cmd) {
        case CMD_SET_DIRECTION:
            if(pkt->length >= 2) {  // 现在需要两个参数：方向和速度
                current_direction = pkt->data[0];
                current_speed = pkt->data[1];
                
                // 限制速度范围
                if(current_speed > 100) {
                    current_speed = 100;
                }
                
                // 设置小车速度
                Robot_SetSpeed(current_speed);
                
                // 设置小车方向
                switch(current_direction) {
                    case DIR_FORWARD:
                        Robot_Move(ROBOT_DIR_FORWARD);
                        printf("Moving Forward at %d%% speed\r\n", current_speed);
                        break;
                    case DIR_BACKWARD:
                        Robot_Move(ROBOT_DIR_BACKWARD);
                        printf("Moving Backward at %d%% speed\r\n", current_speed);
                        break;
                    case DIR_LEFT:
                        Robot_Move(ROBOT_DIR_LEFT);
                        printf("Turning Left at %d%% speed\r\n", current_speed);
                        break;
                    case DIR_RIGHT:
                        Robot_Move(ROBOT_DIR_RIGHT);
                        printf("Turning Right at %d%% speed\r\n", current_speed);
                        break;
                    case DIR_STOP:
                    default:
                        Robot_Move(ROBOT_DIR_STOP);
                        printf("Stopped\r\n");
                        current_direction = DIR_STOP;
                        break;
                }
                
                // 发送确认响应
                response[0] = current_direction;
                response[1] = current_speed;
                uart_send_packet(CMD_SET_DIRECTION, response, 2);
            } else {
                printf("Error: CMD_SET_DIRECTION requires 2 data bytes\r\n");
                uart_send_ack(pkt->cmd, 0xFE);  // 参数错误
            }
            break;
            
        case CMD_SET_SPEED:
            if(pkt->length >= 1) {
                current_speed = pkt->data[0];
                if(current_speed > 100) current_speed = 100;
                
                Robot_SetSpeed(current_speed);
                printf("Speed set to %d%%\r\n", current_speed);
                
                // 发送确认响应
                response[0] = current_speed;
                uart_send_packet(CMD_SET_SPEED, response, 1);
            } else {
                printf("Error: CMD_SET_SPEED requires 1 data byte\r\n");
                uart_send_ack(pkt->cmd, 0xFE);
            }
            break;
            
        case CMD_SET_MOTOR:
            if(pkt->length >= 3) {  // 现在需要3个参数：电机ID、速度和方向
                uint8_t motor_id = pkt->data[0];
                int8_t speed = (int8_t)pkt->data[1];
                uint8_t direction = pkt->data[2];
                
                if(motor_id < MOTOR_COUNT) {
                    // 根据方向设置速度值的正负
                    if(direction == DIR_BACKWARD) {
                        speed = -speed;
                    }
                    
                    motor_speeds[motor_id] = speed;
                    Motor_SetSpeed((Motor_ID)motor_id, speed);
                    printf("Motor %d set to %d%% speed, direction %s\r\n", 
                           motor_id, (int)fabs(speed), 
                           speed >= 0 ? "forward" : "backward");
                    
                    // 发送确认响应
                    response[0] = motor_id;
                    response[1] = (uint8_t)fabs(speed);
                    response[2] = speed >= 0 ? DIR_FORWARD : DIR_BACKWARD;
                    uart_send_packet(CMD_SET_MOTOR, response, 3);
                } else {
                    printf("Error: Invalid motor ID %d\r\n", motor_id);
                    uart_send_ack(pkt->cmd, 0xFD);  // 无效的电机ID
                }
            } else {
                printf("Error: CMD_SET_MOTOR requires 3 data bytes\r\n");
                uart_send_ack(pkt->cmd, 0xFE);
            }
            break;
            
        case CMD_REQUEST_STATUS:
            uart_send_status();
            printf("Status sent\r\n");
            break;
            
        case CMD_SET_POSITION:
            if(pkt->length >= 8) {
                // 从数据包中解析出经纬度
                int32_t lat_int = ((int32_t)pkt->data[0] << 24) | 
                                 ((int32_t)pkt->data[1] << 16) | 
                                 ((int32_t)pkt->data[2] << 8) | 
                                 pkt->data[3];
                
                int32_t lon_int = ((int32_t)pkt->data[4] << 24) | 
                                 ((int32_t)pkt->data[5] << 16) | 
                                 ((int32_t)pkt->data[6] << 8) | 
                                 pkt->data[7];
                
                // 转换回浮点数
                float latitude = (float)lat_int / 1000000.0f;
                float longitude = (float)lon_int / 1000000.0f;
                
                // 更新当前位置
                current_latitude = latitude;
                current_longitude = longitude;
                
                printf("Position received: %.6f, %.6f\r\n", latitude, longitude);
                
                // 发送确认响应
                uart_send_position(latitude, longitude);
            } else {
                printf("Error: CMD_SET_POSITION requires 8 data bytes\r\n");
                uart_send_ack(pkt->cmd, 0xFE);  // 参数错误
            }
            break;
            
        default:
            // 未知命令，发送错误响应
            uart_send_ack(pkt->cmd, 0xFF);
            printf("Unknown command: 0x%02X\r\n", pkt->cmd);
            break;
    }
}

// 发送确认包
void uart_send_ack(uint8_t cmd, uint8_t status) {
    uint8_t ack_data[1] = {status};
    uart_send_packet(cmd, ack_data, 1);
}

// 发送状态信息
void uart_send_status(void) {
    uint8_t status[6];
    
    // 构建状态数据包
    status[0] = current_direction;  // 当前方向
    status[1] = current_speed;      // 当前速度
    
    // 添加四个电机的速度状态
    for(int i = 0; i < MOTOR_COUNT; i++) {
        status[i+2] = (uint8_t)fabs(motor_speeds[i]);  // 发送绝对值
    }
    
    uart_send_packet(CMD_REQUEST_STATUS, status, 6);
}

// 设置小车当前位置
void uart_set_position(float latitude, float longitude) {
    current_latitude = latitude;
    current_longitude = longitude;
    
    // 更新完位置后，主动发送位置信息
    uart_send_position(latitude, longitude);
    printf("Position updated: %.6f, %.6f\r\n", latitude, longitude);
}

// 发送位置信息
void uart_send_position(float latitude, float longitude) {
    uint8_t position_data[8];
    
    // 将浮点数经纬度转换为整数（乘以10^6）
    int32_t lat_int = (int32_t)(latitude * 1000000);
    int32_t lon_int = (int32_t)(longitude * 1000000);
    
    // 将32位整数分解为4个字节
    position_data[0] = (lat_int >> 24) & 0xFF;
    position_data[1] = (lat_int >> 16) & 0xFF;
    position_data[2] = (lat_int >> 8) & 0xFF;
    position_data[3] = lat_int & 0xFF;
    
    position_data[4] = (lon_int >> 24) & 0xFF;
    position_data[5] = (lon_int >> 16) & 0xFF;
    position_data[6] = (lon_int >> 8) & 0xFF;
    position_data[7] = lon_int & 0xFF;
    
    // 发送数据包
    uart_send_packet(CMD_SET_POSITION, position_data, 8);
}

// 发送数据包
void uart_send_packet(uint8_t cmd, uint8_t *data, uint8_t length) {
    uint8_t tx_buffer[32];
    uint8_t index = 0;
    uint8_t checksum = 0;
    
    // 构建数据包
    tx_buffer[index++] = FRAME_HEADER_1;
    tx_buffer[index++] = FRAME_HEADER_2;
    tx_buffer[index++] = cmd;
    tx_buffer[index++] = length;
    
    // 计算校验和并添加数据
    checksum = cmd + length;
    
    for(uint8_t i = 0; i < length; i++) {
        tx_buffer[index++] = data[i];
        checksum += data[i];
    }
    
    // 添加校验和
    tx_buffer[index++] = checksum;
    
    // 发送数据包
    HAL_UART_Transmit(&huart2, tx_buffer, index, 100);
}

// 串口任务，处理接收到的数据
void uart_task(void) {
    uint8_t data;
    
    // 如果环形缓冲区为空，直接返回
    if(rt_ringbuffer_data_len(&rb) == 0) {
        return;
    }
    
    // 从环形缓冲区中读取数据并解析
    while(rt_ringbuffer_data_len(&rb) > 0) {
        // 读取一个字节的数据
        if(rt_ringbuffer_getchar(&rb, &data) == 0) {
            printf("环形缓冲区为空\r\n");
            break;
        }
        
        // 状态机解析数据包
        switch(packet.state) {
            case WAIT_HEADER1:
                if(data == FRAME_HEADER_1) {
                    packet.state = WAIT_HEADER2;
                }
                break;
                
            case WAIT_HEADER2:
                if(data == FRAME_HEADER_2) {
                    packet.state = WAIT_CMD;
                } else {
                    packet.state = WAIT_HEADER1;
                }
                break;
                
            case WAIT_CMD:
                packet.cmd = data;
                packet.state = WAIT_LEN;
                break;
                
            case WAIT_LEN:
                packet.length = data;
                packet.dataIndex = 0;
                packet.checksum = packet.cmd + packet.length;
                
                if(packet.length > 0) {
                    packet.state = WAIT_DATA;
                } else {
                    packet.state = WAIT_CHECK;
                }
                break;
                
            case WAIT_DATA:
                if(packet.dataIndex < packet.length && packet.dataIndex < sizeof(packet.data)) {
                    packet.data[packet.dataIndex++] = data;
                    packet.checksum += data;
                    
                    if(packet.dataIndex >= packet.length) {
                        packet.state = WAIT_CHECK;
                    }
                } else {
                    // 数据长度超出预期，重置状态
                    packet.state = WAIT_HEADER1;
                }
                break;
                
            case WAIT_CHECK:
                if(data == packet.checksum) {
                    // 校验成功，执行命令
                    execute_command(&packet);
                } else {
                    // 校验失败，打印错误信息
                    printf("Checksum error: expected 0x%02X, got 0x%02X\r\n", 
                           packet.checksum, data);
                }
                // 无论校验是否成功，都重置状态准备接收下一个数据包
                packet.state = WAIT_HEADER1;
                break;
                
            default:
                packet.state = WAIT_HEADER1;
                break;
        }
    }
}


