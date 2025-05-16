#ifndef GPS_APP_H
#define GPS_APP_H

#include "bsp_system.h"

// GPS 数据结构体
typedef struct {
    float latitude;        // 纬度 (十进制度数)
    float longitude;       // 经度 (十进制度数)
    uint8_t fix_status;    // 定位状态 (0=无定位, 1=有定位)
    uint8_t satellites;    // 卫星数量
} GPS_Data_t;

// 函数声明
void gps_init(void);
void gps_task(void);
void gps_process_nmea(unsigned char *buffer);
GPS_Data_t gps_get_data(void);

// 外部变量声明
extern GPS_Data_t gps_data;
extern uint8_t uart_dma_buff_gps[1000];
extern uint8_t ringbuff_gps[1000];
extern struct rt_ringbuffer rb_gps;

#endif /* GPS_APP_H */


