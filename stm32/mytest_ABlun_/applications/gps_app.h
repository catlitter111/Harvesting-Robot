#ifndef GPS_APP_H
#define GPS_APP_H

#include "bsp_system.h"

// GPS ���ݽṹ��
typedef struct {
    float latitude;        // γ�� (ʮ���ƶ���)
    float longitude;       // ���� (ʮ���ƶ���)
    uint8_t fix_status;    // ��λ״̬ (0=�޶�λ, 1=�ж�λ)
    uint8_t satellites;    // ��������
} GPS_Data_t;

// ��������
void gps_init(void);
void gps_task(void);
void gps_process_nmea(unsigned char *buffer);
GPS_Data_t gps_get_data(void);

// �ⲿ��������
extern GPS_Data_t gps_data;
extern uint8_t uart_dma_buff_gps[1000];
extern uint8_t ringbuff_gps[1000];
extern struct rt_ringbuffer rb_gps;

#endif /* GPS_APP_H */


