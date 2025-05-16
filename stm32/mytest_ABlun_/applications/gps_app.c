#include "gps_app.h"

// GPS DMA接收缓冲区
uint8_t uart_dma_buff_gps[1000] = {0};
// GPS环形缓冲区
uint8_t ringbuff_gps[1000] = {0};
// GPS环形缓冲区控制结构
struct rt_ringbuffer rb_gps;
// GPS数据结构体
GPS_Data_t gps_data = {0};

/**
 * @brief GPS初始化函数
 */
void gps_init(void) {
    // 初始化GPS环形缓冲区
    rt_ringbuffer_init(&rb_gps, ringbuff_gps, sizeof(ringbuff_gps));
    
    // 启动GPS串口DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_buff_gps, sizeof(uart_dma_buff_gps));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    
    // 初始状态消息
    printf("GPS Initialization Complete\r\n");
}

/**
 * @brief 处理NMEA数据并提取位置信息
 * @param buffer NMEA数据缓冲区
 */
void gps_process_nmea(unsigned char *buffer) {
    // 查找字符串 "$GNGGA" 和 "\r\n$GNGLL"，分别表示开始和结束的标志
    char *start = strstr((const char *)buffer, "$GNGGA");
    char *end = strstr((const char *)buffer, "\r\n$GNGLL");

    // 如果没有找到这两个标志，说明没有找到有效的GPS数据
    if (start == NULL || end == NULL) {
        return;
    }
    
    // 成功找到开始和结束标志，提取数据
    // 创建一个足够存储$GNGGA数据的缓冲区
    char gngga[100];

    // 从buffer中提取从$GNGGA到$GNGLL之间的数据
    int len = end - start;
    if (len >= sizeof(gngga)) {
        len = sizeof(gngga) - 1;
    }
    
    strncpy(gngga, start, len);
    gngga[len] = '\0'; // 确保字符串以NULL结尾

    // 定义分隔符和一个数组来存储解析出的字段
    char *token;
    token = strtok(gngga, ","); // 使用逗号分隔每个字段
    char *nmea_fields[15];      // 最多支持15个字段
    int i = 0;

    // 逐个字段提取并存入nmea_fields数组中
    while (token != NULL && i < 15) {
        nmea_fields[i] = token;
        token = strtok(NULL, ","); // 获取下一个字段
        i++;
    }

    // 如果提取到的字段大于6，说明数据有效
    if (i > 6) {
        // 检查定位状态 (字段[6]，0=无效，1或更大=有效)
        int fix_status = atoi(nmea_fields[6]);
        gps_data.fix_status = (fix_status > 0) ? 1 : 0;
        
        // 如果有有效定位，解析经纬度
        if (gps_data.fix_status) {
            // 解析纬度
            int lat_deg = (int)(atof(nmea_fields[2]) / 100); // 取出度数（例如：2056.122314 -> 20）
            double lat_min = atof(nmea_fields[2]) - (lat_deg * 100); // 取出分数（例如：2056.122314 - 20*100 = 56.122314）

            // 计算纬度
            float latitude_decimal = lat_deg + (lat_min / 60);
            if (nmea_fields[3][0] == 'S') // 如果是南纬，取负
                latitude_decimal = -latitude_decimal;

            // 解析经度
            int lon_deg = (int)(atof(nmea_fields[4]) / 100); // 取出度数（例如：11002.398438 -> 110）
            double lon_min = atof(nmea_fields[4]) - (lon_deg * 100); // 取出分数（例如：11002.398438 - 110*100 = 2.398438）

            // 计算经度
            float longitude_decimal = lon_deg + (lon_min / 60);
            if (nmea_fields[5][0] == 'W') // 如果是西经，取负
                longitude_decimal = -longitude_decimal;
            
            // 保存解析后的经纬度
            gps_data.latitude = latitude_decimal;
            gps_data.longitude = longitude_decimal;
            
            // 保存卫星数量（如果字段存在）
            if (i > 7) {
                gps_data.satellites = atoi(nmea_fields[7]);
            }
            
            // 使用UART协议发送位置信息给上位机
            uart_set_position(latitude_decimal, longitude_decimal);
            
            // 调试输出
            printf("GPS Position: Lat=%.6f, Lon=%.6f, Satellites=%d\r\n", 
                   gps_data.latitude, gps_data.longitude, gps_data.satellites);
        }
    }
}

/**
 * @brief GPS任务函数，由调度器定期调用
 */
void gps_task(void) {
    // 如果环形缓冲区为空，直接返回
    if (rt_ringbuffer_data_len(&rb_gps) == 0) {
        return;
    }
    
    // 创建临时缓冲区，用于存储从环形缓冲区读取的数据
    uint8_t temp_buffer[1000] = {0};
    uint16_t read_size = 0;
    
    // 从环形缓冲区读取数据
    read_size = rt_ringbuffer_get(&rb_gps, temp_buffer, sizeof(temp_buffer) - 1);
    
    // 确保字符串以NULL结尾
    temp_buffer[read_size] = '\0';
    
    // 处理GPS数据
    if (read_size > 0) {
        gps_process_nmea(temp_buffer);
    }
}

/**
 * @brief 获取当前GPS数据
 * @return GPS_Data_t 包含经纬度和定位状态的结构体
 */
GPS_Data_t gps_get_data(void) {
    return gps_data;
}


