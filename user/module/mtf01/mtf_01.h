#ifndef __MTF_01_H__
#define __MTF_01_H__

#include "usart.h"
#include "usart_port.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7

#define MTF_01_USART_HANDLE huart6

/* ping-pong 参数 */
#define MTF_PINGPONG_BUF_SIZE 32u
#define MTF_PINGPONG_BUF_COUNT 2u

/*
    消息ID定义
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,     // 测距传感器
};

/*
    消息结构体定义
*/
typedef struct
{
    uint8_t head;                      
    uint8_t dev_id;                          
    uint8_t sys_id;						
    uint8_t msg_id;                        
    uint8_t seq;                          
    uint8_t len;                               
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN]; 
    uint8_t checksum;                          

    uint8_t status;                           
    uint8_t payload_cnt;                       
} MICOLINK_MSG_t;

/*
    数据负载定义
*/
#pragma pack (1)
// 测距传感器
typedef struct
{
    uint32_t  time_ms;			    // 系统时间 ms
    uint32_t  distance;			    // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t   strength;	            // 信号强度
    uint8_t   precision;	        // 精度
    uint8_t   tof_status;	        // 状态
    uint8_t  reserved1;			    // 预留
    int16_t   flow_vel_x;	        // 光流速度x轴
    int16_t   flow_vel_y;	        // 光流速度y轴
    uint8_t   flow_quality;	        // 光流质量
    uint8_t   flow_status;	        // 光流状态
    uint16_t  reserved2;	        // 预留
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack ()

/* ---- 共享健康/样本/诊断类型（供 mtf_01_stream 和外部使用） ---- */

typedef enum {
    MTF_HEALTH_INIT  = 0,
    MTF_HEALTH_VALID = 1,
    MTF_HEALTH_STALE = 2,
    MTF_HEALTH_LOST  = 3,
} MtfHealth;

typedef struct {
    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
    uint8_t  seq;
    uint32_t received_ms;
} MtfSample;

typedef struct {
    uint32_t rx_event_count;
    uint32_t rx_tc_count;
    uint32_t rx_idle_count;
    uint32_t rx_bytes;
    uint32_t raw_queue_drop_count;
    uint32_t parse_ok_count;
    uint32_t checksum_fail_count;
    uint32_t length_error_count;
    uint32_t unsupported_msg_count;
    uint32_t sequence_drop_count;
    uint32_t last_frame_ms;
    uint32_t last_check_ms;
    MtfHealth health;
    uint8_t  last_flow_quality;
    uint8_t  last_flow_status;
    uint8_t  last_tof_status;
} MtfDiagnostics;

extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

void mtf_01_init(void);
bool micolink_rx_ok(void);
bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);
bool micolink_check_sum(MICOLINK_MSG_t *msg);
void mtf_01_task(void *argument);

/* ---- ping-pong 回调（中断上下文） ---- */
bool mtf_01_pingpong_callback(USART_Data *self,
                              const UsartPingPongChunk *chunk,
                              void *user);

/* ---- 新 getter（基于 mtf_01_stream） ---- */
bool     mtf_01_get_latest_sample(MtfSample *out, uint32_t now_ms);
MtfHealth mtf_01_get_health(uint32_t now_ms);
bool     mtf_01_is_flow_usable(uint32_t now_ms);
void     mtf_01_get_diagnostics(MtfDiagnostics *out);

#endif
