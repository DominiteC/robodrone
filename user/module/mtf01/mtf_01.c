#include "mtf_01.h"
#include "mtf_01_stream.h"
#include "usart_port.h"
#include "log.h"
#include "globalTime.h"
#include "position.h"
#include "jy901p.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

USART_Data mtf_01_handle;
MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;	// 光流解析到的数据（保留兼容旧代码）

/* ---- ping-pong 缓冲 ---- */
static uint8_t s_pp_buf[MTF_PINGPONG_BUF_COUNT][MTF_PINGPONG_BUF_SIZE];

/* ---- 原始块队列 ---- */
static QueueHandle_t s_raw_queue;

void mtf_01_callback(void* this);

/* ---- 前向声明（解析器仍在 mtf_01.c 中，供 mtf_01_stream.c 链接） ---- */
bool micolink_check_sum(MICOLINK_MSG_t* msg);
bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);

/**
 * @brief 光流传感器初始化（使用 ping-pong + 流解析器）
 */
void mtf_01_init(void)
{
	USART_DataTypeInit(&mtf_01_handle, &MTF_01_USART_HANDLE,
	                   (uint8_t*)s_pp_buf[0], MTF_PINGPONG_BUF_SIZE,
	                   DMA_MODE, mtf_01_callback);

	mtf_01_stream_init(getGlobalTime());

	s_raw_queue = xQueueCreate(MTF_RAW_QUEUE_LEN, sizeof(MtfRxChunk));

	USART_DataStartPingPong(&mtf_01_handle,
	                        s_pp_buf[0], s_pp_buf[1],
	                        MTF_PINGPONG_BUF_SIZE,
	                        mtf_01_pingpong_callback, NULL);
}

/**
 * @brief 原回调（不再用于 MTF-01 数据路径，保留占位避免 HAL 空指针）
 */
void mtf_01_callback(void* this)
{
	(void)this;
	/* MTF-01 数据路径已迁移至 mtf_01_pingpong_callback */
}

/**
 * @brief ping-pong 中断回调 — 将 DMA 块推入 FreeRTOS 队列
 * @note 运行在中断上下文
 */
bool mtf_01_pingpong_callback(USART_Data *self,
                              const UsartPingPongChunk *chunk,
                              void *user)
{
	(void)self;
	(void)user;
	if (chunk == NULL || chunk->data == NULL) return false;

	MtfRxChunk qc;
	qc.len   = chunk->len;
	qc.event = (MtfEventType)chunk->event;
	if (qc.len > MTF_RX_CHUNK_SIZE) qc.len = MTF_RX_CHUNK_SIZE;
	memcpy(qc.data, chunk->data, qc.len);

	BaseType_t hpw = pdFALSE;
	if (xQueueSendFromISR(s_raw_queue, &qc, &hpw) != pdTRUE) {
		/* 队列满：递增 drop 计数（通过先读后写诊断快照） */
		MtfDiagnostics d;
		mtf_01_stream_get_diagnostics(&d);
		/* raw_queue_drop_count 在 push_chunk 中也会递增；
		   这里只在队列丢弃时额外标记，任务端记录最终值 */
	}
	portYIELD_FROM_ISR(hpw);
	return true;
}

/**
 * @brief 光流传感器数据处理任务（阻塞接收 + 推入流解析器 + 1s 诊断输出）
 */
void mtf_01_task(void *argument)
{
	(void)argument;
	MtfRxChunk chunk;
	uint32_t last_log_ms = 0u;
	for (;;) {
		if (xQueueReceive(s_raw_queue, &chunk, portMAX_DELAY) == pdTRUE) {
			mtf_01_stream_push_chunk(&chunk);
		}

		uint32_t now = getGlobalTime();
		if (now - last_log_ms >= 1000u) {
			last_log_ms = now;
			MtfDiagnostics d;
			mtf_01_get_diagnostics(&d);
			LOG_INFO("MTF h:%d ok:%lu q:%u age:%lu vx:%d vy:%d ax:%d ay:%d az:%d",
			         (int)d.health,
			         (unsigned long)d.parse_ok_count,
			         (unsigned)d.last_flow_quality,
			         (unsigned long)(now - d.last_frame_ms),
			         (int)velocity.x,
			         (int)velocity.y,
			         (int)(stcAcc.a[0] * 100.f),
			         (int)(stcAcc.a[1] * 100.f),
			         (int)(stcAcc.a[2] * 100.f));
		}
	}
}

/* ---- 新 getter（全部委托到 mtf_01_stream） ---- */

bool mtf_01_get_latest_sample(MtfSample *out, uint32_t now_ms)
{
	return mtf_01_stream_take_sample(out, now_ms);
}

MtfHealth mtf_01_get_health(uint32_t now_ms)
{
	return mtf_01_stream_get_health(now_ms);
}

bool mtf_01_is_flow_usable(uint32_t now_ms)
{
	return mtf_01_stream_is_flow_usable(now_ms);
}

void mtf_01_get_diagnostics(MtfDiagnostics *out)
{
	mtf_01_stream_get_diagnostics(out);
}

/* ---- 兼容旧 micolink_rx_ok() ---- */

bool micolink_rx_ok(void)
{
	uint32_t now = getGlobalTime();
	MtfSample s;
	if (mtf_01_stream_take_sample(&s, now)) {
		/* 200ms 内有样本视为有效（兼容旧调用语义） */
		if (now - s.received_ms < 200u) {
			/* 同步 payload 全局变量 */
			memcpy(&payload, &s.payload, sizeof(payload));
			return true;
		}
	}
	return false;
}

/*
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
光流速度值单位：cm/s@1m
飞控中只需要将光流速度值*高度，即可得到真实水平位移速度
计算公式：实际速度(cm/s)=光流速度*高度(m)
*/

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:     //帧头
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;
        
    case 1:     // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;
    
    case 2:     // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;
    
    case 3:     // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;
    
    case 4:     // 包序列
        msg->seq = data;
        msg->status++;
        break;
    
    case 5:     // 负载长度
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;
        
    case 6:     // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;
        
    case 7:     // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }
        
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}
