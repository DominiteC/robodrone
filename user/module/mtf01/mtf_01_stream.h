/**
 * @file mtf_01_stream.h
 * @brief MTF-01 MicoLink 流解析器 — 类型、健康状态机与诊断接口
 *
 * 本模块与 mtf_01.c 互补：
 * - mtf_01.c 继续负责 UART 初始化和任务调度；
 * - mtf_01_stream 负责原始数据块推入、逐字节解析、完整帧快照发布、
 *   健康状态判定和诊断计数。
 *
 * 接口按 FreeRTOS 任务拆分设计：
 * - ISR/回调 调用 push_chunk / publish_sample（无阻塞）；
 * - 解析任务 调用 take_sample / get_health / get_diagnostics；
 * - 控制任务 调用 is_flow_usable（只读，快照）。
 *
 * 共享类型（MtfHealth/MtfSample/MtfDiagnostics）定义在 mtf_01.h 中。
 */
#ifndef __MTF_01_STREAM_H__
#define __MTF_01_STREAM_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef MTF_01_PC_TEST
/* PC 侧自测：用桩头替代真实 mtf_01.h（避免引入 HAL/usart.h） */
#include "mtf_01_pc.h"
#else
#include "mtf_01.h"
#endif

/* ---- 配置常量 ---- */
#define MTF_RX_CHUNK_SIZE      32u
#define MTF_RAW_QUEUE_LEN      12u
#define MTF_STALE_TIMEOUT_MS   200u
#define MTF_LOST_TIMEOUT_MS    500u
#define MTF_RECOVERY_FRAMES    5u
#define MTF_TOF_MIN_DISTANCE_MM 10u
#define MTF_FLOW_QUALITY_MIN    35u   /* 低于此值视为不可用 */

/* ---- 事件类型 ---- */
typedef enum {
    MTF_EVENT_IDLE = 0,
    MTF_EVENT_TC   = 1,
} MtfEventType;

/* ---- 原始数据块 ---- */
typedef struct {
    uint16_t     len;
    MtfEventType event;
    uint8_t      data[MTF_RX_CHUNK_SIZE];
} MtfRxChunk;

/* ---- 接口 ---- */
void     mtf_01_stream_init(uint32_t now_ms);
bool     mtf_01_stream_push_chunk(const MtfRxChunk *chunk);
bool     mtf_01_stream_publish_sample(const MtfSample *sample);
bool     mtf_01_stream_take_sample(MtfSample *out, uint32_t now_ms);
MtfHealth mtf_01_stream_get_health(uint32_t now_ms);
bool     mtf_01_stream_is_flow_usable(uint32_t now_ms);
void     mtf_01_stream_get_diagnostics(MtfDiagnostics *out);
void     mtf_01_stream_inc_queue_drop(void);   /* ISR 中队列满时递增丢帧计数 */

#endif /* __MTF_01_STREAM_H__ */
