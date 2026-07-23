/**
 * @file mtf_01_stream.c
 * @brief MTF-01 MicoLink 流解析器实现 — 逐字节解析、健康状态机、诊断计数
 */

#include "mtf_01_stream.h"
#include <string.h>

/* ---- 内部状态 ---- */
typedef struct {
    MICOLINK_MSG_t msg;
    uint16_t       recovery_cnt;  /* 帧抵达计数器，INIT→VALID 用 */
    MtfSample      latest;
    bool           has_sample;
    MtfHealth      health;
    uint32_t       last_frame_ms;
    MtfDiagnostics diag;
} MtfStream;

static MtfStream g_stream;

/* ---- 辅助函数 ---- */

/**
 * @brief 根据消息 ID 返回预期的 payload 字节数
 * @return 0 表示未知消息 ID
 */
static uint8_t micolink_payload_len(uint8_t msg_id)
{
    if (msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
        return (uint8_t)sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t);
    }
    return 0u;
}

/**
 * @brief 记录解析失败，根据失败类型递增对应计数器并清零恢复计数
 * @param kind 0=checksum, 1=length_error, else=unsupported msg
 */
static void mtf_01_stream_record_failure(uint32_t kind)
{
    switch (kind) {
    case 0u: g_stream.diag.checksum_fail_count++;   break;
    case 1u: g_stream.diag.length_error_count++;     break;
    default:  g_stream.diag.unsupported_msg_count++; break;
    }
    g_stream.recovery_cnt = 0u;
}

/**
 * @brief 推动内部健康状态机走一拍（每次 tick 调用一次）
 */
static MtfHealth mtf_01_stream_tick(uint32_t now_ms)
{
    g_stream.diag.last_check_ms = now_ms;

    if (!g_stream.has_sample) {
        g_stream.health = MTF_HEALTH_INIT;
        return g_stream.health;
    }

    uint32_t age = now_ms - g_stream.last_frame_ms;

    switch (g_stream.health) {
    case MTF_HEALTH_INIT:
        if (g_stream.recovery_cnt >= MTF_RECOVERY_FRAMES) {
            g_stream.health = MTF_HEALTH_VALID;
            g_stream.recovery_cnt = 0u;
        }
        break;

    case MTF_HEALTH_VALID:
        if (age >= MTF_STALE_TIMEOUT_MS)
            g_stream.health = MTF_HEALTH_STALE;
        break;

    case MTF_HEALTH_STALE:
        if (age >= MTF_LOST_TIMEOUT_MS)
            g_stream.health = MTF_HEALTH_LOST;
        else if (g_stream.recovery_cnt >= MTF_RECOVERY_FRAMES)
            g_stream.health = MTF_HEALTH_VALID;
        break;

    case MTF_HEALTH_LOST:
        if (g_stream.recovery_cnt >= MTF_RECOVERY_FRAMES)
            g_stream.health = MTF_HEALTH_VALID;
        break;
    }

    g_stream.diag.health = g_stream.health;
    return g_stream.health;
}

/* ---- 公开接口 ---- */

void mtf_01_stream_init(uint32_t now_ms)
{
    memset(&g_stream, 0, sizeof(g_stream));
    g_stream.health = MTF_HEALTH_INIT;
    g_stream.last_frame_ms = 0u;
    g_stream.diag.last_check_ms = now_ms;
}

bool mtf_01_stream_push_chunk(const MtfRxChunk *chunk)
{
    if (chunk == NULL || chunk->len == 0u
        || chunk->len > MTF_RX_CHUNK_SIZE) {
        g_stream.diag.raw_queue_drop_count++;
        return false;
    }

    g_stream.diag.rx_event_count++;
    g_stream.diag.rx_bytes += chunk->len;
    if (chunk->event == MTF_EVENT_TC)
        g_stream.diag.rx_tc_count++;
    else
        g_stream.diag.rx_idle_count++;

    for (uint16_t i = 0u; i < chunk->len; ++i) {
        uint8_t prev_status = g_stream.msg.status;
        if (micolink_parse_char(&g_stream.msg, chunk->data[i]) == false) {
            /* 检测帧被拒绝：status 从非 0 变为 0 表示刚完成一帧但失败 */
            if (prev_status != 0u && g_stream.msg.status == 0u) {
                if (prev_status == 7u) {
                    mtf_01_stream_record_failure(0u); /* checksum */
                } else {
                    mtf_01_stream_record_failure(1u); /* length / format */
                }
            }
            continue;
        }

        /* 只处理 RANGE_SENSOR 消息 */
        if (g_stream.msg.msg_id != MICOLINK_MSG_ID_RANGE_SENSOR) {
            mtf_01_stream_record_failure(2u);
            continue;
        }

        /* payload 长度校验 */
        uint8_t expected_len = micolink_payload_len(g_stream.msg.msg_id);
        if (g_stream.msg.len != expected_len) {
            mtf_01_stream_record_failure(1u);
            continue;
        }

        /* 发布样本 */
        MtfSample sample;
        memset(&sample, 0, sizeof(sample));
        memcpy(&sample.payload, g_stream.msg.payload, g_stream.msg.len);
        sample.seq = g_stream.msg.seq;
        sample.received_ms = g_stream.diag.last_check_ms;
        mtf_01_stream_publish_sample(&sample);
    }
    return true;
}

bool mtf_01_stream_publish_sample(const MtfSample *sample)
{
    if (sample == NULL) return false;

    /* seq 跳变检测 */
    if (g_stream.has_sample) {
        uint8_t diff = (uint8_t)(sample->seq - g_stream.latest.seq);
        if (diff > 1u) {
            g_stream.diag.sequence_drop_count++;
        }
    }

    g_stream.latest       = *sample;
    g_stream.has_sample   = true;
    g_stream.last_frame_ms = sample->received_ms;
    g_stream.diag.last_frame_ms   = sample->received_ms;
    g_stream.diag.last_flow_quality = sample->payload.flow_quality;
    g_stream.diag.last_flow_status  = sample->payload.flow_status;
    g_stream.diag.last_tof_status   = sample->payload.tof_status;
    g_stream.diag.parse_ok_count++;
    g_stream.recovery_cnt++;
    return true;
}

bool mtf_01_stream_take_sample(MtfSample *out, uint32_t now_ms)
{
    if (out == NULL) return false;
    if (!g_stream.has_sample) return false;
    *out = g_stream.latest;
    g_stream.diag.last_check_ms = now_ms;
    return true;
}

MtfHealth mtf_01_stream_get_health(uint32_t now_ms)
{
    return mtf_01_stream_tick(now_ms);
}

bool mtf_01_stream_is_flow_usable(uint32_t now_ms)
{
    MtfHealth h = mtf_01_stream_tick(now_ms);
    if (h != MTF_HEALTH_VALID) return false;
    if (!g_stream.has_sample) return false;
    if (g_stream.latest.payload.distance < MTF_TOF_MIN_DISTANCE_MM) return false;
    if (g_stream.latest.payload.flow_quality < MTF_FLOW_QUALITY_MIN) return false;
    return true;
}

void mtf_01_stream_get_diagnostics(MtfDiagnostics *out)
{
    if (out == NULL) return;
    *out = g_stream.diag;
}

void mtf_01_stream_inc_queue_drop(void)
{
    g_stream.diag.raw_queue_drop_count++;
}
