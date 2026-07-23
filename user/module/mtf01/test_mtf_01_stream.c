/**
 * @file test_mtf_01_stream.c
 * @brief MTF-01 MicoLink 流解析器 PC 侧自测套件
 *
 * 编译（在 project 目录执行）：
 *   gcc -std=c99 -Wall -Wextra -I tmp/mtf_pc \
 *       user/module/mtf01/mtf_01_stream.c \
 *       user/module/mtf01/test_mtf_01_stream.c \
 *       -o tmp/test_mtf_01_stream.exe
 *   tmp/test_mtf_01_stream.exe
 *
 * 不加入 Keil 工程，不引入 HAL/FreeRTOS 依赖。
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#define MTF_01_PC_TEST
#include "mtf_01_stream.h"

/* ================================================================
 * PC 桩：micolink_parse_char / micolink_check_sum
 * 从 mtf_01.c 复制，用于独立测试（不链接 HAL / FreeRTOS）
 * ================================================================ */

static bool micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[128];
    uint8_t checksum = 0;
    memcpy(temp, msg, length);
    for (uint8_t i = 0; i < length; i++) checksum += temp[i];
    return checksum == msg->checksum;
}

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch (msg->status) {
    case 0:
        if (data == MICOLINK_MSG_HEAD) { msg->head = data; msg->status++; }
        break;
    case 1: msg->dev_id = data; msg->status++; break;
    case 2: msg->sys_id = data; msg->status++; break;
    case 3: msg->msg_id = data; msg->status++; break;
    case 4: msg->seq = data; msg->status++; break;
    case 5:
        msg->len = data;
        if (msg->len == 0) msg->status += 2;
        else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) msg->status = 0;
        else msg->status++;
        break;
    case 6:
        msg->payload[msg->payload_cnt++] = data;
        if (msg->payload_cnt == msg->len) { msg->payload_cnt = 0; msg->status++; }
        break;
    case 7:
        msg->checksum = data;
        msg->status = 0;
        if (micolink_check_sum(msg)) return true;
        /* fall through */
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }
    return false;
}

/* ================================================================
 * 帧构造工具
 * ================================================================ */

static void frame_build(uint8_t *out, uint8_t msg_id, uint8_t seq,
                        uint8_t len, const uint8_t *payload)
{
    out[0] = MICOLINK_MSG_HEAD;
    out[1] = 1;
    out[2] = 0;
    out[3] = msg_id;
    out[4] = seq;
    out[5] = len;
    memcpy(&out[6], payload, len);
    uint8_t sum = 0;
    for (uint8_t i = 0; i < (uint8_t)(6 + len); i++) sum += out[i];
    out[6 + len] = sum;
}

/* 构建合法的 RANGE_SENSOR 帧（20B payload，distance=100mm） */
static void frame_ok(uint8_t *out, uint8_t seq)
{
    uint8_t payload[20];
    memset(payload, 0, sizeof(payload));
    payload[4] = 100;
    payload[5] = 0;
    payload[6] = 0;
    payload[7] = 0;
    frame_build(out, 0x51, seq, 20, payload);
}

#define FRAME_LEN 27

/* ================================================================
 * 测试辅助宏
 * ================================================================ */

static int g_failures = 0;
static int g_total    = 0;

#define TEST(name)
#define RUN(name) do { \
    g_total++; \
    mtf_01_stream_init(0u); \
    test_##name(); \
    printf("  PASS: %s\n", #name); \
} while (0)

#define ASSERT_EQ(a, b, label) do { \
    if ((unsigned long)(a) != (unsigned long)(b)) { \
        printf("  FAIL %s: " label " expected=%lu got=%lu\n", \
               __func__, (unsigned long)(b), (unsigned long)(a)); \
        g_failures++; \
    } \
} while (0)

#define ASSERT_TRUE(cond, label) do { \
    if (!(cond)) { \
        printf("  FAIL %s: " label "\n", __func__); \
        g_failures++; \
    } \
} while (0)

#define ASSERT_FALSE(cond, label) ASSERT_TRUE(!(cond), label)

#define SET_TIME(ms) do { \
    MtfSample dummy; \
    (void)mtf_01_stream_take_sample(&dummy, (ms)); \
    (void)mtf_01_stream_get_health((ms)); \
} while (0)

/* ================================================================
 * 用例
 * ================================================================ */

static void test_full_frame_in_one_chunk(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 0);
    MtfRxChunk chunk = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(chunk.data, buf, FRAME_LEN);
    ASSERT_TRUE(mtf_01_stream_push_chunk(&chunk), "push failed");
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.parse_ok_count, 1u, "parse_ok_count");
    MtfSample s;
    ASSERT_TRUE(mtf_01_stream_take_sample(&s, 10u), "take_sample failed");
    ASSERT_EQ(s.seq, 0u, "seq");
}

static void test_split_into_two_chunks(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 1);
    MtfRxChunk c1 = { .len = 10, .event = MTF_EVENT_IDLE };
    memcpy(c1.data, buf, 10);
    ASSERT_TRUE(mtf_01_stream_push_chunk(&c1), "push chunk1");
    MtfRxChunk c2 = { .len = 17, .event = MTF_EVENT_IDLE };
    memcpy(c2.data, &buf[10], 17);
    ASSERT_TRUE(mtf_01_stream_push_chunk(&c2), "push chunk2");
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.parse_ok_count, 1u, "parse_ok_count after split");
}

static void test_two_frames_in_one_chunk(void)
{
    uint8_t buf[FRAME_LEN];
    /* 连续 push 两帧 */
    frame_ok(buf, 0);
    MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(c.data, buf, FRAME_LEN);
    ASSERT_TRUE(mtf_01_stream_push_chunk(&c), "push frame0");
    frame_ok(buf, 1);
    memcpy(c.data, buf, FRAME_LEN);
    ASSERT_TRUE(mtf_01_stream_push_chunk(&c), "push frame1");
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.parse_ok_count, 2u, "parse_ok_count two frames");
}

static void test_single_byte_drop(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 0);
    /* 去掉第一字节（帧头），剩 26 字节 */
    MtfRxChunk c1 = { .len = 26, .event = MTF_EVENT_TC };
    memcpy(c1.data, buf + 1, 26);
    mtf_01_stream_push_chunk(&c1);
    /* 下一帧完整 */
    frame_ok(buf, 1);
    MtfRxChunk c2 = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(c2.data, buf, FRAME_LEN);
    mtf_01_stream_push_chunk(&c2);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_TRUE(d.parse_ok_count >= 1u, "must recover after byte drop");
}

static void test_checksum_error(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 0);
    buf[FRAME_LEN - 1] ^= 0xFF;
    MtfRxChunk chunk = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(chunk.data, buf, FRAME_LEN);
    mtf_01_stream_push_chunk(&chunk);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.checksum_fail_count, 1u, "checksum_fail_count");
    ASSERT_EQ(d.parse_ok_count, 0u, "parse_ok_count after bad checksum");
}

static void test_wrong_payload_length(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 0);
    buf[5] = 99;
    MtfRxChunk chunk = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(chunk.data, buf, FRAME_LEN);
    mtf_01_stream_push_chunk(&chunk);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.length_error_count, 1u, "length_error_count");
}

static void test_unsupported_msg_id(void)
{
    uint8_t buf[15];
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    frame_build(buf, 0x52, 0, 8, payload);
    MtfRxChunk chunk = { .len = 15, .event = MTF_EVENT_TC };
    memcpy(chunk.data, buf, 15);
    mtf_01_stream_push_chunk(&chunk);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.unsupported_msg_count, 1u, "unsupported_msg_count");
    ASSERT_EQ(d.parse_ok_count, 0u, "parse_ok_count for unsupported msg");
}

static void test_recovery_to_valid(void)
{
    uint8_t buf[FRAME_LEN];
    for (uint8_t i = 0; i < MTF_RECOVERY_FRAMES; i++) {
        frame_ok(buf, i);
        MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
        memcpy(c.data, buf, FRAME_LEN);
        mtf_01_stream_push_chunk(&c);
        SET_TIME((uint32_t)(i * 10u));
    }
    ASSERT_EQ(mtf_01_stream_get_health(50u), MTF_HEALTH_VALID, "health after recovery");
}

static void test_stale_after_200ms(void)
{
    uint8_t buf[FRAME_LEN];
    for (uint8_t i = 0; i < MTF_RECOVERY_FRAMES; i++) {
        frame_ok(buf, i);
        MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
        memcpy(c.data, buf, FRAME_LEN);
        mtf_01_stream_push_chunk(&c);
        SET_TIME(i * 10u);
    }
    ASSERT_EQ(mtf_01_stream_get_health(50u), MTF_HEALTH_VALID, "health at 50ms");
    ASSERT_EQ(mtf_01_stream_get_health(250u), MTF_HEALTH_STALE, "health at 250ms");
}

static void test_lost_after_500ms(void)
{
    uint8_t buf[FRAME_LEN];
    for (uint8_t i = 0; i < MTF_RECOVERY_FRAMES; i++) {
        frame_ok(buf, i);
        MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
        memcpy(c.data, buf, FRAME_LEN);
        mtf_01_stream_push_chunk(&c);
        SET_TIME(i * 10u);
    }
    ASSERT_EQ(mtf_01_stream_get_health(50u), MTF_HEALTH_VALID, "health at 50ms");
    ASSERT_EQ(mtf_01_stream_get_health(250u), MTF_HEALTH_STALE, "health at 250ms");
    ASSERT_EQ(mtf_01_stream_get_health(550u), MTF_HEALTH_LOST, "health at 550ms");
}

static void test_distance_zero_not_usable(void)
{
    uint8_t buf[FRAME_LEN];
    for (uint8_t i = 0; i <= MTF_RECOVERY_FRAMES; i++) {
        frame_ok(buf, i);
        memset(&buf[10], 0, 4);
        MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
        memcpy(c.data, buf, FRAME_LEN);
        mtf_01_stream_push_chunk(&c);
        if (i > 0) SET_TIME(i * 15u);
    }
    ASSERT_FALSE(mtf_01_stream_is_flow_usable(80u), "flow not usable with distance=0");
}

static void test_zero_length_chunk_dropped(void)
{
    MtfRxChunk c = { .len = 0, .event = MTF_EVENT_TC };
    mtf_01_stream_push_chunk(&c);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.raw_queue_drop_count, 1u, "drop zero-length chunk");
}

static void test_seq_drop_count(void)
{
    uint8_t buf[FRAME_LEN];
    frame_ok(buf, 0);
    MtfRxChunk c = { .len = FRAME_LEN, .event = MTF_EVENT_TC };
    memcpy(c.data, buf, FRAME_LEN);
    mtf_01_stream_push_chunk(&c);
    frame_ok(buf, 5);
    memcpy(c.data, buf, FRAME_LEN);
    mtf_01_stream_push_chunk(&c);
    MtfDiagnostics d;
    mtf_01_stream_get_diagnostics(&d);
    ASSERT_EQ(d.sequence_drop_count, 1u, "sequence_drop_count");
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    printf("=== MTF-01 Stream Parser Self-Test ===\n");

    mtf_01_stream_init(0u); test_full_frame_in_one_chunk();    g_total++;
    mtf_01_stream_init(0u); test_split_into_two_chunks();      g_total++;
    mtf_01_stream_init(0u); test_two_frames_in_one_chunk();    g_total++;
    mtf_01_stream_init(0u); test_single_byte_drop();           g_total++;
    mtf_01_stream_init(0u); test_checksum_error();             g_total++;
    mtf_01_stream_init(0u); test_wrong_payload_length();       g_total++;
    mtf_01_stream_init(0u); test_unsupported_msg_id();         g_total++;
    mtf_01_stream_init(0u); test_recovery_to_valid();          g_total++;
    mtf_01_stream_init(0u); test_stale_after_200ms();          g_total++;
    mtf_01_stream_init(0u); test_lost_after_500ms();           g_total++;
    mtf_01_stream_init(0u); test_distance_zero_not_usable();   g_total++;
    mtf_01_stream_init(0u); test_zero_length_chunk_dropped();  g_total++;
    mtf_01_stream_init(0u); test_seq_drop_count();             g_total++;

    printf("\n=== %d/%d tests passed ===\n", g_total - g_failures, g_total);
    return (g_failures == 0) ? 0 : 1;
}
