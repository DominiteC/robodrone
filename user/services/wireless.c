/**
 * wireless.c — 飞控端无线通信（固定 PRX + ACK Payload 模式）
 *
 * 架构说明：
 *   飞控永远作为 PRX（Primary Receiver），固定 RX 模式。
 *   遥控器作为 PTX 主动发起所有通信。
 *   飞控收到控制数据后，通过 Enhanced ShockBurst 的 ACK Payload
 *   自动将遥测数据带回遥控器，无需飞控切换 TX 模式。
 *
 * 参考：
 *   - Crazyflie nrf24link.c (固定 PRX, ACK payload 回传)
 *   - TMRh20 RF24 GettingStarted_CallResponse
 *   - Nordic nRF24L01+ Enhanced ShockBurst ACK Payload
 */
#include "wireless.h"
#include "C_code_Log.h"
#include "Mydelay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os2.h"

Wireless_ReceiveCallback wireless_callback = NULL;

QueueHandle_t wireless_semaph;
static uint8_t wireless_rx_idle_streak = 0;
#define WIRELESS_RX_IDLE_RECOVER 20   /* 20 × 50ms = 1s 无接收 → 链路恢复 */

/* ──── 内部函数 ──── */

static void Wireless_RecoverLink(void)
{
    L01_Init();
    Wireless_SwitchToRx();
    wireless_rx_idle_streak = 0;
    LOG_WARN("wireless link recovered (flight ctrl PRX)");
}

/* ──── 公开 API ──── */

/**
 * @brief 初始化无线模块（飞控 PRX 模式）
 * @note  初始化后飞控处于 RX 模式持续监听，等待遥控器发送数据。
 */
void Wireless_Init(void)
{
    L01_Init();
    Wireless_SwitchToRx();
    wireless_semaph = xSemaphoreCreateBinary();
    LOG_INFO("wireless init done (flight ctrl PRX mode)");
}

void Wireless_SetReceiveCallback(Wireless_ReceiveCallback callback)
{
    wireless_callback = callback;
}

/**
 * @brief 切换到 RX 模式
 */
void Wireless_SwitchToRx(void)
{
    L01_SetCE(CE_LOW);
    L01_SetPowerUp();
    L01_SetTRMode(RX_MODE);
    L01_FlushRX();
    L01_FlushTX();
    L01_ClearIRQ(IRQ_ALL);
    L01_SetCE(CE_HIGH);
}

/**
 * @brief 切换到 TX 模式（保留 API 兼容性）
 * @note  在 ACK Payload 模式下，飞控不应主动 TX。
 *        此函数仅用于异常恢复等特殊场景。
 */
void Wireless_SwitchToTx(void)
{
    L01_SetCE(CE_LOW);
    L01_SetPowerUp();
    L01_SetTRMode(TX_MODE);
    L01_FlushRX();
    L01_FlushTX();
    L01_ClearIRQ(IRQ_ALL);
}

/**
 * @brief 预加载 ACK Payload（飞控遥测数据）
 * @param data 遥测数据
 * @param len  数据长度（≤32）
 * @note  调用后，下一次收到遥控器数据包时，ACK 会自动携带此 payload。
 *        在每次收到控制包并处理完毕后调用此函数，保证遥测始终新鲜。
 *        初始化时也应调用一次，确保首个 ACK 不为空。
 */
void Wireless_LoadAckPayload(uint8_t *data, uint8_t len)
{
    L01_WriteRXPayload_InAck(data, len);
}

/**
 * @brief 发送处理函数（保留 API 兼容性）
 * @note  在 ACK Payload 模式下，飞控不应主动发送。
 *        此函数改为仅加载 ACK payload 并立即返回。
 * @return 始终返回 0（成功）
 */
uint8_t Wireless_TransmitHandler(uint8_t transmitData[], uint8_t len)
{
    /* 飞控不再主动 TX，改为将数据加载为 ACK payload */
    Wireless_LoadAckPayload(transmitData, len);
    return 0;
}

/* ──── 接收处理 ──── */

/**
 * @brief 中断回调接收数据分析
 * @note  在 EXTI ISR 给出信号量后由 Wireless_ReceiveTask 调用。
 *        处理 RX_DR 中断：读取控制数据 → 回调上层处理 → 清理。
 */
void Wireless_ReceiveAnalysis(void)
{
    if (GET_L01_IRQ() == 0)
    {
        uint8_t irq_src = L01_ReadIRQSource();
        if (irq_src & (1 << RX_DR))
        {
            INT8U len, rcv_buffer[32];
            if ((len = L01_ReadRXPayload(rcv_buffer)) != 0)
            {
                if (wireless_callback != NULL)
                {
                    wireless_callback(rcv_buffer, len);
                }
            }
        }
        L01_FlushRX();
        L01_ClearIRQ(IRQ_ALL);
    }
}

/* ──── EXTI 中断回调 ──── */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == E01_IRQ_Pin)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t give_ok = pdFALSE;

        if (wireless_semaph == NULL)
        {
            LOG_WARN_IT("接收信号量未初始化");
            return;
        }

        give_ok = xSemaphoreGiveFromISR(wireless_semaph, &xHigherPriorityTaskWoken);
        if (give_ok != pdTRUE)
        {
            LOG_WARN_IT("接收信号量释放失败");
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ──── 接收任务 ──── */

/**
 * @brief 无线接收任务
 * @note  优先级必须比所有发送任务高。
 *        等待 EXTI 信号量 → 处理接收数据。
 *        空闲超时（~1s）后自动恢复链路。
 */
void Wireless_ReceiveTask(void *param)
{
    uint8_t idle_recover_count = 0;
    while (1)
    {
        if (xSemaphoreTake(wireless_semaph, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            Wireless_ReceiveAnalysis();
            HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            idle_recover_count = 0;
            continue;
        }

        if (++idle_recover_count >= WIRELESS_RX_IDLE_RECOVER)
        {
            idle_recover_count = 0;
            LOG_WARN("wireless rx idle too long, recover link");
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            Wireless_RecoverLink();
            HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            continue;
        }

        /* 轮询兜底：有时 IRQ 没触发 EXTI，直接查引脚 */
        if (GET_L01_IRQ() == 0)
        {
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            Wireless_ReceiveAnalysis();
            HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
    }
}
