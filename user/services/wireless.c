/*
 * wireless.c
 * 负责无线链路的收发模式切换、链路恢复、接收回调和接收任务。
 * 本文件属于 services 层，底层 nRF24L01P 芯片访问由 module/RF 驱动负责。
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
static uint8_t wireless_tx_fail_streak = 0;
#define WIRELESS_TX_WAIT_MS 35U

static void Wireless_RecoverLink(void)
{
    L01_Init();
    Wireless_SwitchToRx();
    wireless_tx_fail_streak = 0;
}

/*!
 *  @brief        Initialize the wireless module
 *  @param        None
 *  @return       None
 *  @note
*/
void Wireless_Init(void)
{
    // Initialize nRF24L01+ module here
    L01_Init();
    Wireless_SwitchToRx();
    wireless_semaph = xSemaphoreCreateBinary();
}

void Wireless_SetReceiveCallback(Wireless_ReceiveCallback callback)
{
    wireless_callback = callback;
}

/*!
 *  @brief        Switch the current RF mode to RX
 *  @param        None
 *  @return       None
 *  @note
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

/*!
 *  @brief        Switch the current RF mode to TX
 *  @param        None
 *  @return       None
 *  @note
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

/*!
 *  @brief        Transmit handler for transmit mode
 *  @param        None
 *  @return       None
 *  @note
*/
uint8_t Wireless_TransmitHandler(uint8_t transmitData[], uint8_t len)
{
    uint8_t res = 0;
    uint8_t irq_src = 0;
    Wireless_SwitchToTx();
    L01_WriteTXPayload_Ack(transmitData, len);
    L01_SetCE(CE_HIGH);
    while(GET_L01_IRQ() != 0)
    {
        if (myDelay((uint32_t)Wireless_TransmitHandler, WIRELESS_TX_WAIT_MS))
        {
            res = 1;
            break;
        }
    }
    if (res == 0)
    {
        irq_src = L01_ReadIRQSource();
        if (irq_src & (1 << MAX_RT))
        {
            res = 1;
        }
    }
    deleteMyDelay((uint32_t)Wireless_TransmitHandler);
    L01_FlushTX();
    L01_ClearIRQ(IRQ_ALL);
    Wireless_SwitchToRx();
    if (res)
    {
        if (++wireless_tx_fail_streak >= 5)
        {
            LOG_WARN("wireless tx failed continuously, recover link");
            Wireless_RecoverLink();
        }
    }
    else
    {
        wireless_tx_fail_streak = 0;
    }
    return res;
}

/**
 * @brief 涓柇鍥炶皟鎺ュ彈鏁版嵁鍒嗘瀽鍑芥暟
 * 
 */
void Wireless_ReceiveAnalysis(void)
{
    if(GET_L01_IRQ() == 0)
    {
        if(L01_ReadIRQSource() & (1 << RX_DR))//detect RF module recieve interrupt
        {
            INT8U len, rcv_buffer[32];
            if((len = L01_ReadRXPayload(rcv_buffer)) != 0)
            {
                // do something with the received data
                // LED1_ON();
                // uint8_t buff[100];
                // uint8_t buff_len = 0;
                // buff_len += sprintf((char *)(buff + buff_len), "len: %d, data: ", len);
                // for (uint8_t i = 0; i < len; i++)
                // {
                //     buff_len += sprintf((char *)(buff + buff_len), "%x", rcv_buffer[i]);
                // }
                // LOG_DEBUG("%s",(char *)buff);
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

// 璇ヤ换鍔′紭鍏堢骇涓€瀹氳姣斿彂閫佷紭鍏堢骇楂?
void Wireless_ReceiveTask(void* param)
{
    uint8_t idle_recover_count = 0;
    while(1)
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

        if (++idle_recover_count >= 20)
        {
            idle_recover_count = 0;
            LOG_WARN("wireless rx idle too long, recover link");
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            Wireless_RecoverLink();
            HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            continue;
        }

        if (GET_L01_IRQ() == 0)
        {
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            Wireless_ReceiveAnalysis();
            HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
    }
}
