#include "usart_send.h"
#include "FreeRTOS.h"
/**
 * @brief 在关键区域(创建队列)时不能发生切换，关闭中断
 * 
 */
static UBaseType_t uxSavedInterruptStatus;
void USART_Enter_Critical(void)
{
    // __disable_irq();   // 关闭所有中断
    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
}

void USART_Exit_Critical(void)
{
    // __enable_irq();   // 开启所有中断
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
}

#if UART_SEND_MUTITHTEAD == 1
void USART_Mutex_Lock(UART_MUTEX *mutex)
{
    xSemaphoreTake(*mutex, portMAX_DELAY);
    // 这里可以添加锁机制，防止多线程同时访问
}
void USART_Mutex_Unlock(UART_MUTEX *mutex)
{
    xSemaphoreGive(*mutex);
    // 这里可以添加解锁机制
}

void USART_Mutex_Init(UART_MUTEX *mutex)
{
    *mutex = xSemaphoreCreateMutex();
    // 这里可以添加锁初始化机制
}
#endif
/**
 * @brief 发送接口(Unlock版本,在调用此函数时已经进入临界区)
 * 
 * @param this USART_SendType 结构体指针
 * @param data 数据
 * @param size 数据大小
 */
void USART_port_output(USART_SendType *this,uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(this->handle, data, size); // 直接发送队列的数据
}

// 串口发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //在串口使用DMA把数据发送完成后调用以下函数
    USART_SendCallback(huart);
}

// 这里是printf库中的函数哦。
#define USART_UX USART1 //printf使用的串口
void putchar_(char c)
{
    while ((USART_UX->SR & 0X40) == 0); /* 等待上一个字符发送完成 */
    USART_UX->DR = (uint8_t)c; /* 将要发送的字符 ch 写入到DR寄存器 */
    // HAL_UART_Transmit(&huart1, (uint8_t *)&character, 1, 0xFFFF);   // 这是一个阻塞的函数
}
