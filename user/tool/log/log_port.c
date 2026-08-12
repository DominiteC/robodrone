#include "usart_send.h"
#include "log.h"

USART_SendType channel_0;//定义串口输出句柄

// 日志接口初始化
void log_port_init(void)
{
    // 最好直接在这里设置日志输出通道，这里要输入的是你的接口句柄
    log_set_channel(0, &channel_0);
    // 对句柄进行初始化
    USART_SendInit(&channel_0, (&huart1));
}

/**
 * @brief 日志输出接口
 * 
 * @param this 日志通道,这个值是log_set_channel()设置的指针
 * @param data 数据
 * @param size 数据大小
 * @return uint8_t 0:成功 其他失败
 */
uint8_t log_port_output(LogOut_Handle *this, uint8_t *data, uint16_t size)
{
    return USART_SendData(this, data, size, USART_USE_MOLLOC);
}

uint8_t log_port_output_IT(LogOut_Handle *this, uint8_t *data, uint16_t size)
{
    return USART_SendData(this, data, size, USART_USE_RING_BUFF);
}

// 获取时间戳接口
uint32_t log_port_get_time(void) {
    extern uint32_t getGlobalTime();
    return getGlobalTime();
}
#if (LOG_BUFFER_USE_STACK == 0)
static UBaseType_t uxSavedInterruptStatus;
uint8_t log_sprintf_buffer_lock(uint32_t timeout)
{
    //此函数可能会在中断中调用
    // 这里可以添加锁机制，防止多线程同时使用log_buffer
    uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    return 0;
}
void log_sprintf_buffer_unlock(void)
{
    // 这里可以添加解锁机制
    portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
}
#endif