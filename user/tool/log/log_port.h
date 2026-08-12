#ifndef _LOG_PORT_H_
#define _LOG_PORT_H_
#include "usart_send.h"

#define LOG_BUFFER_SIZE 128 // 日志snprintf缓冲区大小,即一次性最大输出长度
#define LOG_BUFFER_USE_STACK 1 // 日志输出缓冲区使用栈空间, 1: 使用栈空间, 0: 使用全局变量

typedef USART_SendType LogOut_Handle;

void log_port_init(void);
uint8_t log_port_output(LogOut_Handle *this, uint8_t *data, uint16_t size);
uint8_t log_port_output_IT(LogOut_Handle *this, uint8_t *data, uint16_t size);
uint32_t log_port_get_time(void);
#if (LOG_BUFFER_USE_STACK == 0)
uint8_t log_sprintf_buffer_lock(uint32_t timeout);
void log_sprintf_buffer_unlock(void);
#endif
#endif