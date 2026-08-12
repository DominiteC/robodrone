#ifndef _USART_SEND_PORT_H_
#define _USART_SEND_PORT_H_
#include "usart.h"
typedef UART_HandleTypeDef UART_sendDevice_Handle;
#define UART_SEND_MUTITHTEAD 1 // 是否启用多线程保护, 1: 启用, 0: 不启用
#if UART_SEND_MUTITHTEAD == 1
#include "FreeRTOS.h"
#include "semphr.h"
typedef SemaphoreHandle_t UART_MUTEX;
void USART_Mutex_Lock(UART_MUTEX *mutex);
void USART_Mutex_Unlock(UART_MUTEX *mutex);
void USART_Mutex_Init(UART_MUTEX *mutex);
#endif
#endif // !_USART_SEND_PORT_H_