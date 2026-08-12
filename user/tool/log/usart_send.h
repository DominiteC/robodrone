#ifndef __USART_SEND_H__
#define __USART_SEND_H__

#include "usart.h"
#include "usart_send_port.h"
#define USART_QUEUE_MAX_SIZE 40 // 单个对象发送队列最大长度
#define USART_QUEUE_RINGBUFFER_SIZE USART_QUEUE_MAX_SIZE // 全局用于环形缓冲区(中断中使用)的队列的长度
#define USART_BUFF_MALLOC_SIZE 1024    // 单个对象malloc出来的发送缓冲区最大长度
#define USART_RING_BUFF_SIZE 8192 // 全局环形缓冲区大小（从4KB增加到8KB以支持高频振动数据）

enum USART_SEND_FLAG
{
    USART_SEND_IDLE = 0,    // 串口发送空闲
    USART_SEND_BUSY         // 串口正在发送
};

// 串口发送状态
typedef enum _usart_send_state
{
    USART_SEND_SUCCESS = 0, // 串口发送成功
    USART_SEND_MALLOC_FAIL, // 串口发送内存分配失败
    USART_SEND_QUEUE_FULL,  // 串口发送队列满
    USART_SEND_BUFF_FULL,   // 串口发送缓冲区满
    USART_SEND_FAIL,        // 串口发送失败
    USART_SEND_TO_RING_BUFF // 串口发送到环形缓冲区
} Usart_SendState;

typedef enum _usart_send_mode
{
    USART_NO_MOLLOC = 0,    // 串口发送不使用malloc, 由用户自己分配内存或其他方式保证数据有效
    USART_USE_MOLLOC,       // 串口发送使用malloc, 由函数内部分配内存
    USART_USE_RING_BUFF     // 串口发送使用环形缓冲区
} Usart_SendMode;

typedef struct _data_queue  // 发送数据队列
{
    uint8_t *data;
    uint16_t size;
    Usart_SendMode mode;
    struct _data_queue *next;
} DataQueue;

typedef struct _usart_send
{
    UART_sendDevice_Handle* handle;
    DataQueue *head; /* 发送队列头 */
    DataQueue *tail; /* 发送队列尾 */
    enum USART_SEND_FLAG usart_tx_sta; /* 发送状态 */
    uint8_t queue_size_malloc; // 当前malloc队列长度
    uint16_t buff_size_malloc; // 当前使用的malloc缓冲区总大小
#if UART_SEND_MUTITHTEAD == 1
    UART_MUTEX mutex; // 多线程保护锁,用来保护queue_size_malloc和buff_size_malloc，其他变量会在中断中使用，不适合加锁
#endif
    struct _usart_send* next;
} USART_SendType;



void USART_SendInit(USART_SendType *this, UART_sendDevice_Handle *huart);
Usart_SendState USART_SendData(USART_SendType *this, uint8_t *data, uint16_t size, Usart_SendMode mode);
Usart_SendState USART_Printf(USART_SendType *this, const char *format, ...);

void USART_SendCallback(UART_sendDevice_Handle *handle);

#endif
