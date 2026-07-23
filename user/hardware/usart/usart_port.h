#ifndef USART_PORT_H
#define USART_PORT_H
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#define USART_UX USART1 //printf使用的串口


/******************************************************************************************/

//#define USART_PORT_NUM 3 /* 串口数量 */
#define USART_REC_LEN 400 /* 定义最大接收字节数 200 */
#define USART_EN_RX 1     /* 使能（1）/禁止（0）串口1接收 */

#define USART_MALLOC 0 /* 串口接收缓冲区使用malloc动态分配内存 */

typedef void (*uartCallBack)(void*);

typedef enum {
	DMA_MODE = 0,
	IT_MODE
} UsartMode;

typedef struct USART_Data {
  UART_HandleTypeDef *huart;
  uint8_t rxBuffer; /* HAL库使用的串口接收缓冲 */

  /*  接收状态
   *  bit15，      接收完成标志
   *  bit14，      接收到0x0d
   *  bit13~0，    接收到的有效字节数目
   */
  uint16_t usart_rx_sta;    
  uint16_t rxSize_Max;  /* 接收缓冲大小 */
  uint8_t *usart_rx_buf; /* 接收缓冲, 最大rxSize_Max个字节,在Init中动态分配 */
  uartCallBack callback; /*中断完成后执行的函数*/
  struct USART_Data *next;
	//uint8_t usart_rx_buf[0];
} USART_Data;
//p=(USART_Data*)malloc(sizeof(USART_Data)+RX_BUF);
//extern USART_Data usartDataHead_handle[USART_PORT_NUM];
#if USART_MALLOC == 1
void USART_DataTypeInit(USART_Data *this, UART_HandleTypeDef *huart,
                        uint16_t rxSize_Max, uint8_t receiveMode,uartCallBack callback);
#else 
void USART_DataTypeInit(USART_Data *this, UART_HandleTypeDef *huart,uint8_t* rxBuffer,
						uint16_t rxSize_Max,uint8_t receiveMode,uartCallBack callback);
#endif

/// @brief 数据接收是否完成
/// @param  this 指向要查看的串口的数据结构体的指针
/// @return true 接收完成 false 接收未完成
static inline bool USART_DataIsReceived(USART_Data *this)
{
    return (this->usart_rx_sta & 0x8000);
}
/// @brief 串口接收到的数据长度
/// @param  this 指向要查看的串口的数据结构体的指针
/// @return  接收到的数据长度，最大为2^14-1
static inline uint16_t USART_DataGetReceivedLen(USART_Data *this)
{
    return (this->usart_rx_sta & 0x3fff);
}

/// @brief 清除接收完成标志位
/// @param  this 指向要查看的串口的数据结构体的指针
static inline void USART_DataResetReceivedFlag(USART_Data *this)
{
    this->usart_rx_sta &= 0x7fff;//清空接收状态
}
/// @brief 获取串口接收缓冲区
/// @param  this 串口数据结构体指针
/// @return 串口接收缓冲区指针 
static inline uint8_t* USART_GetData(USART_Data *this)
{
    return this->usart_rx_buf;
}
/* ---- MTF-01 专用 ping-pong DMA 接口 ---- */
#ifndef USART_MAX_INSTANCES
#define USART_MAX_INSTANCES 8u
#endif

typedef enum {
    USART_PINGPONG_EVT_IDLE = 0,
    USART_PINGPONG_EVT_TC   = 1,
} UsartPingPongEvent;

typedef struct {
    uint8_t          *data;
    uint16_t          len;
    UsartPingPongEvent event;
} UsartPingPongChunk;

typedef bool (*UsartPingPongCallback)(USART_Data *self,
                                      const UsartPingPongChunk *chunk,
                                      void *user);

bool USART_DataStartPingPong(USART_Data *self, uint8_t *buf_a,
                             uint8_t *buf_b, uint16_t buf_size,
                             UsartPingPongCallback cb, void *user);
void USART_DataStopPingPong(USART_Data *self);

#endif  // USART_PORT_H
