/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211103
 * 第一次发布
 *
 * 修改说明：by Sevenfite 2023/12/9
 * 增加了数据结构体，原来只能有一个串口，现在可以有多个串口
 * 数据结构体中增加了接收缓冲区的大小，可以动态分配内存
 * 支持中断接收和DMA接收
 * 删除了串口初始化的内容，兼容了CubeMX生成的代码，底层初始化由CubeMX生成的代码完成
 * 使用时只需要在main函数中调用USART_DataTypeInit函数初始化数据结构体即可
 * 增加了串口接收完成标志位和接收长度的获取函数（内联函数）
 * @warning   -------重要-------  禁止在其他地方定义串口数据结构体（USART_Data），
 * 只能使用本文件中已定义的结构体数组，可以通过宏定义改变数组大小
 * 上述缺陷改进方案：用链表的形式来存储串口数据结构体，这样可以动态增加串口数据结构体
 * 初始化时把串口数据结构体加入到链表中，串口中断处理函数遍历链表处理数据
 * 
 * 修改说明：by Sevenfite 2023/12/10
 * 增加了静态分配内存的方式，可以自己定义一个全局变量作为接收缓冲区，可以避免使用malloc
 * 
 * 修改说明：by Sevenfite 2024/1/12
 * 增加了清除接收完成标志位的函数，在处理完接收到的数据后，可以调用该函数清除接收完成标志位
 * 修改意见：可以增加FIFO的队列来存储接收到的数据，这样可以避免数据丢失
 ****************************************************************************************************
 */

#include "usart_port.h"
#include "usart_send.h"
#include <stdio.h>


#if USART_MALLOC ==1
#include <stdlib.h>
#endif
#include "usart.h"

// /* 如果使用os,则包括下面的头文件即可. */
// #if SYS_SUPPORT_OS
// #include "includes.h" /* os 使用 */
// #endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)           /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t"); /* 声明不使用半主机模式 */
__asm(
    ".global __ARM_use_no_argv \n\t"); /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式
                                        */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE {
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式
 */
int _ttywrch(int ch) {
  ch = ch;
  return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x) { x = x; }

char *_sys_command_string(char *cmd, int len) { return NULL; }

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f) {
  static char dma_buf[64];
  static int  dma_pos = 0;

  dma_buf[dma_pos++] = (char)ch;

  /* 遇到换行或缓冲区满时, 通过 DMA 批量发送, 绝不轮询 USART1->DR */
  if (ch == '\n' || dma_pos >= (int)sizeof(dma_buf) - 1) {
    extern USART_SendType channel_0;
    USART_SendData(&channel_0, (uint8_t *)dma_buf, dma_pos, USART_USE_MOLLOC);
    dma_pos = 0;
  }
  return ch;
}
#endif
/******************************************************************************************/

#if USART_EN_RX /*如果使能了接收*/
USART_Data *usartDataHead_handle = NULL;  // 串口数据结构体链表头
static int USART_DataAttach(USART_Data *this);
#if USART_MALLOC == 1
/// @brief 初始化串口数据类型
/// @note  使用动态分配内存为接收缓冲区分配内存，开启中断接收/开启DMA接收
/// @param  this 指向自己的指针
/// @param huart 串口句柄
/// @param rxSize_Max 接收缓冲区大小
/// @param receiveMode 1:中断接收
/// 0:DMA接收(中断接收要求有结束符\r\n，DMA接收不要求)
void USART_DataTypeInit(USART_Data *this, UART_HandleTypeDef *huart,
                        uint16_t rxSize_Max, uint8_t receiveMode,uartCallBack callback) {
  this->huart = huart;
  this->rxSize_Max = rxSize_Max;
  this->usart_rx_buf = (uint8_t *)malloc(rxSize_Max);
  this->receiveMode = receiveMode;
  this->callback = callback;
  this->next = NULL;
  
  USART_DataAttach(this);

  if (receiveMode) {
  HAL_UART_Receive_IT(this->huart, &this->rxBuffer, 1);
  } else {
  HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->usart_rx_buf, this->rxSize_Max);
  __HAL_DMA_DISABLE_IT(
    this->huart->hdmarx,
    DMA_IT_HT);  // 关闭半传输中断,防止半传输中断影响接收完成标志位和接收长度
  }
}
#else

/**
 * @brief 串口结构体初始化
 * @note  使用静态分配内存为接收缓冲区分配内存，要自己创建一个全局变量，开启中断接收/开启DMA接收
 * @param  this 指向自己的指针，需要自行创建
 * @param huart 串口句柄
 * @param rxBuffer 接收缓冲区，要求大小为rxSize_Max，自己定义的全局变量
 * @param rxSize_Max 接收缓冲区大小
 * @param receiveMode 1:中断接收 0：DMA接收(中断接收要求有结束符\r\n，DMA接收不要求)
 */
void USART_DataTypeInit(USART_Data *this, UART_HandleTypeDef *huart,uint8_t* rxBuffer,
						uint16_t rxSize_Max,UsartMode receiveMode,uartCallBack callback)
{
  this->huart = huart;
  this->rxSize_Max = rxSize_Max;
  this->usart_rx_buf = rxBuffer;
  this->callback = callback;
  this->next = NULL;

  USART_DataAttach(this);

  if (receiveMode) {
  HAL_UART_Receive_IT(this->huart, &this->rxBuffer, 1);
  } else {
  HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->usart_rx_buf, this->rxSize_Max);
  __HAL_DMA_DISABLE_IT(
    this->huart->hdmarx,
    DMA_IT_HT);  // 关闭半传输中断,防止半传输中断影响接收完成标志位和接收长度
  }
}
#endif
/**
 * @brief 将该结构体添加到链表中
 * @param  this 指向自己的指针
 * @return -1表示已经注册，0表示成功注册
 */
static int USART_DataAttach(USART_Data *this)
{
	USART_Data* target = usartDataHead_handle;
	while(target) {
		if(target == this) return -1;	//表示已经注册
		target = target->next;
	}
	this->next = usartDataHead_handle;
	usartDataHead_handle = this;
	return 0;
}

/**
 * @brief       串口数据接收中断回调函数
                数据处理在这里进行（中断接收模式使用）
 * @param       huart:串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  USART_Data* this = usartDataHead_handle;
  while(this) {
    if (huart->Instance == this->huart->Instance) {
      if ((this->usart_rx_sta & 0x8000) == 0) /* 接收未完成 */
      {
        if (this->usart_rx_sta & 0x4000) /* 接收到了0x0d（即回车键） */
        {
          if (this->rxBuffer !=
              0x0a) /* 接收到的不是0x0a（即不是换行键） */
          {
            this->usart_rx_sta = 0; /* 接收错误,重新开始 */
          } else /* 接收到的是0x0a（即换行键） */
          {
            this->usart_rx_sta |= 0x8000; /* 接收完成了 */
            if(this->callback != NULL)
              this->callback((void*)this);
//			printf("1\n");
          }
        } else /* 还没收到0X0d（即回车键） */
        {
          if (this->rxBuffer == 0x0d)
            this->usart_rx_sta |= 0x4000;
          else {
            this->usart_rx_buf[this->usart_rx_sta & 0X3FFF] =
                this->rxBuffer;
            this->usart_rx_sta++;

            if (this->usart_rx_sta > (this->rxSize_Max - 1)) {
              this->usart_rx_sta = 0; /* 接收数据错误,重新开始接收 */
            }
          }
        }
      }
//	  printf("%c",this->rxBuffer);
      HAL_UART_Receive_IT(this->huart, &this->rxBuffer, 1);
    }
    this = this->next;
  }
}
/// @brief 串口空闲中断回调函数（DMA模式使用）
/// @param huart 串口句柄
/// @param Size 接收到的数据长度
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  USART_Data* this = usartDataHead_handle;
  while(this) {
    if (huart->Instance == this->huart->Instance) {
      this->usart_rx_sta = Size;
      this->usart_rx_sta |= 0x8000;  // 接收完成
      HAL_UARTEx_ReceiveToIdle_DMA(this->huart,
                                   this->usart_rx_buf,
                                   this->rxSize_Max);
      __HAL_DMA_DISABLE_IT(
          this->huart->hdmarx,
          DMA_IT_HT);  // 关闭半传输中断,防止半传输中断影响接收完成标志位和接收长度
      if(this->callback != NULL)
        this->callback((void*)this);
    }
    this = this->next;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  USART_Data* this = usartDataHead_handle;
  while(this) {
    if (huart->Instance == this->huart->Instance) {
      this->usart_rx_sta = 0;
      __HAL_UART_CLEAR_PEFLAG(huart);
      __HAL_UART_CLEAR_FEFLAG(huart);
      __HAL_UART_CLEAR_NEFLAG(huart);
      __HAL_UART_CLEAR_OREFLAG(huart);
      HAL_UART_DMAStop(huart);
      HAL_UARTEx_ReceiveToIdle_DMA(this->huart,
                                   this->usart_rx_buf,
                                   this->rxSize_Max);
      __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
      return;
    }
    this = this->next;
  }
}

/**
 * @brief       串口X中断服务函数
                注意,读取USARTx->SR能避免莫名其妙的错误
 * @param       无
 * @retval      无
 */
// void USART_UX_IRQHandler(void)
//{
// #if SYS_SUPPORT_OS                                                   /*
// 使用OS */
//     OSIntEnter();
// #endif
//     HAL_UART_IRQHandler(&g_uart1_handle);                               /*
//     调用HAL库中断处理公用函数 */

//    // while (HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer,
//    RXBUFFERSIZE) != HAL_OK)     /* 重新开启中断并接收数据 */
//    // {
//    //     /* 如果出错会卡死在这里 */
//    // }

// #if SYS_SUPPORT_OS                                                   /*
// 使用OS */
//     OSIntExit();
// #endif
// }
#endif
