#include "bn220.h"
#include "usart_port.h"
#include "minmea.h"
#include "C_code_Log.h"
#include <string.h>

//-------------------------BN220 GPS 串口数据-----------------------------------
USART_Data bn220_handle;       /* BN220 GPS 串口数据接收句柄 (DMA 模式) */
uint8_t bn220_buf[200];        /* BN220 GPS 原始数据缓冲区 (NMEA 句子) */
//-------------------------BN220 GPS 串口数据-----------------------------------
void BN220_Decode(void* this);

/**
 * @brief BN220初始化
 * 
 */
void BN220_Init(void)
{
	USART_DataTypeInit(&bn220_handle,&BN220_HANDLE,bn220_buf,sizeof(bn220_buf),IT_MODE,BN220_Decode);
}

/**
 * @brief 对BN220接收的数据进行解析
 * 
 * @param this 
 */
void BN220_Decode(void* this)
{
	if ((USART_Data*)this == &bn220_handle)
	{
		uint8_t getLen = USART_DataGetReceivedLen((USART_Data*)this);
		uint8_t* data = USART_GetData((USART_Data*)this);
//		LOG_DEBUG_IT("%s",data);
		switch (minmea_sentence_id((char *)data, false)) {
            case MINMEA_SENTENCE_RMC: {
                struct minmea_sentence_rmc frame;	// 存放解析得到的数据
                if (minmea_parse_rmc(&frame, (char *)data)) {
                    LOG_DEBUG_IT("$xxRMC: 原始坐标和速度: (%d/%d,%d/%d) %d/%d",
								 frame.latitude.value, frame.latitude.scale,
								 frame.longitude.value, frame.longitude.scale,
								 frame.speed.value, frame.speed.scale);
                    LOG_DEBUG_IT("$xxRMC 定点坐标和速度缩放到小数点后三位: (%d,%d) %d",
								 minmea_rescale(&frame.latitude, 1000),
								 minmea_rescale(&frame.longitude, 1000),
								 minmea_rescale(&frame.speed, 1000));
                    LOG_DEBUG_IT("$xxRMC 浮点次数坐标和速度: (%f,%f) %f",
								 minmea_tocoord(&frame.latitude),
								 minmea_tocoord(&frame.longitude),
								 minmea_tofloat(&frame.speed));
                } else {
                    LOG_DEBUG_IT("$xxRMC sentence is not parsed");
                }
			} break;
			
			case MINMEA_INVALID:
				LOG_INFO_IT("MINMEA_INVALID");
				break;
			case MINMEA_UNKNOWN:
				LOG_INFO_IT("MINMEA_UNKNOWN");
				break;
			default:
				LOG_INFO_IT("Other ID %d",minmea_sentence_id((char *)data, false));
				break;
		}
		
		memset(bn220_buf,0,sizeof(bn220_buf));
		((USART_Data*)this)->usart_rx_sta = 0;
		USART_DataResetReceivedFlag((USART_Data*)this);
	}
}
