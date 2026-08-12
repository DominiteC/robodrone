#include "jy901p.h"
#include <string.h>
#include "stm32f4xx_hal.h"

static SerialWrite p_WitSerialWriteFunc = NULL;

struct SAcc 	stcAcc={0};
struct SGyro 	stcGyro={0};
struct SAngle 	stcAngle={0};
struct SMag 	stcMag={0};
struct SQ		stcQ={0};

#define p_WitDelaymsFunc HAL_Delay

char CheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}

int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}

/**
 * @brief 设置寄存器
 * 
 * @param uiReg 要设置的寄存器
 * @param usData 要设置的值
 * @return int32_t 
 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;

	if(p_WitSerialWriteFunc == NULL)
		return WIT_HAL_EMPTY;
	ucBuff[0] = 0xFF;
	ucBuff[1] = 0xAA;
	ucBuff[2] = uiReg & 0xFF;
	ucBuff[3] = usData & 0xff;
	ucBuff[4] = usData >> 8;
	p_WitSerialWriteFunc(ucBuff, 5);

	return WIT_HAL_OK;
}

/*Acceleration calibration demo*/
int32_t WitStartAccCali(void)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	p_WitDelaymsFunc(20);
	if(WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopAccCali(void)
{
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*Magnetic field calibration*/
int32_t WitStartMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/**
 * @brief 设置安装方向
 * 
 * @param uiOrient 方向参数
 * @return int32_t 
 */
int32_t WitSetOrient(int32_t uiOrient)
{
	if(!CheckRange(uiOrient,ORIENT_HERIZONE,ORIENT_VERTICLE))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(ORIENT, uiOrient) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/**
 * @brief 修改波特率
 * 
 * @param uiBaudIndex 需要设置的波特率索引
 * @return int32_t 
 */
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

int32_t WitSetSave(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/**
 * @brief 修改输出频率
 * 
 * @param uiRate 
 * @return int32_t 
 */
int32_t WitSetOutputRate(int32_t uiRate)
{	
	// if(!CheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	// {
	// 	return WIT_HAL_INVAL;
	// }
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	p_WitDelaymsFunc(20);
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/**
 * @brief 获取数据
 */
void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[12];
	static unsigned char ucRxCnt = 0;
	static unsigned char sumcrc = 0;

	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		sumcrc = 0;
		return ;
	}
	if (ucRxCnt<11) {sumcrc += ucData; return ;}//数据不满11个，则返回
	else
	{
		if (sumcrc != ucRxBuffer[10])
		{
			ucRxCnt=0;
			sumcrc = 0;
			return ;
		}
		switch(ucRxBuffer[1])
		{
			case READ_TYPE_ACC:		stcAcc.a[0] = ((float)(short)((short)ucRxBuffer[3]<<8|ucRxBuffer[2])/32768)*16*9.8;
									stcAcc.a[1] = ((float)(short)((short)ucRxBuffer[5]<<8|ucRxBuffer[4])/32768)*16*9.8;
									stcAcc.a[2] = ((float)(short)((short)ucRxBuffer[7]<<8|ucRxBuffer[6])/32768)*16*9.8;
									break;
			case READ_TYPE_GYRO:	stcGyro.w[0] = ((float)(short)((short)ucRxBuffer[3]<<8|ucRxBuffer[2])/32768)*2000;
									stcGyro.w[1] = ((float)(short)((short)ucRxBuffer[5]<<8|ucRxBuffer[4])/32768)*2000;
									stcGyro.w[2] = ((float)(short)((short)ucRxBuffer[7]<<8|ucRxBuffer[6])/32768)*2000;
									break;
			case READ_TYPE_ANGLE:	stcAngle.Angle[0] = ((float)(short)((short)ucRxBuffer[3]<<8|ucRxBuffer[2])/32768)*180;
									stcAngle.Angle[1] = ((float)(short)((short)ucRxBuffer[5]<<8|ucRxBuffer[4])/32768)*180;
									stcAngle.Angle[2] = ((float)(short)((short)ucRxBuffer[7]<<8|ucRxBuffer[6])/32768)*180;
									stcAngle.Version = ((unsigned short)ucRxBuffer[9]<<8)|ucRxBuffer[8];
									break;
			case READ_TYPE_MAG:		stcMag.h[0] = ((float)(short)((short)ucRxBuffer[3]<<8|ucRxBuffer[2]));
									stcMag.h[1] = ((float)(short)((short)ucRxBuffer[5]<<8|ucRxBuffer[4]));
									stcMag.h[2] = ((float)(short)((short)ucRxBuffer[7]<<8|ucRxBuffer[6]));
									stcMag.T = ((float)(short)((short)ucRxBuffer[9]<<8|ucRxBuffer[8]))/100;
									break;
			case READ_TYPE_Q:		stcQ.q[0] = ((float)(short)((short)ucRxBuffer[3]<<8|ucRxBuffer[2])/32768);
									stcQ.q[1] = ((float)(short)((short)ucRxBuffer[5]<<8|ucRxBuffer[4])/32768);
									stcQ.q[2] = ((float)(short)((short)ucRxBuffer[7]<<8|ucRxBuffer[6])/32768);
									stcQ.q[3] = ((float)(short)((short)ucRxBuffer[9]<<8|ucRxBuffer[8])/32768);
			
		}
		ucRxCnt=0;
		sumcrc = 0;
	}
	return ;
}
