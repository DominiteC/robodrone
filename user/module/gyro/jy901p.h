#ifndef __JY901_H__
#define __JY901_H__

#include <stdint.h>
#include "jy901p_reg.h"

#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */


#define READ_HEAD 			0x55
#define READ_TYPE_ACC 		0x51
#define READ_TYPE_GYRO 		0X52
#define READ_TYPE_ANGLE 	0x53
#define READ_TYPE_MAG	 	0x54
#define READ_TYPE_Q	 		0x55

typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);

struct SAcc
{
	float a[3];
	float T;	// 温度
};
struct SGyro
{
	float w[3];
	float Vol;	// 电压(非蓝牙产品，该数据无效)
};
struct SAngle
{
	float Angle[3];
	short Version;	// 版本
};
struct SMag
{
	float h[3];
	float T;	// 温度
};
struct SQ
{
	float q[4];
};


//-------------------------JY901P 传感器数据 (全局可见)-----------------------------------
extern struct SAcc 		stcAcc;     /* 加速度计数据 (传感器坐标系) */
extern struct SGyro 	stcGyro;    /* 陀螺仪角速度 (传感器坐标系) */
extern struct SAngle 	stcAngle;   /* 欧拉角融合结果 (传感器坐标系) */
extern struct SMag 		stcMag;     /* 磁力计数据 */
extern struct SQ		stcQ;       /* 四元数姿态数据 */
//-------------------------JY901P 传感器数据 (全局可见)-----------------------------------

int32_t WitSerialWriteRegister(SerialWrite Write_func);
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);

int32_t WitStartAccCali(void);
int32_t WitStopAccCali(void);
int32_t WitStartMagCali(void);
int32_t WitStopMagCali(void);

int32_t WitSetOrient(int32_t uiOrient);

int32_t WitSetUartBaud(int32_t uiBaudIndex);
int32_t WitSetSave(void);
int32_t WitSetOutputRate(int32_t uiRate);
void CopeSerialData(unsigned char ucData);

#endif
