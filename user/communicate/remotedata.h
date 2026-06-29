#ifndef __REMOTEDATA_H__
#define __REMOTEDATA_H__

#include <stdint.h>
#include "vector_types.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define Byte0(data)       ( *( (char *)(&data)	  ) )
#define Byte1(data)       ( *( (char *)(&data) + 1) )
#define Byte2(data)       ( *( (char *)(&data) + 2) )
#define Byte3(data)       ( *( (char *)(&data) + 3) )

#define CMD_CHANGE_CTRL_MODE	0x01	// 控制模式
#define CMD_CHANGE_ATTI_LAND 	0x02	// 姿态模式
#define CMD_FLIGHT_LAND 		0x03	// 飞行着陆
#define CMD_BATTERY_SWITCH		0x04	// 电池开关

#define CMD_ACTUATOR_ALL			0x05	// 控制所有电杆
#define CMD_ACTUATOR_LEFT_FRONT		0x06	// 控制左前电杆
#define CMD_ACTUATOR_RIGHT_FRONT	0x07	// 控制右前电杆
#define CMD_ACTUATOR_LEFT_BACK		0x08	// 控制左后电杆
#define CMD_ACTUATOR_RIGHT_BACK		0x09	// 控制右后电杆

typedef struct
{
    uint8_t mode;       // 模式
	float_angle angle;
	float throttle;   // 油门(高度)
} CtrlData;

typedef struct _cmd_data
{
	uint8_t cmd;	// 命令
	uint8_t param_num;	// 命令参数数量
	uint8_t params[5];	// 命令参数
} CmdData;

extern CtrlData RC_Control;

void RemoteData_Init(void);
uint8_t RemoteData_GetData(CtrlData* rc_out);
void SendToRemote(void *param);

#endif // __REMOTEDATA_H__
