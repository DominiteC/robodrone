/*
 * commander_arbitration.c
 * 遥控超时仲裁：根据连续收包间隔，分级处理"正常解锁 / 悬停锁杆 / 超时自动降落"。
 * 本文件不依赖控制算法和传感器，仅判断定时并使用 commander setter。
 */
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

//-------------------------遥控锁定状态-----------------------------------
static bool isRCLocked = true;  /* 遥控是否处于锁定状态 (失联或安全触发时置 true) */
//-------------------------遥控锁定状态-----------------------------------

static void commanderDropToGround(void)
{
	if(getCommanderKeyFlight())	/* 飞行过程中遥控器信号断开，一键降落 */
	{
		setCommanderKeyland(true);
		setCommanderKeyFlight(false);
	}	
}

/********************************************************
 *ctrlDataUpdate()	更新控制数据
 *返回0表示正常有数据，返回1表示失联，但是使用旧数据，返回2表示使用构造的假数据，返回3表示强制锁定
*********************************************************/
uint8_t ctrlDataUpdate(uint8_t reciveFlag)
{
	static uint32_t last_timestamp = 0;
	uint32_t timestamp = xTaskGetTickCount();	

	uint32_t tickNow = timestamp - last_timestamp;
	if (reciveFlag)
	{
		last_timestamp = timestamp;
	}
	if (tickNow < COMMANDER_WDT_TIMEOUT_STABILIZE) 
	{
		isRCLocked = false;			/*解锁*/
	}
	else if (tickNow < COMMANDER_WDT_TIMEOUT_HOVER)
	{
		isRCLocked = true;			/*断连悬停：锁定但不降落*/
		return 2;					/*返回2触发构造安全数据(零杆+中性油门)代替旧油门*/
	}
    else
	{
		isRCLocked = true;			/*超时锁定*/
		commanderDropToGround();
		return 3;
	}
	return 0;
}

uint8_t getIsLock(void)
{
	return isRCLocked;
}
