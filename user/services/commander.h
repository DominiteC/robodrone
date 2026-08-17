/*
 * commander.h
 * 声明遥控命令服务接口，向控制层提供控制目标和模式状态。
 */
#ifndef __COMMANDER_H__
#define __COMMANDER_H__

#include <stdbool.h>
#include <stdint.h>
#include "flight_mode.h"
#include "vehicle_state.h"
#include "remotedata.h"

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_HOVER      5000   /* 断连5秒内悬停，超时后自动降落 */
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000	·

/* 起飞/降落速度参数（供 commanderGetSetpoint + flyerAutoLand 共用） */
#define TAKEOFF_HEIGHT		60.f
#define TAKEOFF_MIN_SPEED	5.f
#define TAKEOFF_SLOW_ZONE	30.f
#define TAKEOFF_SPEED		15.f	// 起飞爬升速度 cm/s
#define LAND_MIN_SPEED		8.f
#define LAND_SLOW_HEIGHT	25.f
#define LAND_SPEED			20.f	// 降落下降速度 cm/s

/* 触底即时判定阈值（flyerAutoLand 使用，需试飞标定） */
#define IMPACT_ACC_THRESHOLD	3.5f	/* 触底向上加速度尖峰 m/s²（约0.35g，试飞标定） */
#define IMPACT_ACC_MAX			30.0f	/* 触底加速度上限 m/s²（约3g，排除±90串口毛刺） */
#define LAND_MIN_HEIGHT		11.0f	/* 触地高度 cm（支撑架着地约10cm，留余量） */

/* 降落模式开关: 1=正常缓降, 0=直接停机(PWM立即输出0) */
#define LAND_SLOW_ENABLE	1

typedef struct _command
{
	CtrlMode ctrlMode	        : 2;
	AttitudeMode attitudeMode   : 2;
	uint8_t keyFlight 	        : 1;
	uint8_t keyLand 	        : 1;
	uint8_t emerStop 	        : 1;
	uint8_t flightMode 	        : 1;
} commanderBits_t;

void commanderGetSetpoint(setpoint_t *setpoint, state_t *state);
//-------------------------控制目标全局变量-----------------------------------
extern setpoint_t target;                   /* 控制目标集合 (由 commanderGetSetpoint 填充) */
//-------------------------控制目标全局变量-----------------------------------

uint8_t ctrlDataUpdate(uint8_t reciveFlag);

uint8_t getIsLock(void);

void flyerAutoLand(setpoint_t *setpoint, const state_t *state);

void takeoffReset(void);
bool isTakeoffActive(void);
bool takeoffRamp(setpoint_t *setpoint, const state_t *state, float *holdHeight);

void setCommanderCtrlMode(CtrlMode set);
CtrlMode getCommanderCtrlMode(void);

void setCommanderAttitudeMode(AttitudeMode set);
void initCommanderAttitudeMode(AttitudeMode set);
AttitudeMode getCommanderAttitudeMode(void);
bool consumeCommanderAttitudeModeChanged(void);

bool CommanderPersist_LoadModes(uint8_t *attitudeMode, uint8_t *servoMode);
void CommanderPersist_SaveModes(uint8_t attitudeMode, uint8_t servoMode);
void CommanderPersist_SaveAttitudeMode(uint8_t attitudeMode);
void CommanderPersist_SaveServoMode(uint8_t servoMode);

void setCommanderKeyFlight(bool set);
bool getCommanderKeyFlight(void);

/* 边沿检测 —— 由 Control_Task / commanderGetSetpoint 消费后自动清零 */
bool consumeKeyFlightRising(void);
bool consumeKeyFlightFalling(void);

void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);

bool consumeKeyLandRising(void);
bool consumeKeyLandFalling(void);

void setCommanderFlightmode(bool set);
void setCommanderEmerStop(bool set);
void setCommanderSafetyLatched(bool set);
bool getCommanderSafetyLatched(void);

void commanderMutexInit(void);

#endif
