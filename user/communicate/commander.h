#ifndef __COMMANDER_H__
#define __COMMANDER_H__

#include <stdbool.h>
#include <stdint.h>
#include "drone_types.h"
#include "remotedata.h"

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

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
extern setpoint_t target;

uint8_t ctrlDataUpdate(uint8_t reciveFlag);

uint8_t getIsLock(void);

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

void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);

void setCommanderFlightmode(bool set);
void setCommanderEmerStop(bool set);
void setCommanderSafetyLatched(bool set);
bool getCommanderSafetyLatched(void);

void commanderMutexInit(void);

#endif
