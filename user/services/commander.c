/*
 * commander.c
 * 负责把遥控输入、飞行模式和安全状态转换为控制目标 setpoint。
 * 本文件属于业务服务层，不直接实现控制算法和硬件驱动。
 */
#include "commander.h"
#include "C_code_Log.h"
#include "control.h"
#include "PIDcontroller.h"
#include "position.h"
#include "AT24Cxx.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#define TAKEOFF_HEIGHT		100.f
#define TAKEOFF_MIN_SPEED	5.f
#define TAKEOFF_SLOW_ZONE	30.f
#define LAND_MIN_SPEED		4.f
#define LAND_SLOW_HEIGHT	35.f
#define TAKEOFF_SPEED		10.f	// 起飞爬升速度 cm/s
#define LAND_SPEED			6.f	// 降落下降速度 cm/s
#define COMMANDER_DT		0.001f

/* 速度优先定高参数 */
#define PILOT_SPEED_UP		60.f	// 最大爬升速度 cm/s
#define PILOT_SPEED_DN		40.f	// 最大下降速度 cm/s
#define PILOT_ACC_Z			60.f	// 垂直加速度限幅 cm/s²
#define THR_DZ				0.08f	// 摇杆中位死区 (0~1)
#define Z_BRAKE_VEL_DZ		5.0f
#define Z_BRAKE_TIMEOUT_MS	400U
#define XY_HOLD_ENABLE_HEIGHT	25.0f
#define XY_HOLD_DISABLE_HEIGHT	20.0f
#define XY_STICK_VEL_SCALE	6.0f
#define XY_BRAKE_VEL_DZ		5.0f
#define XY_BRAKE_ATT_DZ		3.0f
#define XY_BRAKE_SETTLE_CYCLES	250U
#define XY_BRAKE_TIMEOUT_MS	1200U
#define EEPROM_MODE_MAGIC      0xA5U
#define EEPROM_ADDR_MAGIC      0x00U
#define EEPROM_ADDR_ATTI_MODE  0x01U
#define EEPROM_ADDR_SERVO_MODE 0x02U

static bool isRCLocked = true;				/* 遥控锁定状态 */
static bool safetyLatched = false;          /* 姿态保护锁存，需人工清除 */
static bool attitudeModeChanged = false;    /* 姿态模式是否发生手动切换 */

static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

static commanderBits_t commander;

static void CommanderPersist_EnsureMagic(void)
{
	uint8_t magic = 0;
	if (AT24Cxx_Read_nByteBuf(EEPROM_ADDR_MAGIC, &magic, 1) != 0 || magic != EEPROM_MODE_MAGIC)
	{
		magic = EEPROM_MODE_MAGIC;
		(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_MAGIC, &magic, 1);
	}
}

bool CommanderPersist_LoadModes(uint8_t *attitudeMode, uint8_t *servoMode)
{
	uint8_t magic = 0;
	uint8_t at = 0;
	uint8_t sv = 0;

	if (AT24Cxx_Read_nByteBuf(EEPROM_ADDR_MAGIC, &magic, 1) != 0
		|| AT24Cxx_Read_nByteBuf(EEPROM_ADDR_ATTI_MODE, &at, 1) != 0
		|| AT24Cxx_Read_nByteBuf(EEPROM_ADDR_SERVO_MODE, &sv, 1) != 0)
	{
		return false;
	}

	if (magic != EEPROM_MODE_MAGIC || at > MODE_WALK_45 || sv > 2U)
	{
		return false;
	}

	*attitudeMode = at;
	*servoMode = sv;
	return true;
}

void CommanderPersist_SaveModes(uint8_t attitudeMode, uint8_t servoMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_ATTI_MODE, &attitudeMode, 1);
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_SERVO_MODE, &servoMode, 1);
}

void CommanderPersist_SaveAttitudeMode(uint8_t attitudeMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_ATTI_MODE, &attitudeMode, 1);
}

void CommanderPersist_SaveServoMode(uint8_t servoMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_SERVO_MODE, &servoMode, 1);
}

static void commanderDropToGround(void)
{
	if(commander.keyFlight)	/* 飞行过程中遥控器信号断开，一键降落 */
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}
uint32_t timestamp = 0;
/********************************************************
 *ctrlDataUpdate()	更新控制数据
 *返回0表示正常有数据，返回1表示失联，但是使用旧数据，返回2表示使用构造的假数据，返回3表示强制锁定
*********************************************************/
uint8_t ctrlDataUpdate(uint8_t reciveFlag)
{
	static uint32_t last_timestamp = 0;
	uint32_t timestamp = xTaskGetTickCount();	

	uint32_t tickNow = timestamp - last_timestamp;
	// LOG_DEBUG("commander-tickNow:%d",tickNow);
	if (reciveFlag)
	{
		last_timestamp = timestamp;
	}
	if (tickNow < COMMANDER_WDT_TIMEOUT_STABILIZE) 
	{
		isRCLocked = false;			/*解锁*/
	}
    else
	{
		isRCLocked = true;			/*锁定*/
		commanderDropToGround();
		return 3;
	}
	return 0;
}

/********************************************************
* flyerAutoLand()
* 四轴自动降落
*********************************************************/
void flyerAutoLand(setpoint_t *setpoint,const state_t *state)
{
	static bool landInit = true;
	static float landMinHeight = 0;
	static uint16_t landStallCnt = 0;

	if (landInit)
	{
		landInit = false;
		landMinHeight = state->height;
		landStallCnt = 0;
	}

	float landSpeed = (state->height <= LAND_SLOW_HEIGHT) ? LAND_MIN_SPEED : LAND_SPEED;
	setpoint->vel.z = -landSpeed;
	setpoint->height = state->height;
	setpoint->mode_z = modeVelocity;

	/* 高度低于14cm(架子~12cm) -> 确认着陆 */
	if (state->height <= 14.0f)
	{
		landInit = true;
		landStallCnt = 0;
		commander.keyLand = false;
		commander.keyFlight = false;
	}
	/* 高度停滞检测：5秒内高度变化 < 2cm -> 已触地 */
	else if (state->height < landMinHeight - 2.0f)
	{
		/* 仍在下降，更新最低高度 */
		landMinHeight = state->height;
		landStallCnt = 0;
	}
	else if (state->height <= landMinHeight + 2.0f && state->height < 25.0f)
	{
		/* 低空且高度接近最低点，累计停滞时间 */
		if (++landStallCnt > 2000)
		{
			landInit = true;
			landStallCnt = 0;
			commander.keyLand = false;
			commander.keyFlight = false;
		}
	}
	else
	{
		/* 高度回升（弹跳），重置最低点 */
		landMinHeight = state->height;
		landStallCnt = 0;
	}
}

static bool initHigh = false;
static float holdHeight = TAKEOFF_HEIGHT;
// static bool hasUserAdjustedAltitude; // 不再使用
static bool isAdjustingPosXY = true;/*调整XY位置*/
static uint8_t adjustPosXYTime = 0;		/*XY位置调整时间*/
static float errorPosX = 0.f;		/*X位移误差*/
static float errorPosY = 0.f;		/*Y位移误差*/
static bool isAdjustingPosZ = false;	/*调整Z位置*/
static float holdPosX = 0.f;
static float holdPosY = 0.f;
static bool xyHoldActive = false;
static bool isBrakingPosXY = false;
static uint16_t brakePosXYTime = 0;
static bool isBrakingPosZ = false;
static uint16_t brakePosZTime = 0;
static float errorPosZ = 0.f;
static bool takeoffActive = false;	/* 起飞爬升阶段 */

/**
 * @brief 获取要到达的状态
 * 
 * @param setpoint 设定位置
 * @param state 状态
 */
void commanderGetSetpoint(setpoint_t *setpoint, state_t *state)
{
	static CtrlData last_data;
	CtrlData data;
	uint8_t res = RemoteData_GetData(&data);
	if (res == 0)
	{
		data = last_data;
	}
	else if(res == 2)//使用构造的假数据
	{
		data.angle.roll = 0.f;
		data.angle.pitch = 0.f;
		data.angle.yaw = 0.f;
		data.throttle = last_data.throttle;
	}
	else
	{
		last_data = data;

	}
	if (safetyLatched)
	{
		commander.keyFlight = false;
		commander.keyLand = false;
	}
	state->isRCLocked = (isRCLocked || safetyLatched);
	if (commander.attitudeMode == MODE_WALK || commander.attitudeMode == MODE_WALK_45)
	{
		setpoint->thrust = data.throttle;
		setpoint->vel.x = data.angle.pitch;
		setpoint->vel.y = data.angle.roll;
		setpoint->vel.z = 0;
		setpoint->height = data.throttle;
		setpoint->mode_z = modeDisable;
	}
	else
	{
		if (commander.ctrlMode == MODE_HEIGHT || commander.ctrlMode == MODE_THREEHOLD)
		{
			if(commander.keyLand)/* 一键降落 */
			{
				flyerAutoLand(setpoint, state);
			}
			else if(commander.keyFlight)/* 一键起飞 */
			{
				setpoint->thrust = 0;

				if (initHigh == false)
				{
					initHigh = true;
					takeoffActive = true;
					isAdjustingPosXY = false;
					errorPosX = 0.f;
					errorPosY = 0.f;
					holdPosX = state->position.x;
					holdPosY = state->position.y;
					xyHoldActive = false;
					isBrakingPosXY = false;
					brakePosXYTime = 0;
					isAdjustingPosZ = false;
					isBrakingPosZ = false;
					brakePosZTime = 0;
					errorPosZ = 0.f;

					holdHeight = state->height;
					setpoint->height = holdHeight;
					float takeoffRemain = TAKEOFF_HEIGHT - state->height;
					float takeoffSpeed = TAKEOFF_SPEED;
					if (takeoffRemain < TAKEOFF_SLOW_ZONE)
					{
						takeoffSpeed = TAKEOFF_MIN_SPEED +
							(TAKEOFF_SPEED - TAKEOFF_MIN_SPEED) * (takeoffRemain / TAKEOFF_SLOW_ZONE);
						takeoffSpeed = fmaxf(TAKEOFF_MIN_SPEED, fminf(TAKEOFF_SPEED, takeoffSpeed));
					}
					setpoint->vel.z = takeoffSpeed;
					setpoint->mode_z = modeVelocity;
				}

				if (takeoffActive)
				{
					/* 速度模式爬升至目标高度 */
					if (state->height >= TAKEOFF_HEIGHT)
					{
						takeoffActive = false;
						isAdjustingPosZ = false;
						isBrakingPosZ = false;
						brakePosZTime = 0;
						holdHeight = TAKEOFF_HEIGHT;
						setpoint->height = holdHeight;
						setpoint->vel.z = 0;
						setpoint->mode_z = modeAbs;
					}
					else
					{
						/* 爬升中：速度模式, 跟踪当前高度 */
						float takeoffRemain = TAKEOFF_HEIGHT - state->height;
						float takeoffSpeed = TAKEOFF_SPEED;
						if (takeoffRemain < TAKEOFF_SLOW_ZONE)
						{
							takeoffSpeed = TAKEOFF_MIN_SPEED +
								(TAKEOFF_SPEED - TAKEOFF_MIN_SPEED) * (takeoffRemain / TAKEOFF_SLOW_ZONE);
							takeoffSpeed = fmaxf(TAKEOFF_MIN_SPEED, fminf(TAKEOFF_SPEED, takeoffSpeed));
						}
						setpoint->vel.z = takeoffSpeed;
						holdHeight = state->height;
						setpoint->height = holdHeight;
						setpoint->mode_z = modeVelocity;
					}
				}
				else
				{
					/* 起飞完成后的正常飞行: 摇杆控制高度 */
					float climbRaw = (data.throttle - 50.f) / 50.f;
					if (fabsf(climbRaw) < THR_DZ) climbRaw = 0;

					float climb;
					if (climbRaw > 0.f)
						climb = climbRaw * PILOT_SPEED_UP;
					else
						climb = climbRaw * PILOT_SPEED_DN;

					if (fabsf(climb) > 5.f)
					{
						isAdjustingPosZ = true;
						isBrakingPosZ = false;
						brakePosZTime = 0;
						setpoint->mode_z = modeVelocity;
						setpoint->vel.z = climb;
						setpoint->height = holdHeight;

						if (climbRaw < -0.2f)
						{
							if (maxAccZ < state->acc.z)
								maxAccZ = state->acc.z;
							if (maxAccZ > 250.f)
							{
								commander.keyFlight = false;
							}
						}
						else
						{
							maxAccZ = 0.f;
						}
					}
					else if (isAdjustingPosZ == true)
					{
						if (isBrakingPosZ == false)
						{
							isBrakingPosZ = true;
							brakePosZTime = 0;
						}

						setpoint->mode_z = modeVelocity;
						setpoint->vel.z = 0;

						if (fabsf(state->velocity.z) < Z_BRAKE_VEL_DZ || brakePosZTime++ > Z_BRAKE_TIMEOUT_MS)
						{
							isAdjustingPosZ = false;
							isBrakingPosZ = false;
							brakePosZTime = 0;
							holdHeight = state->height + errorPosZ;
							setpoint->mode_z = modeAbs;
						}
						setpoint->height = holdHeight;
					}
					else
					{
						setpoint->mode_z = modeAbs;
						setpoint->vel.z = 0;
						errorPosZ = holdHeight - state->height;
						errorPosZ = fmaxf(-10.0f, fminf(10.0f, errorPosZ));
						setpoint->height = holdHeight;
					}
				}
			}
			else/* 着陆状态 */
			{
				setpoint->thrust = 0;
				setpoint->vel.z = 0;
				setpoint->mode_z = modeDisable;
				initHigh = false;
				takeoffActive = false;
				isAdjustingPosZ = false;
				isBrakingPosZ = false;
				brakePosZTime = 0;
				errorPosZ = 0.f;
				xyHoldActive = false;
				isAdjustingPosXY = true;
				isBrakingPosXY = false;
				brakePosXYTime = 0;
				holdPosX = state->position.x;
				holdPosY = state->position.y;
				holdHeight = TAKEOFF_HEIGHT;
			}
		}
		else if (commander.ctrlMode == MODE_MANUAL)
		{
			if (commander.keyFlight)
			{
				setpoint->thrust = data.throttle;
				setpoint->vel.x = data.angle.pitch;
				setpoint->vel.y = data.angle.roll;
			}
			else
			{
				setpoint->thrust = 0;
				setpoint->vel.x = 0;
				setpoint->vel.y = 0;
			}
			setpoint->vel.z = 0;
			setpoint->height = data.throttle;
			setpoint->mode_z = modeDisable;
		}
	}
	setpoint->angle.pitch = data.angle.pitch;
	setpoint->angle.roll = data.angle.roll;
	setpoint->angle.yaw = data.angle.yaw;

	// LOG_DEBUG("pitch=%.2f,roll=%.2f,yaw=%.2f",setpoint->angle.pitch,setpoint->angle.roll,setpoint->angle.yaw);

		if(commander.ctrlMode == MODE_THREEHOLD && commander.attitudeMode == MODE_AIRPLANE)	/* 光流数据可用，定点模式 */
		{
			setpoint->angle.yaw *= 1.0f;	/* 定点模式使用完整yaw杆量 */

			/* 临时关闭XY位置环：摇杆只给目标速度，回中目标速度为0 */
			bool stickActive = (fabsf(setpoint->angle.roll) > 1.5f || fabsf(setpoint->angle.pitch) > 1.5f);
			xyHoldActive = false;
			isAdjustingPosXY = false;
			isBrakingPosXY = false;
			brakePosXYTime = 0;
			adjustPosXYTime = 0;
			setpoint->mode_x = modeVelocity;
			setpoint->mode_y = modeVelocity;
			if (stickActive)
			{
				setpoint->vel.x = -setpoint->angle.pitch * XY_STICK_VEL_SCALE;
				setpoint->vel.y = -setpoint->angle.roll * XY_STICK_VEL_SCALE;
			}
			else
			{
				setpoint->vel.x = 0.f;
				setpoint->vel.y = 0.f;
			}
		}
		else	/* 非定点模式，关闭XY位置环 */
		{
			xyHoldActive = false;
			isAdjustingPosXY = true;
			isBrakingPosXY = false;
			brakePosXYTime = 0;
			holdPosX = state->position.x;
			holdPosY = state->position.y;
			setpoint->mode_x = modeDisable;
			setpoint->mode_y = modeDisable;
		}


}

uint8_t getIsLock(void)
{
	return isRCLocked;
}

/**
 * @brief Set the Commander Ctrl Mode object
 * 
 * @param set 需要设置的控制模式
 */
void setCommanderCtrlMode(CtrlMode set)
{
	if (commander.ctrlMode == set)
		return;

	commander.ctrlMode = (set & 0x03);
	LOG_INFO("ctrl mode set to %d", set);
}
CtrlMode getCommanderCtrlMode(void)
{
	return (commander.ctrlMode & 0x03);
}

/**
 * @brief Set the Commander Attitude Mode object
 * 
 * @param set 需要设置的姿态模式
 */
void setCommanderAttitudeMode(AttitudeMode set)
{
	if (commander.attitudeMode == set)
		return;

	if (getCommanderKeyFlight())
	{
		LOG_WARN("reject attitude mode switch while flying");
		return;
	}

	commander.attitudeMode = set;
	attitudeModeChanged = true;
	CommanderPersist_SaveAttitudeMode((uint8_t)set);
	LOG_INFO("attitude mode set to %d", set);
}
void initCommanderAttitudeMode(AttitudeMode set)
{
	commander.attitudeMode = set;
	attitudeModeChanged = false;
}

AttitudeMode getCommanderAttitudeMode(void)
{
	return commander.attitudeMode;
}

bool consumeCommanderAttitudeModeChanged(void)
{
	bool changed = attitudeModeChanged;
	attitudeModeChanged = false;
	return changed;
}

/**
 * @brief 设置一键起飞模式
 * 
 * @param set 
 */
void setCommanderKeyFlight(bool set)
{
	if (set && safetyLatched)
	{
		LOG_WARN("safety latched: reject keyFlight on");
		return;
	}

	if (commander.keyFlight != set)
	{
		commander.keyFlight = set;
		if(set == true)	/* 一键起飞，清零PID积分和最大最小值 */
		{
			ResetFlightControlPIDs();
				position_ResetXY();

			minAccZ = 0.f;
			maxAccZ = 0.f;
			initHigh = false;
			xyHoldActive = false;
		}
		else	/*一键停机，清零PID积分*/
		{
			ResetFlightControlPIDs();
			xyHoldActive = false;
		}
	}
}
bool getCommanderKeyFlight(void)
{
	return commander.keyFlight;
}

/**
 * @brief 设置一键着陆模式
 * 
 * @param set 
 */
void setCommanderKeyland(bool set)
{
	commander.keyLand = set;
}
bool getCommanderKeyland(void)
{
	return commander.keyLand;
}

void setCommanderFlightmode(bool set)
{
	commander.flightMode = set;
}

void setCommanderEmerStop(bool set)
{
	commander.emerStop = set;
}

void setCommanderSafetyLatched(bool set)
{
	safetyLatched = set;
	if (safetyLatched)
	{
		isRCLocked = true;
		commander.keyFlight = false;
		commander.keyLand = false;
	}
}

bool getCommanderSafetyLatched(void)
{
	return safetyLatched;
}






