/*
 * commander_takeoff_land.c
 * 起飞/降落逻辑：起飞速度斜坡 + 自动降落状态机。
 * holdHeight 由 commander.c 维护，通过指针传入。
 */
#include "commander.h"
#include <math.h>   /* fabsf, fmaxf, fminf */

/* ── 降落状态机 ── */
/********************************************************
* flyerAutoLand()
* 四轴自动降落
*********************************************************/
/* 停机缓降参数：触底确认后油门从起始值慢慢减到 0 */
#define LAND_SHUTDOWN_START_THRUST  35.0f  /* 停机缓降起始油门(%) */
#define LAND_SHUTDOWN_STEP          0.05f  /* 停机缓降每拍递减(%)，@1kHz 约0.7s到0 */

void flyerAutoLand(setpoint_t *setpoint,const state_t *state)
{
	static bool landInit = true;
	static float landMinHeight = 0;
	static uint16_t landStallCnt = 0;
	static uint16_t landConfirmCnt = 0;     /* 高度+速度条件连续满足帧数 (防传感器过冲误判) */
	static bool landShutdown = false;       /* 停机缓降中 */
	static float landShutdownThrust = 0;    /* 停机缓降油门 */

	if (landInit)
	{
		landInit = false;
		landMinHeight = state->height;
		landStallCnt = 0;
		landConfirmCnt = 0;
		landShutdown = false;
		landShutdownThrust = 0;
	}

	/* 停机缓降: 触底确认后油门慢慢减到 0, 到 0 后完全停飞 */
	if (landShutdown)
	{
		landShutdownThrust -= LAND_SHUTDOWN_STEP;
		if (landShutdownThrust <= 0)
		{
			landInit = true;
			landShutdown = false;
			landShutdownThrust = 0;
			setCommanderKeyland(false);
			setCommanderKeyFlight(false);
			return;
		}
		setpoint->mode_z = modeDisable;
		setpoint->thrust = landShutdownThrust;
		setpoint->vel.z = 0;
		return;
	}

	/* 触底即时判定: 向上加速度尖峰(撞击) -> 进入停机缓降(防反弹) */
	if (state->acc.z > IMPACT_ACC_THRESHOLD && state->acc.z < IMPACT_ACC_MAX)
	{
		landShutdown = true;
		landShutdownThrust = LAND_SHUTDOWN_START_THRUST;
		setpoint->mode_z = modeDisable;
		setpoint->thrust = landShutdownThrust;
		setpoint->vel.z = 0;
		return;
	}

	float landSpeed = (state->height <= LAND_SLOW_HEIGHT) ? LAND_MIN_SPEED : LAND_SPEED;
	setpoint->vel.z = -landSpeed;
	setpoint->height = state->height;
	setpoint->mode_z = modeVelocity;

	/* 高度低于12cm 且 垂直速度接近零 -> 累计确认后着陆(进入停机缓降) */
	if (state->height <= 12.0f && fabsf(state->velocity.z) < 10.0f)
	{
		if (++landConfirmCnt >= 20)
		{
			landShutdown = true;
			landShutdownThrust = LAND_SHUTDOWN_START_THRUST;
			landConfirmCnt = 0;
			setpoint->mode_z = modeDisable;
			setpoint->thrust = landShutdownThrust;
			setpoint->vel.z = 0;
			return;
		}
	}
	else
	{
		landConfirmCnt = 0;

		/* 高度停滞检测：低空高度变化 < 2cm -> 已触地 */
		if (state->height < landMinHeight - 2.0f)
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
				landShutdown = true;
				landShutdownThrust = LAND_SHUTDOWN_START_THRUST;
				landStallCnt = 0;
				setpoint->mode_z = modeDisable;
				setpoint->thrust = landShutdownThrust;
				setpoint->vel.z = 0;
				return;
			}
		}
		else
		{
			/* 高度回升（弹跳），重置最低点 */
			landMinHeight = state->height;
			landStallCnt = 0;
		}
	}
}

/* ── 起飞斜坡 ── */
//-------------------------起飞状态变量-----------------------------------
static bool initHigh = false;       /* 初始高度是否已锁定 (首次进入起飞斜坡时采样) */
static bool takeoffActive = false;  /* 起飞过程是否正在进行中 */
//-------------------------起飞状态变量-----------------------------------

void takeoffReset(void)
{
	initHigh = false;
	takeoffActive = false;
}

bool isTakeoffActive(void)
{
	return takeoffActive;
}

/**
 * @brief 起飞速度斜坡
 * @param holdHeight 由 commander 维护的目标高度，通过指针读写
 * @return true = 仍在爬升, false = 起飞完成
 */
bool takeoffRamp(setpoint_t *setpoint, const state_t *state, float *holdHeight)
{
	if (initHigh == false)
	{
		/* 首次进入：初始化起飞状态 */
		initHigh = true;
		takeoffActive = true;

		*holdHeight = state->height;
		setpoint->height = *holdHeight;

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
		return true;
	}

	if (takeoffActive)
	{
		/* 速度模式爬升至目标高度 */
		if (state->height >= TAKEOFF_HEIGHT)
		{
			takeoffActive = false;
			*holdHeight = TAKEOFF_HEIGHT;
			setpoint->height = *holdHeight;
			setpoint->vel.z = 0;
			setpoint->mode_z = modeAbs;
			return false;
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
			*holdHeight = state->height;
			setpoint->height = *holdHeight;
			setpoint->mode_z = modeVelocity;
			return true;
		}
	}

	/* 起飞已完成 */
	return false;
}
