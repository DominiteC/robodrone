/*
 * commander.c
 * 负责把遥控输入、飞行模式和安全状态转换为控制目标 setpoint。
 * 本文件属于业务服务层，不直接实现控制算法和硬件驱动。
 */
#include "commander.h"
#include "C_code_Log.h"
#include "alarm.h"
#include "position.h"
#include <math.h>

/* 清 XY 位置环积分（定义在 control.c，通过 extern 避免 commander→control 反向依赖） */
extern void flightClearPosPID(void);

#define COMMANDER_DT		0.001f

/* 速度优先定高参数 */
#define PILOT_SPEED_UP		30.f	// 最大爬升速度 cm/s
#define PILOT_SPEED_DN		20.f	// 最大下降速度 cm/s
#define PILOT_ACC_Z			60.f	// 垂直加速度限幅 cm/s²
#define THR_DZ				0.08f	// 摇杆中位死区 (0~1)
#define Z_BRAKE_VEL_DZ		5.0f
#define Z_BRAKE_TIMEOUT_MS	400U
#define XY_STICK_VEL_SCALE	4.0f
#define XY_BRAKE_VEL_DZ		5.0f
#define XY_BRAKE_TIMEOUT_MS	1200U
#define XY_HOLD_LOOKAHEAD	0.2f

//-------------------------安全与模式标志-----------------------------------
static bool safetyLatched = false;          /* 姿态保护锁存 (由安全模块置位, 需手动清除) */
static bool attitudeModeChanged = false;    /* 姿态模式是否发生手动切换 (供外部消费) */
//-------------------------安全与模式标志-----------------------------------

//-------------------------失控保护: 加速度跟踪-----------------------------------
static float minAccZ = 0.f; 
static float maxAccZ = 0.f;               /* 跟踪下降方向最大加速度 (cm/s²), 用于失控保护判断 */
static uint16_t maxAccZOverCnt = 0;        /* 加速度超阈值持续计数, 避免单帧抖动误触 */
static uint16_t maxAccZWinCnt = 0;         /* 跟踪窗口计数器, 每 200ms 重置一次 maxAccZ */
//-------------------------失控保护: 加速度跟踪-----------------------------------

//-------------------------指挥官核心状态-----------------------------------
static commanderBits_t commander;          /* 指挥官位域: 控制模式/姿态模式/起飞/降落/急停 */
//-------------------------指挥官核心状态-----------------------------------

//-------------------------起飞/降落边沿标志-----------------------------------
static bool keyFlightRising  = false;      /* keyFlight 0→1 上升沿 (消费后清零) */
static bool keyFlightFalling = false;      /* keyFlight 1→0 下降沿 (消费后清零) */
static bool keyLandRising    = false;      /* keyLand 0→1 上升沿 */
static bool keyLandFalling   = false;      /* keyLand 1→0 下降沿 */
//-------------------------起飞/降落边沿标志-----------------------------------

//-------------------------定高/定点控制状态-----------------------------------
static float holdHeight = TAKEOFF_HEIGHT;  /* 定高模式目标高度 (cm), 松杆时锁存 */
static bool isAdjustingPosZ = false;       /* Z 轴高度调整中 (有杆输入时置位) */
static float holdPosX = 0.f;               /* 定点模式 X 保持位置 (cm), 刹车完成后锁存 */
static float holdPosY = 0.f;               /* 定点模式 Y 保持位置 (cm), 刹车完成后锁存 */
static bool xyHoldActive = false;          /* XY 位置保持已激活 (锁点后置位) */
static bool isBrakingPosXY = false;        /* XY 刹车进行中 (松杆后速度环减速) */
static uint16_t brakePosXYTime = 0;        /* XY 刹车计时 (ms), 超时强制切位置环 */
static bool isBrakingPosZ = false;         /* Z 轴刹车进行中 (松杆后速度环减速) */
static uint16_t brakePosZTime = 0;         /* Z 轴刹车计时 (ms), 超时强制切位置环 */
static float errorPosZ = 0.f;              /* Z 轴位置误差 (cm), 松杆锁高时限幅平滑 */
//-------------------------定高/定点控制状态-----------------------------------

/* ========== 内部 dispatch 函数 ========== */

/** 正常飞行(起飞完成后)：摇杆爬升/下降 + 失控保护 + 刹车锁高 */
static void setpointNormalFlight(const CtrlData *data, setpoint_t *setpoint, const state_t *state)
{
	float climbRaw = (data->throttle - 50.f) / 50.f;
	if (fabsf(climbRaw) < THR_DZ) climbRaw = 0;

	float climb;
	if (climbRaw > 0.f)
		climb = climbRaw * PILOT_SPEED_UP;
	else
		climb = climbRaw * PILOT_SPEED_DN;

	/* 下降到支撑架着地高度，或下降中触底撞击 -> 切一键降落触底停飞(正常停飞，不报警) */
	if (state->height <= LAND_MIN_HEIGHT ||
	    (climb < 0.f && state->acc.z > IMPACT_ACC_THRESHOLD && state->acc.z < IMPACT_ACC_MAX))
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(true);
	}

	if (fabsf(climb) > 5.f)
	{
		/* 有杆输入：直接设置目标速度 */
		isAdjustingPosZ = true;
		isBrakingPosZ = false;
		brakePosZTime = 0;
		setpoint->mode_z = modeVelocity;
		setpoint->vel.z = climb;
		setpoint->height = holdHeight;

		/* 失控保护：油门拉下降 + 下降加速度超阈值持续 ~0.8s 触发 */
		if (climbRaw < -0.2f)
		{
			if (state->acc.z > maxAccZ)
				maxAccZ = state->acc.z;

			if (maxAccZ > 250.f)
			{
				if (++maxAccZOverCnt > 200)
				{
					LOG_WARN("失控保护触发: maxAccZ=%.1f cm/s^2, climbRaw=%.2f, 切一键降落",maxAccZ, climbRaw);
					Alarm_SetMode(ALARM_MODE_ERROR);
					setCommanderKeyFlight(false);
					setCommanderKeyland(true);
				}
			}
			else
			{
				maxAccZOverCnt = 0;
			}
		}
	}
	else if (isAdjustingPosZ == true)
	{
		/* 松杆回中：速度环刹车 → 切位置环锁高 */
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
		/* 位置保持 */
		setpoint->mode_z = modeAbs;
		setpoint->vel.z = 0;
		errorPosZ = holdHeight - state->height;
		errorPosZ = fmaxf(-10.0f, fminf(10.0f, errorPosZ));
		setpoint->height = holdHeight;
	}

	/* 跟踪窗口，避免 maxAccZ 长时间累积误触 */
	if (++maxAccZWinCnt > 800)
	{
		maxAccZWinCnt = 0;
		maxAccZ = 0.f;
		maxAccZOverCnt = 0;
	}
}

/** XY 位置控制：光流定点 + 摇杆速度 + 回中刹停锁位
 *  状态A 拨杆: modeVelocity, 目标速度=杆量×4
 *  状态B 刹车: modeVelocity, 目标速度=0 刹停, 速度归零或超时后带前馈锁点
 *  状态C 保持: modeAbs, 目标位置=holdPos 固定 */
static void setpointXY(const CtrlData *data, setpoint_t *setpoint, const state_t *state)
{
	(void)data;
	if(commander.ctrlMode == MODE_THREEHOLD && commander.attitudeMode == MODE_AIRPLANE)
	{
		if (!position_IsXYFlowValid())
		{
			position_ResetFlowState();
			xyHoldActive = false;
			isBrakingPosXY = false;
			brakePosXYTime = 0;
			setpoint->mode_x = modeDisable;
			setpoint->mode_y = modeDisable;
			setpoint->vel.x = 0.f;
			setpoint->vel.y = 0.f;
		}
		else
		{
			setpoint->angle.yaw *= 0.5f;
			bool stickActive = (fabsf(setpoint->angle.roll) > 1.5f || fabsf(setpoint->angle.pitch) > 1.5f);
			if (stickActive)
			{
				/* 状态A 拨杆：速度环直通，位置环旁路 */
				xyHoldActive = false;
				isBrakingPosXY = false;
				brakePosXYTime = 0;
				setpoint->mode_x = modeVelocity;
				setpoint->mode_y = modeVelocity;
				setpoint->vel.x = -setpoint->angle.pitch * XY_STICK_VEL_SCALE;
				setpoint->vel.y = -setpoint->angle.roll * XY_STICK_VEL_SCALE;
			}
			else if (!xyHoldActive)
			{
				/* 状态B 刹车：速度环目标0刹停，位置环不介入；
				 * 等速度归零(或超时)再锁存, 避免锁到滞后的位置估计被反向拉回 */
				if (isBrakingPosXY == false)
				{
					isBrakingPosXY = true;
					brakePosXYTime = 0;
				}
				setpoint->mode_x = modeVelocity;
				setpoint->mode_y = modeVelocity;
				setpoint->vel.x = 0.f;
				setpoint->vel.y = 0.f;
				if ((fabsf(state->velocity.x) < XY_BRAKE_VEL_DZ &&
				     fabsf(state->velocity.y) < XY_BRAKE_VEL_DZ) ||
				    brakePosXYTime++ > XY_BRAKE_TIMEOUT_MS)
				{
					/* 锁存点带速度前馈, 补偿测速滞后 */
					holdPosX = state->position.x + state->velocity.x * XY_HOLD_LOOKAHEAD;
					holdPosY = state->position.y + state->velocity.y * XY_HOLD_LOOKAHEAD;
					xyHoldActive = true;
					isBrakingPosXY = false;
					brakePosXYTime = 0;
					flightClearPosPID();
					setpoint->mode_x = modeAbs;
					setpoint->mode_y = modeAbs;
					setpoint->pos.x = holdPosX;
					setpoint->pos.y = holdPosY;
				}
			}
			else
			{
				/* 状态C 保持 */
				isBrakingPosXY = false;
				brakePosXYTime = 0;
				setpoint->mode_x = modeAbs;
				setpoint->mode_y = modeAbs;
				setpoint->pos.x = holdPosX;
				setpoint->pos.y = holdPosY;
				setpoint->vel.x = 0.f;
				setpoint->vel.y = 0.f;
			}
		}
	}
	else
	{
		xyHoldActive = false;
		isBrakingPosXY = false;
		brakePosXYTime = 0;
		holdPosX = state->position.x;
		holdPosY = state->position.y;
		setpoint->mode_x = modeDisable;
		setpoint->mode_y = modeDisable;
	}
}

/* ========== 主调度 ========== */

/**
 * @brief 获取要到达的状态
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
		data.throttle = 50.0f;		/*中性油门，避免断线后保留旧爬升油门*/
	}
	else
	{
		last_data = data;

	}
	if (safetyLatched)
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}
	state->isRCLocked = (getIsLock() || safetyLatched);

	/* 消费边沿：commander 内部状态复位（PID/position 复位在 Control_Task） */
	if (consumeKeyFlightRising())
	{
		minAccZ = 0.f;
		maxAccZ = 0.f;
		maxAccZOverCnt = 0;
		maxAccZWinCnt = 0;
		takeoffReset();
		xyHoldActive = false;
	}
	if (consumeKeyLandRising())
	{
		xyHoldActive = false;
	}

	if (commander.attitudeMode == MODE_WALK || commander.attitudeMode == MODE_WALK_45)
	{
		float thr = data.throttle;
		// 手动模式油门范围 0~100，映射到以 50 为中心
		if (commander.ctrlMode == MODE_MANUAL)
        	thr = thr * 0.5f + 50.0f;    // 0→50, 100→100
		
		setpoint->thrust = thr;
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
				static bool takeoffXYInitDone = true;
				setpoint->thrust = 0;

				bool stillTakingOff = takeoffRamp(setpoint, state, &holdHeight);

				if (stillTakingOff)
				{
					/* 首帧：锁定 XY 位置起点（后续爬升帧不覆盖） */
					if (takeoffXYInitDone)
					{
						takeoffXYInitDone = false;
						holdPosX = state->position.x;
						holdPosY = state->position.y;
						xyHoldActive = true;
						isBrakingPosXY = false;
						brakePosXYTime = 0;
					}
					isAdjustingPosZ = false;
					isBrakingPosZ = false;
					brakePosZTime = 0;
					errorPosZ = 0.f;
				}
				else
				{
					takeoffXYInitDone = true;
					setpointNormalFlight(&data, setpoint, state);
				}
			}
			else/* 着陆状态 */
			{
				setpoint->thrust = 0;
				setpoint->vel.z = 0;
				setpoint->mode_z = modeDisable;
				takeoffReset();
				isAdjustingPosZ = false;
				isBrakingPosZ = false;
				brakePosZTime = 0;
				errorPosZ = 0.f;
				xyHoldActive = false;
				isBrakingPosXY = false;
				brakePosXYTime = 0;
				holdPosX = state->position.x;
				holdPosY = state->position.y;
				holdHeight = TAKEOFF_HEIGHT;
			}
		}
		else if (commander.ctrlMode == MODE_MANUAL)
		{
			/* 手动模式不执行 flyerAutoLand，直接清理 keyLand 防止残留阻塞后续起飞 */
			if (commander.keyLand)
			{
				setCommanderKeyland(false);
			}

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

	/* RC 失联或自动降落时 yaw 杆量清零 (MiniFly 方式: 约 500ms 后清零 yaw) */
	if (getIsLock() || (commander.keyLand && !commander.keyFlight))
	{
		setpoint->angle.yaw = 0.0f;
	}

	// LOG_DEBUG("pitch=%.2f,roll=%.2f,yaw=%.2f",setpoint->angle.pitch,setpoint->angle.roll,setpoint->angle.yaw);

		setpointXY(&data, setpoint, state);
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
		if (set)
		{
			keyFlightRising  = true;
			keyFlightFalling = false;
		}
		else
		{
			keyFlightFalling = true;
			keyFlightRising  = false;
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
	if (commander.keyLand != set)
	{
		commander.keyLand = set;
		if (set)
		{
			keyLandRising  = true;
			keyLandFalling = false;
		}
		else
		{
			keyLandFalling = true;
			keyLandRising  = false;
		}
	}
}
bool getCommanderKeyland(void)
{
	return commander.keyLand;
}

bool consumeKeyFlightRising(void)
{
	bool v = keyFlightRising;
	keyFlightRising = false;
	return v;
}

bool consumeKeyFlightFalling(void)
{
	bool v = keyFlightFalling;
	keyFlightFalling = false;
	return v;
}

bool consumeKeyLandRising(void)
{
	bool v = keyLandRising;
	keyLandRising = false;
	return v;
}

bool consumeKeyLandFalling(void)
{
	bool v = keyLandFalling;
	keyLandFalling = false;
	return v;
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
		commander.keyFlight = false;
		commander.keyLand = false;
	}
}

bool getCommanderSafetyLatched(void)
{
	return safetyLatched;
}






