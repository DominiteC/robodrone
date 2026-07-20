#include "control.h"
#include "control_state.h"
#include "control_output.h"
#include "control_safety.h"
#include "gyro.h"            /* 陀螺零偏自校准标志: gyro_isGyroZCalibrated() */
#include "stdlib.h"
#include <math.h>
#include "remotedata.h"
#include "commander.h"
#include "position.h"
#include "servo.h"
#include "watchdog_guard.h"
#include "log.h"
#include "change.h"
#include "Mydelay.h"
#include "watchdog_guard.h"

#include "FreeRTOS.h"
#include "task.h"

#define limit(x, min, max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

#define ALTHOLD_THRUST_BASE 49.0f //悬停使用的基准油门
#define YAW_MAX_RATE	42.0f				// 满杆yaw角速率 deg/s
#define YAW_DEADBAND	5.0f				// yaw杆死区

static float thrustLpf = 35;	/*油门低通*/
static float thrustCmd = 0;    /* 实际用于混控前的推力命令 */

static float Desired_yaw = 0.0f;          /* 期望 yaw 角度, 连续 deg, 不做 ±180° 包裹 (MiniFly 方式) */
/* 飞控自积分的连续 yaw 角度 (度), 由 state->gyro.z * ANGEL_PID_DT 累加得到.
   完全不依赖 JY901P 9 轴融合的 state.angle.yaw. */
float yaw_meas_cont = 0.0f;
/* 野点防护: 单拍积分增量 > YAW_INTEG_MAX 度/拍 则丢掉当拍 */
#define YAW_INTEG_MAX_PER_STEP   5.0f
/* yaw 杆 LPF (与 MiniFly ctrlValLpf.yaw 相同思路): 松手后平滑衰减, 避免 Desired_yaw 突停 */
static float lpf_stick_yaw = 0.0f;
static float target_angle_pitch = 0.0f;
static float target_angle_roll = 0.0f;
float debug_xy_velocity_pid_count = 0.0f;
float debug_target_angle_pitch = 0.0f;
float debug_target_angle_roll = 0.0f;
float debug_target_angle_yaw = 0.0f;
float debug_desired_yaw = 0.0f;             /* unfolded target yaw (continuous) */
float debug_yaw_meas_cont = 0.0f;          /* yaw_meas_cont snapshot for telemetry */

static MotorCtrl control;
MotorCtrl debugEsc;
setpoint_t target;
state_t state;

static void ResetYawState(void);
static void Yaw_Unwrap_Debug(state_t* state);

void Roll_Pitch_Control(setpoint_t* target, state_t* state, uint32_t tick);
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick);
float Height_Control(setpoint_t* target, state_t* state,uint32_t tick);

void Flight_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);
void Walk_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);


static float wrapYawDisplay(float yaw);

static void ResetYawState(void)
{
    /* 只清 Desired_yaw 和角度环 PID 积分 (与 MiniFly attitudeControllerResetRollAttitudePID 等价).
       yaw_meas_cont 不清: 它是上电后 0 偏扣除+积分的连续角, 跨飞行次数累计,
       清零会导致第二次起飞瞬间飞机从大角度突然变 0 → 大自旋. */
    Desired_yaw = yaw_meas_cont;
    debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);
    PID_ClearIntegral(&pid_yaw_angle);
    PID_ClearIntegral(&pid_yaw_rate);
}

/* Ground yaw unfold debug - independent from flight control, tracks yaw continuously */
/* debug 用: 仍跟踪 JY901P 内部 state.angle.yaw 的展开值, 纯显示用, 不进控制 */
static float debug_yawOld = 0.0f;
static int32_t debug_yawTurnNum = 0;
static bool debug_yawUnwrapInited = false;

static void Yaw_Unwrap_Debug(state_t* state)
{
    if (!debug_yawUnwrapInited)
    {
        debug_yawOld = state->angle.yaw;
        debug_yawUnwrapInited = true;
    }
    float diff = state->angle.yaw - debug_yawOld;
    if (diff > 180.0f)
        debug_yawTurnNum--;
    else if (diff < -180.0f)
        debug_yawTurnNum++;
    debug_yawOld = state->angle.yaw;

    debug_desired_yaw = state->angle.yaw + debug_yawTurnNum * 360.0f;
}

void ResetFlightControlPIDs(void)
{
    PID_Reset(&pid_roll_angle);
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_yaw_angle);
    PID_Reset(&pid_roll_rate);
    PID_Reset(&pid_pitch_rate);
    PID_Reset(&pid_yaw_rate);
    PID_Reset(&pid_x_position);
    PID_Reset(&pid_y_position);
    PID_Reset(&pid_x_velocity);
    PID_Reset(&pid_y_velocity);
    PID_Reset(&pid_height_position);
    PID_Reset(&pid_z_velocity);

    target_angle_pitch = 0.0f;
    target_angle_roll = 0.0f;
    debug_target_angle_pitch = 0.0f;
    debug_target_angle_roll = 0.0f;
    ResetYawState();
}

void Control_Task(void *param)
{
    /* 从EEPROM恢复掉电前的姿态/舵机模式 */
    uint8_t savedAtti = MODE_AIRPLANE;
    uint8_t savedServo = SERVO_AIRPLANE;
    WatchdogGuard_EnterLongAction(3000);
    if (CommanderPersist_LoadModes(&savedAtti, &savedServo))
    {
        initCommanderAttitudeMode((AttitudeMode)savedAtti);
        initServoMode((ServoMode)savedServo);
        LOG_INFO("从EEPROM恢复姿态:%d 舵机:%d", savedAtti, savedServo);
    }
    else
    {
        initCommanderAttitudeMode(MODE_AIRPLANE);
        initServoMode(SERVO_AIRPLANE);
        CommanderPersist_SaveModes((uint8_t)MODE_AIRPLANE, (uint8_t)SERVO_AIRPLANE);
        LOG_WARN("EEPROM模式无效,回退默认 AIR/AIR 并重写");
    }
    WatchdogGuard_ExitLongAction();

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        /* 消费 commander 边沿：PID/position 复位（原在 setter 内的副作用） */
        if (consumeKeyFlightRising())
        {
            ResetFlightControlPIDs();
            position_ResetXY();
        }
        if (consumeKeyLandRising())
        {
            ResetFlightControlPIDs();
        }

        // 读取陀螺仪和光流数据
        WatchdogGuard_ControlHeartbeat();
        refreshState(&state);
        commanderGetSetpoint(&target,&state);
        // 根据模式进行控制
        if (getCommanderAttitudeMode() == MODE_AIRPLANE && getServoMode() == SERVO_AIRPLANE)
        {
            Flight_Update(&control,&target,&state);
        }
        else if ((getCommanderAttitudeMode() == MODE_WALK && getServoMode() == SERVO_WALK) ||
                 (getCommanderAttitudeMode() == MODE_WALK_45 && getServoMode() == SERVO_WALK_45))
        {
            Walk_Update(&control,&target,&state);
        }
				else 
				{
					if (getCommanderKeyFlight())
					{
						Flight_Update(&control,&target,&state);
					}
					else
					{
						changeAttitude(&control,&state);
					}
				}
        safeCheck(&control,&state);
        debugEsc = control;
        MotorControl(&control);
        vTaskDelayUntil(&lastWakeTime, 1);		/*1ms周期延时*/
    }
}

/**
 * @brief 飞行控制
 * @note 飞机四轴电机示意图
 * 
 * @param ctrl 电计输出结构体
 * @param target 目标位置结构体
 * @param state 当前状态结构体
 * 
 * @note
 *                 机头(X+)
 *          (顺)M2    ↑    M1(逆)
 *                \   |   /
 *                 \  |  /
 *                  \ | /
 *            ————————+————————>Y+
 *                  / | \
 *                 /  |  \
 *                /   |   \
 *           (逆)M3   |    M4(顺)
 * 
 */
void Flight_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state)
{
    float throttle = 0;
    bool flightActive = getCommanderKeyFlight() || getCommanderKeyland();
    static uint32_t tick = 0;

    if (flightActive)
    {
        Roll_Pitch_Control(target,state,tick);
        Yaw_Control(target,state,tick);
			
//        LOG_INFO("height=%.1f, target=%.1f, throttle=%.2f, thrustLpf=%.2f", 
//        state->height, target->height, throttle, thrustLpf);
			
        throttle = Height_Control(target,state,tick);
        thrustCmd = throttle;
        // LOG_DEBUG("throttle:%.2f",throttle);

        tick++;
    }
    else
    {
        ResetFlightControlPIDs();
        (void)Height_Control(target,state,tick);
        Yaw_Unwrap_Debug(state);  /* track unwrapped yaw on ground */
    }


    // LOG_INFO("thr:%.2f,roll:%.2f,pit:%.2f,yaw:%.2f",throttle,pid_roll_rate.Output,pid_pitch_rate.Output,pid_yaw_rate.Output);
    // 设置电调输出
    if (throttle > 5)
    {
//			 ctrl->Esc_Percent_1 = throttle + pid_roll_rate.Output + pid_pitch_rate.Output - pid_yaw_rate.Output;// m3顺(- pid_yaw_rate.Output;)
//			 ctrl->Esc_Percent_2 = throttle - pid_roll_rate.Output + pid_pitch_rate.Output + pid_yaw_rate.Output;// 		(- pid_yaw_rate.Output;)
//			 ctrl->Esc_Percent_3 = throttle - pid_roll_rate.Output - pid_pitch_rate.Output - pid_yaw_rate.Output;// 		(+ pid_yaw_rate.Output;)
//			 ctrl->Esc_Percent_4 = throttle + pid_roll_rate.Output - pid_pitch_rate.Output + pid_yaw_rate.Output;// 		(+ pid_yaw_rate.Output;)

			 // 添加最小油门限制，确保电机不会停转
			 ctrl->Esc_Percent_1 = limit(ctrl->Esc_Percent_1, 0, 90);
			 ctrl->Esc_Percent_2 = limit(ctrl->Esc_Percent_2, 0, 90);
			 ctrl->Esc_Percent_3 = limit(ctrl->Esc_Percent_3, 0, 90);
			 ctrl->Esc_Percent_4 = limit(ctrl->Esc_Percent_4, 0, 90);
//				ctrl->Esc_Percent_2 = 10;

        // LOG_DEBUG("esc:%.2f,%.2f,%.2f,%.2f",ctrl->Esc_Percent_1, ctrl->Esc_Percent_2, ctrl->Esc_Percent_3, ctrl->Esc_Percent_4);
    }
    else
    {
        thrustCmd = 0;
        ctrl->Esc_Percent_1 = 0.0f;
        ctrl->Esc_Percent_2 = 0.0f;
        ctrl->Esc_Percent_3 = 0.0f;
        ctrl->Esc_Percent_4 = 0.0f;
        ResetFlightControlPIDs();
    }
    ctrl->Motor_Left_Front_PWM = 0;
    ctrl->Motor_Right_Front_PWM = 0;
    ctrl->Motor_Left_Back_PWM = 0;
    ctrl->Motor_Right_Back_PWM = 0;
}

/**
 * @brief 横滚俯仰控制
 * 
 * @param target 目标状态
 * @param state 当前状态
 * 
 * @note target_v 目标速度，单位cm/s
 * @note measure_v 测量速度，单位cm/s
 * @note measure_a 测量角度，单位度
 * @note measure_g 测量角速度，单位度/s
 */
void Roll_Pitch_Control(setpoint_t* target, state_t* state, uint32_t tick)
{
    if (RATE_DO_EXECUTE(VELOCITY_PID_RATE,tick))
    {
        if (getCommanderCtrlMode() == MODE_THREEHOLD)
        {
            // 定点模式：位置环 + 速度环级联
            float vel_x_target, vel_y_target;

            // 位置环 (modeAbs时生效，将位置误差转为速度指令)
            if (target->mode_x == modeAbs)
            {
                vel_x_target = 0.1f * PIDCalculate(&pid_x_position, state->position.x, target->pos.x);
                vel_x_target = limit(vel_x_target, -30.0f, 30.0f);
            }
            else
            {
                vel_x_target = target->vel.x;
            }

            if (target->mode_y == modeAbs)
            {
                vel_y_target = 0.1f * PIDCalculate(&pid_y_position, state->position.y, target->pos.y);
                vel_y_target = limit(vel_y_target, -30.0f, 30.0f);
            }
            else
            {
                vel_y_target = target->vel.y;
            }

            // 速度环
            target_angle_pitch = -PIDCalculate(&pid_x_velocity, state->velocity.x, vel_x_target);
            target_angle_roll  = -PIDCalculate(&pid_y_velocity, state->velocity.y, vel_y_target);
            debug_xy_velocity_pid_count += 1.0f;

            debug_target_angle_pitch = target_angle_pitch;
            debug_target_angle_roll = target_angle_roll;

        }
        else
        {
            target_angle_pitch = target->angle.pitch;
            target_angle_roll = target->angle.roll;

            debug_target_angle_pitch = target_angle_pitch;
            debug_target_angle_roll = target_angle_roll;
        }
    }

    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        // 角度环
        
        PIDCalculate(&pid_pitch_angle, state->angle.pitch, target_angle_pitch);
        PIDCalculate(&pid_roll_angle, state->angle.roll, target_angle_roll);
        

        // LOG_DEBUG("state-pit:%.2f,set-pit:%.2f,output:%.2f",state->angle.pitch,target_angle_pitch,pid_pitch_angle.Output);
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // 角速度环
        
        PIDCalculate(&pid_pitch_rate, state->gyro.y, pid_pitch_angle.Output);
        PIDCalculate(&pid_roll_rate, state->gyro.x, pid_roll_angle.Output);
        

        // LOG_DEBUG("state-pit-rate:%.2f,set-pit-rate:%.2f,output:%.2f",state->gyro.y,pid_pitch_angle.Output,pid_pitch_rate.Output);
    }
}

/**
 * @brief 航向控制
 * 
 * @param target 目标状态
 * @param state 当前状态
 * 
 * @note target_a 目标偏航角，单位度
 * @note measure_a 测量角度，单位度
 * @note measure_g 测量角速度，单位度/s
 */
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick)
{
    /* 上电 1.0s 陀螺零偏校准未通过: yaw 控制器直接不工作, 不产生任何差分.
       与 MiniFly sensorsAreCalibrated() 等价的策略: 未校准不进入控制. */
    if (gyro_isGyroZCalibrated() == 0)
    {
        return;
    }

    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        float raw_stick = -target->angle.yaw;

        /* 飞控端用 state->gyro.z 自积分出连续 yaw 角度 (度).
           不再用 JY901P 9 轴融合的 state.angle.yaw, 完全摆脱磁干扰与模块偏置. */
        float yaw_step = state->gyro.z * ANGEL_PID_DT;
        if (fabsf(yaw_step) < YAW_INTEG_MAX_PER_STEP)
        {
            /* 杆回中时 |gyro.z|<0.5°/s 是电机振动导致的零偏偏移, 不积分.
               杆偏时正常积分 (MiniFly 用 Mahony accel 修正同理, 我们没这层). */
            if (fabsf(raw_stick) > YAW_DEADBAND || fabsf(state->gyro.z) > 0.5f)
            {
                yaw_meas_cont += yaw_step;
            }
        }
        debug_yaw_meas_cont = yaw_meas_cont;

        /* LPF 平滑杆量 (与 MiniFly commander.c:117 ctrlValLpf.yaw 思路一致).
           alpha=0.1 @250Hz → τ≈38ms, MiniFly α=0.2 @100Hz → τ≈45ms, 接近. */
        lpf_stick_yaw += (raw_stick - lpf_stick_yaw) * 0.1f;

        if (fabsf(lpf_stick_yaw) > YAW_DEADBAND)
        {
            Desired_yaw += (lpf_stick_yaw / 100.0f) * YAW_MAX_RATE * ANGEL_PID_DT;
        }
        else
        {
            /* 松手锁角 (MiniFly Kp=20.0 不需要, 我们 Kp=2.5 拉不回惯性过冲).
               LPF 衰减进死区后 Desired_yaw = yaw_meas_cont, 角度误差归 0. */
            Desired_yaw = yaw_meas_cont;
        }

        // LOG_DEBUG("desire-yaw:%.2f",Desired_yaw);
        // 角度环

        PIDCalculate(&pid_yaw_angle, yaw_meas_cont, Desired_yaw);
        debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);

        // if (myDelay((uint32_t)Yaw_Control,100))
        //     LOG_DEBUG("state-yaw:%.2f,set-yaw:%.2f,output:%.2f",yaw_meas_cont,Desired_yaw,pid_yaw_angle.Output);
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // 角速度环

//        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);
        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);//target->angle.yaw/10

        // LOG_DEBUG("state-yaw-rate:%.2f,set-yaw-rate:%.2f,output:%.2f",state->gyro.z,pid_yaw_angle.Output,pid_yaw_rate.Output);
    }
}


//void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick)
//{
//    static float yaw_target = 0.0f;   // 期望 yaw 角（deg，连续）
//    static float yaw_meas_cont = 0.0f;
//    static float last_yaw = 0.0f;
//    static int32_t turnNum = 0;

//    /* ---------- yaw 角度展开 ---------- */
//    float yaw_now = state->angle.yaw;

//    if (yaw_now - last_yaw > 300.0f)
//        turnNum--;
//    else if (yaw_now - last_yaw < -300.0f)
//        turnNum++;

//    last_yaw = yaw_now;
//    yaw_meas_cont = yaw_now + 360.0f * turnNum;

//    /* ---------- 角度环（ANGEL_PID_RATE） ---------- */
//    if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick))
//    {
//        /* RC yaw 输入 → 期望 yaw 角速度（deg/s） */
//        float yaw_rate_cmd = target->angle.yaw;  
//        // 建议：target->angle.yaw ∈ [-150, 150] deg/s

//        /* 积分得到期望 yaw 角 */
//        yaw_target += yaw_rate_cmd * ANGEL_PID_DT;

//        PIDCalculate(&pid_yaw_angle, yaw_meas_cont, yaw_target);
//    }

//    /* ---------- 角速度环（RATE_PID_RATE） ---------- */
//    if (RATE_DO_EXECUTE(RATE_PID_RATE, tick))
//    {
//        PIDCalculate(&pid_yaw_rate,
//                     state->gyro.z,              // 实际 yaw 角速度（deg/s）
//                     pid_yaw_angle.Output);      // 角度环输出作为期望角速度
//    }
//}


/**
 * @brief 高度控制
 * @param target_height 目标高度，单位cm
 * @param measure_height 测量高度，单位cm
 * @param measure_vz 测量垂直速度，单位cm/s
 */
float Height_Control(setpoint_t* target, state_t* state,uint32_t tick)
{
	static uint16_t altholdCount = 0;
    static bool lastKeyFlight = false;
    static bool lastKeyLand = false;
    static float thrustRaw = 0;
    static float baseThrust = 0;
    static uint16_t rampCnt = 0;

    #define TAKEOFF_START_THRUST  20.0f
    #define LAND_END_THRUST       40.0f
    #define TAKEOFF_RAMP_CYCLE    500     /* 2秒 @250Hz */
    #define LAND_RAMP_CYCLE       1250    /* 5秒 @250Hz */

    //手动模式直接映射油门，定高模式才使用PID控制高度
    if (getCommanderCtrlMode() == MODE_MANUAL)
        return limit(target->thrust, 0, 100);

    /* 起飞: 检测 keyFlight 0->1 跳变，启动斜坡 */
    if (getCommanderKeyFlight() && !lastKeyFlight)
    {
        baseThrust = TAKEOFF_START_THRUST;
        rampCnt = 0;
    }
    /* 降落: 检测 keyLand 0->1 跳变，启动斜坡 */
    else if (getCommanderKeyland() && !lastKeyLand)
    {
        rampCnt = 0;
    }

    lastKeyFlight = getCommanderKeyFlight();
    lastKeyLand = getCommanderKeyland();

    // 停机时清零，防止之前的积分导致油门数值不正常
    if (!getCommanderKeyFlight() && !getCommanderKeyland())
    {
        thrustLpf = 0;
        thrustRaw = 0;
			
        baseThrust = 0;
        rampCnt = 0;
        PID_ClearIntegral(&pid_height_position);
        PID_ClearIntegral(&pid_z_velocity);
        return 0;
    }

    // PID控制部分
    if (RATE_DO_EXECUTE(POSITION_PID_RATE,tick))
    {
        if (target->mode_z == modeVelocity || target->mode_z == modeDisable)
        {
            /* 速度模式: 位置环旁路，I项清零，防止积分风车 */
            pid_height_position.Output = 0;
            pid_height_position.Iout = 0;
            pid_height_position.ITerm = 0;
        }
        else if (target->mode_z == modeAbs)
        {
            /* 位置保持: 位置环正常级联 */
            PIDCalculate(&pid_height_position, state->height, target->height);
        }
    }

    if (RATE_DO_EXECUTE(VELOCITY_PID_RATE,tick))
    {
        /* 起飞斜坡: 基础油门从 20% 逐步爬升到 44% */
        if (getCommanderKeyFlight() && baseThrust < ALTHOLD_THRUST_BASE)
        {
            if (++rampCnt <= TAKEOFF_RAMP_CYCLE)
                baseThrust = TAKEOFF_START_THRUST +
                    (ALTHOLD_THRUST_BASE - TAKEOFF_START_THRUST) * ((float)rampCnt / TAKEOFF_RAMP_CYCLE);
            else
                baseThrust = ALTHOLD_THRUST_BASE;
        }
        /* 降落斜坡: 基础油门从 44% 逐步降到 35% */
        else if (getCommanderKeyland() && baseThrust > LAND_END_THRUST)
        {
            if (++rampCnt <= LAND_RAMP_CYCLE)
                baseThrust = ALTHOLD_THRUST_BASE -
                    (ALTHOLD_THRUST_BASE - LAND_END_THRUST) * ((float)rampCnt / LAND_RAMP_CYCLE);
            else
                baseThrust = LAND_END_THRUST;
        }

        // 位置环输出限幅，防止异常时修正量过大
        float posCorr = (target->mode_z == modeAbs) ?
            fmaxf(-30.f, fminf(30.f, pid_height_position.Output)) : 0.f;

        // 速度环: 目标 = 位置环修正 + 摇杆速度前馈
        PIDCalculate(&pid_z_velocity, state->velocity.z, posCorr + target->vel.z);

        thrustRaw = baseThrust + pid_z_velocity.Output;
        thrustLpf += (thrustRaw - thrustLpf) * 0.003f;
    }

    // 定高飞行时，如果垂直加速度很小，说明状态比较稳定，可以考虑更新基础油门值
	if(getCommanderKeyFlight() || getCommanderKeyland())
	{
		if(fabs(state->acc.z) < 0.035f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				// if(fabs(configParam.thrustBase - thrustLpf) > 1.f)	/*更新基础油门值*/
				// 	configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland() == false)	/*降落完成，油门清零*/
	{
		return 0;
	}

    return limit(thrustRaw, 20, 70);  // 限制油门上限在70%
}

float getAltholdThrust(void)
{
	return thrustLpf;
}

float getThrustCmd(void)
{
    return thrustCmd;
}

float getYawMeasCont(void)
{
    return yaw_meas_cont;
}

static float wrapYawDisplay(float yaw)
{
    yaw = fmodf(yaw + 180.0f, 360.0f);
    if (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    return yaw - 180.0f;
}

/**
 * @brief 陆行模式控制
 * 
 * @param ctrl 电计输出结构体
 * @param target 目标位置结构体
 * @param state 当前状态结构体
 */
void Walk_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state)
{
    if (state->isRCLocked == false)
    {
        ctrl->Motor_Left_Front_PWM 	= (target->thrust-50)*75 +  target->angle.roll*450;//*45;
        ctrl->Motor_Left_Back_PWM  	= (target->thrust-50)*75 +  target->angle.roll*450;
        ctrl->Motor_Right_Front_PWM = (target->thrust-50)*75 -  target->angle.roll*450;
        ctrl->Motor_Right_Back_PWM 	= (target->thrust-50)*75 -  target->angle.roll*450;
    }
    else
    {
        ctrl->Motor_Left_Front_PWM = 0;
        ctrl->Motor_Left_Back_PWM = 0;
        ctrl->Motor_Right_Front_PWM = 0;
        ctrl->Motor_Right_Back_PWM = 0;
    }

    ctrl->Esc_Percent_1 = 0;
    ctrl->Esc_Percent_2 = 0;
    ctrl->Esc_Percent_3 = 0;
    ctrl->Esc_Percent_4 = 0;
}
