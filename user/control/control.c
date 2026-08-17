#include "control.h"
#include "control_state.h"
#include "control_output.h"
#include "control_safety.h"
#include "gyro.h"            /* 陀螺零偏自校准标志: gyro_isGyroZCalibrated() */
#include "stdlib.h"
#include <math.h>
#include <stdbool.h>
#include "remotedata.h"
#include "commander.h"
#include "position.h"
#include "servo.h"
#include "watchdog_guard.h"
#include "log.h"
#include "change.h"
#include "Mydelay.h"
#include "FreeRTOS.h"
#include "task.h"

#define limit(x, min, max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

#define ALTHOLD_THRUST_BASE 49.5f //悬停使用的基准油门
#define YAW_RATE_LIMIT  40.0f				// yaw角速率限制 deg/s
#define YAW_DEADBAND	5.0f				// yaw杆死区
#define YAW_TRIM       0.0f                /* CW/CCW 电机扭矩平衡偏置: 正值=补偿CCW偏置, 试飞标定 */
//-------------------------油门控制-----------------------------------
static float thrustLpf = 35;	/* 油门低通滤波值 (悬停油门基值，由高度PID持续修正) */
static float thrustCmd = 0;    /* 实际用于混控前的推力命令 (0~100%) */
//-------------------------油门控制-----------------------------------

//-------------------------航向控制状态-----------------------------------
static float Desired_yaw = 0.0f;          /* 期望 yaw 角度, ±180° 包裹 (MiniFly 方式) */
static bool  yaw_needs_init = true;       /* 起飞后首次进 Yaw_Control 时锁存 JY901P 当前航向 */
static bool  yaw_stick_active = false;    /* yaw 杆量激活标志 (松手回中后切换为角度保持) */
static float yaw_rate_target = 0.0f;      /* 期望 yaw 角速率目标值 (角度环输出或杆量直接映射) */
static float lpf_stick_yaw = 0.0f;        /* yaw 杆 LPF: 松手后平滑衰减, 避免 Desired_yaw 突停 */
//-------------------------航向控制状态-----------------------------------

//-------------------------角度环目标值-----------------------------------
static float target_angle_pitch = 0.0f;   /* 俯仰角度目标值 (速度环输出或遥控杆映射) */
static float target_angle_roll = 0.0f;    /* 横滚角度目标值 (速度环输出或遥控杆映射) */
//-------------------------角度环目标值-----------------------------------

//-------------------------地面站观测数据-----------------------------------
float debug_xy_velocity_pid_count = 0.0f;
float debug_target_angle_pitch = 0.0f;
float debug_target_angle_roll = 0.0f;
float debug_target_angle_yaw = 0.0f;
float debug_yaw_rate_target = 0.0f;       /* 最终 yaw 目标角速度 deg/s，供地面站遥测 */
float debug_yaw_meas_cont = 0.0f;          /* yaw_meas_cont snapshot for telemetry */
float debug_pos_pid_out_x = 0.0f;         /* X 位置环 PID 输出的速度指令 (cm/s), 拨杆时=0 */
float debug_pos_pid_out_y = 0.0f;         /* Y 位置环 PID 输出的速度指令 (cm/s), 拨杆时=0 */
float debug_roll_rate_target = 0.0f;      /* 横滚角速度目标 (deg/s), 角度环输出 */
float debug_pitch_rate_target = 0.0f;     /* 俯仰角速度目标 (deg/s), 角度环输出 */
//-------------------------地面站观测数据-----------------------------------


//-------------------------控制输出与状态-----------------------------------
static MotorCtrl control;      /* 电机/舵机控制输出 (经混控后写入硬件) */
MotorCtrl debugEsc;            /* 调试用控制输出镜像 (供地面站读取) */
setpoint_t target;             /* 控制目标集合 (角度/速度/高度/位置/油门) */
state_t state;                 /* 飞行器当前状态 (姿态/角速度/位置/高度) */
//-------------------------控制输出与状态-----------------------------------

static void ResetYawState(void);
void Roll_Pitch_Control(setpoint_t* target, state_t* state, uint32_t tick);
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick);
float Height_Control(setpoint_t* target, state_t* state,uint32_t tick);

void Flight_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);
void Walk_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);


static void ResetYawState(void)
{
    /* 起飞/降落时: 复位标志、LPF、PID 积分.
       不写 Desired_yaw — 下次进入 Yaw_Control 时由 yaw_needs_init 锁存 JY901P 当前航向.
       (与 MiniFly 低油门时 attitudeDesired.yaw = state->attitude.yaw 思路一致) */
    yaw_needs_init = true;
    yaw_stick_active = false;
    yaw_rate_target = 0.0f;
    debug_yaw_rate_target = 0.0f;
    debug_target_angle_yaw = 0.0f;
    lpf_stick_yaw = 0.0f;
    PID_ClearIntegral(&pid_yaw_angle);
    PID_ClearIntegral(&pid_yaw_rate);
}


void flightReset(FlightResetLevel level)
{
    if (level >= FLIGHT_RESET_LAND)
    {
        PID_Reset(&pid_roll_angle);   PID_Reset(&pid_pitch_angle);
        PID_Reset(&pid_yaw_angle);    PID_Reset(&pid_roll_rate);
        PID_Reset(&pid_pitch_rate);   PID_Reset(&pid_yaw_rate);
        PID_Reset(&pid_x_position);   PID_Reset(&pid_y_position);
        PID_Reset(&pid_x_velocity);   PID_Reset(&pid_y_velocity);
        PID_Reset(&pid_height_position); PID_Reset(&pid_z_velocity);
        target_angle_pitch = 0.0f;
        target_angle_roll  = 0.0f;
    }
    if (level == FLIGHT_RESET_FULL)
    {
        position_ResetXY();
        ResetYawState();
    }
    if (level == FLIGHT_RESET_YAW)
    {
        PID_ClearIntegral(&pid_yaw_angle);
        PID_ClearIntegral(&pid_yaw_rate);
    }
}

void flightClearPosPID(void)
{
    PID_ClearIntegral(&pid_x_position);
    PID_ClearIntegral(&pid_y_position);
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
            flightReset(FLIGHT_RESET_FULL);
        if (consumeKeyLandRising())
            flightReset(FLIGHT_RESET_LAND);

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
//        safeCheck(&control,&state);
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
        flightReset(FLIGHT_RESET_LAND);
        (void)Height_Control(target,state,tick);
    }


    // LOG_INFO("thr:%.2f,roll:%.2f,pit:%.2f,yaw:%.2f",throttle,pid_roll_rate.Output,pid_pitch_rate.Output,pid_yaw_rate.Output);
    // 设置电调输出
    if (throttle > 5)
    {
			 ctrl->Esc_Percent_1 =  throttle + pid_roll_rate.Output + pid_pitch_rate.Output + pid_yaw_rate.Output - YAW_TRIM;
			 ctrl->Esc_Percent_2 =  throttle - pid_roll_rate.Output + pid_pitch_rate.Output - pid_yaw_rate.Output + YAW_TRIM;
			 ctrl->Esc_Percent_3 =  throttle - pid_roll_rate.Output - pid_pitch_rate.Output + pid_yaw_rate.Output - YAW_TRIM;
			 ctrl->Esc_Percent_4 =  throttle + pid_roll_rate.Output - pid_pitch_rate.Output - pid_yaw_rate.Output + YAW_TRIM;

			 // 添加最小油门限制，确保电机不会停转
			 ctrl->Esc_Percent_1 = limit(ctrl->Esc_Percent_1, 0, 100);
			 ctrl->Esc_Percent_2 = limit(ctrl->Esc_Percent_2, 0, 100);
			 ctrl->Esc_Percent_3 = limit(ctrl->Esc_Percent_3, 0, 100);
			 ctrl->Esc_Percent_4 = limit(ctrl->Esc_Percent_4, 0, 100);
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
        flightReset(FLIGHT_RESET_LAND);
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
    static uint8_t last_mode_x = modeDisable;
    static uint8_t last_mode_y = modeDisable;

    if (RATE_DO_EXECUTE(VELOCITY_PID_RATE,tick))
    {
        if (getCommanderCtrlMode() == MODE_THREEHOLD)
        {
            // 定点模式：位置环 + 速度环级联
            float vel_x_target, vel_y_target;

            // 位置环 (modeAbs时生效，将位置误差转为速度指令)
            if (target->mode_x == modeAbs)
            {
                if (last_mode_x != modeAbs) PID_PrepareReengage(&pid_x_position);
                vel_x_target = 0.5f * PIDCalculate(&pid_x_position, state->position.x, target->pos.x);
                vel_x_target = limit(vel_x_target, -30.0f, 30.0f);
                debug_pos_pid_out_x = vel_x_target;
            }
            else
            {
                vel_x_target = target->vel.x;
                debug_pos_pid_out_x = 0.0f;
            }

            if (target->mode_y == modeAbs)
            {
                if (last_mode_y != modeAbs) PID_PrepareReengage(&pid_y_position);
                vel_y_target = 0.5f * PIDCalculate(&pid_y_position, state->position.y, target->pos.y);
                vel_y_target = limit(vel_y_target, -30.0f, 30.0f);
                debug_pos_pid_out_y = vel_y_target;
            }
            else
            {
                vel_y_target = target->vel.y;
                debug_pos_pid_out_y = 0.0f;
            }
            last_mode_x = target->mode_x;
            last_mode_y = target->mode_y;

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
        
        /* 外环输出写入 setpoint，内环从 setpoint 读（不再直接读 PID 内部字段） */
        target->attitudeRate.pitch = pid_pitch_angle.Output;
        target->attitudeRate.roll  = pid_roll_angle.Output;
        debug_pitch_rate_target = pid_pitch_angle.Output;
        debug_roll_rate_target  = pid_roll_angle.Output;

        // LOG_DEBUG("state-pit:%.2f,set-pit:%.2f,output:%.2f",state->angle.pitch,target_angle_pitch,pid_pitch_angle.Output);
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // 角速度环
        
        PIDCalculate(&pid_pitch_rate, state->gyro.y, target->attitudeRate.pitch);
        PIDCalculate(&pid_roll_rate, state->gyro.x, target->attitudeRate.roll);
        

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

    /* 陀螺零偏校准通过后首次进入: 锁存 JY901P 当前航向为 Desired_yaw.
       同时初始化 yaw_rate_target = 0, yaw_stick_active = false. */
    if (yaw_needs_init)
    {
        Desired_yaw = state->angle.yaw;
        yaw_rate_target = 0.0f;
        yaw_stick_active = false;
        yaw_needs_init = false;
    }

    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        float raw_stick = -target->angle.yaw;

        /* LPF 平滑杆量 (与 MiniFly commander.c:117 ctrlValLpf.yaw 思路一致).
           alpha=0.2 @250Hz → τ≈20ms (MiniFly α=0.2 @100Hz → τ≈50ms). */
        lpf_stick_yaw += (raw_stick - lpf_stick_yaw) * 0.2f;

        bool stick_active_now = (fabsf(raw_stick) > YAW_DEADBAND);

        if (stick_active_now)
        {
            /* 拨杆分支: 直接设置角速度目标, Desired_yaw 跟随实际 yaw 避免边沿突变 */
            yaw_rate_target = limit(lpf_stick_yaw, -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
            Desired_yaw = state->angle.yaw;
        }
        else
        {
            /* 回中分支: 仅当从拨杆切换过来时锁存 Desired_yaw 并清角度 PID 积分 */
            if (yaw_stick_active)
            {
                Desired_yaw = state->angle.yaw;
                flightReset(FLIGHT_RESET_YAW);
            }
            /* ±180° 最短角误差, 角度 PID 输出作为 yaw 角速度目标 */
            float yawError = Desired_yaw - state->angle.yaw;
            if (yawError > 180.0f)       yawError -= 360.0f;
            else if (yawError < -180.0f) yawError += 360.0f;
            PIDCalculate(&pid_yaw_angle, Desired_yaw - yawError, Desired_yaw);
            yaw_rate_target = pid_yaw_angle.Output;
        }

        yaw_stick_active = stick_active_now;
        debug_target_angle_yaw = Desired_yaw;
        debug_yaw_rate_target = yaw_rate_target;

        /* yaw 外环输出写入 setpoint（拨杆时 = 直接角速率，回中时 = 角度环输出） */
        target->attitudeRate.yaw = yaw_rate_target;
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // 角速度环
        PIDCalculate(&pid_yaw_rate, state->gyro.z, target->attitudeRate.yaw);
    }
}


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
    static uint8_t last_mode_z = modeDisable;

    #define TAKEOFF_START_THRUST  20.0f
    #define LAND_HIGH_FLOOR       45.0f   /* 高空降落地板(%) */
    #define LAND_IDLE_THRUST      37.5f   /* 低空怠速地板(%) */
    #define LAND_FLOOR_SLEW       0.008f  /* 地板一阶逼近系数(越小越平滑) */
    #define TAKEOFF_RAMP_CYCLE    500     /* 2秒 @250Hz */

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

    /* 停机缓降: 触底确认后 flyerAutoLand 设 modeDisable + 递减油门, 这里直接返回递减值 */
    if (getCommanderKeyland() && target->mode_z == modeDisable)
    {
        return limit(target->thrust, 0, 100);
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
            if (last_mode_z != modeAbs) PID_PrepareReengage(&pid_height_position);
            PIDCalculate(&pid_height_position, state->height, target->height);
        }
        last_mode_z = target->mode_z;
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
        /* 降落地板: 高空用 LAND_HIGH_FLOOR(速度环主导), 低空降到 LAND_IDLE_THRUST(怠速) */
        else if (getCommanderKeyland())
        {
            float landFloor = (state->height > LAND_SLOW_HEIGHT) ? LAND_HIGH_FLOOR : LAND_IDLE_THRUST;
            /* 一阶平滑逼近地板, 避免油门突变导致瞬时掉高 */
            baseThrust += (landFloor - baseThrust) * LAND_FLOOR_SLEW;
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

    return limit(thrustRaw, 0, 100);  // 限制油门上限在70%
}

float getAltholdThrust(void)
{
	return thrustLpf;
}

float getThrustCmd(void)
{
    return thrustCmd;
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
