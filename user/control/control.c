#include "control.h"
#include "control_state.h"
#include "control_output.h"
#include "control_safety.h"
#include "stdlib.h"
#include <math.h>
#include "remotedata.h"
#include "commander.h"
#include "servo.h"
#include "watchdog_guard.h"
#include "log.h"
#include "change.h"
#include "Mydelay.h"
#include "watchdog_guard.h"

#include "FreeRTOS.h"
#include "task.h"

#define limit(x, min, max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

#define ALTHOLD_THRUST_BASE 50.0f
#define YAW_MAX_RATE	42.0f	// 满杆yaw角速率 deg/s
#define YAW_DEADBAND	5.0f	// yaw杆死区

static float thrustLpf = 35;	/*油门低通*/
static float thrustCmd = 0;    /* 实际用于混控前的推力命令 */

static float Desired_yaw;
static float yawOld = 0.0f;
static int32_t yawTurnNum = 0;
static bool yawUnwrapInited = false;
static bool isAdjustingYaw = false;
static float target_angle_pitch = 0.0f;
static float target_angle_roll = 0.0f;
float debug_xy_velocity_pid_count = 0.0f;
float debug_target_angle_pitch = 0.0f;
float debug_target_angle_roll = 0.0f;
float debug_target_angle_yaw = 0.0f;

static MotorCtrl control;
MotorCtrl debugEsc;
setpoint_t target;
state_t state;

static void ResetYawState(void);

void Roll_Pitch_Control(setpoint_t* target, state_t* state, uint32_t tick);
void Yaw_Control(setpoint_t* target, state_t* state, uint32_t tick);
float Height_Control(setpoint_t* target, state_t* state,uint32_t tick);

void Flight_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);
void Walk_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state);


static float wrapYawDisplay(float yaw);

static void ResetYawState(void)
{
    Desired_yaw = state.angle.yaw;
    debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);
    yawOld = state.angle.yaw;
    yawTurnNum = 0;
    yawUnwrapInited = false;
    isAdjustingYaw = false;
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
 *          (顺)M3    ↑    M1(逆)
 *                \   |   /
 *                 \  |  /
 *                  \ | /
 *            ————————+————————>Y+
 *                  / | \
 *                 /  |  \
 *                /   |   \
 *           (逆)M2   |    M4(顺)
 * 
 */
void Flight_Update(MotorCtrl* ctrl, setpoint_t* target, state_t* state)
{
    float throttle = 0;
    bool flightActive = getCommanderKeyFlight() || getCommanderKeyland();
    static uint32_t tick = 0;

    if (!state->isRCLocked && flightActive)
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
    }


    // LOG_INFO("thr:%.2f,roll:%.2f,pit:%.2f,yaw:%.2f",throttle,pid_roll_rate.Output,pid_pitch_rate.Output,pid_yaw_rate.Output);
    // 设置电调输出
    if (!state->isRCLocked && throttle > 5)
    {
			 ctrl->Esc_Percent_1 = throttle + pid_roll_rate.Output + pid_pitch_rate.Output;// - pid_yaw_rate.Output;
			 ctrl->Esc_Percent_2 = throttle - pid_roll_rate.Output - pid_pitch_rate.Output;// - pid_yaw_rate.Output;
			 ctrl->Esc_Percent_3 = throttle - pid_roll_rate.Output + pid_pitch_rate.Output;// + pid_yaw_rate.Output;
			 ctrl->Esc_Percent_4 = throttle + pid_roll_rate.Output - pid_pitch_rate.Output;// + pid_yaw_rate.Output;

			 // 添加最小油门限制，确保电机不会停转
			 ctrl->Esc_Percent_1 = limit(ctrl->Esc_Percent_1, 10, 24);
			 ctrl->Esc_Percent_2 = limit(ctrl->Esc_Percent_2, 10, 24);
			 ctrl->Esc_Percent_3 = limit(ctrl->Esc_Percent_3, 10, 24);
			 ctrl->Esc_Percent_4 = limit(ctrl->Esc_Percent_4, 10, 24);
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
    if (RATE_DO_EXECUTE(ANGEL_PID_RATE,tick))
    {
        // 角度缠绕展开
        if (!yawUnwrapInited)
        {
            yawOld = state->angle.yaw;
            Desired_yaw = state->angle.yaw;
            yawUnwrapInited = true;
        }
        float diff = state->angle.yaw - yawOld;
        if (diff > 180.0f)
            {yawTurnNum--;LOG_INFO("turnNum: %d",yawTurnNum);}
        else if (diff < -180.0f)
            {yawTurnNum++;LOG_INFO("turnNum: %d",yawTurnNum);}
        yawOld = state->angle.yaw;

        float yaw_meas_cont = state->angle.yaw + yawTurnNum * 360.0f;
        float stick = -target->angle.yaw;

        if (fabsf(stick) > YAW_DEADBAND)
        {
            // 摇杆偏离中心：按比例调整目标角度
            Desired_yaw += (stick / 100.0f) * YAW_MAX_RATE * ANGEL_PID_DT;
            isAdjustingYaw = true;
        }
        else
        {
            // 摇杆回中：位置保持
            if (isAdjustingYaw)
            {
                // 刚回中 → 锁定当前角度
                Desired_yaw = yaw_meas_cont;
                PID_ClearIntegral(&pid_yaw_angle);
                PID_ClearIntegral(&pid_yaw_rate);
                pid_yaw_angle.Output = 0;
                pid_yaw_angle.Last_Output = 0;
                pid_yaw_rate.Output = 0;
                pid_yaw_rate.Last_Output = 0;
                isAdjustingYaw = false;
            }
            // 否则 Desired_yaw 不变，继续位置保持
        }

        // LOG_DEBUG("desire-yaw:%.2f",Desired_yaw);
        // 角度环
        
        PIDCalculate(&pid_yaw_angle, yaw_meas_cont, Desired_yaw);
        debug_target_angle_yaw = wrapYawDisplay(Desired_yaw);
        if (fabsf(pid_yaw_angle.Err) < pid_yaw_angle.DeadBand)
        {
            pid_yaw_angle.Output = 0;
            pid_yaw_angle.Last_Output = 0;
        }
        

        // if (myDelay((uint32_t)Yaw_Control,100))
        //     LOG_DEBUG("state-yaw:%.2f,set-yaw:%.2f,output:%.2f",state->angle.yaw,Desired_yaw,pid_yaw_angle.Output);
    }

    if (RATE_DO_EXECUTE(RATE_PID_RATE,tick))
    {
        // float error = fabs(target->angle.yaw/10 - state->gyro.z);
        // if (error < 5.0f)
        // {
        //     pid_yaw_rate.Kp = 0.1f;
        // }
        // // else if (error < 15.0f)
        // // {
        // //     pid_yaw_rate.Kp = 0.3f;
        // // }
        // else
        // {
        //     pid_yaw_rate.Kp = 1.0f;
        // }
        // 角速度环
        
//        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);
        PIDCalculate(&pid_yaw_rate, state->gyro.z, pid_yaw_angle.Output);//target->angle.yaw/10
        if (fabsf(pid_yaw_rate.Err) < pid_yaw_rate.DeadBand)
        {
            pid_yaw_rate.Output = 0;
            pid_yaw_rate.Last_Output = 0;
        }
        

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
    #define LAND_END_THRUST       45.0f
    #define TAKEOFF_RAMP_CYCLE    500     /* 2秒 @250Hz */
    #define LAND_RAMP_CYCLE       1250    /* 5秒 @250Hz */

    //手动模式直接映射油门，定高模式才使用PID控制高度
    if (getCommanderCtrlMode() == MODE_MANUAL)
        return limit(target->thrust, 0, 70);

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
