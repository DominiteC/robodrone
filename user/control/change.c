#include "change.h"
#include "servo.h"
#include "commander.h"
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "actuator.h"
#include "esc.h"
#include "watchdog_guard.h"
#include "AT24Cxx.h"

#define SERVO_TIME_TICK 5       // 每个舵机转了之后延时时间
#define SERVO_ANGLE_TICK 0      // 每次多个端机转了之后的延时时间

#define SERVO_SINGLE_ANGLE  1.5   // 舵机单次需要转的角度
#define SERVO_SINGLE_ANGLE_2  0.3   // 舵机单次需要转的角度

#define SERVO_WAIT_TIMEOUT_TICKS pdMS_TO_TICKS(20000)

#define WHEEL_OSCILLATE_PWM            400   // 轮子振荡 PWM 幅度
#define WHEEL_OSCILLATE_HALF_PERIOD_MS 500   // 半周期 ms

//-------------------------舵机模式状态-----------------------------------
static ServoMode servo_mode = SERVO_AIRPLANE;  /* 当前舵机模式 (飞行/陆行/45度陆行) */
//-------------------------舵机模式状态-----------------------------------

static bool ServoWaitTimedOut(TickType_t startTick, const char* tag)
{
    if ((xTaskGetTickCount() - startTick) > SERVO_WAIT_TIMEOUT_TICKS)
    {
        LOG_ERROR("%s timeout, force safety latch", tag);
        setCommanderSafetyLatched(true);
        setCommanderKeyFlight(false);
        setCommanderKeyland(false);
        return true;
    }
    return false;
}

ServoMode getServoMode(void)
{
    return servo_mode;
}

void initServoMode(ServoMode mode)
{
    servo_mode = mode;
    LOG_INFO("初始化舵机模式为%d", mode);
}

/**
 * @brief 使一号前舵机慢慢的转动
 * 
 * @param angle 要转动到的位置
 */
void setServoSlow_1_Front(ServoAngle angle)
{
    // float finish[8]; // 结束角度
    // bool finish_flag[8] = {0};    // 是否更新完成
    // if (angle == SERVO_INIT)
    // {
    //     finish[LEFT_FRONT_1] = LEFT_FRONT_1_INIT;
    //     finish[RIGHT_FRONT_1] = RIGHT_FRONT_1_INIT;
    // }
    // else if (angle == SERVO_UP)
    // {
    //     finish[LEFT_FRONT_1] = LEFT_FRONT_1_UP;
    //     finish[RIGHT_FRONT_1] = RIGHT_FRONT_1_UP;
    // }
        
    // // 当只要有一个没有达到时
    // while (!(finish_flag[LEFT_FRONT_1] && finish_flag[RIGHT_FRONT_1]))
    // {
    //     if (!finish_flag[LEFT_FRONT_1])
    //     {
    //         finish_flag[LEFT_FRONT_1] = setServoSlowByIndex(LEFT_FRONT_1,finish[LEFT_FRONT_1],SERVO_SINGLE_ANGLE);
    //         vTaskDelay(SERVO_TIME_TICK);
    //     }
    //     if (!finish_flag[RIGHT_FRONT_1])
    //     {
    //         finish_flag[RIGHT_FRONT_1] = setServoSlowByIndex(RIGHT_FRONT_1,finish[RIGHT_FRONT_1],SERVO_SINGLE_ANGLE);
    //         vTaskDelay(SERVO_TIME_TICK);
    //     }
    //     vTaskDelay(SERVO_ANGLE_TICK);
    // }
        
    // Actuator_Set(LEFT_FRONT,angle == SERVO_INIT ? true : false);
    // Actuator_Set(RIGHT_FRONT,angle == SERVO_INIT ? true : false);
}
/**
 * @brief 使一号后舵机慢慢的转动
 * 
 * @param angle 要转动到的位置
 */
void setServoSlow_1_Back(ServoAngle angle)
{
    // uint16_t finish[8]; // 结束角度
    // bool finish_flag[8] = {0};    // 是否更新完成
    // if (angle == SERVO_INIT)
    // {
    //     finish[LEFT_BACK_1] = LEFT_BACK_1_INIT;
    //     finish[RIGHT_BACK_1] = RIGHT_BACK_1_INIT;
    // }
    // else if (angle == SERVO_UP)
    // {
    //     finish[LEFT_BACK_1] = LEFT_BACK_1_UP;
    //     finish[RIGHT_BACK_1] = RIGHT_BACK_1_UP;
    // }
        
    // // 当只要有一个没有达到时
    // while (!(finish_flag[LEFT_BACK_1] && finish_flag[RIGHT_BACK_1]))
    // {
    //     if (!finish_flag[LEFT_BACK_1])
    //     {
    //         finish_flag[LEFT_BACK_1] = setServoSlowByIndex(LEFT_BACK_1,finish[LEFT_BACK_1],SERVO_SINGLE_ANGLE);
    //         vTaskDelay(SERVO_TIME_TICK);
    //     }
    //     if (!finish_flag[RIGHT_BACK_1])
    //     {
    //         finish_flag[RIGHT_BACK_1] = setServoSlowByIndex(RIGHT_BACK_1,finish[RIGHT_BACK_1],SERVO_SINGLE_ANGLE);
    //         vTaskDelay(SERVO_TIME_TICK);
    //     }
    //     vTaskDelay(SERVO_ANGLE_TICK);
    // }
        
    // Actuator_Set(LEFT_BACK,angle == SERVO_INIT ? true : false);
    // Actuator_Set(RIGHT_BACK,angle == SERVO_INIT ? true : false);
}
/**
 * @brief 使二号前舵机慢慢的转动
 * 
 * @param angle 要转动到的位置
 */
static void setServoSlow_2_Front(ServoAngle angle)
{
    uint16_t finish[8]; // 结束角度
    bool finish_flag[8] = {0};    // 是否更新完成
    if (angle == SERVO_INIT)
    {
        finish[LEFT_FRONT_2] = LEFT_FRONT_2_INIT;
        finish[RIGHT_FRONT_2] = RIGHT_FRONT_2_INIT;
    }
    else if (angle == SERVO_UP)
    {
        finish[LEFT_FRONT_2] = LEFT_FRONT_2_UP;
        finish[RIGHT_FRONT_2] = RIGHT_FRONT_2_UP;
    }
        
    // 当只要有一个没有达到时
    TickType_t startTick = xTaskGetTickCount();
    while (!(finish_flag[LEFT_FRONT_2] && finish_flag[RIGHT_FRONT_2]))
    {
        if (ServoWaitTimedOut(startTick, "setServoSlow_2_Front"))
        {
            break;
        }
        if (!finish_flag[LEFT_FRONT_2])
        {
            finish_flag[LEFT_FRONT_2] = setServoSlowByIndex(LEFT_FRONT_2,finish[LEFT_FRONT_2],SERVO_SINGLE_ANGLE);
            vTaskDelay(SERVO_TIME_TICK);
        }
        if (!finish_flag[RIGHT_FRONT_2])
        {
            finish_flag[RIGHT_FRONT_2] = setServoSlowByIndex(RIGHT_FRONT_2,finish[RIGHT_FRONT_2],SERVO_SINGLE_ANGLE);
            vTaskDelay(SERVO_TIME_TICK);
        }
        vTaskDelay(SERVO_ANGLE_TICK);
    }
        
}

/**
 * @brief 使二号后舵机慢慢的转动
 * 
 * @param angle 要转动到的位置
 */
void setServoSlow_2_Back(ServoAngle angle)
{
    uint16_t finish[8]; // 结束角度
    bool finish_flag[8] = {0};    // 是否更新完成
    if (angle == SERVO_INIT)
    {
        finish[LEFT_BACK_2] = LEFT_BACK_2_INIT;
        finish[RIGHT_BACK_2] = RIGHT_BACK_2_INIT;
    }
    else if (angle == SERVO_UP)
    {
        finish[LEFT_BACK_2] = LEFT_BACK_2_UP;
        finish[RIGHT_BACK_2] = RIGHT_BACK_2_UP;
    }
        
    // 当只要有一个没有达到时
    TickType_t startTick = xTaskGetTickCount();
    while (!(finish_flag[LEFT_BACK_2] && finish_flag[RIGHT_BACK_2]))
    {
        if (ServoWaitTimedOut(startTick, "setServoSlow_2_Back"))
        {
            break;
        }
        if (!finish_flag[LEFT_BACK_2])
        {
            finish_flag[LEFT_BACK_2] = setServoSlowByIndex(LEFT_BACK_2,finish[LEFT_BACK_2],SERVO_SINGLE_ANGLE);
            vTaskDelay(SERVO_TIME_TICK);
        }
        if (!finish_flag[RIGHT_BACK_2])
        {
            finish_flag[RIGHT_BACK_2] = setServoSlowByIndex(RIGHT_BACK_2,finish[RIGHT_BACK_2],SERVO_SINGLE_ANGLE);
            vTaskDelay(SERVO_TIME_TICK);
        }
        vTaskDelay(SERVO_ANGLE_TICK);
    }
        
}

/**
 * @brief 使二号舵机慢慢的转动
 * 
 * @param angle 要转动到的位置
 */
void setServoSlow_2(ServoAngle angle,float tick_angle)
{
    uint16_t finish[8]; // 结束角度
    bool finish_flag[8] = {0};    // 是否更新完成
    if (angle == SERVO_INIT)
    {
        finish[LEFT_FRONT_2] = LEFT_FRONT_2_INIT;
        finish[LEFT_BACK_2] = LEFT_BACK_2_INIT;
        finish[RIGHT_FRONT_2] = RIGHT_FRONT_2_INIT;
        finish[RIGHT_BACK_2] = RIGHT_BACK_2_INIT;
    }
    else if (angle == SERVO_UP)
    {
        finish[LEFT_FRONT_2] = LEFT_FRONT_2_UP;
        finish[LEFT_BACK_2] = LEFT_BACK_2_UP;
        finish[RIGHT_FRONT_2] = RIGHT_FRONT_2_UP;
        finish[RIGHT_BACK_2] = RIGHT_BACK_2_UP;
    }
    else if (angle == SERVO_WALK_2)
    {
        finish[LEFT_FRONT_2] = LEFT_FRONT_2_WALK;
        finish[LEFT_BACK_2] = LEFT_BACK_2_WALK;
        finish[RIGHT_FRONT_2] = RIGHT_FRONT_2_WALK;
        finish[RIGHT_BACK_2] = RIGHT_BACK_2_WALK;
    }
        
    // 当只要有一个没有达到时
    TickType_t startTick = xTaskGetTickCount();
    while (!(finish_flag[LEFT_FRONT_2] && finish_flag[LEFT_BACK_2] && finish_flag[RIGHT_FRONT_2] && finish_flag[RIGHT_BACK_2]))
    {
        if (ServoWaitTimedOut(startTick, "setServoSlow_2"))
        {
            break;
        }
        if (!finish_flag[LEFT_FRONT_2])
        {
            finish_flag[LEFT_FRONT_2] = setServoSlowByIndex(LEFT_FRONT_2,finish[LEFT_FRONT_2],tick_angle);
            vTaskDelay(SERVO_TIME_TICK);
        }
        if (!finish_flag[LEFT_BACK_2])
        {
            finish_flag[LEFT_BACK_2] = setServoSlowByIndex(LEFT_BACK_2,finish[LEFT_BACK_2],tick_angle);
            vTaskDelay(SERVO_TIME_TICK);
        }
        if (!finish_flag[RIGHT_FRONT_2])
        {
            finish_flag[RIGHT_FRONT_2] = setServoSlowByIndex(RIGHT_FRONT_2,finish[RIGHT_FRONT_2],tick_angle);
            vTaskDelay(SERVO_TIME_TICK);
        }
        if (!finish_flag[RIGHT_BACK_2])
        {
            finish_flag[RIGHT_BACK_2] = setServoSlowByIndex(RIGHT_BACK_2,finish[RIGHT_BACK_2],tick_angle);
            vTaskDelay(SERVO_TIME_TICK);
        }
        vTaskDelay(SERVO_ANGLE_TICK);
    }
        
}

/**
 * @brief 变形时启动电杆 + 轮子来回滚动，到时停止
 */
static void transformActuatorWithWheels(bool extend, uint16_t duration_ms, int16_t pwm_amplitude, uint16_t half_period_ms)
{
    Actuator_Start(1, extend);
    Actuator_Start(2, extend);
    Actuator_Start(3, extend);
    Actuator_Start(4, extend);

    uint32_t elapsed = 0;
    int16_t pwm = pwm_amplitude;

    while (elapsed < duration_ms)
    {
        Motor_Set_PWM(pwm, pwm, pwm, pwm);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
        elapsed += half_period_ms;
        pwm = -pwm;
    }

    Actuator_Stop(1);
    Actuator_Stop(2);
    Actuator_Stop(3);
    Actuator_Stop(4);
    Motor_Set_PWM(0, 0, 0, 0);
}

/**
 * @brief 切换舵机状态
 *
 * @param mode 需要切换的模式
 */
void changeServoMode(MotorCtrl* ctrl,ServoMode mode)
{
    WatchdogGuard_EnterLongAction(30000);
    // 切换模式为飞行模式，从陆行模式切换为飞行模式应该是先变一舵机再变二舵机
    if (mode == SERVO_AIRPLANE && servo_mode != SERVO_AIRPLANE)
    {

//        setServoSlow_2(SERVO_UP,SERVO_SINGLE_ANGLE_2);
// 				setServoSlow_1_Front(SERVO_INIT);			
//				Actuator_Set_2(LEFT_FRONT,LEFT_BACK,true,15000);				
//				Actuator_Set_2(RIGHT_FRONT,RIGHT_BACK,true,15000);				
//        setServoSlow_2_Front(SERVO_INIT);
// 				setServoSlow_1_Back(SERVO_INIT);
//        Actuator_Set_2(LEFT_BACK,RIGHT_BACK,true,5000);
//        setServoSlow_2_Back(SERVO_INIT);
			
        transformActuatorWithWheels(true, 15000, WHEEL_OSCILLATE_PWM, WHEEL_OSCILLATE_HALF_PERIOD_MS);

        // if (servo_mode == SERVO_WALK_45)
        // {
        //     setServoSlow_2(SERVO_INIT,SERVO_SINGLE_ANGLE_2);
        // }
			
        servo_mode = SERVO_AIRPLANE;
        CommanderPersist_SaveServoMode((uint8_t)SERVO_AIRPLANE);
        LOG_INFO("舵机转换为飞行模式");
    }
//    // 切换模式为倾斜陆行模式
//    else if (mode == SERVO_WALK_45 && servo_mode != SERVO_WALK_45)
//    {
//        if (servo_mode == SERVO_AIRPLANE)
//        {
//            // ESC_Set_PWM(1055,1055,1055,1055);
//            // Actuator_AllSet(0,25000);
//            // ESC_Set_PWM(0,0,0,0);

//            setServoSlow_2_Front(SERVO_UP);
//            Actuator_Set_2(LEFT_FRONT,RIGHT_FRONT,false,3250);
//            // setServoSlow_1_Front(SERVO_UP);

//            setServoSlow_2_Back(SERVO_UP);
//            Actuator_Set_2(LEFT_BACK,RIGHT_BACK,false,3250);
//            Actuator_AllSet(false,6500);
//            // setServoSlow_1_Back(SERVO_UP);
//            // vTaskDelay(300);

//            setServoSlow_2(SERVO_WALK_2,SERVO_SINGLE_ANGLE_2);
//            Motor_Set_PWM(0,0,0,0);
//        }
//        else if (servo_mode == SERVO_WALK)
//        {
//            setServoSlow_2(SERVO_WALK_2,SERVO_SINGLE_ANGLE_2);
//        }

//        servo_mode = SERVO_WALK_45;

//        LOG_INFO("舵机转换为45度陆行模式");
//    }
    // 切换模式为陆行模式，从飞行模式切换为陆行模式应该是先变二舵机再变一舵机
    else if (mode == SERVO_WALK && servo_mode != SERVO_WALK)
    {
        if (servo_mode == SERVO_AIRPLANE)
        {
            // ESC_Set_PWM(1055,1055,1055,1055);
            // ESC_Set_PWM(0,0,0,0);

//            setServoSlow_2_Front(SERVO_UP);
//            Actuator_Set_2(LEFT_FRONT,RIGHT_FRONT,false,5000);
            // setServoSlow_1_Front(SERVO_UP);

//            setServoSlow_2_Back(SERVO_UP);
//            Actuator_Set_2(LEFT_BACK,RIGHT_BACK,false,5000);
					

//					Actuator_Set_2(LEFT_FRONT,LEFT_BACK,false,13500);	+
//					Actuator_Set_2(RIGHT_FRONT,RIGHT_BACK,false,13300);	

            transformActuatorWithWheels(false, 16500, 1200, 500);
            // setServoSlow_1_Back(SERVO_UP);
        }
        else if (servo_mode == SERVO_WALK_45)
        {
            // setServoSlow_2(SERVO_INIT,SERVO_SINGLE_ANGLE_2);
        }
//        setServoSlow_2(SERVO_INIT,SERVO_SINGLE_ANGLE_2);
//        Motor_Set_PWM(0,0,0,0);
        LOG_INFO("舵机转换为陆行模式");

        servo_mode = SERVO_WALK;
        CommanderPersist_SaveServoMode((uint8_t)SERVO_WALK);
    }

    WatchdogGuard_ExitLongAction();
}

void changeAttitude(MotorCtrl* ctrl, state_t* state)
{
    // 安全检查：只有停机状态才允许变形
    if (state->isRCLocked || state->height >= 500)
    {
        ctrl->Esc_Percent_1 = 0;
        ctrl->Esc_Percent_2 = 0;
        ctrl->Esc_Percent_3 = 0;
        ctrl->Esc_Percent_4 = 0;
        ctrl->Motor_Left_Front_PWM = 0;
        ctrl->Motor_Left_Back_PWM = 0;
        ctrl->Motor_Right_Front_PWM = 0;
        ctrl->Motor_Right_Back_PWM = 0;
        return;
    }

//-------------------------变形状态机-----------------------------------
    static uint8_t flag = 0;  /* 变形状态机步骤: 0→清输出, 1→执行变形 */
//-------------------------变形状态机-----------------------------------
    switch (flag)
    {
    case 0:
        ctrl->Esc_Percent_1 = 0;
        ctrl->Esc_Percent_2 = 0;
        ctrl->Esc_Percent_3 = 0;
        ctrl->Esc_Percent_4 = 0;
        ctrl->Motor_Left_Front_PWM = 0;
        ctrl->Motor_Left_Back_PWM = 0;
        ctrl->Motor_Right_Front_PWM = 0;
        ctrl->Motor_Right_Back_PWM = 0;
        flag = 1;
        break;
    case 1:
        if (consumeCommanderAttitudeModeChanged())
        {
            if (getCommanderAttitudeMode() == MODE_WALK)
            {
                changeServoMode(ctrl,SERVO_WALK);
            }
            else if (getCommanderAttitudeMode() == MODE_WALK_45)
            {
                changeServoMode(ctrl,SERVO_WALK_45);
            }
            else if (getCommanderAttitudeMode() == MODE_AIRPLANE)
            {
                changeServoMode(ctrl,SERVO_AIRPLANE);
            }
        }
        flag = 0;
        break;

    default:
        break;
    }
}

