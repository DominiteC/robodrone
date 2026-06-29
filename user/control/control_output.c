#include "control_output.h"

#include "esc.h"
#include "motor.h"

/**
 * @brief 电机控制
 *
 * @param ctrl 电机控制数据结构体
 */
void MotorControl(MotorCtrl* ctrl)
{
    ESC_Set_Percent(ctrl->Esc_Percent_1, ctrl->Esc_Percent_2, ctrl->Esc_Percent_3, ctrl->Esc_Percent_4);
    Motor_Set_PWM(ctrl->Motor_Left_Front_PWM,ctrl->Motor_Left_Back_PWM,
                ctrl->Motor_Right_Front_PWM,ctrl->Motor_Right_Back_PWM);
    // if (myDelay((uint32_t)MotorControl,100))
    //     LOG_DEBUG("esc:%.2f,%.2f,%.2f,%.2f  motor:%d,%d",ctrl->Esc_Percent_1, ctrl->Esc_Percent_2, ctrl->Esc_Percent_3, ctrl->Esc_Percent_4,
    //                 ctrl->Motor_Left_Front_PWM,ctrl->Motor_Right_Front_PWM);
}