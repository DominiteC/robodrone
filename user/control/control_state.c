#include "control_state.h"

#include "gyro.h"
#include "position.h"
#include "log.h"
#include "control.h"

void printState(state_t* state)
{
    // LOG_INFO("velocity-x:%.2f,y:%.2f,z:%.2f",state->velocity.x,state->velocity.y,state->velocity.z);
    LOG_INFO("angle-roll:%.2f,pitch:%.2f,yaw:%.2f",state->angle.roll,state->angle.pitch,state->angle.yaw);
}

/**
 * @brief 刷新状态
 *
 * @param state 状态结构体
 */
void refreshState(state_t* state)
{
    // 陀螺仪
    gyro_getAngularVelocity(&state->gyro);
    gyro_getAngle(&state->angle);
    debug_yaw_meas_cont = state->angle.yaw;  /* 遥测: 每拍刷新, 无论 yaw 控制是否激活 */
    gyro_getAcc(&state->acc);
    // 光流（内部已做倾斜补偿 + 旋转补偿，仅在新数据到达时执行一次）
    position_GetVelocity(&state->velocity, &state->angle, &state->gyro);
    position_GetHeight(&state->height);
    position_GetPosition(&state->position);

    // printState(state);
}
