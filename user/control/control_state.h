#ifndef __CONTROL_STATE_H__
#define __CONTROL_STATE_H__

#include "vehicle_state.h"

/*
 * 控制状态模块只负责采集并刷新控制任务需要的飞行器状态。
 * 这里不放控制算法，避免状态采集和控制计算继续混在 control.c 里。
 */
void refreshState(state_t* state);
void printState(state_t* state);

#endif