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

#define ALTHOLD_THRUST_BASE 50.0f //锟斤拷停使锟矫的伙拷准锟斤拷锟斤拷
#define YAW_MAX_RATE	42.0f				// 锟斤拷锟斤拷yaw锟斤拷锟斤拷锟斤拷 deg/s
#define YAW_DEADBAND	5.0f				// yaw锟斤拷锟斤拷锟斤拷

static float thrustLpf = 35;	/*锟斤拷锟脚碉拷通*/
static float thrustCmd = 0;    /* 实锟斤拷锟斤拷锟节伙拷锟角帮拷锟斤拷锟斤拷锟斤拷锟斤拷锟