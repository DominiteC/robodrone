#ifndef __CONTROL_RATES_H__
#define __CONTROL_RATES_H__

#include <stdint.h>

/*
 * 控制周期配置集中放在这里。
 * 控制算法只通过这些宏判断执行频率，避免周期参数散落在控制接口头文件中。
 */
#define RATE_5_HZ      5
#define RATE_10_HZ     10
#define RATE_25_HZ     25
#define RATE_50_HZ     50
#define RATE_100_HZ    100
#define RATE_200_HZ    200
#define RATE_250_HZ    250
#define RATE_500_HZ    500
#define RATE_1000_HZ   1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#define MAIN_LOOP_RATE          RATE_1000_HZ
#define MAIN_LOOP_DT            (uint32_t)(1000 / MAIN_LOOP_RATE)

#define ATTITUDE_ESTIMAT_RATE   RATE_250_HZ
#define ATTITUDE_ESTIMAT_DT     (1.0 / ATTITUDE_ESTIMAT_RATE)

#define POSITION_ESTIMAT_RATE   RATE_250_HZ
#define POSITION_ESTIMAT_DT     (1.0 / POSITION_ESTIMAT_RATE)

#define RATE_PID_RATE           RATE_500_HZ
#define RATE_PID_DT             (1.0 / RATE_500_HZ)

#define ANGEL_PID_RATE          ATTITUDE_ESTIMAT_RATE
#define ANGEL_PID_DT            (1.0 / ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE       POSITION_ESTIMAT_RATE
#define VELOCITY_PID_DT         (1.0 / POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE       POSITION_ESTIMAT_RATE
#define POSITION_PID_DT         (1.0 / POSITION_ESTIMAT_RATE)

#endif