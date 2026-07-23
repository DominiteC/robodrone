/*
 * app_tasks.c
 * 负责集中创建 FreeRTOS 业务任务，并维护任务栈大小和优先级。
 * 本文件不负责设备初始化和控制算法实现。
 */
#include "app_tasks.h"
#include "app_config.h"
#include "alarm.h"
#include "ANO_DT.h"
#include "remotedata.h"
#include "wireless.h"
#include "control.h"
#include "mtf_01.h"
#include "usmart.h"
#include "gyro.h"            /* gyro_calibrateGyroZOffset */
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"

/* 任务资源配置集中放在这里，便于后续调整时核对栈大小和优先级。 */
#define TASK_STACK_ALARM      250
#define TASK_PRIO_ALARM       2
#define TASK_STACK_USMART     250
#define TASK_PRIO_USMART      2
#define TASK_STACK_ANO_DT     600
#define TASK_PRIO_ANO_DT      2
#define TASK_STACK_SEND       350
#define TASK_PRIO_SEND        2
#define TASK_STACK_RECEIVE    450
#define TASK_PRIO_RECEIVE     4
#define TASK_STACK_CONTROL    700
#define TASK_PRIO_CONTROL     4
#define TASK_STACK_MTF01      400
#define TASK_PRIO_MTF01       3
#define TASK_STACK_GYRO_CALIB 200
#define TASK_PRIO_GYRO_CALIB  1
#define TASK_STACK_MONITOR    250
#define TASK_PRIO_MONITOR     2    /* 与 Control/ANO 同级, 时间片轮转, 不会被饿死 */

/* 任务句柄, 用于监控各任务栈剩余 */
TaskHandle_t hAlarm  = NULL;
TaskHandle_t hUsmart = NULL;
TaskHandle_t hANO    = NULL;
TaskHandle_t hSend   = NULL;
TaskHandle_t hRecv   = NULL;
TaskHandle_t hControl= NULL;
TaskHandle_t hMtf    = NULL;

static void usmart_task(void* param);
static void gyroCalibTask(void* param);
static void stkMonTask(void* param);

void App_CreateTasks(void)
{
  // Freertos任务创建
  xTaskCreate(Alarm_Update,"Alarm",TASK_STACK_ALARM,NULL,TASK_PRIO_ALARM,&hAlarm);            // 报警处理任务，包括低电压检测，姿态检测
#if ENABLE_GCS_SERIAL
  #if USE_USMART_OR_ANO == 1
    xTaskCreate(usmart_task,"usmart",TASK_STACK_USMART,NULL,TASK_PRIO_USMART,&hUsmart);            // usmart任务，在串口中调用函数，与ANO_DT_Data_Exchange任务互斥
  #elif USE_USMART_OR_ANO == 0
    xTaskCreate(ANO_DT_Data_Exchange,"ANO_DT",TASK_STACK_ANO_DT,NULL,TASK_PRIO_ANO_DT,&hANO);   // 与地面站数据交换任务，通过串口发送数据，与usmart任务互斥
  #endif
#endif
  xTaskCreate(SendToRemote,"Send",TASK_STACK_SEND,NULL,TASK_PRIO_SEND,&hSend);             			// 发送遥控数据任务，通过无线模块发送数据
  xTaskCreate(Wireless_ReceiveTask,"Receive",TASK_STACK_RECEIVE,NULL,TASK_PRIO_RECEIVE,&hRecv); // 接收遥控数据任务，通过无线模块接收数据
  xTaskCreate(Control_Task,"Control",TASK_STACK_CONTROL,NULL,TASK_PRIO_CONTROL,&hControl);          	// 控制任务，飞控核心任务，包括姿态解算，位置解算，控制算法等，控制电机驱动，电调和舵机。
  xTaskCreate(mtf_01_task,"mtf_01",TASK_STACK_MTF01,NULL,TASK_PRIO_MTF01,&hMtf);            			// 光流模块任务
  xTaskCreate(gyroCalibTask,"GyroCalib",TASK_STACK_GYRO_CALIB,NULL,TASK_PRIO_GYRO_CALIB,NULL);  // 陀螺零偏自校准任务(1.0s后自删)
  xTaskCreate(stkMonTask,"StkMon",TASK_STACK_MONITOR,NULL,TASK_PRIO_MONITOR,NULL);             // 任务栈监控(prio 2, 独立栈, 不会被饿死)
}

static void usmart_task(void* param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        usmart_scan();
        vTaskDelayUntil(&lastWakeTime, 20);        /*20ms周期延时*/
    }
}

/* 陀螺零偏自校准任务: 上电后 1.0s 采 200 帧, 校准通过后自删.
   优先级 1 低于 Control(prio 2) 高于 Idle(prio 0),
   不阻塞启动流程, 校准未完成期间 Yaw_Control 早返不产生差分. */
static void gyroCalibTask(void* param)
{
    (void)param;
    gyro_calibrateGyroZOffset();
    vTaskDelete(NULL);
}

/* 任务栈监控: 每 2 秒打印各任务剩余栈(words, 越小越危险).
   prio 2, 与其他任务同级, 独立栈 250 words — LOG_WARN 安全. */
static void stkMonTask(void* param)
{
    (void)param;
    TickType_t lastWake = xTaskGetTickCount();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(3000));  /* 等业务任务跑起来 */
    while(1) {
        LOG_WARN("STK A:%u U:%s N:%u S:%u R:%u C:%u M:%u",
            hAlarm   ? uxTaskGetStackHighWaterMark(hAlarm)  : 0,
            hUsmart  ? "on " : "off",
            hANO     ? uxTaskGetStackHighWaterMark(hANO)    : 0,
            hSend    ? uxTaskGetStackHighWaterMark(hSend)   : 0,
            hRecv    ? uxTaskGetStackHighWaterMark(hRecv)   : 0,
            hControl ? uxTaskGetStackHighWaterMark(hControl): 0,
            hMtf     ? uxTaskGetStackHighWaterMark(hMtf)    : 0);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));
    }
}
