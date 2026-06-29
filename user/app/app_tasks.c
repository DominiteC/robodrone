#include "app_tasks.h"
#include "app_config.h"
#include "alarm.h"
#include "ANO_DT.h"
#include "remotedata.h"
#include "wireless.h"
#include "control.h"
#include "mtf_01.h"
#include "watchdog_guard.h"
#include "usmart.h"

#include "FreeRTOS.h"
#include "task.h"

/* 任务资源配置集中放在这里，便于后续调整时核对栈大小和优先级。 */
#define TASK_STACK_ALARM      150
#define TASK_PRIO_ALARM       2
#define TASK_STACK_USMART     250
#define TASK_PRIO_USMART      2
#define TASK_STACK_ANO_DT     350
#define TASK_PRIO_ANO_DT      2
#define TASK_STACK_SEND       350
#define TASK_PRIO_SEND        2
#define TASK_STACK_RECEIVE    350
#define TASK_PRIO_RECEIVE     4
#define TASK_STACK_CONTROL    500
#define TASK_PRIO_CONTROL     2
#define TASK_STACK_MTF01      150
#define TASK_PRIO_MTF01       3
#define TASK_STACK_WDOG       180
#define TASK_PRIO_WDOG        5

static void usmart_task(void* param);

void App_CreateTasks(void)
{
  // Freertos任务创建
  xTaskCreate(Alarm_Update,"Alarm",TASK_STACK_ALARM,NULL,TASK_PRIO_ALARM,NULL);            // 报警处理任务，包括低电压检测，姿态检测
#if ENABLE_GCS_SERIAL
  #if USE_USMART_OR_ANO == 1
    xTaskCreate(usmart_task,"usmart",TASK_STACK_USMART,NULL,TASK_PRIO_USMART,NULL);            // usmart任务，在串口中调用函数，与ANO_DT_Data_Exchange任务互斥
  #elif USE_USMART_OR_ANO == 0
    xTaskCreate(ANO_DT_Data_Exchange,"ANO_DT",TASK_STACK_ANO_DT,NULL,TASK_PRIO_ANO_DT,NULL);   // 与地面站数据交换任务，通过串口发送数据，与usmart任务互斥
  #endif
#endif
  xTaskCreate(SendToRemote,"Send",TASK_STACK_SEND,NULL,TASK_PRIO_SEND,NULL);             // 发送遥控数据任务，通过无线模块发送数据
  xTaskCreate(Wireless_ReceiveTask,"Receive",TASK_STACK_RECEIVE,NULL,TASK_PRIO_RECEIVE,NULL);  // 接收遥控数据任务，通过无线模块接收数据
  xTaskCreate(Control_Task,"Control",TASK_STACK_CONTROL,NULL,TASK_PRIO_CONTROL,NULL);          // 控制任务，飞控核心任务，包括姿态解算，位置解算，控制算法等，控制电机驱动，电调和舵机。
  xTaskCreate(mtf_01_task,"mtf_01",TASK_STACK_MTF01,NULL,TASK_PRIO_MTF01,NULL);            // 光流模块任务
  xTaskCreate(WatchdogGuard_Task,"WDog",TASK_STACK_WDOG,NULL,TASK_PRIO_WDOG,NULL);       // 看门狗任务
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
