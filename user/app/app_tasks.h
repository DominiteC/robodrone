/*
 * app_tasks.h
 * 声明 app 层任务创建入口，供启动编排模块调用。
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "FreeRTOS.h"
#include "task.h"

/* 创建业务 FreeRTOS 任务，任务参数在实现文件中集中维护。 */
void App_CreateTasks(void);

/* 各任务句柄, 供 stkMonTask 监控各任务栈剩余.
   这些变量定义在 app_tasks.c, xTaskCreate 时自动赋值. */
extern TaskHandle_t hAlarm;
extern TaskHandle_t hUsmart;
extern TaskHandle_t hANO;
extern TaskHandle_t hSend;
extern TaskHandle_t hRecv;
extern TaskHandle_t hControl;
extern TaskHandle_t hMtf;

#endif /* __APP_TASKS_H */
