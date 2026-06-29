/*
 * app_tasks.h
 * 声明 app 层任务创建入口，供启动编排模块调用。
 */
#ifndef __APP_TASKS_H
#define __APP_TASKS_H

/* 创建业务 FreeRTOS 任务，任务参数在实现文件中集中维护。 */
void App_CreateTasks(void);

#endif /* __APP_TASKS_H */
