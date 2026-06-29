#include "app_init.h"
#include "app_system.h"
#include "app_tasks.h"
#include "app_diagnostics.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * app 层初始化任务负责启动编排，并在完成后删除自身。
 * 系统初始化、任务创建和启动诊断分别由 app 内部分模块负责。
 */
void App_Init(void)
{
  App_InitSystemModules();
}

void App_StartTasks(void)
{
  App_CreateTasks();
}

void App_InitTask(void *param)
{
  (void)param;
  App_Init();
  App_StartTasks();
  App_LogStartupComplete();
  vTaskDelete(NULL);
}
