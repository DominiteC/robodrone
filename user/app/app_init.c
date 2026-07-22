#include "app_init.h"
#include "app_system.h"
#include "app_tasks.h"
#include "C_code_Log.h"

#include "FreeRTOS.h"
#include "task.h"

static void App_LogStartupComplete(void)
{
  LOG_INFO("Free heap: %d bytes", xPortGetFreeHeapSize());      /*打印剩余堆栈大小*/
  LOG_INFO("你好世界");
}

/*
 * app 层初始化任务负责启动编排，并在完成后删除自身。
 * 系统初始化、任务创建和启动诊断均在 app 层启动流程中完成。
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
