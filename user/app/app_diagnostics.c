/*
 * app_diagnostics.c
 * 负责启动完成后的基础诊断日志输出。
 */
#include "app_diagnostics.h"
#include "C_code_Log.h"

#include "FreeRTOS.h"
#include "task.h"

void App_LogStartupComplete(void)
{
  LOG_INFO("Free heap: %d bytes", xPortGetFreeHeapSize());      /*打印剩余堆栈大小*/
  LOG_INFO("你好世界");
}
