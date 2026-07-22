/*
 * app_system.c
 * 负责系统启动阶段的全局服务、设备和业务模块初始化编排。
 * 本文件只维护初始化顺序，不直接创建 FreeRTOS 任务。
 */
#include "app_system.h"
#include "app_config.h"
#include "usart_port.h"
#include "C_code_Log.h"
#include "globalTime.h"
#include "IT_Callback.h"
#include "usmart.h"
#include "bn220.h"
#include "gyro.h"
#include "bmp280.h"
#include "Mydelay.h"
#include "esc.h"
#include "servo.h"
#include "alarm.h"
#include "ANO_DT.h"
#include "motor.h"
#include "remotedata.h"
#include "control.h"
#include "position.h"
#include "actuator.h"
#include "watchdog_guard.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

USART_Data usartDebug;
uint8_t USART_buf[200];

static void App_InitGlobalServices(void);
static void App_InitDevices(void);
static void App_InitBusinessModules(void);

void App_InitSystemModules(void)
{
  App_InitGlobalServices();

  App_InitDevices();

  App_InitBusinessModules();

  WatchdogGuard_Init(300);

//  调试追踪，目前未启用，要使用比较麻烦
//  xTraceInitialize();
//  xTraceEnable(TRC_START);
//  xTraceEnable(TRC_START_FROM_HOST);

}

static void App_InitBusinessModules(void)
{
  position_init();  // 光流之类的进行初始化
  Control_Init();   // PID之类的进行初始化
  // 动力系统初始化
  Motor_Init();     // 电机驱动初始化
  vTaskDelay(500);
//  ServoInit();      // 舵机初始化
  Actuator_Init();  // 电杆初始化
  WatchdogGuard_ExitLongAction();
#if ENABLE_GCS_SERIAL && (USE_USMART_OR_ANO == 1)
  usmart_init(168);
#endif
}

static void App_InitGlobalServices(void)
{
  // 全局组件初始化
  c_code_log_init();
  Mydelay_Init();
  HAL_TIM_Base_Start_IT(&TIM_HANDLE_GLOBAL_TIME);
#if ENABLE_GCS_SERIAL
  #if USE_USMART_OR_ANO == 0
    USART_DataTypeInit(&usartDebug, &huart1, USART_buf, sizeof(USART_buf), 0, ANO_DT_CallBack);
  #else
    USART_DataTypeInit(&usartDebug, &huart1, USART_buf, sizeof(USART_buf), 0, NULL);
  #endif
#else
  /* 地面站关闭，但仍保留 USART1 + 日志系统可用（不做 log_disable） */
#endif
}

static void App_InitDevices(void)
{
	Alarm_Init();       // 报警处理初始化
//	ESC_Calibrate();  // 电调校准，新电调刚开始必须要校准，使用时请取消注释。:请在固定好的情况下使用，不可在无固定下使用校准
  ESC_Init();       // 电调初始化: 输出最小油门PWM, 防止磁校准期间电调滴答
  // 传感器初始化
  BN220_Init();       // GPS初始化
  gyro_init();        // 陀螺仪初始化
  sDrv_BMP280_Init(); // 气压计初始化
  RemoteData_Init();  // 无线通信初始化
}
