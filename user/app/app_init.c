#include "app_init.h"
#include "app_config.h"
#include "usart_port.h"
#include "C_code_Log.h"
#include "globalTime.h"
#include "IT_Callback.h"
#include "mtf_01.h"
#include "usmart.h"
#include "bn220.h"
#include "gyro.h"
#include "bmp280.h"
#include "wireless.h"
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

USART_Data usartDebug;
uint8_t USART_buf[200];
static void App_InitGlobalServices(void);
static void App_InitDevices(void);
static void App_InitBusinessModules(void);
static void usmart_task(void* param);

/*
 * app 层初始化任务负责启动编排，并在完成后删除自身。
 * 底层初始化和任务创建实现已经迁入 app 层，启动入口统一使用 App_* 接口。
 */
void App_Init(void)
{
  App_InitSystemModules();
}

void App_StartTasks(void)
{
  // Freertos任务创建
  xTaskCreate(Alarm_Update,"Alarm",TASK_STACK_ALARM,NULL,TASK_PRIO_ALARM,NULL);            // 报警处理任务，包括低电压检测，姿态检测
#if ENABLE_GCS_SERIAL
  #if USE_USMART_OR_ANO == 1
    xTaskCreate(usmart_task,"usmart",TASK_STACK_USMART,NULL,TASK_PRIO_USMART,NULL);            // usmart任务，在串口中调用函数，与ANO_DT_Data_Exchange任务互斥
  #elif USE_USMART_OR_ANO == 0
    xTaskCreate(ANO_DT_Data_Exchange,"ANO_DT",TASK_STACK_ANO_DT,NULL,TASK_PRIO_ANO_DT,NULL);   // 与地面站数据交换任务(通过串口发送数据)，与usmart任务互斥
  #endif
#endif
  xTaskCreate(SendToRemote,"Send",TASK_STACK_SEND,NULL,TASK_PRIO_SEND,NULL);             // 发送遥控数据任务，通过无线模块发送数据
  xTaskCreate(Wireless_ReceiveTask,"Receive",TASK_STACK_RECEIVE,NULL,TASK_PRIO_RECEIVE,NULL);  // 接收遥控数据任务，通过无线模块接收数据
  xTaskCreate(Control_Task,"Control",TASK_STACK_CONTROL,NULL,TASK_PRIO_CONTROL,NULL);          // 控制任务，飞控核心任务，包括姿态解算，位置解算，控制算法等，控制电机驱动，电调和舵机。
  xTaskCreate(mtf_01_task,"mtf_01",TASK_STACK_MTF01,NULL,TASK_PRIO_MTF01,NULL);            // 光流模块任务
  xTaskCreate(WatchdogGuard_Task,"WDog",TASK_STACK_WDOG,NULL,TASK_PRIO_WDOG,NULL);       // 看门狗任务
}

void App_InitTask(void *param)
{
  (void)param;
  App_Init();
  App_StartTasks();
  App_LogStartupComplete();
  vTaskDelete(NULL);
}


void App_InitSystemModules(void)
{
  if (WatchdogGuard_WasIwdgReset())
  {
    LOG_WARN("boot after IWDG reset");
  }

  App_InitGlobalServices();

  App_InitDevices();

  App_InitBusinessModules();

//  调试追踪，目前未启用，要使用比较麻烦
//  xTraceInitialize();
//  xTraceEnable(TRC_START);
//  xTraceEnable(TRC_START_FROM_HOST);
  // WatchdogGuard_Init(300);
}


static void App_InitBusinessModules(void)
{
  position_init();  // 光流之类的进行初始化
  Control_Init();   // PID之类的进行初始化
  // 动力系统初始化
  //ESC_Calibrate();  // 电调校准，新电调刚开始必须要校准，使用时请取消注释。:请在固定好的情况下使用，不可在无固定下使用校准
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

#if !ENABLE_GCS_SERIAL
  // 完全禁用地面站串口时，同步关闭日志串口，避免USART1发送占用DMA/中断
  log_disable();
#endif
  Mydelay_Init();
  HAL_TIM_Base_Start_IT(&TIM_HANDLE_GLOBAL_TIME);
  // ANO_DT_Init();
#if ENABLE_GCS_SERIAL
  #if USE_USMART_OR_ANO == 0
    USART_DataTypeInit(&usartDebug, &huart1, USART_buf, sizeof(USART_buf), 0, ANO_DT_CallBack);
  #else
    USART_DataTypeInit(&usartDebug, &huart1, USART_buf, sizeof(USART_buf), 0, NULL);
  #endif
#else
  // 彻底静默USART1，防止对无线链路造成中断/DMA层面的干扰
  HAL_UART_DMAStop(&huart1);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn); // USART1_RX DMA
  HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn); // USART1_TX DMA
  HAL_UART_DeInit(&huart1);
#endif
}

static void App_InitDevices(void)
{
	Alarm_Init();       // 报警处理初始化
  ESC_Init();       // 电调初始化: 输出最小油门PWM, 防止磁校准期间电调滴答
  // 传感器初始化
  BN220_Init();       // GPS初始化
  gyro_init();        // 陀螺仪初始化
  sDrv_BMP280_Init(); // 气压计初始化
  RemoteData_Init();  // 无线通信初始化
}


void App_LogStartupComplete(void)
{
  LOG_INFO("Free heap: %d bytes", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/
  LOG_INFO("你好世界");
}


static void usmart_task(void* param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        usmart_scan();
        vTaskDelayUntil(&lastWakeTime, 20);		/*20ms周期延时*/
    }
}
