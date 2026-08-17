#include "watchdog_guard.h"
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

//-------------------------看门狗守护状态-----------------------------------
static volatile uint32_t control_heartbeat_tick = 0;  /* 控制任务心跳时间戳 (ms), 喂狗参考 */
static volatile bool     wd_inited             = false; /* 看门狗是否已完成初始化 */
//-------------------------看门狗守护状态-----------------------------------

bool WatchdogGuard_WasIwdgReset(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        return true;
    }
    return false;
}

void WatchdogGuard_Init(uint32_t timeout_ms)
{
    if (wd_inited) return;

    /* LSI ~= 32kHz, prescaler=64 => 500Hz counter clock */
    uint32_t reload = (uint32_t)((timeout_ms * 500U) / 1000U);
    if (reload > 4095U) reload = 4095U;
    if (reload < 1U)    reload = 1U;

    IWDG->KR  = 0x5555U;
    IWDG->PR  = 0x04U;
    IWDG->RLR = (reload & 0x0FFFU);
    IWDG->KR  = 0xAAAAU;
    IWDG->KR  = 0xCCCCU;

    control_heartbeat_tick = xTaskGetTickCount();
    wd_inited = true;
}

void WatchdogGuard_ControlHeartbeat(void)
{
    control_heartbeat_tick = xTaskGetTickCount();
}

void WatchdogGuard_FeedNow(void)
{
    IWDG->KR = 0xAAAAU;
}

void WatchdogGuard_EnterLongAction(uint32_t allow_ms)
{
    (void)allow_ms;
}

void WatchdogGuard_ExitLongAction(void)
{
}

/*
 * vApplicationIdleHook — 借鉴 MiniFly 设计.
 * 只要 RTOS 调度器正常, 空闲钩子定期喂狗.
 * 任何任务卡死 → 空闲钩子停 → IWDG 复位.
 */
void vApplicationIdleHook(void)
{
    static uint32_t tickLastFeed = 0;

    if (!wd_inited) return;

    uint32_t now = xTaskGetTickCount();

    /* 每 150ms 喂一次狗, 避免不必要的 IWDG 寄存器写入 */
    if ((now - tickLastFeed) >= pdMS_TO_TICKS(150))
    {
        tickLastFeed = now;
        WatchdogGuard_FeedNow();
    }
}
