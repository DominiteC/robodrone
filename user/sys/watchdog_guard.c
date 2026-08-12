#include "watchdog_guard.h"
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

static volatile uint32_t control_heartbeat_tick = 0;
static volatile bool wd_inited = false;
static volatile uint32_t long_action_deadline_tick = 0;

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
    if (wd_inited)
    {
        return;
    }

    /* LSI ~= 32kHz, prescaler=64 => 500Hz counter clock */
    uint32_t reload = (uint32_t)((timeout_ms * 500U) / 1000U);
    if (reload > 4095U)
    {
        reload = 4095U;
    }
    if (reload < 1U)
    {
        reload = 1U;
    }

    /* Enable write access */
    IWDG->KR = 0x5555U;
    /* Prescaler = 64 */
    IWDG->PR = 0x04U;
    /* Reload value */
    IWDG->RLR = (reload & 0x0FFFU);
    /* Reload counter */
    IWDG->KR = 0xAAAAU;
    /* Start IWDG */
    IWDG->KR = 0xCCCCU;

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
    long_action_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(allow_ms);
}

void WatchdogGuard_ExitLongAction(void)
{
    long_action_deadline_tick = 0;
}

void WatchdogGuard_Task(void *param)
{
    (void)param;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (wd_inited)
        {
            uint32_t now = xTaskGetTickCount();
            uint32_t delta = now - control_heartbeat_tick;
            bool in_long_action = (long_action_deadline_tick != 0) &&
                                  ((int32_t)(long_action_deadline_tick - now) > 0);

            /* Control loop is expected at 1ms; allow transient jitter */
            if (in_long_action || delta <= 50U)
            {
                WatchdogGuard_FeedNow();
            }
            else
            {
                LOG_ERROR("control heartbeat timeout=%lums, stop feeding IWDG", delta);
            }
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
    }
}
