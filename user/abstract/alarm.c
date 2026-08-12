#include "alarm.h"
#include "Mydelay.h"
#include "adc.h"
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wireless.h"

AlarmMode current_mode = ALARM_MODE_LOCKED;
float battery_voltage = 22.2; // 初始电池电压
float battery_current = 0.0;    // 初始电池电流
uint8_t alarm_flag = 0;
static TickType_t error_mode_start_tick = 0;
static bool error_buzzer_muted = false;

uint16_t adc_value[2];

/**
 * @brief 警报初始化
 * 
 */
void Alarm_Init(void)
{
    // HAL_ADC_Start(&hadc3);
    // while (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(adc_value[0]));
    while((HAL_ADC_GetState(&hadc3) & HAL_ADC_STATE_READY) != HAL_ADC_STATE_READY);
    // HAL_GPIO_WritePin(BATTERY_SWITCH_GPIO_PORT, BATTERY_SWITCH_GPIO_PIN, GPIO_PIN_RESET);
    Alarm_SetBatterySwitch(true);
    LOG_INFO("初始电池电压: %.2f V,初始电流:%.2f A", GET_BATTERY_VOLTAGE(adc_value),GET_BATTERY_CURRENT(adc_value));
}

/**
 * @brief 设置警报模式
 * 
 * @param mode 需要设置的模式
 */
void Alarm_SetMode(AlarmMode mode)
{
    current_mode = mode;
    alarm_flag = 0;
    if (mode == ALARM_MODE_ERROR)
    {
        error_mode_start_tick = xTaskGetTickCount();
        error_buzzer_muted = false;
    }
    else
    {
        error_mode_start_tick = 0;
        error_buzzer_muted = false;
    }
}

static void Alarm_CheckBattery(void)
{
    static uint8_t count=0;
    if (myDelay((uint32_t)Alarm_CheckBattery, 1000))
    {
        // HAL_ADC_Start(&hadc3);
        // while (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK);
        HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(adc_value[0]));
        while((HAL_ADC_GetState(&hadc3) & HAL_ADC_STATE_READY) != HAL_ADC_STATE_READY);
        battery_voltage = GET_BATTERY_VOLTAGE(adc_value);
        battery_current = GET_BATTERY_CURRENT(adc_value);
        // LOG_DEBUG("电池电压: %.2f V,电流:%.2f A", battery_voltage,battery_current);
        if (current_mode != ALARM_MODE_LOW_BATTERY && battery_voltage < 22.0)
        {
            if(count >= 2)
            {
                Alarm_SetMode(ALARM_MODE_LOW_BATTERY);
            }
            else
            {
                count ++;
            }
        }
        else if (current_mode == ALARM_MODE_LOW_BATTERY && battery_voltage >= 22.2)
        {
            if(count >= 2)
            {
                Alarm_SetMode(ALARM_MODE_LOCKED);
            }
            else
            {
                count ++;
            }
        }
        else {
            count =0;
        }
    }
}

float Alarm_GetBatteryVoltage(void)
{
    return battery_voltage;
}

float Alarm_GetBatteryCurrent(void)
{
    return battery_current;
}

void Alarm_SetBatterySwitch(bool state)
{
    HAL_GPIO_WritePin(BATTERY_SWITCH_GPIO_PORT, BATTERY_SWITCH_GPIO_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Alarm_SetBatteryToggle(void)
{
    HAL_GPIO_TogglePin(BATTERY_SWITCH_GPIO_PORT, BATTERY_SWITCH_GPIO_PIN);
}

/**
 * @brief 警报更新
 * 
 */
void Alarm_Update(void *param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        extern uint8_t battery_switch_flag;

        if (battery_switch_flag)
        {
            battery_switch_flag = 0;
            HAL_Delay(100); // 等待继电器动作完成
            L01_Init();
            Wireless_SwitchToRx();
        }

        Alarm_CheckBattery();
        switch (current_mode)
        {
        case ALARM_MODE_LOCKED:
            if (!alarm_flag)
            {
                LED_1_OFF();
                LED_2_ON();
                LED_3_OFF();
                BUZZER_OFF();
                alarm_flag = 1;
            }
            break;

        case ALARM_MODE_UNLOCKED:
            if (!alarm_flag)
            {
                LED_1_ON();
                LED_2_OFF();
                LED_3_OFF();
                BUZZER_ON();
                alarm_flag = 1;
            }
            break;

        case ALARM_MODE_LOW_BATTERY:
            if (alarm_flag == 0 && myDelay((uint32_t)Alarm_Update,500))
            {
                LED_1_OFF();
                LED_2_OFF();
                LED_3_OFF();
                BUZZER_OFF();
                alarm_flag = 1;
            }
            else if (alarm_flag == 1 && myDelay((uint32_t)Alarm_Update,500))
            {
                LED_1_OFF();
                LED_2_OFF();
                LED_3_ON();
                BUZZER_ON();
                alarm_flag = 0;
            }
            break;

        case ALARM_MODE_ERROR:
            if (!alarm_flag)
            {
                LED_1_ON();
                LED_2_ON();
                LED_3_ON();
                BUZZER_ON();
                alarm_flag = 1;
            }
            if (!error_buzzer_muted)
            {
                TickType_t now_tick = xTaskGetTickCount();
                if ((now_tick - error_mode_start_tick) >= pdMS_TO_TICKS(3000))
                {
                    BUZZER_OFF();
                    error_buzzer_muted = true;
                }
            }
            break;

        default:
            break;
        }

        vTaskDelayUntil(&lastWakeTime, 100);		/*20ms周期延时*/
    }

}


void LED_Test(void)
{
    HAL_GPIO_TogglePin(LED_1_GPIO_PORT, LED_1_GPIO_PIN);
    HAL_Delay(1500);
    HAL_GPIO_TogglePin(LED_2_GPIO_PORT, LED_2_GPIO_PIN);
    HAL_Delay(1500);
    HAL_GPIO_TogglePin(LED_3_GPIO_PORT, LED_3_GPIO_PIN);
    HAL_Delay(1500);
    HAL_GPIO_TogglePin(LED_1_GPIO_PORT, LED_1_GPIO_PIN);
    HAL_Delay(1500);
    HAL_GPIO_TogglePin(LED_2_GPIO_PORT, LED_2_GPIO_PIN);
    HAL_Delay(1500);
    HAL_GPIO_TogglePin(LED_3_GPIO_PORT, LED_3_GPIO_PIN);
    HAL_Delay(1500);
}
