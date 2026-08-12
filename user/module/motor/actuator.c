#include "actuator.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 电动推杆初始化
 * 
 */
void Actuator_Init(void)
{
//    HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, GPIO_PIN_RESET);

//    vTaskDelay(15000);

    HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 启动电动推杆
 * 
 * @param actuator_num 需要启动的推杆编号(1~4)
 * @param state 推杆状态(true伸出，false缩回)
 */
void Actuator_Start(uint8_t actuator_num, bool state)
{
    switch (actuator_num)
    {
        case 1:
            HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

/**
 * @brief 停止电动推杆
 * 
 * @param actuator_num 需要停止的推杆编号(1~4)
 */
void Actuator_Stop(uint8_t actuator_num)
{
    switch (actuator_num)
    {
        case 1:
            HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

/**
 * @brief 设置电动推杆状态
 * 
 * @param actuator_num 推杆编号(1~4)
 * @param state 推杆状态(true伸出，false缩回)
 * @param delay_time 推杆动作持续时间(单位: ms)
 */
void Actuator_Set(uint8_t actuator_num, bool state, uint16_t delay_time)
{
    Actuator_Start(actuator_num, state);

    vTaskDelay(delay_time);

    Actuator_Stop(actuator_num);
}

/**
 * @brief 同时设置两个电动推杆状态
 * 
 * @param actuator_num1 推杆编号1(1~4)
 * @param actuator_num2 推杆编号2(1~4)
 * @param state 推杆状态(true伸出，false缩回)
 * @param delay_time 推杆动作持续时间(单位: ms)
 */
void Actuator_Set_2(uint8_t actuator_num1,uint8_t actuator_num2, bool state, uint16_t delay_time)
{
    Actuator_Start(actuator_num1, state);
    Actuator_Start(actuator_num2, state);

    vTaskDelay(delay_time);

    Actuator_Stop(actuator_num1);
	
    Actuator_Stop(actuator_num2);
}

/**
 * @brief 同时设置所有电动推杆状态
 * 
 * @param state 推杆状态(true伸出，false缩回)
 * @param delay_time 推杆动作持续时间(单位: ms)
 */
void Actuator_AllSet(bool state, uint16_t delay_time)
{
    HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);

    vTaskDelay(delay_time);

    HAL_GPIO_WritePin(ACTUATOR_1_1_Port, ACTUATOR_1_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_1_2_Port, ACTUATOR_1_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_2_1_Port, ACTUATOR_2_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_2_2_Port, ACTUATOR_2_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_3_1_Port, ACTUATOR_3_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_3_2_Port, ACTUATOR_3_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_4_1_Port, ACTUATOR_4_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR_4_2_Port, ACTUATOR_4_2_Pin, GPIO_PIN_RESET);
}