#include "remotedata.h"
#include "wireless.h"
#include "alarm.h"
#include "structConfig.h"
#include "gyro.h"
#include "IT_Callback.h"
#include "position.h"
#include "commander.h"
#include "C_code_Log.h"
#include "actuator.h"
#include "control.h"
#include "Mydelay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

uint8_t RC_ID = 255;
uint8_t RemoteFlag = 0; // 遥控器数据接收标志位
CtrlData RC_Control;

QueueHandle_t remote_send_semaph;

uint8_t battery_switch_flag = 0;
static TickType_t s_last_atti_cmd_tick = 0;

void ButtonCommand(CmdData* cmd);
void RemoteData_RecieveHandler(uint8_t data[], uint8_t len);

void RemoteData_Init(void)
{
    Wireless_Init();
    Wireless_SetReceiveCallback(RemoteData_RecieveHandler);
    remote_send_semaph = xSemaphoreCreateBinary();
}

/**
 * @brief 无线数据处理
 * 
 * @param data 需要处理的数据
 * @param len 数据长度
 */
void RemoteData_RecieveHandler(uint8_t data[], uint8_t len)
{
    // 遥控器数据处理
    // 解析data数组，更新控制命令
    if (len <= 2 || data[0] != 0xAA || data[1] != 0xFF)
    {
        return; // 数据帧头错误，丢弃数据
    }
    if (data[2] == 0x01 && len >= 13) // 检查数据类型和长度
    {
        RC_Control.mode = data[3];
        // 高字节在前,低字节在后
        RC_Control.angle.pitch     = ((int16_t)((data[5] << 8) | data[6]))/100.0;
        RC_Control.angle.roll    = ((int16_t)((data[7] << 8) | data[8]))/100.0;
        RC_Control.angle.yaw = ((int16_t)((data[9] << 8) | data[10]))/100.0;
        RC_Control.throttle      = ((int16_t)((data[11] << 8) | data[12]))/100.0;

        setCommanderCtrlMode(data[3]);
        xSemaphoreGive(remote_send_semaph);
        RemoteFlag = 1;
        // LOG_INFO_IT("pitch=%.2f,roll=%.2f,thr=%.2f,yaw=%.2f",RC_Control.angle.pitch,RC_Control.angle.roll,RC_Control.throttle,RC_Control.angle.yaw);
    }
    if (data[2] == 0x02 && len >= 5) // 检查数据类型和长度
    {
        CmdData cmd = {0};
        cmd.cmd = data[3];
        cmd.param_num = data[4];
        for (uint8_t i = 0; i < cmd.param_num && i < 5; i++)
        {
            cmd.params[i] = data[5 + i];
        }
        ButtonCommand(&cmd);
        xSemaphoreGive(remote_send_semaph);
    }

}

/**
 * @brief 获取接收到的数据
 * 
 * @param rc_out 存在数据的指针
 * @return uint8_t 1=有新数据 0=无新数据 2,使用构造的假数据 
 */
uint8_t RemoteData_GetData(CtrlData* rc_out)
{
    uint8_t status = ctrlDataUpdate(RemoteFlag);
    if (status == 0)
    {
        *rc_out = RC_Control;
        RemoteFlag = 0; // 清除标志位
        return 1; // 返回1表示有新数据
    }
    else if(status == 1)
    {
        return 0;
    }
    else if(status == 2)
    {
        return 2;
    }
    else{
        return 0;
    }
        
    return 0; // 返回0表示无新数据
}

/**
 * @brief 按键命令处理
 * 
 * @param cmd 获取到的按键命令
 */
void ButtonCommand(CmdData* cmd)
{
    if (cmd->cmd == CMD_CHANGE_CTRL_MODE)
    {
        if (getCommanderCtrlMode() == MODE_HEIGHT)
        {
            setCommanderCtrlMode(MODE_MANUAL);
        }
        else if (getCommanderCtrlMode() == MODE_MANUAL)
        {
            setCommanderCtrlMode(MODE_THREEHOLD);
        }
        else
        {
            setCommanderCtrlMode(MODE_HEIGHT);
        }
    }
    else if (cmd->cmd == CMD_CHANGE_ATTI_LAND)
    {
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_atti_cmd_tick) < pdMS_TO_TICKS(300))
        {
            return;
        }
        s_last_atti_cmd_tick = now;

        if (cmd->param_num < 1)
        {
            return;
        }

        if (cmd->params[0] == MODE_AIRPLANE)
        {
            setCommanderAttitudeMode(MODE_AIRPLANE);
            return;
        }

        if (cmd->params[0] == MODE_WALK || cmd->params[0] == MODE_WALK_45)
        {
            /* Remote UI is two-state; map WALK_45 requests to WALK for compatibility. */
            setCommanderAttitudeMode(MODE_WALK);
        }
    }
    else if (cmd->cmd == CMD_FLIGHT_LAND)
    {
        if (getCommanderSafetyLatched())
        {
            LOG_WARN_IT("safety latched, ignore CMD_FLIGHT_LAND");
            setCommanderKeyFlight(false);
            setCommanderKeyland(false);
            return;
        }

        if (getCommanderKeyland())
        {
            LOG_INFO_IT("landing in progress, ignore CMD_FLIGHT_LAND");
            return;
        }

        if(getCommanderKeyFlight() != true)
        {
            setCommanderKeyFlight(true);
            setCommanderKeyland(false);
            LOG_INFO_IT("设置为起飞形态");
        }
        else
        {
            setCommanderKeyland(true);
            setCommanderKeyFlight(false);
            LOG_INFO_IT("设置为降落形态");
        }
    }
    else if (cmd->cmd == CMD_BATTERY_SWITCH)
    {
        // 电池开关处理逻辑
        Alarm_SetBatteryToggle();
        battery_switch_flag = 1;
        LOG_INFO_IT("收到电池开关命令");
    }
    else if (cmd->cmd >= CMD_ACTUATOR_ALL && cmd->cmd <= CMD_ACTUATOR_RIGHT_BACK)
    {
        /*
            param: 0: 关闭 1: 正转 2: 反转 
        */
        uint8_t actuator_index[4];
        actuator_index[0] = LEFT_FRONT;
        actuator_index[1] = RIGHT_FRONT;
        actuator_index[2] = LEFT_BACK;
        actuator_index[3] = RIGHT_BACK;
        if (cmd->cmd == CMD_ACTUATOR_ALL && cmd->param_num >= 1)
        {
            // 控制所有电杆
            for (uint8_t i = 0; i < cmd->param_num; i++)
            {
                if (cmd->params[i] == 0)
                {
                    Actuator_Stop(actuator_index[i]);
                }
                else if (cmd->params[i] == 1)
                {
                    Actuator_Start(actuator_index[i], true);
                }
                else if (cmd->params[i] == 2)
                {
                    Actuator_Start(actuator_index[i], false);
                }

            }

            LOG_INFO_IT("收到所有电杆控制命令，状态: %d %d %d %d", cmd->params[0], cmd->params[1], cmd->params[2], cmd->params[3]);
            return;
        }
        else
        {
            uint8_t index = cmd->cmd - CMD_ACTUATOR_LEFT_FRONT;
            if (cmd->param_num >= 1)
            {
                if (cmd->params[0] == 0)
                {
                    Actuator_Stop(actuator_index[index]);
                }
                else if (cmd->params[0] == 1)
                {
                    Actuator_Start(actuator_index[index], true);
                }
                else if (cmd->params[0] == 2)
                {
                    Actuator_Start(actuator_index[index], false);
                }
                LOG_INFO_IT("收到电杆%d控制命令,状态: %d", index + 1, cmd->params[0]);
            }
        }

    }
}

/**
 * @brief 向遥控器发送无线数据
 * 
 * @param param 无
 */
void SendToRemote(void *param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    static uint8_t err_count;
    while(1)
    {
        xSemaphoreTake(remote_send_semaph, pdMS_TO_TICKS(20));
        int16_t temp;
        uint8_t SI24R1_TX_DATA[29] = {0};

        float_angle Att_Angle;
        float height;
        float battery_voltage;
        float battery_current;
        gyro_getAngle(&Att_Angle);
        position_GetHeight(&height);
        battery_voltage = Alarm_GetBatteryVoltage();
        battery_current = Alarm_GetBatteryCurrent();

        SI24R1_TX_DATA[0] = 0xFF;//帧头
        
        SI24R1_TX_DATA[1] = 0xAA; //标志位组
        
        // 高字节在前,低字节在后
        temp = (int)(RC_Control.throttle*100); //油门
        SI24R1_TX_DATA[2] = Byte1(temp);
        SI24R1_TX_DATA[3] = Byte0(temp);
        temp = (int)(Att_Angle.yaw*100); //航向
        SI24R1_TX_DATA[4] = Byte1(temp);
        SI24R1_TX_DATA[5] = Byte0(temp);
        temp = (int)(Att_Angle.pitch*100); //俯仰
        SI24R1_TX_DATA[6] = Byte1(temp);
        SI24R1_TX_DATA[7] = Byte0(temp);
        temp = (int)(Att_Angle.roll*100); //横滚
        SI24R1_TX_DATA[8] = Byte1(temp);
        SI24R1_TX_DATA[9] = Byte0(temp);
        temp = (int)(height*100);   //高度留待
        SI24R1_TX_DATA[10] = Byte1(temp);
        SI24R1_TX_DATA[11] = Byte0(temp);
        temp = (int)(battery_voltage*100); //飞机电池电压
        SI24R1_TX_DATA[12] = Byte1(temp);
        SI24R1_TX_DATA[13] = Byte0(temp);
        temp = (int)(battery_current*100); //飞机电池电流
        SI24R1_TX_DATA[14] = Byte1(temp);
        SI24R1_TX_DATA[15] = Byte0(temp);
        SI24R1_TX_DATA[16] = getCommanderCtrlMode();
        SI24R1_TX_DATA[17] = getCommanderAttitudeMode();
        SI24R1_TX_DATA[18] = getCommanderKeyFlight() ? 1 : 0;

        float thrustCmd = getThrustCmd();
        temp = (int16_t)(thrustCmd * 100);
        SI24R1_TX_DATA[19] = Byte1(temp);
        SI24R1_TX_DATA[20] = Byte0(temp);
        temp = (int16_t)(target.height * 100);
        SI24R1_TX_DATA[21] = Byte1(temp);
        SI24R1_TX_DATA[22] = Byte0(temp);
        temp = (int16_t)(debug_target_angle_pitch * 100);
        SI24R1_TX_DATA[23] = Byte1(temp);
        SI24R1_TX_DATA[24] = Byte0(temp);
        temp = (int16_t)(debug_target_angle_roll * 100);
        SI24R1_TX_DATA[25] = Byte1(temp);
        SI24R1_TX_DATA[26] = Byte0(temp);
        temp = (int16_t)(debug_target_angle_yaw * 100);
        SI24R1_TX_DATA[27] = Byte1(temp);
        SI24R1_TX_DATA[28] = Byte0(temp);

        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        uint8_t err = Wireless_TransmitHandler(SI24R1_TX_DATA,sizeof(SI24R1_TX_DATA));
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        if (err)
        {
            if (myDelay((uint32_t)SendToRemote, 500))
            {
                LOG_WARN("wireless tx timeout");
            }
            err_count++;
            if (err_count >= 10)
            {
                LOG_ERROR("一直断连,执行延时大法");
                vTaskDelay(20);
                err_count = 0;
            }
        }
        else
        {
            err_count = 0;
        }
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));		/*20ms周期发送心跳*/
    }
}
