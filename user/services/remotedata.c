/**
 * remotedata.c — 飞控端遥控数据处理
 *
 * 架构说明（ACK Payload 模式）：
 *   飞控永远作为 PRX，不主动切 TX 模式发包。
 *   收到遥控器控制数据后，处理完毕即用 Wireless_LoadAckPayload()
 *   将最新遥测预加载到 ACK 队列中。
 *   下一次遥控器发包时，ACK 自动携带遥测数据返回。
 *
 *   同时有一个周期任务 TelemetryAckUpdateTask 持续刷新 ACK payload，
 *   确保即使没有新控制包，遥测数据也是新鲜的。
 */
#include "remotedata.h"
#include "wireless.h"
#include "alarm.h"
#include "vector_types.h"
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
uint8_t RemoteFlag = 0;
CtrlData RC_Control;

uint8_t battery_switch_flag = 0;
static TickType_t s_last_atti_cmd_tick = 0;

void ButtonCommand(CmdData *cmd);
void RemoteData_RecieveHandler(uint8_t data[], uint8_t len);
static void UpdateAckTelemetry(void);

void RemoteData_Init(void)
{
    Wireless_Init();
    Wireless_SetReceiveCallback(RemoteData_RecieveHandler);

    /* 预加载初始 ACK payload（全零遥测），确保首个 ACK 不为空 */
    uint8_t init_telemetry[29] = {0};
    init_telemetry[0] = 0xFF;
    init_telemetry[1] = 0xAA;
    Wireless_LoadAckPayload(init_telemetry, sizeof(init_telemetry));
}

/**
 * @brief 接收遥控器控制数据处理回调
 * @note  由 Wireless_ReceiveAnalysis 在 RX_DR 中断上下文中调用。
 *        处理完毕后立即刷新 ACK payload，保证下一次 ACK 携带最新遥测。
 */
void RemoteData_RecieveHandler(uint8_t data[], uint8_t len)
{
    if (len <= 2 || data[0] != 0xAA || data[1] != 0xFF)
    {
        return;
    }
    if (data[2] == 0x01 && len >= 13)   /* 控制数据帧 */
    {
        RC_Control.mode = data[3];
        RC_Control.angle.pitch = ((int16_t)((data[5]  << 8) | data[6]))  / 100.0;
        RC_Control.angle.roll  = ((int16_t)((data[7]  << 8) | data[8]))  / 100.0;
        RC_Control.angle.yaw   = ((int16_t)((data[9]  << 8) | data[10])) / 100.0;
        RC_Control.throttle    = ((int16_t)((data[11] << 8) | data[12])) / 100.0;

        setCommanderCtrlMode(data[3]);
        RemoteFlag = 1;

        /* 收到控制包后立即刷新 ACK payload（回传最新遥测） */
        UpdateAckTelemetry();
    }
    if (data[2] == 0x02 && len >= 5)    /* 命令帧 */
    {
        CmdData cmd = {0};
        cmd.cmd = data[3];
        cmd.param_num = data[4];
        for (uint8_t i = 0; i < cmd.param_num && i < 5; i++)
        {
            cmd.params[i] = data[5 + i];
        }
        ButtonCommand(&cmd);

        /* 收到命令后立即刷新 ACK payload（确认收到） */
        UpdateAckTelemetry();
    }
}

/**
 * @brief 构建遥测帧并加载到 ACK payload
 * @note  帧格式：[0]=0xFF, [1]=0xAA, [2..28]=遥测数据（29 字节）
 *        与遥控器 remotestate.c ParseTelemetry 解析格式严格对齐
 */
static void UpdateAckTelemetry(void)
{
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

    SI24R1_TX_DATA[0] = 0xFF;   /* 帧头 */
    SI24R1_TX_DATA[1] = 0xAA;

    temp = (int)(RC_Control.throttle * 100);
    SI24R1_TX_DATA[2]  = Byte1(temp);
    SI24R1_TX_DATA[3]  = Byte0(temp);
    temp = (int)(getYawMeasCont() * 100);
    SI24R1_TX_DATA[4]  = Byte1(temp);
    SI24R1_TX_DATA[5]  = Byte0(temp);
    temp = (int)(Att_Angle.pitch * 100);
    SI24R1_TX_DATA[6]  = Byte1(temp);
    SI24R1_TX_DATA[7]  = Byte0(temp);
    temp = (int)(Att_Angle.roll * 100);
    SI24R1_TX_DATA[8]  = Byte1(temp);
    SI24R1_TX_DATA[9]  = Byte0(temp);
    temp = (int)(height * 100);
    SI24R1_TX_DATA[10] = Byte1(temp);
    SI24R1_TX_DATA[11] = Byte0(temp);
    temp = (int)(battery_voltage * 100);
    SI24R1_TX_DATA[12] = Byte1(temp);
    SI24R1_TX_DATA[13] = Byte0(temp);
    temp = (int)(battery_current * 100);
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

    Wireless_LoadAckPayload(SI24R1_TX_DATA, sizeof(SI24R1_TX_DATA));
}

/**
 * @brief 获取接收到的遥控器控制数据
 */
uint8_t RemoteData_GetData(CtrlData *rc_out)
{
    uint8_t status = ctrlDataUpdate(RemoteFlag);
    if (status == 0)
    {
        *rc_out = RC_Control;
        RemoteFlag = 0;
        return 1;
    }
    else if (status == 1)
    {
        return 0;
    }
    else if (status == 2)
    {
        return 2;
    }
    else
    {
        return 0;
    }

    return 0;
}

/* ──── 按键命令处理 ──── */

void ButtonCommand(CmdData *cmd)
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

        if (getCommanderKeyFlight() != true)
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
        Alarm_SetBatteryToggle();
        battery_switch_flag = 1;
        LOG_INFO_IT("收到电池开关命令");
    }
    else if (cmd->cmd >= CMD_ACTUATOR_ALL &&
             cmd->cmd <= CMD_ACTUATOR_RIGHT_BACK)
    {
        uint8_t actuator_index[4];
        actuator_index[0] = LEFT_FRONT;
        actuator_index[1] = RIGHT_FRONT;
        actuator_index[2] = LEFT_BACK;
        actuator_index[3] = RIGHT_BACK;
        if (cmd->cmd == CMD_ACTUATOR_ALL && cmd->param_num >= 1)
        {
            for (uint8_t i = 0; i < cmd->param_num; i++)
            {
                if (cmd->params[i] == 0)
                    Actuator_Stop(actuator_index[i]);
                else if (cmd->params[i] == 1)
                    Actuator_Start(actuator_index[i], true);
                else if (cmd->params[i] == 2)
                    Actuator_Start(actuator_index[i], false);
            }
            LOG_INFO_IT("收到所有电杆控制命令，状态: %d %d %d %d",
                        cmd->params[0], cmd->params[1],
                        cmd->params[2], cmd->params[3]);
            return;
        }
        else
        {
            uint8_t index = cmd->cmd - CMD_ACTUATOR_LEFT_FRONT;
            if (cmd->param_num >= 1)
            {
                if (cmd->params[0] == 0)
                    Actuator_Stop(actuator_index[index]);
                else if (cmd->params[0] == 1)
                    Actuator_Start(actuator_index[index], true);
                else if (cmd->params[0] == 2)
                    Actuator_Start(actuator_index[index], false);
                LOG_INFO_IT("收到电杆%d控制命令,状态: %d",
                            index + 1, cmd->params[0]);
            }
        }
    }
}

/* ──── ACK 遥测周期刷新任务 ──── */

/**
 * @brief 周期刷新 ACK payload 中的遥测数据
 * @note  每 20ms 更新一次 ACK payload，确保即使遥控器不发控制包，
 *        飞控 ACK 也始终携带最新遥测。
 *        此任务替换了原来独立 TX 发送遥测的 SendToRemote 任务。
 */
void SendToRemote(void *param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /* 禁用 EXTI 防止高优先级的 Receive 任务抢占 SPI 总线 */
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        UpdateAckTelemetry();
        HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
    }
}
