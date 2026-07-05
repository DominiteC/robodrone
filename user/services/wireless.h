/*
 * wireless.h
 * 声明无线链路服务接口，供遥控数据服务和 app 任务创建模块调用。
 *
 * 架构说明（ACK Payload 模式）：
 *   飞控固定为 PRX (RX mode)，不主动 TX。
 *   遥测数据通过 Wireless_LoadAckPayload() 预加载，
 *   在遥控器下次发包时自动随 ACK 回传。
 */
#ifndef __WIRELESS_H_
#define __WIRELESS_H_

#include "nRF24L01P.h"

typedef void (*Wireless_ReceiveCallback)(uint8_t data[], uint8_t len);

void Wireless_Init(void);
void Wireless_SwitchToRx(void);
void Wireless_SwitchToTx(void);
uint8_t Wireless_TransmitHandler(uint8_t transmitData[], uint8_t len);
void Wireless_SetReceiveCallback(Wireless_ReceiveCallback callback);
void Wireless_ReceiveAnalysis(void);
void Wireless_ReceiveTask(void *param);

/* 预加载 ACK payload（飞控遥测），将在下次收到遥控器数据包时随 ACK 回传 */
void Wireless_LoadAckPayload(uint8_t *data, uint8_t len);

#endif  // !__WIRELESS_H_
