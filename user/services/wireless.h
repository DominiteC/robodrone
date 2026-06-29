/*
 * wireless.h
 * 声明无线链路服务接口，供遥控数据服务和 app 任务创建模块调用。
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
void Wireless_ReceiveTask(void* param);

#endif  // !__GYRO_H_
