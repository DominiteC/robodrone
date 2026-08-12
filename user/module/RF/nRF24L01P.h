/**@file  	    nRF24L01P.h
* @brief            nRF24L01+ low level operations and configurations.
* @author           hyh
* @date             2021.6.9
* @version          1.0
* @copyright        Chengdu Ebyte Electronic Technology Co.Ltd
**********************************************************************************
*/
#ifndef nRF24L01P_H
#define nRF24L01P_H

#include "spi.h"
#include "nRF24L01P_REG.h"

/*Data type definations*/
typedef unsigned char INT8U;
typedef unsigned short INT16U;
typedef unsigned int INT32U;

/*Macro function*/
#define MAX(a, b)                ((a) < (b) ? (b) : (a))
#define MIN(a, b)                ((a) > (b) ? (b) : (a))

#define IRQ_ALL ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))
/*Data Rate selection*/
typedef enum {DRATE_250K,DRATE_1M,DRATE_2M}L01_DRATE;
/*Power selection*/
typedef enum {POWER_N_0,POWER_N_6,POWER_N_12,POWER_N_18}L01_PWR;
/*Mode selection*/
typedef enum {TX_MODE,RX_MODE}L01_MODE;
/*CE pin level selection*/
typedef enum {CE_LOW,CE_HIGH}CE_STAUS;
/*
================================================================================
============================Configurations and Options==========================
================================================================================
*/
#define DYNAMIC_PACKET      1 //1:DYNAMIC packet length, 0:fixed
#define FIXED_PACKET_LEN    32//Packet size in fixed size mode
#define TX_ADDR             5,2,0,13,14
#define RX_ADDR             20,12,0,7,12
#define INIT_ADDR           TX_ADDR

#define L01_SPI_HANDLE  hspi2               //SPI handle
#define PORT_L01_CSN    E01_CS_GPIO_Port    //CSN pin port
#define PIN_L01_CSN     E01_CS_Pin          //CSN pin number
#define PORT_L01_CE     E01_CE_GPIO_Port    //CE pin port
#define PIN_L01_CE      E01_CE_Pin          //CE pin number
#define PORT_L01_IRQ    E01_IRQ_GPIO_Port   //IRQ pin port
#define PIN_L01_IRQ     E01_IRQ_Pin         //IRQ pin number
/*
================================================================================
==========================List of externally provided functions ================
================================================================================
*/

static inline uint8_t SPI_TransmitReceive(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&L01_SPI_HANDLE,&data,&rx_data,1,1000);
    return rx_data;
}

#define L01_CSN_LOW()      HAL_GPIO_WritePin(PORT_L01_CSN,PIN_L01_CSN,GPIO_PIN_RESET)//Pull down the SPI chip select
#define L01_CSN_HIGH()     HAL_GPIO_WritePin(PORT_L01_CSN,PIN_L01_CSN,GPIO_PIN_SET)//Pull up the SPI chip select
#define L01_CE_LOW()       HAL_GPIO_WritePin(PORT_L01_CE,PIN_L01_CE,GPIO_PIN_RESET)//Set CE low level
#define L01_CE_HIGH()      HAL_GPIO_WritePin(PORT_L01_CE,PIN_L01_CE,GPIO_PIN_SET)//Set CE high level
#define GET_L01_IRQ()      HAL_GPIO_ReadPin(PORT_L01_IRQ,PIN_L01_IRQ)//Get the IRQ pin status
#define SPI_ExchangeByte(data)      SPI_TransmitReceive(data)
                                    //BSP_SPI_ExchangeByte(data) //Exchange data by the SPI
/*
================================================================================
-------------------------------------Exported APIs------------------------------
================================================================================
*/
/*Set the level status of the CE pin low or high*/
void L01_SetCE(CE_STAUS status);
/*Read the value from the specified register */
INT8U L01_ReadSingleReg(INT8U addr);
/*Read the values of the specified registers and store them in buffer*/
void L01_ReadMultiReg(INT8U start_addr,INT8U *buffer,INT8U size);
/*Write a value to the specified register*/
void L01_WriteSingleReg(INT8U addr,INT8U value);
/*Write buffer to the specified registers */
void L01_WriteMultiReg(INT8U start_addr,INT8U *buffer,INT8U size);
/*Set the nRF24L01 into PowerDown mode */
void L01_SetPowerDown(void);
/*Set the nRF24L01 into PowerUp mode*/
void L01_SetPowerUp(void);
/*Flush the TX buffer*/
void L01_FlushTX(void);
/*Flush the RX buffer*/
void L01_FlushRX(void);
/*Reuse the last transmitted payload*/
void L01_ReuseTXPayload(void);
/*Read the status register of the nRF24L01*/
INT8U L01_ReadStatusReg(void);
/*Clear the IRQ caused by the nRF24L01+*/
void L01_ClearIRQ(INT8U irqMask);
/*Read the IRQ status of the nRF24L01+*/
INT8U L01_ReadIRQSource(void);
/*Read the payload width of the top buffer of the FIFO */
INT8U L01_ReadTopFIFOWidth(void);
/*Read the RX payload from the FIFO and store them in buffer*/
INT8U L01_ReadRXPayload(INT8U *buffer);
/*Write TX Payload to a data pipe,and PRX will return ACK back*/
void L01_WriteTXPayload_Ack(INT8U *buffer,INT8U size);
/*Write TX Payload to a data pipe,and PRX won't return ACK back*/
void L01_WriteTXPayload_NoAck(INT8U *buffer,INT8U size);
/*Write TX Payload to a data pipe when RX mode*/
void L01_WriteRXPayload_InAck(INT8U *buffer,INT8U size);
/*Write Transmit address into TX_ADDR register */
void L01_SetTXAddr(INT8U *Addrbuffer,INT8U Addr_size);
/*Write address for the RX pipe*/
void L01_SetRXAddr(INT8U pipeNum,INT8U *addrBuffer,INT8U addr_size);
/*Set the data rate of the nRF24L01+ */
void L01_SetDataRate(L01_DRATE drate);
/*Set the power of the nRF24L01+ */
void L01_SetPower(L01_PWR power);
/*Set the frequency of the nRF24L01+*/
void L01_WriteHoppingPoint(INT8U freq);
/*Set the nRF24L01+ as TX/RX mode*/
void L01_SetTRMode(L01_MODE mode);
/*Initialize the nRF24L01+ */
void L01_Init(void);

#endif
