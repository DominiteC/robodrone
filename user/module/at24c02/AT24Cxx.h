#ifndef __AT24CXX_H__
#define __AT24CXX_H__

#ifdef __cplusplus
extern "C"{
#endif
#include "main.h"

#define AT24Xxx_USING_HARDWARE_I2C 1 //使用硬件i2c
#define AT24Xxx_USING_SOFTWARE_I2C 0 //使用软件i2c(先初始化软件i2c)

// #define AT24C01
#define AT24C02
// #define AT24C04
// #define AT24C08
// #define AT24C16

#if AT24Xxx_USING_HARDWARE_I2C
#include "i2c.h"
#define I2Cx hi2c1
#endif
#if AT24Xxx_USING_SOFTWARE_I2C
#include "software_i2c.h"
#endif

#ifdef AT24C01
#define AT24CXX_PAGE_SIZE 8
#define AT24CXX_BLOCK_SIZE 128
#endif
#ifdef AT24C02
#define AT24CXX_PAGE_SIZE 8
#define AT24CXX_BLOCK_SIZE 256
#endif
#ifdef AT24C04
#define AT24CXX_PAGE_SIZE 16
#define AT24CXX_BLOCK_SIZE 256
#endif
#ifdef AT24C08
#define AT24CXX_PAGE_SIZE 16
#define AT24CXX_BLOCK_SIZE 256
#endif
#ifdef AT24C016
#define AT24CXX_PAGE_SIZE 16
#define AT24CXX_BLOCK_SIZE 256
#endif

#define AT24CXX_ADDRESS 0xA0     //(1K/2K[1 0 1 0 A2 A1 A0 R/W])(4K/8K/16K[1 0 1 0 X X X R/W])

#define AT24Cxx_Delay_ms(__xms) HAL_Delay(__xms)
#define AT24CXX_MAX_TIMEOUT 1000 //i2c最大阻塞时间
#define AT24CXX_MAX_TRIALS 1000  //i2c判断次数

uint8_t AT24Cxx_Write_nByte_In_One_Block(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize);
uint8_t AT24Cxx_Read_nByteBuf(uint16_t ReadMemAddr, uint8_t *ReadBuf, uint16_t ReadDataSize);

#ifdef __cplusplus
}
#endif

#endif //__AT24CXX_H__
