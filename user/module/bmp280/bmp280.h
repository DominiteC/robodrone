#ifndef __BMP280_H__
#define __BMP280_H__

#include "spi.h"

// #define BMP280_SPI_HANDLE   hspi1
#define BMP280_CS_PORT      BMP_280_CS_GPIO_Port
#define BMP280_CS_PIN       BMP_280_CS_Pin

#define BMP280_SCK_PORT  BMP_280_SCK_GPIO_Port
#define BMP280_SCK_PIN   BMP_280_SCK_Pin

#define BMP280_MOSI_PORT BMP_280_MOSI_GPIO_Port
#define BMP280_MOSI_PIN  BMP_280_MOSI_Pin

#define BMP280_MISO_PORT BMP_280_MISO_GPIO_Port
#define BMP280_MISO_PIN  BMP_280_MISO_Pin

#define BMP280_CS_LOW()   HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET)
#define BMP280_CS_HIGH()  HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET)
#define BMP280_SCK_LOW()  HAL_GPIO_WritePin(BMP280_SCK_PORT, BMP280_SCK_PIN, GPIO_PIN_RESET)
#define BMP280_SCK_HIGH() HAL_GPIO_WritePin(BMP280_SCK_PORT, BMP280_SCK_PIN, GPIO_PIN_SET)
#define BMP280_MOSI_LOW() HAL_GPIO_WritePin(BMP280_MOSI_PORT, BMP280_MOSI_PIN, GPIO_PIN_RESET)
#define BMP280_MOSI_HIGH() HAL_GPIO_WritePin(BMP280_MOSI_PORT, BMP280_MOSI_PIN, GPIO_PIN_SET)
#define BMP280_READ_MISO() HAL_GPIO_ReadPin(BMP280_MISO_PORT, BMP280_MISO_PIN)

//使用最大数据分辨率,也只实现了这一个
#define sDRV_BMP280_USE_MAX_RESOLUTION
//使用定点数做数据校准运算,注释掉将会使用浮点运算
//#define sDRV_BMP280_USE_FIXED_POINT_COMPE

//为了方便使用指针,这里顺序有要求
typedef struct{
    uint8_t press_msb;  //这些都是从寄存器里读到的值
    uint8_t press_lsb;
    uint8_t press_xlsb;
    uint8_t temp_msb;
    uint8_t temp_lsb;
    uint8_t temp_xlsb;
    int32_t temp;      //最终温度数据
    int32_t press;     //最终气压数据
    uint8_t cal_val[26];//获取到的寄存器里的校准值
    uint16_t dig_T1;    //这里参考数据手册
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
}sDrv_BMP280_t;

extern sDrv_BMP280_t bmp280;

uint8_t BMP280_ReadChipID_SoftSPI(void);

HAL_StatusTypeDef sDrv_BMP280_Init(void);
HAL_StatusTypeDef sDrv_BMP280_GetMeasure(void);
double sDrv_BMP280_GetPress(void);
double sDrv_BMP280_GetTemp(void);
double calculate_altitude(double pressure);

#endif /* __BMP280_H__ */
