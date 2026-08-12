#include "bmp280.h"
#include <math.h>
#include "log.h"

//这是BMP280的I2C地址
#define BMP280_ADDR           (0x76 << 1)

//这些是BMP280的寄存器地址
#define BMP280_REG_ID         (0xD0)
#define BMP280_REG_RESET      (0xE0)
#define BMP280_REG_STATUS     (0xF3)
#define BMP280_REG_CTRL_MEAS  (0xF4)
#define BMP280_REG_CONFIG     (0xF5)
#define BMP280_REG_PRESS_MSB  (0xF7)
#define BMP280_REG_PRESS_LSB  (0xF8)
#define BMP280_REG_PRESS_XLSB (0xF9)
#define BMP280_REG_TEMP_MSB   (0xFA)
#define BMP280_REG_TEMP_LSB   (0xFB)
#define BMP280_REG_TEMP_XLSB  (0xFC)
#define BMP280_REG_CALIB      (0x88)    //第0个校准值寄存器的地址
//这些是一些值
#define BMP280_VAL_RESET      (0xB6)
#define BMP280_VAL_CHIPID     (0x58)

#define Delay_ms HAL_Delay

// 校准参数结构体
typedef struct {
    uint16_t dig_T1;
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
} bmp280_calib_t;

static bmp280_calib_t bmp280_calib;

//用来存储BMP280的一些信息
sDrv_BMP280_t bmp280;

//这是算法的校准值,参考博世的数据手册代码
int32_t t_fine;

// 简单延时，控制 SPI 时序（根据主频调整）
static void spi_delay(void)
{
    for (volatile int i = 0; i < 20; i++);
}

// 发送并接收一个字节 (SPI Mode 0)
uint8_t SoftSPI_Transfer(uint8_t data)
{
    uint8_t recv = 0;

    for (int i = 0; i < 8; i++)
    {
        // MOSI: 先放数据
        if (data & 0x80)
            BMP280_MOSI_HIGH();
        else
            BMP280_MOSI_LOW();

        data <<= 1;

        // 拉高 SCK，采样 MISO
        BMP280_SCK_HIGH();
        spi_delay();

        recv <<= 1;
        if (BMP280_READ_MISO())
            recv |= 0x01;

        // 拉低 SCK
        BMP280_SCK_LOW();
        spi_delay();
    }

    return recv;
}

// 读取 BMP280 的 Chip ID (0xD0)，返回值应为 0x58
uint8_t BMP280_ReadChipID_SoftSPI(void)
{
    uint8_t reg = 0xD0 | 0x80;  // 读操作
    uint8_t id = 0;

    BMP280_CS_LOW();
    spi_delay();

    SoftSPI_Transfer(reg);   // 发送寄存器地址
    id = SoftSPI_Transfer(0x00); // dummy -> 接收数据

    BMP280_CS_HIGH();

    return id;
}

uint8_t BMP280_ReadReg(uint8_t reg)
{
    uint8_t val;
    BMP280_CS_LOW();
    SoftSPI_Transfer(reg | 0x80);   // 读操作
    val = SoftSPI_Transfer(0x00);
    BMP280_CS_HIGH();
    return val;
}

void BMP280_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMP280_CS_LOW();
    SoftSPI_Transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SoftSPI_Transfer(0x00);
    BMP280_CS_HIGH();
}

void BMP280_WriteReg(uint8_t reg, uint8_t val)
{
    BMP280_CS_LOW();
    SoftSPI_Transfer(reg & 0x7F);   // 写操作
    SoftSPI_Transfer(val);
    BMP280_CS_HIGH();
}

/*@brief  BMP280初始化
*
* @param  无
*
* @return HAL_StatusTypeDef:如果初始化失败(通信异常)会返回HAL_ERROR,否则返回HAL_OK
*/
HAL_StatusTypeDef sDrv_BMP280_Init(void){
    //读取BMP280里的chip_id寄存器与标准值比较,用于检查通信是否正常
    uint8_t chip_id = 0;

    chip_id = BMP280_ReadReg(BMP280_REG_ID);

    //如果通信异常就返回ERR
    if(chip_id != BMP280_VAL_CHIPID){
        LOG_INFO("BMP280 chip id: 0x%02X,is error", chip_id);
        return HAL_ERROR;
    }

    //读取校准寄存器
    uint8_t cal_reg[] = {BMP280_REG_CALIB|0x80};
    BMP280_ReadMulti(BMP280_REG_CALIB, bmp280.cal_val, 26);
    Delay_ms(100);

    //复位BMP280
    uint8_t reset_reg[] = {BMP280_REG_RESET&0x7F,BMP280_VAL_RESET};
    BMP280_WriteReg(reset_reg[0],reset_reg[1]);
    Delay_ms(10);
    
    //设置为超高解析度
    #ifdef sDRV_BMP280_USE_MAX_RESOLUTION
    //气压过采样:x16,温度过采样:x2,IIR滤波:16,Timing:0.5ms,Mode:Normal
    uint8_t conf_seq[] = {BMP280_REG_CTRL_MEAS&0x7F,0x57,BMP280_REG_CONFIG&0x7F,0x14};
    BMP280_WriteReg(conf_seq[0],conf_seq[1]);
    BMP280_WriteReg(conf_seq[2],conf_seq[3]);
    Delay_ms(5);
    #endif

//    LOG_INFO("calibration data:");
//    for(uint8_t i = 0;i < 26;i++){
//        LOG_INFO("0x%02X",bmp280.cal_val[i]);
//    }
    
    //把接收到的数据变成数据手册里的样子
    //下面两块代码等价
//    for(uint8_t i = 0;i < 12;i++){
//        *((uint16_t*)(&(bmp280.dig_T1)) + (uint16_t)i) =  \
//        (((uint16_t)bmp280.cal_val[((i * 2)) + 1]) << 8) | \
//        ((uint16_t)bmp280.cal_val[(i * 2)]);               
//    }
    //指针真好玩
    for(uint8_t i=0;i<12;i++)*((uint16_t*)(&(bmp280.dig_T1))+(uint16_t)i)=(((uint16_t)bmp280.cal_val[((i*2))+1])<<8)|((uint16_t)bmp280.cal_val[(i*2)]);
    
	LOG_INFO("bmp280初始化成功");
	
    return HAL_OK;
}

uint8_t  BMP280_GetStatus(uint8_t status_flag)
{
	uint8_t flag;
	flag = BMP280_ReadReg(BMP280_REG_STATUS);
	if(flag&status_flag)	return SET;
	else return RESET;
}

/*@brief  BMP280获取测量值
*
* @param  无
*
* @return HAL_StatusTypeDef:如果传感器在忙会返回HAL_ERROR,否则返回HAL_OK
*/
HAL_StatusTypeDef sDrv_BMP280_GetMeasure(void){
    uint8_t status_reg_val = 0xFF;
    //读取状态寄存器
    while(BMP280_GetStatus(0x08) != RESET);
	while(BMP280_GetStatus(0x01) != RESET)
	;
    // status_reg_val = BMP280_ReadReg(BMP280_REG_STATUS);
    // LOG_INFO("status_reg_val:0x%02X",status_reg_val);
    // //读取忙标志
    // if((status_reg_val & 0x08) == 1){
    //     return HAL_ERROR;
    // }
    //获取温度气压数据
    BMP280_ReadMulti(BMP280_REG_PRESS_MSB, &bmp280.press_msb, 6);
    // LOG_INFO("press_msb:0x%02X,press_lsb:0x%02X,press_xlsb:0x%02X",bmp280.press_msb,bmp280.press_lsb,bmp280.press_xlsb);
    // LOG_INFO("temp_msb:0x%02X,temp_lsb:0x%02X,temp_xlsb:0x%02X",bmp280.temp_msb,bmp280.temp_lsb,bmp280.temp_xlsb);
    
    return HAL_OK;
}

/*@brief  BMP280获取压力数据
*
* @param  无
*
* @return double:气压数据,单位Pa
*/
double sDrv_BMP280_GetPress(void){
    //数据合并
    bmp280.press = ((uint32_t)(bmp280.press_msb) << 12) | ((uint32_t)(bmp280.press_lsb) << 4) | (((uint32_t)(bmp280.press_xlsb) >> 4));
    
    //如果使用定点运算
    #ifdef sDRV_BMP280_USE_FIXED_POINT_COMPE
    //这些是博世提供的算法
    int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bmp280.dig_P6;
	var2 = var2 + ((var1*(int64_t)bmp280.dig_P5)<<17);
	var2 = var2 + (((int64_t)bmp280.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp280.dig_P3)>>8) + ((var1 * (int64_t)bmp280.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280.dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-bmp280.press;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bmp280.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bmp280.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280.dig_P7)<<4);
	return (double)p / 256.0;
    #else
    //使用浮点运算
    //这些是博世提供的算法
    double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)bmp280.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)bmp280.dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)bmp280.dig_P4) * 65536.0);
	var1 = (((double)bmp280.dig_P3) * var1 * var1 / 524288.0 + ((double)bmp280.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)bmp280.dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)bmp280.press;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)bmp280.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)bmp280.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)bmp280.dig_P7)) / 16.0;
	return p;
    #endif
}

/*@brief  BMP280获取温度数据
*
* @param  无
*
* @return double:温度数据,单位degC
*/
double sDrv_BMP280_GetTemp(void){
    //数据合并
    bmp280.temp = ((uint32_t)(bmp280.temp_msb) << 12) | ((uint32_t)(bmp280.temp_lsb) << 4) | (((uint32_t)(bmp280.press_xlsb) >> 4));
    
    //如果使用定点运算
    #ifdef sDRV_BMP280_USE_FIXED_POINT_COMPE
    int32_t var1, var2, T;
	var1 = ((((bmp280.temp>>3) - ((int32_t)bmp280.dig_T1<<1))) * ((int32_t)bmp280.dig_T2)) >> 11;
	var2 = (((((bmp280.temp>>4) - ((int32_t)bmp280.dig_T1)) * ((bmp280.temp>>4) - ((int32_t)bmp280.dig_T1))) >> 12) * 
	((int32_t)bmp280.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return (double)T / 100.0;
    #else
    //浮点运算
    double var1, var2, T;
	var1 = (((double)bmp280.temp)/16384.0 - ((double)bmp280.dig_T1)/1024.0) * ((double)bmp280.dig_T2);
	var2 = ((((double)bmp280.temp)/131072.0 - ((double)bmp280.dig_T1)/8192.0) *
	(((double)bmp280.temp)/131072.0 - ((double)bmp280.dig_T1)/8192.0)) * ((double)bmp280.dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
    return T;
    #endif
}


double calculate_altitude(double pressure) {  
    double pressure_sea_level = 101325; // 标准海平面大气压力  
    double altitude = 44330 * (1 - pow((pressure / pressure_sea_level), 1 / 5.255));  
    return altitude;  
}

void sDrv_BMP280_Test(void)
{
    sDrv_BMP280_GetMeasure();
    LOG_INFO("pressure:%lf,temp:%lf",sDrv_BMP280_GetPress(),sDrv_BMP280_GetTemp());
}
