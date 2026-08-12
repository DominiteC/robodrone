#include "./USMART/usmart.h"
#include "./USMART/usmart_str.h"

/******************************************************************************************/
/* 用户配置区
 * 这下面要包含所用到的函数所申明的头文件(用户自己添加)
 */
#include "main.h"
#include <stdbool.h>
#include "change.h"

// extern void led_set(uint8_t sta);
// extern void test_fun(void(*ledset)(uint8_t), uint8_t sta);
void led_set(uint8_t sta) {
  if (sta) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  }
}
//调参用的
int gl_value1,gl_value2,gl_value3,gl_value4,gl_value5,gl_value6;
void setGlValue(int v1,int v2,int v3,int v4,int v5,int v6){
  gl_value1 =v1;
  gl_value2 = v2;
  gl_value3 = v3;
  gl_value4 = v4;
  gl_value5 = v5;
  gl_value6 = v6;
}
void sDrv_BMP280_Test(void);
uint8_t BMP280_ReadChipID_SoftSPI(void);
void AT24Cxx_Test(void);
void setServoAngleByIndex(uint8_t servoNum, float angle);
void setServoSlow_2_Back(ServoAngle angle);
void setServoSlow_1_Front(ServoAngle angle);
void setServoSlow_1_Back(ServoAngle angle);
void setServoAngleByIndex_int(uint8_t servoNum, uint16_t angle);
void Motor_Set_PWM(int16_t leftFrontPWM,int16_t leftBackPWM,int16_t rightFrontPWM,int16_t rightBackPWM);
void Actuator_Set(uint8_t actuator_num, bool state, uint16_t delay_time);
void Actuator_AllSet(bool state, uint16_t delay_time);
void ESC_Set_PWM(uint16_t PWM_1,uint16_t PWM_2,uint16_t PWM_3,uint16_t PWM_4);
/* 函数名列表初始化(用户自己添加)
 * 用户直接在这里输入要执行的函数名及其查找串
 */
struct _m_usmart_nametab usmart_nametab[] = {
#if USMART_USE_WRFUNS == 1 /* 如果使能了读写操作 */
    (void *)read_addr,  "uint32_t read_addr(uint32_t addr)",
    (void *)write_addr, "void write_addr(uint32_t addr,uint32_t val)",
#endif

   (void *)setServoAngleByIndex_int,    "void setServoAngleByIndex_int(uint8_t servoNum, uint16_t angle)",
   (void *)sDrv_BMP280_Test,    "void sDrv_BMP280_Test(void)",
   (void *)BMP280_ReadChipID_SoftSPI,    "uint8_t BMP280_ReadChipID_SoftSPI(void)",
   (void *)AT24Cxx_Test,    "void AT24Cxx_Test(void)",
   (void *)setServoSlow_2_Back,    "void setServoSlow_2_Back(ServoAngle angle)",
   (void *)setServoSlow_1_Front,    "void setServoSlow_1_Front(ServoAngle angle)",
   (void *)setServoSlow_1_Back,    "void setServoSlow_1_Back(ServoAngle angle)",
   (void *)Motor_Set_PWM,    "void Motor_Set_PWM(int16_t leftFrontPWM,int16_t leftBackPWM,int16_t rightFrontPWM,int16_t rightBackPWM)",
   (void *)Actuator_Set,    "void Actuator_Set(uint8_t actuator_num, bool state, uint16_t delay_time)",
   (void *)Actuator_AllSet,    "void Actuator_AllSet(bool state, uint16_t delay_time)",
   (void *)ESC_Set_PWM,    "void ESC_Set_PWM(uint16_t PWM_1,uint16_t PWM_2,uint16_t PWM_3,uint16_t PWM_4)",
};

/******************************************************************************************/

/* 函数控制管理器初始化
 * 得到各个受控函数的名字
 * 得到函数总数量
 */
struct _m_usmart_dev usmart_dev = {
    usmart_nametab,
    usmart_init,
    usmart_cmd_rec,
    usmart_exe,
    usmart_scan,
    sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), /* 函数数量 */
    0, /* 参数数量 */
    0, /* 函数ID */
    1, /* 参数显示类型,0,10进制;1,16进制 */
    0, /* 参数类型.bitx:,0,数字;1,字符串 */
    0, /* 每个参数的长度暂存表,需要MAX_PARM个0初始化 */
    0, /* 函数的参数,需要PARM_LEN个0初始化 */
};
