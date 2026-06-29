/**
 * @file gyro.c
 * @author Sevenfite (Sevenfite@163.com)
 * @brief 陀螺仪抽象层
 * @version 0.1
 * @date 2024-05-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "gyro.h"
#include "C_code_Log.h"
#include "usart_port.h"
#include "usart.h"
#include "Mydelay.h"
#include "vector_types.h"
#include "arm_transform.h"
#include "watchdog_guard.h"
#include "FreeRTOS.h"
#include "task.h"


//#include "usart_send.h"
//extern USART_SendType this;
static float gl_Pitch,gl_Roll,gl_Yaw;//俯仰、横滚、偏航角 (放置位置不确定)
USART_Data gyroData;

#define GYRO_ENABLE_ACC_CALI_ON_BOOT 0
#define GYRO_ENABLE_MAG_CALI_ON_BOOT 0

uint8_t pitch_init_flag = 0; // 陀螺仪pitch初始化为0标志位
float_angle offset; //偏移值
static float acc_z_gravity_offset = 0.0f;

float32_t bufA[16], bufB[16], bufC[16];
arm_matrix_instance_f32 T;


static void gyro_dataBufferInit(void);
static void gyro_calibration_delay_ms(uint32_t delay_ms);
void gyro_calibration(void);
void gyro_callback(void* this);
void JY901_SerialWrite(uint8_t *p_ucData, uint32_t uiLen);

void gyro_init(void) {
  WitSerialWriteRegister(JY901_SerialWrite);    // 注册串口发送函数
  // 修改默认参数
  WitSetUartBaud(WIT_BAUD_115200);             // 设置波特率
  MX_USART_UART_Init(GYRO_USART_HANDLE,USART3,115200);  // 初始化串口为115200
  WitSetSave(); // 保存参数
  WitSetOutputRate(RRATE_200HZ);    // 设置输出频率为200Hz
  WitSetOrient(ORIENT_HERIZONE);	// 设置为水平模式(目前是斜着放的，如果后面改成竖着放要改为垂直模式)
  vTaskDelay(pdMS_TO_TICKS(220));
  gyro_dataBufferInit();          // 初始化数据缓冲区
	
  gyro_calibration(); // 校准陀螺仪
  LOG_INFO("gyro init");
}

static uint8_t dataBuffer[44];	// 待完善
//static USART_Data gyroData;
static void gyro_dataBufferInit(void)
{
    USART_DataTypeInit(&gyroData,GYRO_USART_HANDLE,dataBuffer,sizeof(dataBuffer),DMA_MODE,gyro_callback);
    LOG_INFO("gyro dataBufferInit");
}

void gyro_calibration(void)
{
#if GYRO_ENABLE_ACC_CALI_ON_BOOT
    WitStartAccCali();
    LOG_INFO("陀螺仪开始加速度计校准，请保持正面朝上并静止");
    gyro_calibration_delay_ms(4000);
    WitStopAccCali();
    LOG_INFO("陀螺仪加速度计校准完成");
    gyro_calibration_delay_ms(500);
#endif
    acc_z_gravity_offset = stcAcc.a[2];
    LOG_INFO("加速度计Z轴重力偏置: %.2f", acc_z_gravity_offset);
#if GYRO_ENABLE_MAG_CALI_ON_BOOT
    WatchdogGuard_EnterLongAction(20000);
    WitStartMagCali();
    LOG_INFO("陀螺仪开始磁力计校准, 绕三个轴分别旋转1-2圈");
    gyro_calibration_delay_ms(20000);
    WitStopMagCali();
    WatchdogGuard_ExitLongAction();
    LOG_INFO("陀螺仪磁力计校准完成");
#endif
}

static void gyro_calibration_delay_ms(uint32_t delay_ms)
{
    uint32_t elapsed = 0;

    while (elapsed < delay_ms)
    {
        WatchdogGuard_FeedNow();
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }
}


void gyro_callback(void* this)
{
	if(((USART_Data*)this) == &gyroData)
	{
		uint16_t len = USART_DataGetReceivedLen(&gyroData);
		for (uint8_t i = 0; i < len; i++)
		{
			CopeSerialData(gyroData.usart_rx_buf[i]);	//应该存在更好的查找数据头的方法
		}
        if (!pitch_init_flag) // 初始化pitch为0
        {
            offset.roll = stcAngle.Angle[1];
            offset.pitch = stcAngle.Angle[0];
            offset.yaw = stcAngle.Angle[2];
            stcAngle.Angle[0] = 0;
//			if (myDelay((uint32_t)gyro_callback,10))
						pitch_init_flag = 1;
            LOG_INFO_IT("陀螺仪pitch初始化为0");
        }
        else
        {
            stcAngle.Angle[1] -= offset.roll; // 修正roll值
            stcAngle.Angle[0] -= offset.pitch; // 修正pitch值
            stcAngle.Angle[2] -= offset.yaw;  // 修正yaw值
            if (stcAngle.Angle[2] > 180)
                stcAngle.Angle[2] -= 360;
            else if (stcAngle.Angle[2] < -180)
                stcAngle.Angle[2] += 360;
        }
        
				// 加速度坐标系变换
        float acc_temp = stcAcc.a[0];
        stcAcc.a[0] = stcAcc.a[1];		// X_b = Y_g
        stcAcc.a[1] = acc_temp;			// Y_b = X_g
        
				// 角速度坐标变换
        float gyro_temp = stcGyro.w[0];
        stcGyro.w[0] = -stcGyro.w[1];		// X_b = -Y_g
        stcGyro.w[1] = gyro_temp;			// Y_b = X_g
        
				// 欧拉角变换
				gl_Roll = -stcAngle.Angle[1] ;	// 绕X轴
        gl_Pitch = stcAngle.Angle[0]+0.1;	// 绕Y轴
				gl_Yaw = stcAngle.Angle[2];	// 绕Z轴
        
		// LOG_DEBUG_IT("roll:%.2f,%.2f,%.2f",gl_Roll,gl_Pitch,gl_Yaw);
//		LOG_DEBUG_IT("acc:%f,%f,%f",stcAcc.a[0],stcAcc.a[1],stcAcc.a[2]);
//		USART_SendData(&this, dataBuffer, len, USART_USE_RING_BUFF);
	}
}

void JY901_SerialWrite(uint8_t *p_ucData, uint32_t uiLen)
{
    if(p_ucData == NULL || uiLen == 0)return;
    HAL_UART_Transmit(GYRO_USART_HANDLE, p_ucData, uiLen, 1000);
}

/// @brief 陀螺仪获取数据
/// @note 检测陀螺仪串口，并处理数据，存储到本地变量中，如果要查看，可以调用对应的get函数
/// @example while(gyro_getData() != 1);pitch=gyro_getPitch;
/// @return 1 获取成功, 0 还未接收完成，-1 获取失败, 数据长度不对
int8_t gyro_getData()
{
   if(USART_DataIsReceived(&gyroData))
   {
       uint16_t len = USART_DataGetReceivedLen(&gyroData);
       USART_DataResetReceivedFlag(&gyroData);
       if(len == 33)
       {
          return 1;
       }
       return -1;
   }
   return 0;
}

/**
 * @brief 获取俯仰角
 * 
 * @return float 
 * @note 规定上坡时为负值，下坡时为正值
 */
float gyro_getPitch(void)
{
	
    return gl_Pitch;
}
/**
 * @brief 获取横滚角
 * 
 * @return float 
 */
float gyro_getRoll(void)
{

    return gl_Roll;
}
/**
 * @brief 获取偏航角
 * 
 * @return float 
 */
float gyro_getYaw(void)
{

    return gl_Yaw;
}

void gyro_getAngle(float_angle* angle)
{
    // static uint8_t coordinate_init_flag;
    // float32_t v_in[3]  = {gl_Roll * PI/180.0f, gl_Pitch * PI/180.0f, gl_Yaw * PI/180.0f};
    // float32_t v_out[3];
    if(angle)
    {
        // if (coordinate_init_flag == 0 && roll_init_flag)
        // {
        //     coordinate_init_flag = 1;
        //     arm_transform_from_euler(0, 0, 0, offset.roll * PI/180.0f, offset.pitch * PI/180.0f, 0, &T, bufA);
        //     LOG_INFO("offset:%.2f,%.2f,%.2f",offset.roll,offset.pitch,offset.yaw);
        // }
        // arm_transform_apply_vector(&T, v_in, v_out);
        // LOG_INFO("angle-roll:%.2f,pitch:%.2f,yaw:%.2f",gl_Roll,gl_Pitch,gl_Yaw);
        angle->roll = gl_Roll;
        angle->pitch = gl_Pitch;
        angle->yaw = gl_Yaw;
    }
}

void gyro_getAcc(float_acc* acc)
{
    if (acc)
    {
        acc->x = stcAcc.a[0];
        acc->y = stcAcc.a[1];
        acc->z = stcAcc.a[2] - acc_z_gravity_offset;
    }
}

void gyro_getAngularVelocity(float_gyro* gyro)
{
    if (gyro)
    {
        gyro->x = stcGyro.w[0];
        gyro->y = stcGyro.w[1];
        gyro->z = stcGyro.w[2];
    }
}
