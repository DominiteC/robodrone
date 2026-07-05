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

/* ===== 陀螺零偏自校准 (上电后静止采样) ===== */
/* 与 MiniFly processGyroBias 思路一致：上电后用 1.0 秒采 200 帧,
   仅当三轴方差都低于阈值时, 才把均值当作零偏. */
/* 采样长度 200 帧 @200Hz ≈ 1.0s, 与 MiniFly 的 1024 帧@1kHz≈1.0s 时长相当 */
#define GYRO_CALI_SAMPLES        200
/* 原始数据是 °/s, 方差单位 (°/s)^2. 静止时 MPU6500 典型 <0.05 deg/s,
   JY901P 略大, 这里放宽到 4.0, 等同 MiniFly 4000 (他们量纲不同但思路相同) */
#define GYRO_CALI_VAR_MAX        4.0f
/* 兜底: 万一校准前已被调用, 也能取个合理值 */
static float gyro_z_offset = 0.0f;
static uint8_t gyro_z_calibrated = 0;  /* 0=未完成, 1=已校准通过 */

float32_t bufA[16], bufB[16], bufC[16];
arm_matrix_instance_f32 T;


static void gyro_dataBufferInit(void);
static void gyro_calibration_delay_ms(uint32_t delay_ms);
void gyro_calibration(void);
void gyro_calibrateGyroZOffset(void);
uint8_t gyro_isGyroZCalibrated(void);
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
  gyro_calibrateGyroZOffset(); // 1.0s 静止自校准陀螺零偏, 上电时跑
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

/* 上电后 1.0 秒采 GYRO_CALI_SAMPLES 帧陀螺, 仅当三轴方差都低于阈值
   时接受均值当零偏. 失败时 gyro_z_offset 保持为 0 并打 LOG_WARN.
   与 MiniFly sensors.c:411 processGyroBias 思路一致: 上电时静采 + 方差校验. */
void gyro_calibrateGyroZOffset(void)
{
    int32_t cnt = 0;
    float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
    float sum_x2 = 0.f, sum_y2 = 0.f, sum_z2 = 0.f;
    float mean_x, mean_y, mean_z;
    float var_x, var_y, var_z;
    float gx, gy, gz;

    /* 等 JY901P 输出稳定 (上电后 200ms) */
    gyro_calibration_delay_ms(200);

    while (cnt < GYRO_CALI_SAMPLES)
    {
        gx = stcGyro.w[0];
        gy = stcGyro.w[1];
        gz = stcGyro.w[2];

        sum_x  += gx;  sum_x2 += gx * gx;
        sum_y  += gy;  sum_y2 += gy * gy;
        sum_z  += gz;  sum_z2 += gz * gz;
        cnt++;

        /* 5ms 间隔, 200 帧 ≈ 1.0s, 喂狗避免 1s 阻塞期间看门狗复位 */
        vTaskDelay(pdMS_TO_TICKS(5));
        WatchdogGuard_FeedNow();
    }

    mean_x = sum_x / GYRO_CALI_SAMPLES;
    mean_y = sum_y / GYRO_CALI_SAMPLES;
    mean_z = sum_z / GYRO_CALI_SAMPLES;
    var_x  = (sum_x2 - (float)GYRO_CALI_SAMPLES * mean_x * mean_x) / GYRO_CALI_SAMPLES;
    var_y  = (sum_y2 - (float)GYRO_CALI_SAMPLES * mean_y * mean_y) / GYRO_CALI_SAMPLES;
    var_z  = (sum_z2 - (float)GYRO_CALI_SAMPLES * mean_z * mean_z) / GYRO_CALI_SAMPLES;

    if (var_x < GYRO_CALI_VAR_MAX && var_y < GYRO_CALI_VAR_MAX && var_z < GYRO_CALI_VAR_MAX)
    {
        gyro_z_offset = mean_z;  /* yaw 方向 */
        gyro_z_calibrated = 1;
        LOG_INFO("gyro z offset=%.4f deg/s (var_z=%.4f, mean=%.4f,%.4f,%.4f) cali OK",
                 gyro_z_offset, var_z, mean_x, mean_y, mean_z);
    }
    else
    {
        gyro_z_offset = 0.0f;
        gyro_z_calibrated = 0;
        LOG_WARN("gyro z cali FAIL (var=%.4f,%.4f,%.4f > %.4f), using offset=0, yaw will drift slowly",
                 var_x, var_y, var_z, GYRO_CALI_VAR_MAX);
    }
}

uint8_t gyro_isGyroZCalibrated(void)
{
    return gyro_z_calibrated;
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
        /* 扣零偏: 与 MiniFly sensors.c:508-510 同样的处理.
           即便 gyro_z_calibrated=0, gyro_z_offset 仍为 0, 不影响正常情况. */
        gyro->z = stcGyro.w[2] - gyro_z_offset;
    }
}
