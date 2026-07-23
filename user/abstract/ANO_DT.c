/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "usart.h"
#include "time.h"
#include "ANO_DT.h"
#include "gyro.h"
#include "usart_send.h"
#include "control.h"
#include "PIDcontroller.h"
#include "usart_port.h"
#include "IT_Callback.h"
#include "commander.h"
#include "C_code_Log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern USART_SendType channel_0;

// Observer ANO_DT_observer = {
// 	.id = 0,
// 	.update = ANO_DT_Data_Exchange,
// 	.freq = 0,
// 	.count = 0,
// 	.next = NULL
// };

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

static volatile TickType_t s_ano_last_rx_tick = 0;

static void ANO_UpdateSendHealth(Usart_SendState send_state)
{
    (void)send_state;
}

dt_flag_t f;					//需要发送数据的标志
uint8_t data_to_send[80];	//发送数据缓存（扩大以支持高频振动数据帧）


void ANO_DT_CallBack(void* this)
{
	uint8_t *data = USART_GetData((USART_Data*)this);
	uint16_t len = USART_DataGetReceivedLen((USART_Data*)this);
	if(len < 5 || data[0] != 0xAA || data[1] != 0xAF)
		return;
	s_ano_last_rx_tick = xTaskGetTickCountFromISR();
	ANO_DT_Data_Receive_Anl(data, len);
}

void ANO_DT_Init(void)
{
	// gl_TIM6_IT.add(&gl_TIM6_IT, &ANO_DT_observer);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//每20ms调用一次
void ANO_DT_Data_Exchange(void *param)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		static uint8_t cnt = 0;
		static uint8_t senser_cnt 	= 1;
		static uint8_t status_cnt 	= 10;
		static uint8_t rcdata_cnt 	= 10;
		static uint8_t motopwm_cnt	= 10;
		static uint8_t power_cnt	= 50;
		static uint8_t pid_debug_cnt	= 1;
		senser_cnt = 1;
		rcdata_cnt = 10;
		motopwm_cnt = 10;
		pid_debug_cnt = 1;

		if((cnt % senser_cnt) == (senser_cnt-1))
			f.send_senser = 1;

		if((cnt % status_cnt) == (status_cnt-1))
			f.send_status = 1;

		if((cnt % rcdata_cnt) == (rcdata_cnt-1))
			f.send_rcdata = 1;

		if((cnt % motopwm_cnt) == (motopwm_cnt-1))
			f.send_motopwm = 1;

		if((cnt % power_cnt) == (power_cnt-1))
			f.send_power = 1;

		if((cnt % pid_debug_cnt) == (pid_debug_cnt-1))
			f.send_pid_debug = 1;

		cnt++;
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_version)
		{
			f.send_version = 0;
			ANO_DT_Send_Version(4,300,100,400,0);
		}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_status)
		{
			f.send_status = 0;
			float_angle angle;
			gyro_getAngle(&angle);
			ANO_DT_Send_Status(angle.roll,angle.pitch,debug_yaw_meas_cont,0,0,getIsLock());
		}	
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_senser)
		{
			f.send_senser = 0;
			float_acc acc;
			float_gyro gyr;
			gyro_getAcc(&acc);
			gyro_getAngularVelocity(&gyr);
			int16_t acc_x = (int16_t)acc.x;
			int16_t acc_y = (int16_t)acc.y;
			int16_t acc_z = (int16_t)acc.z;
			int16_t gyro_x = (int16_t)gyr.x;
			int16_t gyro_y = (int16_t)gyr.y;
			int16_t gyro_z = (int16_t)gyr.z;
			int16_t mag_x = stcMag.h[0];
			int16_t mag_y = stcMag.h[1];
			int16_t mag_z = stcMag.h[2];
			
			ANO_DT_Send_Senser(acc_x, acc_y, acc_z,
								gyro_x, gyro_y, gyro_z,
								mag_x, mag_y, mag_z,0);

			// Data_Send_AngleRate(stcGyro.w[0],RC_Control.angle.roll*10,pid_roll_rate.Err,pid_roll_rate.Output,
			// 										pid_roll_angle.Err,pid_roll_angle.Output,0,0);

			// Data_Send_AngleRate(stcGyro.w[1],RC_Control.angle.pitch*10,pid_pitch_rate.Err,pid_pitch_rate.Output,
			// 										pid_pitch_angle.Err,pid_pitch_angle.Output,0,0);

			// 保护PID数据访问
			
			int16_t gyro_z_val = (int16_t)gyr.z;
			PIDDump dump_yaw_rate;
			pid_dump(&pid_yaw_rate, &dump_yaw_rate);
			float pid_yaw_rate_err = dump_yaw_rate.Err;
			float pid_yaw_rate_out = dump_yaw_rate.Output;
			float pid_yaw_rate_pout = dump_yaw_rate.Pout;
			float pid_yaw_rate_iout = dump_yaw_rate.Iout;
			float pid_yaw_rate_dout = dump_yaw_rate.Dout;
			
			/* yaw rate setpoint = 角度环输出 (MiniFly 方式) */
			float yaw_rate_setpoint = pid_yaw_angle.Output;
			
			Data_Send_AngleRate(gyro_z_val, yaw_rate_setpoint, pid_yaw_rate_err, pid_yaw_rate_out,
													pid_yaw_rate_pout, pid_yaw_rate_iout, pid_yaw_rate_dout, RC_Control.throttle);
		}	
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_rcdata)
		{
			f.send_rcdata = 0;
			PIDDump dump_yaw_rate;
			pid_dump(&pid_yaw_rate, &dump_yaw_rate);
			PIDDump dump_yaw_angle;
			pid_dump(&pid_yaw_angle, &dump_yaw_angle);
ANO_DT_Send_RCDataFloat(
												debug_target_angle_yaw,        // 1-目标 yaw
												state.angle.yaw,        // 2-实际 yaw
                        pid_yaw_rate.Output,      // 3-yaw rate PID输出
												pid_z_velocity.Ref,    // 4-目标Z速度 cm/s
                        state.velocity.z,         // 5-实际Z速度 cm/s
												pid_z_velocity.Output, // 6-Z速度环PID输出
                        debug_target_angle_roll,      // 7-target roll angle
                        state.velocity.x,           // 8-X velocity cm/s
                        state.velocity.y,           // 9-Y velocity cm/s
                        state.gyro.z,              // 10-yaw 角度误差
                        pid_yaw_angle.Output, // 11-yaw 角度环PID输出
												debug_yaw_rate_target   // 12-yaw 角速度目标
                        );
		}	
/////////////////////////////////////////////////////////////////////////////////////	
		if(f.send_motopwm)
		{
			f.send_motopwm = 0;
			ANO_DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
		}	
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_power)
		{
			f.send_power = 0;
			ANO_DT_Send_Power(123,456);
		}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid1)
		{
			f.send_pid1 = 0;

			PID_Init_Config_s cfg_rr, cfg_pr, cfg_yr;
			pid_get_config(&pid_roll_rate, &cfg_rr);
			pid_get_config(&pid_pitch_rate, &cfg_pr);
			pid_get_config(&pid_yaw_rate, &cfg_yr);
			ANO_DT_Send_PID(1,cfg_rr.Kp,cfg_rr.Ki,cfg_rr.Kd,
										cfg_pr.Kp,cfg_pr.Ki,cfg_pr.Kd,
										cfg_yr.Kp,cfg_yr.Ki,cfg_yr.Kd);

		}	
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid2)
		{
			f.send_pid2 = 0;
			
			PID_Init_Config_s cfg_ra, cfg_pa, cfg_ya;
			pid_get_config(&pid_roll_angle, &cfg_ra);
			pid_get_config(&pid_pitch_angle, &cfg_pa);
			pid_get_config(&pid_yaw_angle, &cfg_ya);
			ANO_DT_Send_PID(2,cfg_ra.Kp,cfg_ra.Ki,cfg_ra.Kd,
										cfg_pa.Kp,cfg_pa.Ki,cfg_pa.Kd,
										cfg_ya.Kp,cfg_ya.Ki,cfg_ya.Kd);
			
		}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid3)
		{
			f.send_pid3 = 0;

			PID_Init_Config_s cfg_zv, cfg_hp, cfg_xp;
			pid_get_config(&pid_z_velocity, &cfg_zv);
			pid_get_config(&pid_height_position, &cfg_hp);
			pid_get_config(&pid_x_position, &cfg_xp);
			ANO_DT_Send_PID(3,cfg_zv.Kp,cfg_zv.Ki,cfg_zv.Kd,
										cfg_hp.Kp,cfg_hp.Ki,cfg_hp.Kd,
										cfg_xp.Kp,cfg_xp.Ki,cfg_xp.Kd);

		}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid4)
		{
			f.send_pid4 = 0;

			PID_Init_Config_s cfg_yp, cfg_xv, cfg_yv;
			pid_get_config(&pid_y_position, &cfg_yp);
			pid_get_config(&pid_x_velocity, &cfg_xv);
			pid_get_config(&pid_y_velocity, &cfg_yv);
			ANO_DT_Send_PID(4,cfg_yp.Kp,cfg_yp.Ki,cfg_yp.Kd,
										cfg_xv.Kp,cfg_xv.Ki,cfg_xv.Kd,
										cfg_yv.Kp,cfg_yv.Ki,cfg_yv.Kd);

		}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid5)
		{
			f.send_pid5 = 0;
			/* 当前协议布局 PID5 列无对应 PID */
		}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid6)
	{
		f.send_pid6 = 0;
		/* 当前协议布局 PID6 列无对应 PID */
	}
/////////////////////////////////////////////////////////////////////////////////////
		if(f.send_pid_debug)
		{
			f.send_pid_debug = 0;

			// 获取实际欧拉角（直接读取传感器融合结果，不依赖PID计算）
			float_angle angle;
			gyro_getAngle(&angle);

			PIDDump dump_roll_angle, dump_pitch_angle, dump_roll_rate, dump_pitch_rate;
			pid_dump(&pid_roll_angle, &dump_roll_angle);
			pid_dump(&pid_pitch_angle, &dump_pitch_angle);
			pid_dump(&pid_roll_rate, &dump_roll_rate);
			pid_dump(&pid_pitch_rate, &dump_pitch_rate);
// Roll 数据
float			roll_angle_target = dump_roll_angle.Ref;
float			roll_angle_measure = angle.roll;          // 直接用传感器融合结果
float			roll_rate_target = dump_roll_angle.Output;
float			roll_rate_measure = dump_roll_rate.Measure;
		
			// Pitch 数据
float			pitch_angle_target = dump_pitch_angle.Ref;
float			pitch_angle_measure = angle.pitch;         // 直接用传感器融合结果
float			pitch_rate_target = dump_pitch_angle.Output;
float			pitch_rate_measure = dump_pitch_rate.Measure;

			// 发送 0xF3 帧 - Roll/Pitch 角度和角速度
			ANO_DT_Send_PID_Debug(0xF3,
				target.angle.roll, roll_angle_measure, roll_rate_target, roll_rate_measure,
				target.angle.pitch, pitch_angle_measure, pitch_rate_target, pitch_rate_measure,
				dump_roll_rate.Output, dump_pitch_rate.Output, dump_roll_rate.Err, dump_pitch_rate.Err);
		}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

        vTaskDelayUntil(&lastWakeTime, 20);		/*20ms周期延时*/
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
	Usart_SendState send_state = USART_SendData(&channel_0, dataToSend, length, USART_USE_RING_BUFF);
	ANO_UpdateSendHealth(send_state);
#ifdef ANO_DT_USE_USB_HID
	Usb_Hid_Adddata(data_to_send,length);
#endif
#ifdef ANO_DT_USE_USART2
	Usart2_Send(data_to_send, length);
#endif
}

static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			// mpu6050.Acc_CALIBRATE = 1;
		if(*(data_buf+4)==0X02)
			// mpu6050.Gyro_CALIBRATE = 1;
		if(*(data_buf+4)==0X03)
		{
			// mpu6050.Acc_CALIBRATE = 1;		
			// mpu6050.Gyro_CALIBRATE = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//			Para_ResetToFactorySetup();
		}
	}

		if(*(data_buf+2)==0X10)							//PID1
    {
        PID_Init_Config_s cfg_rr, cfg_pr, cfg_yr;
        pid_get_config(&pid_roll_rate, &cfg_rr);
        pid_get_config(&pid_pitch_rate, &cfg_pr);
        pid_get_config(&pid_yaw_rate, &cfg_yr);
        cfg_rr.Kp  = 0.001*( (vint16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        cfg_rr.Ki  = 0.001*( (vint16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        cfg_rr.Kd  = 0.001*( (vint16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        cfg_pr.Kp = 0.001*( (vint16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        cfg_pr.Ki = 0.001*( (vint16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        cfg_pr.Kd = 0.001*( (vint16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        cfg_yr.Kp 	= 0.001*( (vint16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        cfg_yr.Ki 	= 0.001*( (vint16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        cfg_yr.Kd 	= 0.001*( (vint16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        pid_set_config(&pid_roll_rate, &cfg_rr);
        pid_set_config(&pid_pitch_rate, &cfg_pr);
        pid_set_config(&pid_yaw_rate, &cfg_yr);
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        PID_Init_Config_s cfg_ra, cfg_pa, cfg_ya;
        pid_get_config(&pid_roll_angle, &cfg_ra);
        pid_get_config(&pid_pitch_angle, &cfg_pa);
        pid_get_config(&pid_yaw_angle, &cfg_ya);
        cfg_ra.Kp 	= 0.001*( (vint16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        cfg_ra.Ki 	= 0.001*( (vint16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        cfg_ra.Kd 	= 0.001*( (vint16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        cfg_pa.Kp 	= 0.001*( (vint16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        cfg_pa.Ki 	= 0.001*( (vint16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        cfg_pa.Kd 	= 0.001*( (vint16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        cfg_ya.Kp	= 0.001*( (vint16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        cfg_ya.Ki 	= 0.001*( (vint16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        cfg_ya.Kd 	= 0.001*( (vint16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        pid_set_config(&pid_roll_angle, &cfg_ra);
        pid_set_config(&pid_pitch_angle, &cfg_pa);
        pid_set_config(&pid_yaw_angle, &cfg_ya);
        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
    }
        if(*(data_buf+2)==0X12)							//PID3
    {
        PID_Init_Config_s cfg_zv, cfg_hp, cfg_xp;
        pid_get_config(&pid_z_velocity, &cfg_zv);
        pid_get_config(&pid_height_position, &cfg_hp);
        pid_get_config(&pid_x_position, &cfg_xp);
        cfg_zv.Kp  = 0.001*( (vint16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        cfg_zv.Ki  = 0.001*( (vint16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        cfg_zv.Kd  = 0.001*( (vint16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        cfg_hp.Kp  = 0.001*( (vint16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        cfg_hp.Ki  = 0.001*( (vint16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        cfg_hp.Kd  = 0.001*( (vint16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        cfg_xp.Kp 	= 0.001*( (vint16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        cfg_xp.Ki 	= 0.001*( (vint16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        cfg_xp.Kd 	= 0.001*( (vint16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        pid_set_config(&pid_z_velocity, &cfg_zv);
        pid_set_config(&pid_height_position, &cfg_hp);
        pid_set_config(&pid_x_position, &cfg_xp);
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
		if(*(data_buf+2)==0X13)							//PID4
	{
        PID_Init_Config_s cfg_yp, cfg_xv, cfg_yv;
        pid_get_config(&pid_y_position, &cfg_yp);
        pid_get_config(&pid_x_velocity, &cfg_xv);
        pid_get_config(&pid_y_velocity, &cfg_yv);
        cfg_yp.Kp  = 0.001*( (vint16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        cfg_yp.Ki  = 0.001*( (vint16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        cfg_yp.Kd  = 0.001*( (vint16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        cfg_xv.Kp  = 0.001*( (vint16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        cfg_xv.Ki  = 0.001*( (vint16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        cfg_xv.Kd  = 0.001*( (vint16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        cfg_yv.Kp 	= 0.001*( (vint16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        cfg_yv.Ki 	= 0.001*( (vint16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        cfg_yv.Kd 	= 0.001*( (vint16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        pid_set_config(&pid_y_position, &cfg_yp);
        pid_set_config(&pid_x_velocity, &cfg_xv);
        pid_set_config(&pid_y_velocity, &cfg_yv);
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
		if(*(data_buf+2)==0X14)							//PID5 (空)
	{
		/* 当前协议布局 PID5 列无对应 PID，地面站若发此帧仅 ACK 即可 */
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
		if(*(data_buf+2)==0X15)							//PID6 (空)
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}

}

void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	vint16_t _temp;
	vint32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	uint8_t _cnt=0;
	vint16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr, int16_t yaw, int16_t rol, int16_t pit, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	vint16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/**************************************************** 自定义帧 **************************************************/
//角速度环调试,波形显示
void Data_Send_AngleRate(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8)
{
	uint8_t _cnt=0,sum = 0,i;
	float _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
	
	_temp = data1;//RadtoDeg
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data5;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data6;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data7;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data8;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}


/**************************************************** 自定义帧 **************************************************/
//高频振动数据帧，用于振动分析与PID调试
void ANO_DT_Send_VibrationData(uint32_t timestamp, 
                                float raw_gx, float raw_gy, float raw_gz,
                               float filt_gx, float filt_gy, float filt_gz,
                               float pid_roll_out, float pid_pitch_out, float pid_yaw_out,
                               float pid_roll_err, float pid_pitch_err, float pid_yaw_err,
                                float acc_x, float acc_y, float acc_z)
{
	uint8_t _cnt=0,sum = 0,i;
	float _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2; // 功能码：高频振动数据
	data_to_send[_cnt++]=0;    // 长度占位，后面填充
	
	// 时间戳 (uint32_t)
	data_to_send[_cnt++]=BYTE3(timestamp);
	data_to_send[_cnt++]=BYTE2(timestamp);
	data_to_send[_cnt++]=BYTE1(timestamp);
	data_to_send[_cnt++]=BYTE0(timestamp);
	
 	// 原始陀螺仪数据 (3×float)
	_temp = raw_gx;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = raw_gy;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = raw_gz;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	// 滤波后角速度 (3×float)
	_temp = filt_gx;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = filt_gy;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = filt_gz;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	// PID输出 (3×float)
	_temp = pid_roll_out;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pid_pitch_out;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pid_yaw_out;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	// PID误差 (3×float)
	_temp = pid_roll_err;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pid_pitch_err;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = pid_yaw_err;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
 	// 加速度计数据 (3×float)
	_temp = acc_x;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = acc_y;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = acc_z;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	// 填充长度字段
	data_to_send[3] = _cnt-4;
	
	// 计算校验和
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	// 发送数据
	ANO_DT_Send_Data(data_to_send, _cnt);
}



// 高频振动数据任务实现
static const float ALPHA_LPF = 0.2f; // 低通滤波器系数 (200Hz采样, 20Hz截止)



/**************************************************** 自定义帧 **************************************************/
// PID调试数据帧 - Roll/Pitch 角度和角速度 (0xF3)
// 发送 roll/pitch 的 target 和 measure (角度 + 角速度)
void ANO_DT_Send_PID_Debug(uint8_t frame_type,
    float roll_angle_target, float roll_angle_measure, float roll_rate_target, float roll_rate_measure,
    float pitch_angle_target, float pitch_angle_measure, float pitch_rate_target, float pitch_rate_measure,
    float yaw_angle_target, float yaw_angle_measure, float yaw_rate_target, float yaw_rate_measure)
{
	uint8_t _cnt = 0, sum = 0, i;
	float _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = frame_type;  // 0xF3
	data_to_send[_cnt++] = 0;  // 长度占位

	// Roll 数据 (4个float)
	_temp = roll_angle_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = roll_angle_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = roll_rate_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = roll_rate_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

	// Pitch 数据 (4个float)
	_temp = pitch_angle_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = pitch_angle_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = pitch_rate_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = pitch_rate_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

	// Yaw 数据 (4个float)
	_temp = yaw_angle_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = yaw_angle_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = yaw_rate_target;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);
	_temp = yaw_rate_measure;
	data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

	// 填充长度字段
	data_to_send[3] = _cnt - 4;

	// 计算校验和
	for(i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_RCDataFloat(float yaw, float roll, float pitch, float x_velocity_output, float y_velocity_output, float target_height, float velocity_x, float velocity_y, float velocity_z, float target_vel_x, float target_vel_y, float acc_z)
{
    uint8_t _cnt = 0, sum = 0, i;
    float _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF4;  // 自定义帧类型
    data_to_send[_cnt++] = 0;     // 长度占位

    _temp = yaw;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = roll;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = pitch;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = x_velocity_output;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = y_velocity_output;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = target_height;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = velocity_x;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = velocity_y;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = velocity_z;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = target_vel_x;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = target_vel_y;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    _temp = acc_z;
    data_to_send[_cnt++] = BYTE3(_temp); data_to_send[_cnt++] = BYTE2(_temp); data_to_send[_cnt++] = BYTE1(_temp); data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    for(i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
