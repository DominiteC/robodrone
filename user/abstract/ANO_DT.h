#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"
#include <stdint.h>

typedef struct
{
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t send_pid3;
		uint8_t send_pid4;
		uint8_t send_pid5;
		uint8_t send_pid6;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;
		uint8_t send_vibration;  // 高频振动数据发送标志
		uint8_t send_pid_debug;   // PID调试数据发送标志

}dt_flag_t;

extern dt_flag_t f;

typedef volatile int16_t vint16_t;
typedef volatile int32_t vint32_t;


void ANO_DT_Data_Exchange(void *param);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);
void ANO_DT_Send_RCData(uint16_t thr, int16_t yaw, int16_t rol, int16_t pit, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6);
void ANO_DT_Send_Power(uint16_t votage, uint16_t current);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void Data_Send_AngleRate(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8);
void ANO_DT_Send_VibrationData(uint32_t timestamp,
                                float raw_gx, float raw_gy, float raw_gz,
                               float filt_gx, float filt_gy, float filt_gz,
                               float pid_roll_out, float pid_pitch_out, float pid_yaw_out,
                               float pid_roll_err, float pid_pitch_err, float pid_yaw_err,
                                float acc_x, float acc_y, float acc_z);

// PID调试数据帧 - Roll/Pitch 角度和角速度 (0xF3)
// 发送 roll/pitch 的 target 和 measure (角度 + 角速度)
void ANO_DT_Send_PID_Debug(uint8_t frame_type,
    float roll_angle_target, float roll_angle_measure, float roll_rate_target, float roll_rate_measure,
    float pitch_angle_target, float pitch_angle_measure, float pitch_rate_target, float pitch_rate_measure,
    float yaw_angle_target, float yaw_angle_measure, float yaw_rate_target, float yaw_rate_measure);

void ANO_DT_Init(void);
void ANO_DT_CallBack(void* this);



// 自定义 RCData 帧 - 使用 float 类型发送（解决负数显示问题）
void ANO_DT_Send_RCDataFloat(float yaw, float roll, float pitch, float x_velocity_output, float y_velocity_output, float target_height, float velocity_x, float velocity_y, float velocity_z, float target_vel_x, float target_vel_y, float acc_z);

#endif

