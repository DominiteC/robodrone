#include "control_pid.h"

#define DEFAULT_LPF_RC 0.00159f
#define DEFAULT_PID_INTEGRATION_LIMIT 		1000.f //默认pid的积分限幅

/*
 * PID 参数集中放在这里，便于后续独立校准和持久化。
 * 本阶段只迁移定义位置，不修改任何 PID 参数数值。
 */
PID_Init_Config_s pid_height_position_config = {
    .Kp = 1.5f,
    .Ki = 0.0f,
    .Kd = 0.005f,
    .MaxOut = 80.0f, //限幅
    .DeadBand = 0,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = 80.f,
};
PIDInstance pid_height_position;    // 高度位置环

PID_Init_Config_s pid_x_position_config = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 1200.0f, //输出速度限幅 cm/s
    .DeadBand = 3.0f,
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f,
    .DeltaT_Limit_Min = 0.0001f,
    .IntegralLimit = 1200.f,
};
PID_Init_Config_s pid_y_position_config = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 1200.0f,
    .DeadBand = 3.0f,
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f,
    .DeltaT_Limit_Min = 0.0001f,
    .IntegralLimit = 1200.f,
};
PIDInstance pid_x_position;    // X位置环
PIDInstance pid_y_position;    // Y位置环

PID_Init_Config_s pid_x_velocity_config = {
    .Kp = 0.11f,
    .Ki = 0.001f,
    .Kd = 0.003f,
    .MaxOut = 50.0f, //限幅
    .DeadBand =3.0f,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit| PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
	  .Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};
PID_Init_Config_s pid_y_velocity_config = {
    .Kp = 0.11f,
    .Ki = 0.001f,
    .Kd = 0.003f,
    .MaxOut = 50.0f, //限幅
    .DeadBand =3.0f,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit| PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
	  .Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};


PID_Init_Config_s pid_z_velocity_config = {
    .Kp = 0.050f,
    .Ki = 0.005f,
    .Kd = 0.005f,
    .MaxOut = 50.0f, //限幅
    .DeadBand =0.2,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_HOLD,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit| PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};
PIDInstance pid_x_velocity; // x轴方向速度环
PIDInstance pid_y_velocity; // y轴方向速度环
PIDInstance pid_z_velocity; // z轴方向速度环

PID_Init_Config_s pid_roll_angle_config = {
//    .Kp = 3.5f,
		.Kp = 1.7f,//4.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 80.0f, //限幅
    .DeadBand =0.1,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};
PID_Init_Config_s pid_pitch_angle_config = {
//    .Kp = 4.5f,
	.Kp = 1.7f,//4.0
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 80.0f, //限幅
    .DeadBand =0.1,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};
PID_Init_Config_s pid_yaw_angle_config = {
//    .Kp = 3.0f,
	.Kp = 2.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = 100.0f, //限幅
    .DeadBand =0.1,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_OutputFilter | PID_Integral_Limit,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .DeltaT_Limit_Max = 0.1f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = DEFAULT_PID_INTEGRATION_LIMIT,
};
PIDInstance pid_roll_angle;     // 横滚角度环
PIDInstance pid_pitch_angle;    // 俯仰角度环
PIDInstance pid_yaw_angle;      // 航向角度环

PID_Init_Config_s pid_roll_rate_config = {
    .Kp = 0.08f,//0.15
    .Ki = 0.008,
    .Kd = 0.001f,
    .MaxOut = 15.0f, //限幅
    .DeadBand =0.2,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
    .Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.01f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = 5,
};
PID_Init_Config_s pid_pitch_rate_config = {
    .Kp = 0.08f,//0.2
    .Ki = 0.008f,
    .Kd = 0.001f,
    .MaxOut = 15.0f, //限幅
    .DeadBand =0.2,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
		.Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.01f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = 5,
};
PID_Init_Config_s pid_yaw_rate_config = {
    .Kp = 0.6f,
    .Ki = 0.008f,
    .Kd = 0.001f,
    .MaxOut = 8.0f, //限幅
    .DeadBand =0.3,//差值大于5才进行PID调节，防止抖动
    .DeadBandOutputMode = PID_DEADBAND_OUTPUT_ZERO,
    .Improve = PID_DeltaT_Limit | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    .Output_LPF_RC = DEFAULT_LPF_RC,
		.Derivative_LPF_RC = 0.003f,
    .DeltaT_Limit_Max = 0.01f, //时间间隔限幅
    .DeltaT_Limit_Min = 0.0001f, //时间间隔限幅
    .IntegralLimit = 8.f,
};
PIDInstance pid_roll_rate;  // 横滚角速度环
PIDInstance pid_pitch_rate; // 俯仰角速度环
PIDInstance pid_yaw_rate;   // 航向角速度环
void Control_Init(void)
{
    // 暂时在main函数中初始化模块和动力，之后会移到这里
    // PID初始化
    PIDInit(&pid_height_position, &pid_height_position_config);
    PIDInit(&pid_x_position, &pid_x_position_config);
    PIDInit(&pid_y_position, &pid_y_position_config);
    PIDInit(&pid_x_velocity, &pid_x_velocity_config);
    PIDInit(&pid_y_velocity, &pid_y_velocity_config);
    PIDInit(&pid_z_velocity, &pid_z_velocity_config);
    PIDInit(&pid_roll_angle, &pid_roll_angle_config);
    PIDInit(&pid_pitch_angle, &pid_pitch_angle_config);
    PIDInit(&pid_yaw_angle, &pid_yaw_angle_config);
    PIDInit(&pid_roll_rate, &pid_roll_rate_config);
    PIDInit(&pid_pitch_rate, &pid_pitch_rate_config);
    PIDInit(&pid_yaw_rate, &pid_yaw_rate_config);
}
