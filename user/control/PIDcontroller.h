/**
 ******************************************************************************
 * @file	 PIDcontroller.h
 * @author  sevenfite  email:lin481399413@163.com
 * @date    2025/5/13
 * @brief pid控制器头文件
 ******************************************************************************
 * @attention
 * 看README文件
 * @note
 *  参照了湖南大学RobotMaster跃鹿战队的代码(author  Wang Hongxi)
 ******************************************************************************
 */
#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

#include <stdint.h>
#include "globalTime.h"

#define  PID_DEADBAND_OUTPUT_ZERO  (0x00)
#define  PID_DEADBAND_OUTPUT_HOLD  (0x01)

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式

#define  PID_IMPROVE_NONE  (0x00)                   // 0000 0000
#define  PID_Integral_Limit  (0x01<<0)              // 0000 0001  // 积分限幅
#define  PID_Derivative_On_Measurement  (0x01<<1)   // 0000 0010  // 微分先行
#define  PID_Trapezoid_Intergral  (0x01<<2)         // 0000 0100  // 梯形积分
#define  PID_DeltaT_Limit  (0x01<<3)                // 0000 1000  // 时间间隔限幅
#define  PID_OutputFilter  (0x01<<4)                // 0001 0000  // 输出滤波器
#define  PID_ChangingIntegrationRate  (0x01<<5)     // 0010 0000  // 变速积分
#define  PID_DerivativeFilter  (0x01<<6)            // 0100 0000  // 微分滤波器
#define  PID_Integral_Separate  (0x01<<7)           // 1000 0000  // 积分分离

// 若希望使用多个环节的优化，这样就行：Integral_Limit |Trapezoid_Intergral|...|...

/* ── ⚠️ 外部代码禁止直接读写 PIDInstance 字段 ──
 * 读运行状态请用 pid_dump(), 读配置参数请用 pid_get_config(),
 * 写配置参数请用 pid_set_config(). 违反此约定会导致并发不安全。 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;
    uint8_t DeadBandOutputMode;

    // improve parameter
    uint8_t Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数
    float Intergral_Separate; // 积分分离值
    float DeltaT_Limit_Max;     // 时间间隔限幅
    float DeltaT_Limit_Min;     // 时间间隔限幅
    float NominalDt;            // 固定控制周期(s)，正常时使用此值避免HAL_GetTick量化误差

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t lastTime;
    float dt;
    uint8_t initialized;   // P0: 首次计算标志，Reset后第一拍不输出D项
    uint8_t dt_timeout;    // P2: 超期标志，跳过D项避免错误微分

} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区
    uint8_t DeadBandOutputMode;

    // improve parameter
    uint8_t Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
    float Intergral_Separate; // 积分分离值
    float DeltaT_Limit_Max;     // 时间间隔限幅
    float DeltaT_Limit_Min;     // 时间间隔限幅
    float NominalDt;            // 固定控制周期(s)，如500Hz环=0.002f
} PID_Init_Config_s;

/**
 * @brief PID 运行状态快照，用于调试/遥测，不包含配置参数。
 * @note  外部代码通过 pid_dump() 获取原子快照，禁止直接读 PIDInstance 内部字段。
 */
typedef struct
{
    float Output;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Err;
    float Measure;
    float Ref;
} PIDDump;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

/**
 * @brief 清零 PID 所有输出变量
 */
void PID_ClearIntegral(PIDInstance *pid);

void PID_PrepareReengage(PIDInstance *pid); // P1: 旁路PID重新启用前状态同步

void PID_Reset(PIDInstance *pid);

/**
 * @brief 获取 PID 运行状态快照（调试/遥测用）
 * @param pid  PID 实例指针
 * @param out  快照输出
 */
void pid_dump(const PIDInstance *pid, PIDDump *out);

/**
 * @brief 读取 PID 配置参数（Kp/Ki/Kd 等），不含运行状态
 * @param pid  PID 实例指针
 * @param out  配置输出
 */
void pid_get_config(const PIDInstance *pid, PID_Init_Config_s *out);

/**
 * @brief 写入 PID 配置参数。调用侧需确保不在 PIDCalculate 并发上下文。
 * @param pid  PID 实例指针
 * @param cfg  新配置
 */
void pid_set_config(PIDInstance *pid, const PID_Init_Config_s *cfg);

/**
 * @brief 获得时间间隔
 * 
 * @param lastTime 上一次的时间戳
 * @return float 
 * @note 此函数为接口函数，需自行实现，以下实现为示例。要求返回与上一次的时间间隔，并更新lastTime，此时间间隔用于积分项与微分项和滤波器的计算
 * @note 间隔时间建议以秒为单位，这样在关于滤波器截止频率的计算中会比较方便
 */
static inline float GetDeltaT(uint32_t *lastTime)
{
    uint32_t HAL_GetTick();//函数声明
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - *lastTime) / 1000.0f; // 转换为秒
    *lastTime = now;
    return dt;
}

#endif
