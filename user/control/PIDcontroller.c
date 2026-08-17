#include "PIDcontroller.h"
#include <string.h>
#include <math.h>
#define is_same_sign(a, b) ((a >= 0 && b >= 0) || (a < 0 && b < 0))
/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (is_same_sign(pid->Err, pid->Iout))
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}
static void f_Integral_Separate(PIDInstance *pid)
{
    if (fabsf(pid->Err) > pid->Intergral_Separate)
    {
        pid->Iout = 0;// 误差小于分离值,不使用积分
        pid->ITerm = 0;
    }
    
}
static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        // if (pid->Err * pid->Iout > 0) // 积分却还在累积
        // {
        //     pid->ITerm = 0; // 当前积分项置零
        // }
        if((pid->Ki>0&&is_same_sign(pid->Err, pid->Iout))
            ||pid->Ki<0&&(!is_same_sign(pid->Err, pid->Iout))){
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

//时间间隔限幅
static void f_DeltaT_Limit(PIDInstance *pid)
{
    if (pid->dt > pid->DeltaT_Limit_Max)
        pid->dt = pid->DeltaT_Limit_Max;
    if (pid->dt < pid->DeltaT_Limit_Min)
        pid->dt = pid->DeltaT_Limit_Min;
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // 清零整个PID实例
    memset(pid, 0, sizeof(PIDInstance));
    
    // 显式复制配置参数（避免依赖内存布局）
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    pid->DeadBandOutputMode = config->DeadBandOutputMode;
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    pid->Intergral_Separate = config->Intergral_Separate;
    pid->DeltaT_Limit_Max = config->DeltaT_Limit_Max;
    pid->DeltaT_Limit_Min = config->DeltaT_Limit_Min;
    pid->NominalDt = config->NominalDt;
    
    // 初始化时间戳
    GetDeltaT(&pid->lastTime);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{

    // P2: 默认使用固定 nominal dt，避免 HAL_GetTick 1ms 量化误差导致 D 项抖动
    {
        uint32_t now = HAL_GetTick();
        uint32_t tick_diff = now - pid->lastTime;
        pid->lastTime = now;
        // 实际间隔 > nominal*2 视为明显超期（HAL_GetTick 1ms 量化抖动不会触发此阈值）
        uint8_t dt_timeout = (tick_diff > (uint32_t)(pid->NominalDt * 2000.0f));
        pid->dt = dt_timeout ? (float)tick_diff / 1000.0f : pid->NominalDt;
        pid->dt_timeout = dt_timeout;
    }
    if(pid->Improve & PID_DeltaT_Limit)
        f_DeltaT_Limit(pid); // 时间间隔限幅（超期时防止 dt 过大导致 I 项失控）

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // P0: 首次计算只建立历史基线，不输出 D 项，避免 (0-Measure)/dt 或 (Err-0)/dt 产生虚假尖峰
    if (!pid->initialized)
    {
        pid->Last_Measure = pid->Measure;
        pid->Last_Err     = pid->Err;
        pid->Last_Dout    = 0;
        pid->Last_ITerm   = 0;
        pid->Last_Output  = pid->Output;
        pid->initialized  = 1;
        return pid->Output;
    }

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) >= pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;

        // P2: 超期时跳过 D 项全部计算，避免错误微分
        if (pid->dt_timeout)
        {
            pid->Dout = 0;
        }
        else
        {
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
            // 微分先行
            if (pid->Improve & PID_Derivative_On_Measurement)
                f_Derivative_On_Measurement(pid);
            // 微分滤波器
            if (pid->Improve & PID_DerivativeFilter)
                f_Derivative_Filter(pid);
        }

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 积分分离
        if (pid->Improve & PID_Integral_Separate)
            f_Integral_Separate(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区
    {
        if (pid->DeadBandOutputMode == PID_DEADBAND_OUTPUT_HOLD)
        {
            pid->Output = pid->Last_Output;
        }
        else
        {
            pid->Output = 0;
            pid->Last_Output = 0;
        }
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

void PID_ClearIntegral(PIDInstance *pid)
{
    pid->Iout = 0;
    pid->ITerm = 0;
    pid->Last_ITerm = 0;
}

/**
 * @brief P1: 旁路 PID 重新启用前的状态同步。
 *        更新时间戳防止 dt 异常，触发下次 PIDCalculate 重新建基线（不输出 D 项）。
 * @note  不清零积分/输出，适用于模式切换（如速度模式→定点模式）。
 */
void PID_PrepareReengage(PIDInstance *pid)
{
    GetDeltaT(&pid->lastTime);  // 同步时间戳，避免下次 dt 过大
    pid->initialized = 0;       // 触发下次计算重新建基线
}

void PID_Reset(PIDInstance *pid)
{
    pid->Measure = 0;
    pid->Last_Measure = 0;
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Ref = 0;
    pid->dt = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->ITerm = 0;
    pid->Output = 0;
    pid->Last_Output = 0;
    pid->Last_Dout = 0;
    pid->Last_ITerm = 0;
    GetDeltaT(&pid->lastTime);
    pid->initialized = 0;   // P0: 下次 PIDCalculate 第一拍只建基线，不输出 D
}

/**
 * @brief 获取 PID 运行状态快照（调试/遥测用）
 * @param pid  PID 实例指针
 * @param out  快照输出
 * @note  纯值拷贝，不加锁。在单任务控制场景下并发安全。
 *        若未来改为多任务控制，需要在调用侧统一加临界区。
 */
void pid_dump(const PIDInstance *pid, PIDDump *out)
{
    if (!pid || !out) return;
    out->Output  = pid->Output;
    out->Pout    = pid->Pout;
    out->Iout    = pid->Iout;
    out->Dout    = pid->Dout;
    out->ITerm   = pid->ITerm;
    out->Err     = pid->Err;
    out->Measure = pid->Measure;
    out->Ref     = pid->Ref;
}

/**
 * @brief 读取 PID 配置参数（Kp/Ki/Kd 等），不含运行状态
 */
void pid_get_config(const PIDInstance *pid, PID_Init_Config_s *out)
{
    if (!pid || !out) return;
    out->Kp   = pid->Kp;
    out->Ki   = pid->Ki;
    out->Kd   = pid->Kd;
    out->MaxOut = pid->MaxOut;
    out->DeadBand = pid->DeadBand;
    out->DeadBandOutputMode = pid->DeadBandOutputMode;
    out->Improve = pid->Improve;
    out->IntegralLimit = pid->IntegralLimit;
    out->CoefA = pid->CoefA;
    out->CoefB = pid->CoefB;
    out->Output_LPF_RC = pid->Output_LPF_RC;
    out->Derivative_LPF_RC = pid->Derivative_LPF_RC;
    out->Intergral_Separate = pid->Intergral_Separate;
    out->DeltaT_Limit_Max = pid->DeltaT_Limit_Max;
    out->DeltaT_Limit_Min = pid->DeltaT_Limit_Min;
}

/**
 * @brief 写入 PID 配置参数。调用侧需确保不在 PIDCalculate 并发上下文。
 */
void pid_set_config(PIDInstance *pid, const PID_Init_Config_s *cfg)
{
    if (!pid || !cfg) return;
    pid->Kp   = cfg->Kp;
    pid->Ki   = cfg->Ki;
    pid->Kd   = cfg->Kd;
    pid->MaxOut = cfg->MaxOut;
    pid->DeadBand = cfg->DeadBand;
    pid->DeadBandOutputMode = cfg->DeadBandOutputMode;
    pid->Improve = cfg->Improve;
    pid->IntegralLimit = cfg->IntegralLimit;
    pid->CoefA = cfg->CoefA;
    pid->CoefB = cfg->CoefB;
    pid->Output_LPF_RC = cfg->Output_LPF_RC;
    pid->Derivative_LPF_RC = cfg->Derivative_LPF_RC;
    pid->Intergral_Separate = cfg->Intergral_Separate;
    pid->DeltaT_Limit_Max = cfg->DeltaT_Limit_Max;
    pid->DeltaT_Limit_Min = cfg->DeltaT_Limit_Min;
}
