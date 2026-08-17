#include "position.h"
#include "mtf_01.h"
#include "mtf_01_stream.h"
#include "globalTime.h"
#include "C_code_Log.h"
#include "PIDcontroller.h"
#include "control_pid.h"
#include "jy901p.h"
#include <math.h>

/* ---- IMU 短时预测参数 ---- */
#define IMU_PRED_BLEND_WEIGHT   0.15f   /* Z 轴 IMU 融合权重 (光流辅助) */
#define IMU_PRED_MAX_DT         0.30f   /* 纯预测最大持续时间(s) */
#define IMU_GRAVITY             980.665f /* 重力加速度 cm/s² */
#define INAV_W_OPFLOW_VEL       2.0f    /* INAV CorrectVel 速度残差权重, 参考 MiniFly wOpflowV */
#define INAV_W_OPFLOW_POS       1.0f    /* INAV CorrectPos 位置残差权重, 参考 MiniFly wOpflowP */

//-------------------------位置/速度状态变量-----------------------------------
float_velocity velocity;              /* 飞行器三轴速度 (cm/s), X/Y 来自光流+IMU, Z 来自超声波 */
static float height_compensated = 0.0f;  /* 倾斜补偿后的高度值 (cm), 由超声波原始距离 × cos(roll)×cos(pitch) */
static float_xy_pos position;          /* 飞行器 XY 位置 (cm), 由速度时间积分得到 */
static bool pos_initialized = false;  /* XY 位置是否已初始化 (首次有效光流数据后置1) */
//-------------------------位置/速度状态变量-----------------------------------

#define POSITION_GYRO_COMP_GAIN        1.0f  /* 陀螺仪旋转补偿增益 (PX4: 理论值=1.0) */
#define POSITION_GYRO_COMP_DEADBAND    3.0f  /* 陀螺死区 (deg/s), 静止时过滤零偏噪声 */
#define POSITION_MIN_TILT_FACTOR       0.3f
#define POSITION_INVALID_VEL_DECAY     0.8f
#define POSITION_VEL_LPF_ALPHA         0.20f
#define POSITION_VEL_DEADBAND          1.0f
#define POSITION_MAX_VELOCITY_CM_S     120.0f
#define POSITION_MAX_INTEGRATE_DT      0.5f
#define POSITION_Z_UPDATE_MS           100U
#define POSITION_LOG_INTERVAL_MS       100U

void position_Update(const float_angle* angle, const float_gyro* gyro);

/* ---- IMU 短时预测状态（文件级，供 ResetFlowState 访问）---- */
//-------------------------IMU 短时预测状态-----------------------------------
static uint32_t last_imu_ms = 0U;     /* 上次 IMU 加速度积分时间戳 (ms) */
static float    imu_pred_vx = 0.0f;    /* IMU 加速度积分预测的 X 速度 (cm/s) */
static float    imu_pred_vy = 0.0f;    /* IMU 加速度积分预测的 Y 速度 (cm/s) */
static float    imu_pred_vz = 0.0f;    /* IMU 加速度积分预测的 Z 速度 (cm/s) */
static float    imu_pred_dt_total = 0.0f;  /* IMU 纯预测累计持续时间 (s), 超时后衰减归零 */
//-------------------------IMU 短时预测状态-----------------------------------

//-------------------------光流独立累积位移-----------------------------------
static float    opflow_pos_sum_x = 0.0f;    /* 光流原始速度独立累积 X 位移 (cm), 用于残差反馈修正 */
static float    opflow_pos_sum_y = 0.0f;    /* 光流原始速度独立累积 Y 位移 (cm), 用于残差反馈修正 */
//-------------------------光流独立累积位移-----------------------------------

//-------------------------Predict 调试变量 (供 ANO_DT 遥测)------------------
float debug_acc_hx = 0.0f;                  /* 重力补偿后水平加速度 X (cm/s²), Predict 输入 */
float debug_acc_hy = 0.0f;                  /* 重力补偿后水平加速度 Y (cm/s²) */
float debug_flow_residual_x = 0.0f;         /* 光流位置残差 (opflow_pos - position) X (cm) */
//-------------------------Predict 调试变量-----------------------------------

void position_ResetXY(void)
{
    position.x = 0.0f;
    position.y = 0.0f;
    opflow_pos_sum_x = 0.0f;
    opflow_pos_sum_y = 0.0f;
    pos_initialized = false;
}

static float kalmanFilter_A(float inData)
{
    static float prevData = 0.0f;
    static float p = 10.0f, q = 0.001f, r = 0.4f, kGain = 0.0f;

    p = p + q;
    kGain = p / (p + r);

    inData = prevData + (kGain * (inData - prevData));
    p = (1.0f - kGain) * p;

    prevData = inData;

    return inData;
}

static void decay_xy_velocity(void)
{
    velocity.x *= POSITION_INVALID_VEL_DECAY;
    velocity.y *= POSITION_INVALID_VEL_DECAY;

    if (fabsf(velocity.x) < POSITION_VEL_DEADBAND) velocity.x = 0.0f;
    if (fabsf(velocity.y) < POSITION_VEL_DEADBAND) velocity.y = 0.0f;
}

static float limit_velocity(float value)
{
    if (value > POSITION_MAX_VELOCITY_CM_S) return POSITION_MAX_VELOCITY_CM_S;
    if (value < -POSITION_MAX_VELOCITY_CM_S) return -POSITION_MAX_VELOCITY_CM_S;
    return value;
}

static float filter_xy_velocity(float previous, float measurement)
{
    float filtered = previous + POSITION_VEL_LPF_ALPHA * (measurement - previous);

    filtered = limit_velocity(filtered);
    if (fabsf(filtered) < POSITION_VEL_DEADBAND) filtered = 0.0f;

    return filtered;
}

void position_init(void)
{
    mtf_01_init();
}

void position_GetVelocity(float_velocity* vel, const float_angle* angle, const float_gyro* gyro)
{
    position_Update(angle, gyro);
    if (vel)
    {
        vel->x = velocity.x;
        vel->y = velocity.y;
        vel->z = velocity.z;
    }
}

void position_GetHeight(float* height)
{
    if (height) *height = height_compensated;
}

void position_GetPosition(float_xy_pos* pos)
{
    if (pos)
    {
        pos->x = position.x;
        pos->y = position.y;
    }
}

void position_Update(const float_angle* angle, const float_gyro* gyro)
{
    static uint32_t last_xy_time = 0U;
    static uint32_t last_z_time = 0U;
    static uint32_t last_log_time = 0U;
    static int32_t last_distance_filtered = 0;
    static float last_velocity_z = 0.0f;
    static bool angle_comp_initialized = false;
    static float last_tan_pitch = 0.0f;
    static float last_tan_roll = 0.0f;

    uint32_t now = getGlobalTime();
    MtfSample s;
    bool have_sample = mtf_01_get_latest_sample(&s, now);
    bool flow_valid = mtf_01_is_flow_usable(now);

    /* ---- IMU 短时预测（加速度积分）---- */
    float imu_dt = 0.0f;
    float acc_hx = 0.0f;   /* 重力补偿后水平加速度 X (cm/s²), 供位置预测复用 */
    float acc_hy = 0.0f;   /* 重力补偿后水平加速度 Y (cm/s²), 供位置预测复用 */
    if (last_imu_ms != 0U) {
        imu_dt = (now - last_imu_ms) / 1000.0f;
        if (imu_dt > 0.0f && imu_dt < IMU_PRED_MAX_DT) {
            float acc_x = stcAcc.a[0] * 100.f; /* m/s² → cm/s² */
            float acc_y = stcAcc.a[1] * 100.f;
            float pitch_rad = angle->pitch * DEG_TO_RAD;
            float roll_rad  = angle->roll * DEG_TO_RAD;

            /* 重力补偿：剔除倾斜引起的重力分量 */
            acc_hx = acc_x - IMU_GRAVITY * sinf(pitch_rad);
            acc_hy = acc_y - IMU_GRAVITY * sinf(roll_rad);

            /* 死区：静止时 ±4cm/s² 以下不积分，防零偏/姿态误差漂移 (MiniFly 同款) */
            if (fabsf(acc_hx) < 4.0f) acc_hx = 0.0f;
            if (fabsf(acc_hy) < 4.0f) acc_hy = 0.0f;

            /* Z 方向重力补偿: 倾斜时重力分量 = G * cos(pitch) * cos(roll) */
            float acc_z = stcAcc.a[2] * 100.f;
            float acc_hz = acc_z - IMU_GRAVITY * cosf(pitch_rad) * cosf(roll_rad);

            /* 积分得到 IMU 预测速度 */
            imu_pred_vx += acc_hx * imu_dt;
            imu_pred_vy += acc_hy * imu_dt;
            imu_pred_vz += acc_hz * imu_dt;
        }
    }
    last_imu_ms = now;
    /* 曝光 Predict 输入源供地面站验证 (ANO_DT 通道 10/11) */
    debug_acc_hx = acc_hx;
    debug_acc_hy = acc_hy;

    if (have_sample)
    {
        uint32_t distance_raw = s.payload.distance;

        /* 跳变保护：相邻两帧原始距离变化 > 50% 时丢弃当前帧，防止 TOF 异常值
         * 在 kalmanFilter 之前拦截，避免滤波内部状态被污染 */
        static uint32_t distance_raw_prev = 0U;
        bool raw_valid = (distance_raw > 0U);
        if (raw_valid && distance_raw_prev > 0U) {
            uint32_t delta = (distance_raw > distance_raw_prev)
                           ? (distance_raw - distance_raw_prev)
                           : (distance_raw_prev - distance_raw);
            if (delta > distance_raw_prev / 2U) {
                distance_raw = distance_raw_prev;  /* 用上一帧值替代 */
            }
        }
        if (raw_valid) distance_raw_prev = distance_raw;

        uint32_t distance_filtered = (uint32_t)(kalmanFilter_A((float)distance_raw) + 0.5f);
        bool tof_valid = (distance_raw > 0U) && (distance_filtered > 0U);

        float raw_vx = 0.0f;
        float raw_vy = 0.0f;
        float tilt_factor = 1.0f;
        float final_vx = velocity.x;
        float final_vy = velocity.y;
        float xy_dt = 0.0f;

        if (tof_valid)
        {
            float raw_height = distance_filtered / 10.0f; // cm
            float tan_pitch = 0.0f;
            float tan_roll = 0.0f;
            bool angle_valid = false;

            if (angle && raw_height > 0.1f)
            {
                float cos_roll = cosf(angle->roll * DEG_TO_RAD);
                float cos_pitch = cosf(angle->pitch * DEG_TO_RAD);
                tilt_factor = cos_roll * cos_pitch;
                tan_pitch = tanf(angle->pitch * DEG_TO_RAD);
                tan_roll = tanf(angle->roll * DEG_TO_RAD);
                angle_valid = true;

                if (tilt_factor > POSITION_MIN_TILT_FACTOR)
                {
                    height_compensated = raw_height * tilt_factor;
                }
                else
                {
                    tilt_factor = 1.0f;
                    height_compensated = raw_height;
                }
            }
            else
            {
                height_compensated = raw_height;
            }

            if (flow_valid)
            {
                if (last_xy_time != 0U)
                {
                    xy_dt = (now - last_xy_time) / 1000.0f;
                    if ((xy_dt <= 0.0f) || (xy_dt > POSITION_MAX_INTEGRATE_DT))
                    {
                        xy_dt = 0.0f;
                    }
                }

                raw_vx =   s.payload.flow_vel_y * (distance_filtered / 1000.0f); // cm/s
                raw_vy = - s.payload.flow_vel_x * (distance_filtered / 1000.0f); // cm/s

                /* 前置低通: 压制光流尖峰再进融合 (MiniFly α=0.15) */
                {
                    static float raw_vx_lpf = 0.0f, raw_vy_lpf = 0.0f;
                    raw_vx_lpf += (raw_vx - raw_vx_lpf) * 0.15f;
                    raw_vy_lpf += (raw_vy - raw_vy_lpf) * 0.15f;
                    raw_vx = raw_vx_lpf;
                    raw_vy = raw_vy_lpf;
                }

                /* 陀螺仪旋转补偿: 剔除机身旋转引起的视运动 (PX4 flow.cpp 同款)
                 * ω(rad/s) × h(m) × 100 = 旋转伪速度(cm/s), GAIN≈1.0 理论值 */
                if (gyro &&
                    (fabsf(gyro->x) > POSITION_GYRO_COMP_DEADBAND ||
                     fabsf(gyro->y) > POSITION_GYRO_COMP_DEADBAND)) {
                    float height_m = distance_filtered / 1000.0f;
                    float omega_x = gyro->y * DEG_TO_RAD;  /* pitch角速度 → X伪速度 */
                    float omega_y = gyro->x * DEG_TO_RAD;  /* roll角速度 → Y伪速度 */
                    raw_vx -= omega_x * height_m * 100.0f * POSITION_GYRO_COMP_GAIN;
                    raw_vy -= omega_y * height_m * 100.0f * POSITION_GYRO_COMP_GAIN;
                }

                final_vx = raw_vx * tilt_factor;
                final_vy = raw_vy * tilt_factor;

                /* INAV CorrectVel: 光流速度残差修正 IMU 预测速度 */
                imu_pred_dt_total = 0.0f;
                {
                    float e_vx = final_vx - imu_pred_vx;  /* 残差 = 光流观测 - IMU预测 */
                    float e_vy = final_vy - imu_pred_vy;
                    final_vx = imu_pred_vx + e_vx * INAV_W_OPFLOW_VEL * xy_dt;
                    final_vy = imu_pred_vy + e_vy * INAV_W_OPFLOW_VEL * xy_dt;
                }
                /* 同步 IMU 预测到融合结果，避免切换跳变 */
                imu_pred_vx = final_vx;
                imu_pred_vy = final_vy;

                final_vx = filter_xy_velocity(velocity.x, final_vx);
                final_vy = filter_xy_velocity(velocity.y, final_vy);

                velocity.x = final_vx;
                velocity.y = final_vy;

                if (xy_dt > 0.0f)
                {
                    if (!pos_initialized)
                    {
                        position.x = 0.0f;
                        position.y = 0.0f;
                        opflow_pos_sum_x = 0.0f;
                        opflow_pos_sum_y = 0.0f;
                        pos_initialized = true;
                    }

                    /* 光流独立累积位移 (原始速度, 未融合 IMU, 不含角速度补偿) */
                    opflow_pos_sum_x += raw_vx * xy_dt;
                    opflow_pos_sum_y += raw_vy * xy_dt;

                    /* 融合速度积分位置 + INAV 加速度预测 (pos += v·dt + a·dt²/2) */
                    float vel_inc_x = velocity.x * xy_dt;
                    float vel_inc_y = velocity.y * xy_dt;
                    float pred_inc_x = acc_hx * xy_dt * xy_dt * 0.5f;
                    float pred_inc_y = acc_hy * xy_dt * xy_dt * 0.5f;
                    position.x += vel_inc_x + pred_inc_x;
                    position.y += vel_inc_y + pred_inc_y;

                    /* INAV CorrectPos: 位置残差修正位置, 同时联动修正速度 */
                    {
                        float e_px = opflow_pos_sum_x - position.x;
                        float e_py = opflow_pos_sum_y - position.y;
                        float ewdt_x = e_px * INAV_W_OPFLOW_POS * xy_dt;
                        float ewdt_y = e_py * INAV_W_OPFLOW_POS * xy_dt;
                        position.x += ewdt_x;
                        position.y += ewdt_y;
                        velocity.x += INAV_W_OPFLOW_POS * ewdt_x;  /* 联动: vel += wP²·e·dt */
                        velocity.y += INAV_W_OPFLOW_POS * ewdt_y;
                    }

                    /* 曝光残差供地面站验证 (ANO_DT 通道 12) */
                    debug_flow_residual_x = opflow_pos_sum_x - position.x;
                }

                if (angle_valid)
                {
                    last_tan_pitch = tan_pitch;
                    last_tan_roll = tan_roll;
                    angle_comp_initialized = true;
                }
                else
                {
                    angle_comp_initialized = false;
                }
                last_xy_time = now;
            }
            else
            {
                /* 光流失效：IMU 短时预测 + 超时衰减 */
                opflow_pos_sum_x = 0.0f;
                opflow_pos_sum_y = 0.0f;
                imu_pred_dt_total += imu_dt;
                if (imu_pred_dt_total < IMU_PRED_MAX_DT) {
                    final_vx = imu_pred_vx;
                    final_vy = imu_pred_vy;
                } else {
                    /* 超时：衰减归零 */
                    imu_pred_vx *= 0.9f;
                    imu_pred_vy *= 0.9f;
                    final_vx = imu_pred_vx;
                    final_vy = imu_pred_vy;
                }
                last_xy_time = 0U;
                angle_comp_initialized = false;
            }

            if ((now - last_z_time) >= POSITION_Z_UPDATE_MS)
            {
                if (last_z_time != 0U)
                {
                    float dt = (now - last_z_time) / 1000.0f;
                    if (dt > 0.0f)
                    {
                        velocity.z = ((float)((int32_t)distance_filtered - last_distance_filtered)) / dt * 0.1f; // cm/s
                        velocity.z = velocity.z * 0.8f + last_velocity_z * 0.2f;
                        /* 融合 IMU 短时预测，提升速度带宽 (MiniFly INAV 思路) */
                        velocity.z += IMU_PRED_BLEND_WEIGHT * (imu_pred_vz - velocity.z);
                        imu_pred_vz = velocity.z;
                        last_velocity_z = velocity.z;
                    }
                }
                last_distance_filtered = (int32_t)distance_filtered;
                last_z_time = now;
            }
        }
        else
        {
            decay_xy_velocity();
            final_vx = velocity.x;
            final_vy = velocity.y;
            last_xy_time = 0U;
            last_z_time = 0U;
            angle_comp_initialized = false;
        }

        if ((now - last_log_time) >= POSITION_LOG_INTERVAL_MS)
        {
            last_log_time = now;
//            LOG_INFO("flow:%d,%d dist:%lu,%lu q:%u st:%u,%u att:%.2f,%.2f gyro:%.2f,%.2f raw:%.2f,%.2f gc:%.2f,%.2f vel:%.2f,%.2f",
//                     payload.flow_vel_x,
//                     payload.flow_vel_y,
//                     (unsigned long)distance_raw,
//                     (unsigned long)distance_filtered,
//                     payload.flow_quality,
//                     payload.tof_status,
//                     payload.flow_status,
//                     angle ? angle->roll : 0.0f,
//                     angle ? angle->pitch : 0.0f,
//                     gyro ? gyro->x : 0.0f,
//                     gyro ? gyro->y : 0.0f,
//                     raw_vx,
//                     raw_vy,
//                     gyro_comp_x,
//                     gyro_comp_y,
//                     final_vx,
//                     final_vy);
        }
    }
}

bool position_IsXYFlowValid(void)
{
    return mtf_01_is_flow_usable(getGlobalTime());
}

void position_ResetFlowState(void)
{
    mtf_01_stream_init(getGlobalTime());
    position_ResetXY();
    velocity.x = 0.0f;
    velocity.y = 0.0f;
    imu_pred_vx = 0.0f;
    imu_pred_vy = 0.0f;
    imu_pred_vz = 0.0f;
    imu_pred_dt_total = 0.0f;
    opflow_pos_sum_x = 0.0f;
    opflow_pos_sum_y = 0.0f;
    last_imu_ms = 0U;
}
