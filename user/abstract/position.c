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
#define IMU_PRED_BLEND_WEIGHT   0.15f   /* 光流正常时 IMU 融合权重 */
#define IMU_PRED_MAX_DT         0.30f   /* 纯预测最大持续时间(s) */
#define IMU_GRAVITY             980.665f /* 重力加速度 cm/s² */

float_velocity velocity;
static float height_compensated = 0.0f;
static float_xy_pos position;
static bool pos_initialized = false;

#define POSITION_GYRO_COMP_GAIN        1.0f
#define POSITION_GYRO_COMP_DEADBAND    2.0f
#define POSITION_ANGLE_DIFF_COMP_GAIN  1.0f
#define POSITION_USE_ANGLE_DIFF_COMP   1
#define POSITION_USE_GYRO_COMP         0
#define POSITION_MIN_TILT_FACTOR       0.3f
#define POSITION_INVALID_VEL_DECAY     0.8f
#define POSITION_VEL_LPF_ALPHA         0.20f
#define POSITION_VEL_DEADBAND          1.0f
#define POSITION_MAX_VELOCITY_CM_S     120.0f
#define POSITION_MAX_INTEGRATE_DT      0.5f
#define POSITION_Z_UPDATE_MS           300U
#define POSITION_LOG_INTERVAL_MS       100U

void position_Update(const float_angle* angle, const float_gyro* gyro);

/* ---- IMU 短时预测状态（文件级，供 ResetFlowState 访问）---- */
static uint32_t last_imu_ms = 0U;
static float    imu_pred_vx = 0.0f;
static float    imu_pred_vy = 0.0f;
static float    imu_pred_dt_total = 0.0f;

void position_ResetXY(void)
{
    position.x = 0.0f;
    position.y = 0.0f;
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
    if (last_imu_ms != 0U) {
        imu_dt = (now - last_imu_ms) / 1000.0f;
        if (imu_dt > 0.0f && imu_dt < IMU_PRED_MAX_DT) {
            float acc_x = stcAcc.a[0] * 100.f; /* m/s² → cm/s² */
            float acc_y = stcAcc.a[1] * 100.f;
            float pitch_rad = stcAngle.Angle[1] * DEG_TO_RAD;
            float roll_rad  = stcAngle.Angle[0] * DEG_TO_RAD;

            /* 重力补偿：剔除倾斜引起的重力分量 */
            float acc_hx = acc_x - IMU_GRAVITY * sinf(pitch_rad);
            float acc_hy = acc_y + IMU_GRAVITY * sinf(roll_rad);

            /* 积分得到 IMU 预测速度 */
            imu_pred_vx += acc_hx * imu_dt;
            imu_pred_vy += acc_hy * imu_dt;
        }
    }
    last_imu_ms = now;

    if (have_sample)
    {
        uint32_t distance_raw = s.payload.distance;
        uint32_t distance_filtered = (uint32_t)(kalmanFilter_A((float)distance_raw) + 0.5f);
        bool tof_valid = (distance_raw > 0U) && (distance_filtered > 0U);

        float raw_vx = 0.0f;
        float raw_vy = 0.0f;
        float tilt_factor = 1.0f;
        float gyro_comp_x = 0.0f;
        float gyro_comp_y = 0.0f;
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

                raw_vx =   s.payload.flow_vel_y * (distance_filtered / 1000.0f); // cm/s, 前进(pitch)
                raw_vy = - s.payload.flow_vel_x * (distance_filtered / 1000.0f); // cm/s, 左右(roll)
                final_vx = raw_vx * tilt_factor;
                final_vy = raw_vy * tilt_factor;

                /* IMU 预测融合：光流为主，IMU 为辅 */
                imu_pred_dt_total = 0.0f;
                final_vx += IMU_PRED_BLEND_WEIGHT * (imu_pred_vx - final_vx);
                final_vy += IMU_PRED_BLEND_WEIGHT * (imu_pred_vy - final_vy);
                /* 同步 IMU 预测到融合结果，避免切换跳变 */
                imu_pred_vx = final_vx;
                imu_pred_vy = final_vy;

#if POSITION_USE_ANGLE_DIFF_COMP
                if (angle_valid && angle_comp_initialized && (xy_dt > 0.0f))
                {
                    gyro_comp_x = ((tan_pitch - last_tan_pitch) / xy_dt) *
                                  height_compensated * POSITION_ANGLE_DIFF_COMP_GAIN;
                    gyro_comp_y = ((tan_roll - last_tan_roll) / xy_dt) *
                                  height_compensated * POSITION_ANGLE_DIFF_COMP_GAIN;
                    final_vx -= gyro_comp_x;
                    final_vy -= gyro_comp_y;
                }
#elif POSITION_USE_GYRO_COMP
                if (gyro &&
                    ((fabsf(gyro->y) > POSITION_GYRO_COMP_DEADBAND) ||
                     (fabsf(gyro->x) > POSITION_GYRO_COMP_DEADBAND)))
                {
                    float height_m = height_compensated / 100.0f;
                    gyro_comp_x = gyro->y * DEG_TO_RAD * height_m * 100.0f * POSITION_GYRO_COMP_GAIN;
                    gyro_comp_y = gyro->x * DEG_TO_RAD * height_m * 100.0f * POSITION_GYRO_COMP_GAIN;
                    final_vx -= gyro_comp_x;
                    final_vy -= gyro_comp_y;
                }
#endif

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
                        pos_initialized = true;
                    }
                    position.x += velocity.x * xy_dt;
                    position.y += velocity.y * xy_dt;
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
    imu_pred_dt_total = 0.0f;
    last_imu_ms = 0U;
}
