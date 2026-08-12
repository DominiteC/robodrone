#include "arm_transform.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ----------- 构造变换矩阵 -----------
void arm_transform_from_euler(float tx, float ty, float tz,
                              float roll, float pitch, float yaw,
                              arm_matrix_instance_f32 *T,
                              float32_t *buf)
{
    arm_mat_init_f32(T, 4, 4, buf);

    float cr = cosf(roll), sr = sinf(roll);
    float cbeta = cosf(pitch), sbeta = sinf(pitch);
    float calpha = cosf(yaw), salpha = sinf(yaw);

    // ZYX
    buf[0] = calpha*cbeta;
    buf[1] = calpha*sbeta*sr - salpha*cr;
    buf[2] = calpha*sbeta*cr + salpha*sr;
    buf[3] = tx;

    buf[4] = salpha*cbeta;
    buf[5] = salpha*sbeta*sr + calpha*cr;
    buf[6] = salpha*sbeta*cr - calpha*sr;
    buf[7] = ty;

    buf[8]  = -sbeta;
    buf[9]  = cbeta*sr;
    buf[10] = cbeta*cr;
    buf[11] = tz;

    buf[12] = 0.0f;
    buf[13] = 0.0f;
    buf[14] = 0.0f;
    buf[15] = 1.0f;
}

// ----------- 应用变换到点 -----------
void arm_transform_apply_point(const arm_matrix_instance_f32 *T,
                               const float32_t p_in[4],
                               float32_t p_out[4])
{
    arm_matrix_instance_f32 Pin, Pout;
    arm_mat_init_f32((arm_matrix_instance_f32*)&Pin, 4, 1, (float32_t*)p_in);
    arm_mat_init_f32((arm_matrix_instance_f32*)&Pout,4, 1, p_out);
    arm_mat_mult_f32(T, &Pin, &Pout);
}

// ----------- 应用变换到向量 (只旋转) -----------
void arm_transform_apply_vector(const arm_matrix_instance_f32 *T,
                                const float32_t v_in[3],
                                float32_t v_out[3])
{
    // 只取 T 的左上 3x3 部分
    for (int i=0;i<3;i++) {
        v_out[i] = T->pData[i*4+0]*v_in[0] +
                   T->pData[i*4+1]*v_in[1] +
                   T->pData[i*4+2]*v_in[2];
    }
}

// ----------- 复合两个变换 -----------
arm_status arm_transform_combine(const arm_matrix_instance_f32 *A,
                                 const arm_matrix_instance_f32 *B,
                                 arm_matrix_instance_f32 *C,
                                 float32_t *buffer16)
{
    arm_mat_init_f32(C, 4, 4, buffer16);
    return arm_mat_mult_f32(A, B, C);
}

// ----------- 通用矩阵求逆 -----------
arm_status arm_transform_inverse(const arm_matrix_instance_f32 *T,
                                 arm_matrix_instance_f32 *Tinv,
                                 float32_t *buffer16)
{
    arm_mat_init_f32(Tinv, 4, 4, buffer16);
    return arm_mat_inverse_f32(T, Tinv);
}

/*
void arm_transform_test(void)
{
    float32_t bufA[16], bufB[16], bufC[16];
    arm_matrix_instance_f32 TA, TB, TC;

    // 构造两个变换: 平移+旋转
    arm_transform_from_euler(1.0f, 2.0f, 3.0f, 0, 0, 30*PI/180.0f, &TA, bufA);
    arm_transform_from_euler(0.0f, 1.0f, 0.0f, 0, 45*PI/180.0f, 0, &TB, bufB);

    // 复合变换: TC = TA * TB
    arm_transform_combine(&TA, &TB, &TC, bufC);

    // 测试点
    float32_t p_in[4]  = {1.0f, 0.0f, 0.0f, 1.0f};
    float32_t p_out[4];

    arm_transform_apply_point(&TC, p_in, p_out);

    printf("p_out = (%f, %f, %f)\n", p_out[0], p_out[1], p_out[2]);
}
*/
