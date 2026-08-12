#ifndef __ARM_TRANSFORM_H__
#define __ARM_TRANSFORM_H__
#endif

#include "stm32f4xx.h"
#include "arm_math.h"

// ----------- API -----------

#ifdef __cplusplus
extern "C" {
#endif

// 构造 4x4 齐次变换矩阵 (欧拉角 ZYX + 平移)
void arm_transform_from_euler(float tx, float ty, float tz,
                              float roll, float pitch, float yaw,
                              arm_matrix_instance_f32 *T,
                              float32_t *buffer16);

// 应用变换到点 (x,y,z,1)
void arm_transform_apply_point(const arm_matrix_instance_f32 *T,
                               const float32_t p_in[4],
                               float32_t p_out[4]);

// 应用变换到向量 (只旋转)
void arm_transform_apply_vector(const arm_matrix_instance_f32 *T,
                                const float32_t v_in[3],
                                float32_t v_out[3]);

// 复合两个变换: C = A * B
arm_status arm_transform_combine(const arm_matrix_instance_f32 *A,
                                 const arm_matrix_instance_f32 *B,
                                 arm_matrix_instance_f32 *C,
                                 float32_t *buffer16);

// 通用矩阵求逆 (可用于非刚体)
arm_status arm_transform_inverse(const arm_matrix_instance_f32 *T,
                                 arm_matrix_instance_f32 *Tinv,
                                 float32_t *buffer16);

#ifdef __cplusplus
}
#endif
