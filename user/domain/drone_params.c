#include "drone_params.h"

/*
 * 机体参数定义集中放在领域层。
 * 这些参数描述飞行器本体，不属于控制任务主循环实现。
 */
const DroneParams drone = {
    .Dia         = 0.2667f, // m
    .rho         = 1.225f,
    .mass        = 5.027f,
    .Jxx         = 0.12387f,
    .Jyy         = 0.16104f,
    .Jzz         = 0.23051f,
    .r_x         = 7.7f,
    .r_y         = 1.0f,
    .r_z         = 3.0f,
    .thrust_coef = 0.147f,   // kg
    .torque_coef = 0.00983f,
    .arm_len_x   = 0.23412f,
    .arm_len_y   = 0.176725f,
};