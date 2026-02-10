/**
 * @file steering.h
 * @brief 哨兵 Swerve 转向角解算：vx,vy -> 两路转向目标角 + 驱动方向，带最短路径
 */
#ifndef STEERING_H
#define STEERING_H

/**
 * @brief 将编码器刻度值转换为角度（度）
 * @param ticks 编码器刻度值
 * @return 角度值（度）
 */
float SteeringTicksToDegrees(float ticks);

/**
 * @brief 将角度值转换为编码器刻度值
 * @param degrees 角度值（度）
 * @return 编码器刻度值
 */
float SteeringDegreesToTicks(float degrees);

/**
 * @brief 计算swerve转向目标角度
 * @param vx 前后方向速度
 * @param vy 左右方向速度
 * @param init_angle_a 转向电机A的初始角度（编码器刻度）
 * @param init_angle_b 转向电机B的初始角度（编码器刻度）
 * @param current_angle_a 转向电机A当前角度（编码器刻度）
 * @param current_angle_b 转向电机B当前角度（编码器刻度）
 * @param out_angle_a 输出：转向电机A目标角度（编码器刻度）
 * @param out_angle_b 输出：转向电机B目标角度（编码器刻度）
 * @param out_direction 输出：驱动方向（1.0正向，-1.0反向）
 */
void SteeringCalculate(float vx, float vy,
                       float init_angle_a, float init_angle_b,
                       float current_angle_a, float current_angle_b,
                       float *out_angle_a, float *out_angle_b,
                       float *out_direction);

#endif // STEERING_H
