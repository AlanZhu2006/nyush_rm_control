/**
 * @file steering.c
 * @brief 哨兵 Swerve 转向角解算：vx,vy -> 两路转向目标角 + 驱动方向，带最短路径
 *        参考 robomaster-control: init_angle_a=1084, init_angle_b=2434
 */
#include "steering.h"
#include "robot_def.h"
#include <math.h>

#ifndef STEER_ECD_PER_REV
#define STEER_ECD_PER_REV 8192.0f
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TICKS_PER_REV STEER_ECD_PER_REV
#define RAD_TO_TICKS  (TICKS_PER_REV / (2.0f * M_PI))

float SteeringTicksToDegrees(float ticks)
{
    return ticks * (360.0f / TICKS_PER_REV);
}

static float wrap_ticks(float t)
{
    while (t >= TICKS_PER_REV) t -= TICKS_PER_REV;
    while (t < 0.0f) t += TICKS_PER_REV;
    return t;
}

static float shortest_delta(float target, float current)
{
    float d = target - current;
    if (d > TICKS_PER_REV / 2.0f)  d -= TICKS_PER_REV;
    if (d < -TICKS_PER_REV / 2.0f) d += TICKS_PER_REV;
    return d;
}

void SteeringCalculate(float vx, float vy,
                       float init_angle_a, float init_angle_b,
                       float current_angle_a, float current_angle_b,
                       float *out_angle_a, float *out_angle_b,
                       float *out_direction)
{
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 1e-6f) {
        *out_angle_a = init_angle_a;
        *out_angle_b = init_angle_b;
        *out_direction = 1.0f;
        return;
    }

    float angle_rad = atan2f(vx, vy);
    float angle_ticks = angle_rad * RAD_TO_TICKS;
    if (angle_ticks < 0.0f) angle_ticks += TICKS_PER_REV;

    float target_a = wrap_ticks(init_angle_a + angle_ticks);
    float target_b = wrap_ticks(init_angle_b + angle_ticks);
    float alt_a    = wrap_ticks(target_a + TICKS_PER_REV / 2.0f);
    float alt_b    = wrap_ticks(target_b + TICKS_PER_REV / 2.0f);

    float diff_main_a = shortest_delta(target_a, current_angle_a);
    float diff_alt_a  = shortest_delta(alt_a, current_angle_a);
    int use_alt = (fabsf(diff_alt_a) < fabsf(diff_main_a)) ? 1 : 0;

    if (use_alt) {
        *out_angle_a   = alt_a;
        *out_angle_b   = alt_b;
        *out_direction = -1.0f;
    } else {
        *out_angle_a   = target_a;
        *out_angle_b   = target_b;
        *out_direction = 1.0f;
    }
}
