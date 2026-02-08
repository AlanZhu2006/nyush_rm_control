/**
 * @file sentry_controller.c
 * @brief 哨兵 Swerve 底盘：4x M3508 驱动 + 2x GM6020 转向
 *        与 robomaster-control 哨兵复用思路一致，通过 ROBOT_TYPE_sentry 编译接入
 */
#include "sentry_controller.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "stm32f4xx_hal.h"
#include "bsp_log.h"
#include <math.h>
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "steering.h"
#include "general_def.h"  /* DEGREE_2_RAD */
#include "arm_math.h"

#ifdef ONE_BOARD
static Publisher_t *chassis_pub;
static Subscriber_t *chassis_sub;
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;
static SuperCapInstance *cap;
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static DJIMotorInstance *motor_steer_a, *motor_steer_b;
static referee_info_t *referee_data;
static Referee_Interactive_info_t ui_data;

#define HALF_WHEEL_BASE   (WHEEL_BASE / 2.0f)
#define HALF_TRACK_WIDTH  (TRACK_WIDTH / 2.0f)
#define STEER_DEG_TO_TICKS(deg) ((deg) * (STEER_ECD_PER_REV / 360.0f))

#define STEER_ALIGNMENT_THRESHOLD 100.0f   // encoder ticks
#define STEER_ALIGNMENT_TIMEOUT_MS 5000u
#define STEER_ALIGNMENT_STABLE_CYCLES 10u

static float chassis_vx, chassis_vy;
static float vt_lf, vt_rf, vt_lb, vt_rb;

void SentryChassisInit(void)
{
    (void)chassis_feedback_data;
    (void)ui_data;

    Motor_Init_Config_s drive_config = {
        .can_init_config.can_handle = &CHASSIS_CAN_BUS,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = CHASSIS_SPEED_PID_KP,
                .Ki = CHASSIS_SPEED_PID_KI,
                .Kd = CHASSIS_SPEED_PID_KD,
                .IntegralLimit = CHASSIS_SPEED_PID_INT_LIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = CHASSIS_SPEED_PID_MAX_OUT,
            },
            .current_PID = {
                .Kp = CHASSIS_CURRENT_PID_KP,
                .Ki = CHASSIS_CURRENT_PID_KI,
                .Kd = CHASSIS_CURRENT_PID_KD,
                .IntegralLimit = CHASSIS_CURRENT_PID_INT_LIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = CHASSIS_CURRENT_PID_MAX_OUT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_LF_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_LF_REVERSE;
    motor_lf = DJIMotorInit(&drive_config);

    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_RF_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_RF_REVERSE;
    motor_rf = DJIMotorInit(&drive_config);

    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_LB_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_LB_REVERSE;
    motor_lb = DJIMotorInit(&drive_config);

    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_RB_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_RB_REVERSE;
    motor_rb = DJIMotorInit(&drive_config);

    /* 转向电机 A/B (GM6020)，外环角度 + 内环速度 */
    Motor_Init_Config_s steer_config = {
        .can_init_config = {
            .can_handle = &CHASSIS_CAN_BUS,
            .tx_id = STEER_MOTOR_A_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = STEER_ANGLE_PID_KP,
                .Ki = STEER_ANGLE_PID_KI,
                .Kd = STEER_ANGLE_PID_KD,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = STEER_ANGLE_PID_INT_LIMIT,
                .MaxOut = STEER_ANGLE_PID_MAX_OUT,
            },
            .speed_PID = {
                .Kp = STEER_SPEED_PID_KP,
                .Ki = STEER_SPEED_PID_KI,
                .Kd = STEER_SPEED_PID_KD,
                .IntegralLimit = STEER_SPEED_PID_INT_LIMIT,
                .MaxOut = STEER_SPEED_PID_MAX_OUT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .current_PID = {
                .Kp = STEER_CURRENT_PID_KP,
                .Ki = STEER_CURRENT_PID_KI,
                .Kd = STEER_CURRENT_PID_KD,
                .IntegralLimit = STEER_CURRENT_PID_INT_LIMIT,
                .MaxOut = STEER_CURRENT_PID_MAX_OUT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  /* GM6020 不需要电流环，与 robomaster/gimbal 一致 */
            .motor_reverse_flag = STEER_MOTOR_A_REVERSE,
        },
        .motor_type = GM6020,
    };
    motor_steer_a = DJIMotorInit(&steer_config);

    steer_config.can_init_config.tx_id = STEER_MOTOR_B_ID;
    steer_config.controller_setting_init_config.motor_reverse_flag = STEER_MOTOR_B_REVERSE;
    motor_steer_b = DJIMotorInit(&steer_config);

    referee_data = UITaskInit(&huart6, &ui_data);
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302,
            .rx_id = 0x301,
        }};
    cap = SuperCapInit(&cap_conf);

#ifdef ONE_BOARD
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

static void SentryLimitChassisOutput(void)
{
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

void SentryChassisTask(void)
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

    static uint8_t steer_aligned = 0;
    static uint32_t align_start_tick = 0u;
    static uint32_t align_stable_count = 0u;

    /* 舵轮对齐阶段：动力轮必停，仅舵轮同步转到 init，参考 robomaster */
    if (!steer_aligned) {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorEnable(motor_steer_a);
        DJIMotorEnable(motor_steer_b);

        if (align_start_tick == 0u)
            align_start_tick = HAL_GetTick();

        /* 两轮同步：都朝 init 转，用同一套最短路径逻辑 */
        uint16_t cur_ecd_a = motor_steer_a->measure.ecd;
        uint16_t cur_ecd_b = motor_steer_b->measure.ecd;
        float cur_angle_a = cur_ecd_a * (360.0f / STEER_ECD_PER_REV);
        float cur_angle_b = cur_ecd_b * (360.0f / STEER_ECD_PER_REV);
        float target_a = STEER_MOTOR_A_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);
        float target_b = STEER_MOTOR_B_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);
        float delta_a = target_a - cur_angle_a;
        float delta_b = target_b - cur_angle_b;
        if (delta_a > 180.0f) delta_a -= 360.0f;
        if (delta_a < -180.0f) delta_a += 360.0f;
        if (delta_b > 180.0f) delta_b -= 360.0f;
        if (delta_b < -180.0f) delta_b += 360.0f;

        DJIMotorSetRef(motor_steer_a, motor_steer_a->measure.total_angle + delta_a);
        DJIMotorSetRef(motor_steer_b, motor_steer_b->measure.total_angle + delta_b);

        /* 不调用 SentryLimitChassisOutput：动力轮已 Stop，会发 0；不再设置 ref 避免任何输出 */

        float err_a = fabsf((float)STEER_MOTOR_A_INIT_ANGLE - (float)cur_ecd_a);
        float err_b = fabsf((float)STEER_MOTOR_B_INIT_ANGLE - (float)cur_ecd_b);
        if (err_a > STEER_ECD_PER_REV / 2.0f) err_a = STEER_ECD_PER_REV - err_a;
        if (err_b > STEER_ECD_PER_REV / 2.0f) err_b = STEER_ECD_PER_REV - err_b;

        if (err_a <= STEER_ALIGNMENT_THRESHOLD && err_b <= STEER_ALIGNMENT_THRESHOLD)
            align_stable_count++;
        else
            align_stable_count = 0u;

        if (align_stable_count >= STEER_ALIGNMENT_STABLE_CYCLES ||
            (HAL_GetTick() - align_start_tick >= STEER_ALIGNMENT_TIMEOUT_MS))
            steer_aligned = 1;

        /* 校准用：每 500ms 打印一次 ecd 值，手动将两轮朝前后可读取填入 STEER_MOTOR_A/B_INIT_ANGLE */
        {
            static uint32_t last_log = 0;
            if (HAL_GetTick() - last_log >= 500u) {
                last_log = HAL_GetTick();
                LOGINFO("[sentry] steer A ecd=%u B ecd=%u (calib: align wheels forward, read values)",
                        (unsigned)motor_steer_a->measure.ecd,
                        (unsigned)motor_steer_b->measure.ecd);
            }
        }

#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
        return;
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(motor_steer_a);
        DJIMotorStop(motor_steer_b);
#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
        return;
    }

    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
    DJIMotorEnable(motor_steer_a);
    DJIMotorEnable(motor_steer_b);

    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW:
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE:
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

    /* 云台系 -> 底盘系 */
    float cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = -chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
    chassis_vy =  chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    /* Swerve: 仅用 vx,vy 决定方向与大小，wz 暂不参与转向角 (可后续加) */
    float vx_n = chassis_vx;
    float vy_n = chassis_vy;
    float mag = sqrtf(vx_n * vx_n + vy_n * vy_n);
    if (mag > 1.0f) mag = 1.0f;
#define CHASSIS_VEL_DEADBAND 0.08f  /* 死区，避免 reset 时 RC/雷达残值导致前冲 */
    if (mag < CHASSIS_VEL_DEADBAND) {
        mag = 0.0f;
        vx_n = vy_n = 0.0f;  /* 关键：reset 时必须传 0 给 SteeringCalculate，否则舵轮会朝残值方向转 */
    }

    /* SteeringCalculate 工作在 ticks [0, 8192)，用 ecd 而非 total_angle
     * 参考 robomaster：mag=0 时两轮都回 init，驱动必停 */
    float cur_ticks_a = (float)motor_steer_a->measure.ecd;
    float cur_ticks_b = (float)motor_steer_b->measure.ecd;

    float angle_a, angle_b, direction;
    SteeringCalculate(vx_n, vy_n,
                     STEER_MOTOR_A_INIT_ANGLE, STEER_MOTOR_B_INIT_ANGLE,
                     cur_ticks_a, cur_ticks_b,
                     &angle_a, &angle_b, &direction);

    /* DJIMotor 用 total_angle 反馈，ref 需在同一坐标系（最短路径） */
    float target_deg_a = SteeringTicksToDegrees(angle_a);
    float target_deg_b = SteeringTicksToDegrees(angle_b);
    float cur_single_a = motor_steer_a->measure.angle_single_round;
    float cur_single_b = motor_steer_b->measure.angle_single_round;
    float delta_a = target_deg_a - cur_single_a;
    float delta_b = target_deg_b - cur_single_b;
    if (delta_a > 180.0f) delta_a -= 360.0f;
    if (delta_a < -180.0f) delta_a += 360.0f;
    if (delta_b > 180.0f) delta_b -= 360.0f;
    if (delta_b < -180.0f) delta_b += 360.0f;
    DJIMotorSetRef(motor_steer_a, motor_steer_a->measure.total_angle + delta_a);
    DJIMotorSetRef(motor_steer_b, motor_steer_b->measure.total_angle + delta_b);

    /* 驱动轮速度：同向，大小 * 方向 */
    float scale = 8000.0f;
    vt_lf = direction * mag * scale;
    vt_rf = direction * mag * scale;
    vt_lb = direction * mag * scale;
    vt_rb = direction * mag * scale;

    SentryLimitChassisOutput();

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
}
