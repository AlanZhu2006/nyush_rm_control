/**
 * @file sentry_controller.c
 * @brief 哨兵 Swerve 底盘：4x M3508 驱动 + 2x GM6020 转向
 *        与 robomaster-control 哨兵复用思路一致，通过 ROBOT_TYPE_sentry 编译接入
 */
#include "sentry_controller.h"
#include "robot_def.h"
#include "dji_motor.h"
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
#define STEER_ECD_PER_REV 8192.0f
#define STEER_DEG_TO_TICKS(deg) ((deg) * (STEER_ECD_PER_REV / 360.0f))

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
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    motor_steer_a = DJIMotorInit(&steer_config);

    steer_config.can_init_config.tx_id = STEER_MOTOR_B_ID;
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

    float cur_deg_a = DJIMotorGetTotalAngle(motor_steer_a);
    float cur_deg_b = DJIMotorGetTotalAngle(motor_steer_b);
    float cur_ticks_a = STEER_DEG_TO_TICKS(cur_deg_a);
    float cur_ticks_b = STEER_DEG_TO_TICKS(cur_deg_b);

    float angle_a, angle_b, direction;
    SteeringCalculate(vx_n, vy_n,
                     STEER_MOTOR_A_INIT_ANGLE, STEER_MOTOR_B_INIT_ANGLE,
                     cur_ticks_a, cur_ticks_b,
                     &angle_a, &angle_b, &direction);

    float ref_deg_a = SteeringTicksToDegrees(angle_a);
    float ref_deg_b = SteeringTicksToDegrees(angle_b);
    DJIMotorSetRef(motor_steer_a, ref_deg_a);
    DJIMotorSetRef(motor_steer_b, ref_deg_b);

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
