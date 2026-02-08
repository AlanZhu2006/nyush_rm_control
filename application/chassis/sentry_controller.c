/**
 * @file sentry_controller.c
 * @brief 哨兵 Swerve 底盘：4x M3508 驱动 + 2x GM6020 转向
 *        与 robomaster-control 哨兵复用思路一致，通过 ROBOT_TYPE_sentry 编译接入
 */
#include "sentry_controller.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "stm32f4xx_hal.h"
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

#define STEER_ALIGNMENT_THRESHOLD 100.0f   // encoder ticks (robomaster: 100)
#define STEER_ALIGNMENT_TIMEOUT_MS 5000u   // 5 seconds
#define STEER_ALIGNMENT_STABLE_CYCLES 10u  // require stable for 10 cycles (~50ms @ 200Hz)

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
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  // GM6020不需要电流环
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

    // Initialize chassis_cmd_recv to SAFE defaults
    // 注意：不要设置为ZERO_FORCE，否则对齐逻辑永远不会运行
    chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW;  // 允许对齐运行
    chassis_cmd_recv.vx = 0.0f;
    chassis_cmd_recv.vy = 0.0f;
    chassis_cmd_recv.wz = 0.0f;
    chassis_cmd_recv.offset_angle = 0.0f;

    // Stop drive motors initially, but leave steer motors enabled for alignment
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
    // Don't stop steer motors - let alignment logic control them
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

    /* 舵轮对齐阶段: 必须在 chassis_mode 检查之前执行，否则上电时 ZERO_FORCE 会导致对齐永远不跑
     * 参考 robomaster: Sentry_WaitForSteerAlignment 每 50ms 计算电流并发送；
     * nyush 通过 DJIMotorSetRef + MotorControlTask 实现，需每周期更新 ref 以保证闭环收敛 */
    if (!steer_aligned) {
        // Only enable steer motors during alignment
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorEnable(motor_steer_a);
        DJIMotorEnable(motor_steer_b);

        if (align_start_tick == 0u) {
            align_start_tick = HAL_GetTick();
        }

        /* 每周期计算目标 total_angle（最短路径），与 robomaster SteerController_CascadeControl 思路一致 */
        uint16_t cur_ecd_a = motor_steer_a->measure.ecd;
        uint16_t cur_ecd_b = motor_steer_b->measure.ecd;
        float cur_angle_a = cur_ecd_a * (360.0f / STEER_ECD_PER_REV);
        float cur_angle_b = cur_ecd_b * (360.0f / STEER_ECD_PER_REV);
        float target_angle_a = STEER_MOTOR_A_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);
        float target_angle_b = STEER_MOTOR_B_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);
        float delta_a = target_angle_a - cur_angle_a;
        float delta_b = target_angle_b - cur_angle_b;
        if (delta_a > 180.0f) delta_a -= 360.0f;
        if (delta_a < -180.0f) delta_a += 360.0f;
        if (delta_b > 180.0f) delta_b -= 360.0f;
        if (delta_b < -180.0f) delta_b += 360.0f;
        DJIMotorSetRef(motor_steer_a, motor_steer_a->measure.total_angle + delta_a);
        DJIMotorSetRef(motor_steer_b, motor_steer_b->measure.total_angle + delta_b);

        // 每次循环都设置驱动轮为0
        vt_lf = vt_rf = vt_lb = vt_rb = 0.0f;
        SentryLimitChassisOutput();

        // 检查是否对齐完成（重用上面的cur_ecd_a/b）
        float err_a = fabsf((float)STEER_MOTOR_A_INIT_ANGLE - (float)cur_ecd_a);
        float err_b = fabsf((float)STEER_MOTOR_B_INIT_ANGLE - (float)cur_ecd_b);
        // 考虑编码器环绕
        if (err_a > STEER_ECD_PER_REV / 2.0f) err_a = STEER_ECD_PER_REV - err_a;
        if (err_b > STEER_ECD_PER_REV / 2.0f) err_b = STEER_ECD_PER_REV - err_b;

        if (err_a <= STEER_ALIGNMENT_THRESHOLD && err_b <= STEER_ALIGNMENT_THRESHOLD)
            align_stable_count++;
        else
            align_stable_count = 0u;

        if (align_stable_count >= STEER_ALIGNMENT_STABLE_CYCLES ||
            (HAL_GetTick() - align_start_tick >= STEER_ALIGNMENT_TIMEOUT_MS))
            steer_aligned = 1;

#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
        return;
    }

    /* 对齐完成后再检查 chassis_mode */
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

    // Alignment complete, now enable all motors
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
    /* 死区: mag 过小时动力轮必须为0，实现原地 reset (仅舵轮回 init) */
#define CHASSIS_VEL_DEADBAND 0.01f
    if (mag < CHASSIS_VEL_DEADBAND) mag = 0.0f;

    /* SteeringCalculate 工作在 ticks [0, 8192)，使用 ecd 而非 total_angle */
    float cur_ticks_a = (float)motor_steer_a->measure.ecd;
    float cur_ticks_b = (float)motor_steer_b->measure.ecd;

    float angle_a, angle_b, direction;
    SteeringCalculate(vx_n, vy_n,
                     STEER_MOTOR_A_INIT_ANGLE, STEER_MOTOR_B_INIT_ANGLE,
                     cur_ticks_a, cur_ticks_b,
                     &angle_a, &angle_b, &direction);

    /* DJIMotor 使用 total_angle 反馈，ref 需在同一坐标系。使用最短路径将目标 ticks 转为 total_angle */
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

    /* 驱动轮速度：mag=0 时必为 0，保证原地 reset */
    /* 提高速度scale以获得更快的移动速度 (原8000.0f) */
    float scale = 18000.0f;
    vt_lf = direction * mag * scale;
    vt_rf = direction * mag * scale;
    vt_lb = direction * mag * scale;
    vt_rb = direction * mag * scale;

    SentryLimitChassisOutput();

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
}
