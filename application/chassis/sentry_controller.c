/**
 * @file sentry_controller.c
 * @brief 哨兵 Swerve 底盘：4x M3508 驱动 + 2x GM6020 转向
 *        与 robomaster-control 哨兵复用思路一致，通过 ROBOT_TYPE_sentry
 * 编译接入
 */
#include "sentry_controller.h"

#include <math.h>

#include "arm_math.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "general_def.h" /* DEGREE_2_RAD */
#include "message_center.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "robot_def.h"
#include "steering.h"
#include "stm32f4xx_hal.h"
#include "super_cap.h"

/* Make the implementation visible in editors/IntelliSense by defining
    ROBOT_TYPE_sentry for the IDE parser only. This does not affect normal
    builds because common IDE parsers define one of these helper macros. */
#if (defined(__INTELLISENSE__) || defined(__clang_analyzer__) || \
     defined(__GNUC__ONLY_FOR_IDE__)) &&                         \
    !defined(ROBOT_TYPE_sentry)
/* Intentional editor-only define to avoid greyed-out #else region */
#define ROBOT_TYPE_sentry 1
#endif

#if !defined(ROBOT_TYPE_sentry)
/* When not building for sentry, provide empty stubs so this translation unit
    can be compiled/linked without requiring sentry-specific config macros. */
void SentryChassisInit(void) {}
void SentryChassisTask(void) {}
#else

#ifdef ONE_BOARD
static Publisher_t* chassis_pub;
static Subscriber_t* chassis_sub;
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;          // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;  // 底盘回传的反馈数据

static referee_info_t* referee_data;
static Referee_Interactive_info_t ui_data;
static SuperCapInstance* cap;

/* ======================== 电机实例 ======================== */
/* 仅使用对角线上的两组舵轮：
 *   舵轮 A (左前位置): GM6020 转向 + M3508 驱动
 *   舵轮 B (右后位置): GM6020 转向 + M3508 驱动
 * RF/LB 全向轮不初始化、不供电
 */
static DJIMotorInstance* motor_drive_a;  // 舵轮A的驱动电机 (M3508, 左前位置)
static DJIMotorInstance* motor_drive_b;  // 舵轮B的驱动电机 (M3508, 右后位置)
static DJIMotorInstance* motor_steer_a;  // 舵轮A的转向电机 (GM6020)
static DJIMotorInstance* motor_steer_b;  // 舵轮B的转向电机 (GM6020)

/* ======================== 宏定义 ======================== */
#define STEER_ALIGNMENT_THRESHOLD 100.0f   // 对齐判定阈值（编码器 ticks）
#define STEER_ALIGNMENT_TIMEOUT_MS 5000u   // 对齐超时（ms）
#define STEER_ALIGNMENT_STABLE_CYCLES 10u  // 连续稳定周期数

#define CHASSIS_VEL_DEADBAND 0.08f  // 速度死区，防止遥控器残值导致意外运动
#define DRIVE_SPEED_SCALE 12000.0f   // 驱动轮速度缩放系数

/* ======================== 中间变量 ======================== */
static float chassis_vx, chassis_vy;  // 底盘坐标系下的速度分量
static float vt_drive_a, vt_drive_b;  // 两个驱动轮的速度输出

/* ================================================================== */
/*                          初始化                                      */
/* ================================================================== */
void SentryChassisInit(void) {
    (void)chassis_feedback_data;
    (void)ui_data;

    /* ---------- 驱动电机 M3508（速度环 + 电流环） ---------- */
    Motor_Init_Config_s drive_config = {
        .can_init_config.can_handle = &CHASSIS_CAN_BUS,
        .controller_param_init_config =
            {
                .speed_PID =
                    {
                        .Kp = CHASSIS_SPEED_PID_KP,
                        .Ki = CHASSIS_SPEED_PID_KI,
                        .Kd = CHASSIS_SPEED_PID_KD,
                        .IntegralLimit = CHASSIS_SPEED_PID_INT_LIMIT,
                        .Improve = PID_Trapezoid_Intergral |
                                   PID_Integral_Limit |
                                   PID_Derivative_On_Measurement,
                        .MaxOut = CHASSIS_SPEED_PID_MAX_OUT,
                    },
                .current_PID =
                    {
                        .Kp = CHASSIS_CURRENT_PID_KP,
                        .Ki = CHASSIS_CURRENT_PID_KI,
                        .Kd = CHASSIS_CURRENT_PID_KD,
                        .IntegralLimit = CHASSIS_CURRENT_PID_INT_LIMIT,
                        .Improve = PID_Trapezoid_Intergral |
                                   PID_Integral_Limit |
                                   PID_Derivative_On_Measurement,
                        .MaxOut = CHASSIS_CURRENT_PID_MAX_OUT,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            },
        .motor_type = M3508,
    };

    // 驱动轮 A — 使用左前电机 ID
    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_LB_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_LB_REVERSE;
    motor_drive_a = DJIMotorInit(&drive_config);

    // 驱动轮 B — 使用右后电机 ID
    drive_config.can_init_config.tx_id = CHASSIS_MOTOR_RB_ID;
    drive_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_RB_REVERSE;
    motor_drive_b = DJIMotorInit(&drive_config);

    /* ---------- 转向电机 GM6020（角度环 + 速度环） ---------- */
    Motor_Init_Config_s steer_config = {
        .can_init_config =
            {
                .can_handle = &CHASSIS_CAN_BUS,
                .tx_id = STEER_MOTOR_A_ID,
            },
        .controller_param_init_config =
            {
                .angle_PID =
                    {
                        .Kp = STEER_ANGLE_PID_KP,
                        .Ki = STEER_ANGLE_PID_KI,
                        .Kd = STEER_ANGLE_PID_KD,
                        .DeadBand = 0.0f,
                        .Improve = PID_Trapezoid_Intergral |
                                   PID_Integral_Limit |
                                   PID_Derivative_On_Measurement,
                        .IntegralLimit = STEER_ANGLE_PID_INT_LIMIT,
                        .MaxOut = STEER_ANGLE_PID_MAX_OUT,
                    },
                .speed_PID =
                    {
                        .Kp = STEER_SPEED_PID_KP,
                        .Ki = STEER_SPEED_PID_KI,
                        .Kd = STEER_SPEED_PID_KD,
                        .IntegralLimit = STEER_SPEED_PID_INT_LIMIT,
                        .MaxOut = STEER_SPEED_PID_MAX_OUT,
                        .Improve = PID_Trapezoid_Intergral |
                                   PID_Integral_Limit |
                                   PID_Derivative_On_Measurement,
                    },
                .current_PID =
                    {
                        .Kp = STEER_CURRENT_PID_KP,
                        .Ki = STEER_CURRENT_PID_KI,
                        .Kd = STEER_CURRENT_PID_KD,
                        .IntegralLimit = STEER_CURRENT_PID_INT_LIMIT,
                        .MaxOut = STEER_CURRENT_PID_MAX_OUT,
                        .Improve = PID_Trapezoid_Intergral |
                                   PID_Integral_Limit |
                                   PID_Derivative_On_Measurement,
                    },
            },
        .controller_setting_init_config =
            {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = ANGLE_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                .motor_reverse_flag = STEER_MOTOR_A_REVERSE,
            },
        .motor_type = GM6020,
    };
    motor_steer_a = DJIMotorInit(&steer_config);

    steer_config.can_init_config.tx_id = STEER_MOTOR_B_ID;
    steer_config.controller_setting_init_config.motor_reverse_flag =
        STEER_MOTOR_B_REVERSE;
    motor_steer_b = DJIMotorInit(&steer_config);

    /* ---------- 裁判系统 & 超级电容 ---------- */
    referee_data = UITaskInit(&huart6, &ui_data);
    SuperCap_Init_Config_s cap_conf = {.can_config = {
                                           .can_handle = &hcan2,
                                           .tx_id = 0x302,
                                           .rx_id = 0x301,
                                       }};
    cap = SuperCapInit(&cap_conf);

    /* ---------- 发布/订阅 ---------- */
#ifdef ONE_BOARD
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

/* ================================================================== */
/*                         限幅 & 设置输出                               */
/* ================================================================== */
static void SentryLimitChassisOutput(void) {
    // 功率限制待添加（可参考 referee_data->PowerHeatData）
    DJIMotorSetRef(motor_drive_a, vt_drive_a);
    DJIMotorSetRef(motor_drive_b, vt_drive_b);
}

/* ================================================================== */
/*                          舵轮对齐阶段                                 */
/* ================================================================== */
/**
 * @brief 上电后舵轮先转到初始位置（朝前），驱动轮保持停止
 * @return 1: 对齐完成  0: 仍在对齐中
 */
static uint8_t SentrySteerAlign(void) {
    static uint8_t aligned = 0;
    static uint32_t align_start_tick = 0u;
    static uint32_t stable_count = 0u;

    if (aligned) return 1;

    /* 驱动轮停止，仅舵轮工作 */
    DJIMotorStop(motor_drive_a);
    DJIMotorStop(motor_drive_b);
    DJIMotorEnable(motor_steer_a);
    DJIMotorEnable(motor_steer_b);

    if (align_start_tick == 0u) align_start_tick = HAL_GetTick();

    /* 计算到 init 位置的最短路径 */
    uint16_t cur_ecd_a = motor_steer_a->measure.ecd;
    uint16_t cur_ecd_b = motor_steer_b->measure.ecd;
    float cur_deg_a = cur_ecd_a * (360.0f / STEER_ECD_PER_REV);
    float cur_deg_b = cur_ecd_b * (360.0f / STEER_ECD_PER_REV);
    float tgt_deg_a = STEER_MOTOR_A_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);
    float tgt_deg_b = STEER_MOTOR_B_INIT_ANGLE * (360.0f / STEER_ECD_PER_REV);

    float delta_a = tgt_deg_a - cur_deg_a;
    float delta_b = tgt_deg_b - cur_deg_b;
    if (delta_a > 180.0f) delta_a -= 360.0f;
    if (delta_a < -180.0f) delta_a += 360.0f;
    if (delta_b > 180.0f) delta_b -= 360.0f;
    if (delta_b < -180.0f) delta_b += 360.0f;

    DJIMotorSetRef(motor_steer_a, motor_steer_a->measure.total_angle + delta_a);
    DJIMotorSetRef(motor_steer_b, motor_steer_b->measure.total_angle + delta_b);

    /* 判断是否到位 */
    float err_a = fabsf((float)STEER_MOTOR_A_INIT_ANGLE - (float)cur_ecd_a);
    float err_b = fabsf((float)STEER_MOTOR_B_INIT_ANGLE - (float)cur_ecd_b);
    if (err_a > STEER_ECD_PER_REV / 2.0f) err_a = STEER_ECD_PER_REV - err_a;
    if (err_b > STEER_ECD_PER_REV / 2.0f) err_b = STEER_ECD_PER_REV - err_b;

    if (err_a <= STEER_ALIGNMENT_THRESHOLD &&
        err_b <= STEER_ALIGNMENT_THRESHOLD)
        stable_count++;
    else
        stable_count = 0u;

    if (stable_count >= STEER_ALIGNMENT_STABLE_CYCLES ||
        (HAL_GetTick() - align_start_tick >= STEER_ALIGNMENT_TIMEOUT_MS))
        aligned = 1;

    /* 校准日志：每 500ms 打印当前编码器值 */
    {
        static uint32_t last_log = 0;
        if (HAL_GetTick() - last_log >= 500u) {
            last_log = HAL_GetTick();
            LOGINFO(
                "[sentry] steer A ecd=%u B ecd=%u (calib: align wheels "
                "forward)",
                (unsigned)motor_steer_a->measure.ecd,
                (unsigned)motor_steer_b->measure.ecd);
        }
    }

    return 0;
}

/* ================================================================== */
/*                   运动学解算：vx,vy -> 舵轮角度 + 驱动速度             */
/* ================================================================== */
/**
 * @brief 两轮舵轮平移运动学解算
 *
 *  对于纯平移（无旋转），两个舵轮目标方向相同，驱动速度相同。
 *  利用 SteeringCalculate() 计算目标角度及最短路径方向。
 *
 * @param vx_body 底盘坐标系 x 方向速度（前后）
 * @param vy_body 底盘坐标系 y 方向速度（左右）
 */
static void SentryTranslationCalculate(float vx_body, float vy_body) {
    float mag = sqrtf(vx_body * vx_body + vy_body * vy_body);

    /* 死区处理：速度过小时舵轮回归 init 位置，驱动停止 */
    if (mag < CHASSIS_VEL_DEADBAND) {
        vx_body = 0.0f;
        vy_body = 0.0f;
        mag = 0.0f;
    }

    /* 速度归一化，用于限幅（cmd 传入的 vx/vy 范围约 [-1, 1]） */
    if (mag > 1.0f) mag = 1.0f;

    /* 调用 steering 模块计算目标角度 (编码器 ticks) 和驱动方向 */
    float cur_ticks_a = (float)motor_steer_a->measure.ecd;
    float cur_ticks_b = (float)motor_steer_b->measure.ecd;

    float angle_a, angle_b, direction;
    SteeringCalculate(vx_body, vy_body, STEER_MOTOR_A_INIT_ANGLE,
                      STEER_MOTOR_B_INIT_ANGLE, cur_ticks_a, cur_ticks_b,
                      &angle_a, &angle_b, &direction);

    /* 将 ticks 目标转换为角度，通过最短路径增量设置给 GM6020 */
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

    DJIMotorSetRef(
        motor_steer_a,
        (motor_steer_a->measure.total_angle + delta_a));
    DJIMotorSetRef(
        motor_steer_b,
        (motor_steer_b->measure.total_angle + delta_b));

    /* 驱动轮速度：两轮同速同向（平移运动） */
    vt_drive_a = direction * mag * DRIVE_SPEED_SCALE;
    vt_drive_b = direction * mag * DRIVE_SPEED_SCALE;
}

/* ================================================================== */
/*                          主任务                                      */
/* ================================================================== */
void SentryChassisTask(void) {
    /* 获取控制命令 */
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

    /* ---- 阶段1：舵轮对齐（上电后执行一次） ---- */
    if (!SentrySteerAlign()) {
#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void*)&chassis_feedback_data);
#endif
        return;
    }

    /* ---- 急停模式 ---- */
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_drive_a);
        DJIMotorStop(motor_drive_b);
        DJIMotorStop(motor_steer_a);
        DJIMotorStop(motor_steer_b);
#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void*)&chassis_feedback_data);
#endif
        return;
    }

    /* ---- 正常工作：使能所有电机 ---- */
    DJIMotorEnable(motor_drive_a);
    DJIMotorEnable(motor_drive_b);
    DJIMotorEnable(motor_steer_a);
    DJIMotorEnable(motor_steer_b);

    /* ---- 根据控制模式修正 wz ---- */
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            chassis_cmd_recv.wz = 0;
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle *
                                  fabsf(chassis_cmd_recv.offset_angle);
            break;
        case CHASSIS_ROTATE:
            // 两轮舵轮结构暂不支持原地旋转，忽略 wz
            chassis_cmd_recv.wz = 0;
            break;
        default:
            break;
    }

    /* ---- 云台坐标系 -> 底盘坐标系 ---- */
    float cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx =-(
        -chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta);
    chassis_vy =
        chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    /* ---- 运动学解算 ---- */
    SentryTranslationCalculate(chassis_vx, chassis_vy);

    /* ---- 设置驱动轮输出 ---- */
    SentryLimitChassisOutput();

    /* ---- 推送反馈 ---- */
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void*)&chassis_feedback_data);
#endif
}

#endif /* ROBOT_TYPE_sentry */
