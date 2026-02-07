#pragma once
#ifndef ROBOT_SENTRY_CONFIG_H
#define ROBOT_SENTRY_CONFIG_H

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define CHASSIS_TYPE_SWERVE

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 电机正反转配置 (MOTOR_DIRECTION_NORMAL=0, MOTOR_DIRECTION_REVERSE=1)
#define GIMBAL_YAW_MOTOR_REVERSE MOTOR_DIRECTION_NORMAL
#define GIMBAL_PITCH_MOTOR_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_LF_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_RF_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_LB_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_RB_REVERSE MOTOR_DIRECTION_NORMAL
#define STEER_MOTOR_A_REVERSE MOTOR_DIRECTION_NORMAL  // Swerve转向电机A
#define STEER_MOTOR_B_REVERSE MOTOR_DIRECTION_NORMAL  // Swerve转向电机B
#define SHOOT_FRICTION_L_REVERSE MOTOR_DIRECTION_NORMAL
#define SHOOT_FRICTION_R_REVERSE MOTOR_DIRECTION_NORMAL  // 与左摩擦轮同向
#define SHOOT_LOADER_REVERSE MOTOR_DIRECTION_NORMAL

// 底盘驱动电机CAN配置 (M3508)
#define CHASSIS_CAN_BUS hcan1
#define CHASSIS_MOTOR_LF_ID 1  // 左前驱动轮
#define CHASSIS_MOTOR_RF_ID 2  // 右前驱动轮
#define CHASSIS_MOTOR_LB_ID 4  // 左后驱动轮
#define CHASSIS_MOTOR_RB_ID 3  // 右后驱动轮

// Swerve转向电机CAN配置 (GM6020)
#define STEER_MOTOR_A_ID 5            // 转向电机A (CAN1)
#define STEER_MOTOR_B_ID 8            // 转向电机B (CAN1) - 修改为8以避免与云台Yaw冲突
#define STEER_ECD_PER_REV 8192.0f     // GM6020编码器分辨率 (0-8191)
#define STEER_MOTOR_A_INIT_ANGLE 0.0f // 转向电机A机械零点对应的编码器值(单位:刻度 0-8191),需实测校准
#define STEER_MOTOR_B_INIT_ANGLE 0.0f // 转向电机B机械零点对应的编码器值(单位:刻度 0-8191),需实测校准

// 云台电机CAN配置 (GM6020)
#define GIMBAL_YAW_CAN_BUS hcan1
#define GIMBAL_YAW_MOTOR_ID 6
#define GIMBAL_PITCH_CAN_BUS hcan2
#define GIMBAL_PITCH_MOTOR_ID 7

// 发射机构电机CAN配置
#define SHOOT_CAN_BUS hcan2
#define SHOOT_FRICTION_L_ID 1  // 左摩擦轮
#define SHOOT_FRICTION_R_ID 2  // 右摩擦轮
#define SHOOT_LOADER_ID 3      // 拨盘

// 底盘运动参数
#define CHASSIS_ROTATE_SPEED 4000.0f  // 小陀螺模式旋转速度
#define CHASSIS_RC_MOVE_RATIO_X 10.0f // 遥控器模式底盘前后移动速度系数
#define CHASSIS_RC_MOVE_RATIO_Y 10.0f // 遥控器模式底盘左右移动速度系数
#define CHASSIS_KB_MOVE_SPEED_X 300.0f // 键鼠模式底盘前后移动速度
#define CHASSIS_KB_MOVE_SPEED_Y 300.0f // 键鼠模式底盘左右移动速度

// PID参数 - 底盘驱动轮 (M3508)
#define CHASSIS_SPEED_PID_KP 10.0f
#define CHASSIS_SPEED_PID_KI 0.0f
#define CHASSIS_SPEED_PID_KD 0.0f
#define CHASSIS_SPEED_PID_INT_LIMIT 3000.0f
#define CHASSIS_SPEED_PID_MAX_OUT 12000.0f

#define CHASSIS_CURRENT_PID_KP 0.5f
#define CHASSIS_CURRENT_PID_KI 0.0f
#define CHASSIS_CURRENT_PID_KD 0.0f
#define CHASSIS_CURRENT_PID_INT_LIMIT 3000.0f
#define CHASSIS_CURRENT_PID_MAX_OUT 15000.0f

// PID参数 - Swerve转向电机 (GM6020)
// 角度环PID
#define STEER_ANGLE_PID_KP 10.0f
#define STEER_ANGLE_PID_KI 0.0f
#define STEER_ANGLE_PID_KD 0.0f
#define STEER_ANGLE_PID_INT_LIMIT 100.0f
#define STEER_ANGLE_PID_MAX_OUT 500.0f

// 速度环PID
#define STEER_SPEED_PID_KP 50.0f
#define STEER_SPEED_PID_KI 200.0f
#define STEER_SPEED_PID_KD 0.0f
#define STEER_SPEED_PID_INT_LIMIT 3000.0f
#define STEER_SPEED_PID_MAX_OUT 20000.0f

// 电流环PID (GM6020一般不使用电流环，保留接口)
#define STEER_CURRENT_PID_KP 0.0f
#define STEER_CURRENT_PID_KI 0.0f
#define STEER_CURRENT_PID_KD 0.0f
#define STEER_CURRENT_PID_INT_LIMIT 3000.0f
#define STEER_CURRENT_PID_MAX_OUT 15000.0f

// PID参数 - 云台
#define GIMBAL_YAW_ANGLE_PID_KP 8.0f
#define GIMBAL_YAW_ANGLE_PID_KI 0.0f
#define GIMBAL_YAW_ANGLE_PID_KD 0.0f
#define GIMBAL_YAW_ANGLE_PID_DEADBAND 0.1f
#define GIMBAL_YAW_ANGLE_PID_INT_LIMIT 100.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT 500.0f

#define GIMBAL_YAW_SPEED_PID_KP 50.0f
#define GIMBAL_YAW_SPEED_PID_KI 200.0f
#define GIMBAL_YAW_SPEED_PID_KD 0.0f
#define GIMBAL_YAW_SPEED_PID_INT_LIMIT 3000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_OUT 20000.0f

#define GIMBAL_PITCH_ANGLE_PID_KP 10.0f
#define GIMBAL_PITCH_ANGLE_PID_KI 0.0f
#define GIMBAL_PITCH_ANGLE_PID_KD 0.0f
#define GIMBAL_PITCH_ANGLE_PID_INT_LIMIT 100.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT 500.0f

#define GIMBAL_PITCH_SPEED_PID_KP 50.0f
#define GIMBAL_PITCH_SPEED_PID_KI 350.0f
#define GIMBAL_PITCH_SPEED_PID_KD 0.0f
#define GIMBAL_PITCH_SPEED_PID_INT_LIMIT 2500.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT 20000.0f

// PID参数 - 发射
#define SHOOT_FRICTION_SPEED_PID_KP 0.0f
#define SHOOT_FRICTION_SPEED_PID_KI 0.0f
#define SHOOT_FRICTION_SPEED_PID_KD 0.0f
#define SHOOT_FRICTION_SPEED_PID_INT_LIMIT 10000.0f
#define SHOOT_FRICTION_SPEED_PID_MAX_OUT 15000.0f

#define SHOOT_FRICTION_CURRENT_PID_KP 0.0f
#define SHOOT_FRICTION_CURRENT_PID_KI 0.0f
#define SHOOT_FRICTION_CURRENT_PID_KD 0.0f
#define SHOOT_FRICTION_CURRENT_PID_INT_LIMIT 10000.0f
#define SHOOT_FRICTION_CURRENT_PID_MAX_OUT 15000.0f

#define SHOOT_LOADER_ANGLE_PID_KP 0.0f
#define SHOOT_LOADER_ANGLE_PID_KI 0.0f
#define SHOOT_LOADER_ANGLE_PID_KD 0.0f
#define SHOOT_LOADER_ANGLE_PID_MAX_OUT 200.0f

#define SHOOT_LOADER_SPEED_PID_KP 0.0f
#define SHOOT_LOADER_SPEED_PID_KI 0.0f
#define SHOOT_LOADER_SPEED_PID_KD 0.0f
#define SHOOT_LOADER_SPEED_PID_INT_LIMIT 5000.0f
#define SHOOT_LOADER_SPEED_PID_MAX_OUT 5000.0f

#define SHOOT_LOADER_CURRENT_PID_KP 0.0f
#define SHOOT_LOADER_CURRENT_PID_KI 0.0f
#define SHOOT_LOADER_CURRENT_PID_KD 0.0f
#define SHOOT_LOADER_CURRENT_PID_INT_LIMIT 5000.0f
#define SHOOT_LOADER_CURRENT_PID_MAX_OUT 5000.0f

#endif // ROBOT_SENTRY_CONFIG_H
