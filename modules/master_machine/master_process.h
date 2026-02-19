#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 36u // 扩展接收大小以支持导航控制命令
#define VISION_SEND_SIZE 36u

/* ================== 自定义数据包Header定义 (参考SMBU协议) ================== */
// 下位机 -> 上位机 (C板发送)
#define PACKET_HEADER_VISION_TX     0x5A  // 视觉数据 (IMU姿态)
#define PACKET_HEADER_ALL_ROBOT_HP  0x5B  // 所有机器人血量
#define PACKET_HEADER_GAME_STATUS   0x5C  // 比赛状态
#define PACKET_HEADER_ROBOT_STATUS  0x5D  // 机器人状态

// 上位机 -> 下位机 (C板接收)
#define PACKET_HEADER_VISION_RX     0xA5  // 视觉目标数据
#define PACKET_HEADER_TWIST         0xA4  // 底盘速度控制 (由radar_comm模块处理，此处仅定义)
#define PACKET_HEADER_ROBOT_CTRL    0xA3  // 机器人控制命令 (云台扫描等)

// flags_register bit layout (LSB -> MSB)
// [1:0]  fire_mode
// [3:2]  target_state
// [7:4]  target_type
// [9:8]  enemy_color
// [11:10] work_mode
// [15:12] bullet_speed_code
#define VISION_FLAG_FIRE_MODE_SHIFT 0
#define VISION_FLAG_FIRE_MODE_MASK  0x0003
#define VISION_FLAG_TARGET_STATE_SHIFT 2
#define VISION_FLAG_TARGET_STATE_MASK  0x000C
#define VISION_FLAG_TARGET_TYPE_SHIFT 4
#define VISION_FLAG_TARGET_TYPE_MASK  0x00F0
#define VISION_FLAG_ENEMY_COLOR_SHIFT 8
#define VISION_FLAG_ENEMY_COLOR_MASK  0x0300
#define VISION_FLAG_WORK_MODE_SHIFT 10
#define VISION_FLAG_WORK_MODE_MASK  0x0C00
#define VISION_FLAG_BULLET_SPEED_SHIFT 12
#define VISION_FLAG_BULLET_SPEED_MASK  0xF000

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
	volatile uint8_t new_data; // set by decode callback, cleared after cmd processes it
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;

/* ================== 新增：裁判系统数据上传结构体 ================== */
// 所有机器人血量 (对应SMBU的ReceivePacketAllRobotHP)
typedef struct
{
	uint16_t red_1_robot_hp;
	uint16_t red_2_robot_hp;
	uint16_t red_3_robot_hp;
	uint16_t red_4_robot_hp;
	uint16_t red_5_robot_hp;
	uint16_t red_7_robot_hp;  // 哨兵
	uint16_t red_outpost_hp;
	uint16_t red_base_hp;
	uint16_t blue_1_robot_hp;
	uint16_t blue_2_robot_hp;
	uint16_t blue_3_robot_hp;
	uint16_t blue_4_robot_hp;
	uint16_t blue_5_robot_hp;
	uint16_t blue_7_robot_hp;  // 哨兵
	uint16_t blue_outpost_hp;
	uint16_t blue_base_hp;
} All_Robot_HP_Send_s;

// 比赛状态 (对应SMBU的ReceivePacketGameStatus)
typedef struct
{
	uint8_t game_progress;      // 比赛阶段: 0未开始 1准备 2自检 3倒计时 4比赛中 5结算
	uint16_t stage_remain_time; // 当前阶段剩余时间(秒)
} Game_Status_Send_s;

// 机器人状态 (对应SMBU的ReceivePacketRobotStatus)
typedef struct
{
	uint8_t robot_id;       // 机器人ID
	uint16_t current_hp;    // 当前血量
	uint16_t shooter_heat;  // 枪口热量
	uint8_t team_color;     // 队伍颜色 0-红 1-蓝
	uint8_t is_attacked;    // 是否受到攻击
} Robot_Status_Send_s;

/* ================== 新增：上位机下发控制命令结构体 ================== */
// 机器人控制命令 (对应SMBU的SendPacketRobotControl) - 注：导航速度由radar_comm处理
typedef struct
{
	uint8_t stop_gimbal_scan;   // 是否停止云台扫描 0-继续扫描 1-停止
	float chassis_spin_vel;     // 底盘小陀螺旋转速度 (rad/s)
	volatile uint8_t new_data;
} Robot_Ctrl_Recv_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据 (IMU姿态)
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

/* ================== 新增：裁判系统数据发送接口 ================== */

/**
 * @brief 发送所有机器人血量数据到上位机
 * @param hp 血量数据指针
 */
void SendAllRobotHP(const All_Robot_HP_Send_s *hp);

/**
 * @brief 发送比赛状态数据到上位机
 * @param status 比赛状态数据指针
 */
void SendGameStatus(const Game_Status_Send_s *status);

/**
 * @brief 发送机器人状态数据到上位机
 * @param status 机器人状态数据指针
 */
void SendRobotStatus(const Robot_Status_Send_s *status);

/* ================== 新增：获取上位机下发数据接口 ================== */

/**
 * @brief 获取机器人控制命令数据指针 (云台扫描控制等)
 * @note 导航速度命令 (vx,vy,wz) 由 radar_comm 模块处理
 * @return Robot_Ctrl_Recv_s* 机器人控制命令数据
 */
Robot_Ctrl_Recv_s *GetRobotCtrl(void);

#endif // !MASTER_PROCESS_H
