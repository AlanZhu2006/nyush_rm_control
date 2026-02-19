/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc8.h"
#include "crc16.h"
#include "string.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static Robot_Ctrl_Recv_s robot_ctrl_data; // 机器人控制命令 (云台扫描等)
static DaemonInstance *vision_daemon_instance;

static uint8_t BulletSpeedToCode(Bullet_Speed_e speed)
{
    switch (speed)
    {
    case BIG_AMU_10:
        return 1;
    case SMALL_AMU_15:
        return 2;
    case BIG_AMU_16:
        return 3;
    case SMALL_AMU_18:
        return 4;
    case SMALL_AMU_30:
        return 5;
    case BULLET_SPEED_NONE:
    default:
        return 0;
    }
}

static void VisionDecodeFlags(uint16_t flags, Vision_Recv_s *out)
{
    if (!out)
        return;
    out->fire_mode = (Fire_Mode_e)((flags & VISION_FLAG_FIRE_MODE_MASK) >> VISION_FLAG_FIRE_MODE_SHIFT);
    out->target_state = (Target_State_e)((flags & VISION_FLAG_TARGET_STATE_MASK) >> VISION_FLAG_TARGET_STATE_SHIFT);
    out->target_type = (Target_Type_e)((flags & VISION_FLAG_TARGET_TYPE_MASK) >> VISION_FLAG_TARGET_TYPE_SHIFT);
}

static uint16_t VisionEncodeFlags(const Vision_Send_s *in)
{
    uint16_t flags = 0;
    if (!in)
        return flags;
    flags |= (((uint16_t)in->enemy_color << VISION_FLAG_ENEMY_COLOR_SHIFT) & VISION_FLAG_ENEMY_COLOR_MASK);
    flags |= (((uint16_t)in->work_mode << VISION_FLAG_WORK_MODE_SHIFT) & VISION_FLAG_WORK_MODE_MASK);
    flags |= (((uint16_t)BulletSpeedToCode(in->bullet_speed) << VISION_FLAG_BULLET_SPEED_SHIFT) & VISION_FLAG_BULLET_SPEED_MASK);
    return flags;
}

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/* ================== 获取上位机下发数据接口 ================== */
// 注：导航速度命令 (vx,vy,wz) 由 radar_comm 模块处理
Robot_Ctrl_Recv_s *GetRobotCtrl(void)
{
    return &robot_ctrl_data;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 解析上位机下发的机器人控制命令 (Header: 0xA3) - UART版本
 * @note 导航速度命令 (vx,vy,wz) 由 radar_comm 模块处理
 */
static void DecodeRobotCtrlUART(uint8_t *data, uint16_t len)
{
    // 数据包格式: [0xA3][stop_gimbal_scan(1B)][chassis_spin_vel(4B)][CRC16(2B)] = 8字节
    if (len < 8) return;
    
    // 验证CRC16
    uint16_t crc_calc = crc_16(data, len - 2);
    uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
    if (crc_calc != crc_recv) return;
    
    robot_ctrl_data.stop_gimbal_scan = data[1];
    memcpy(&robot_ctrl_data.chassis_spin_vel, &data[2], sizeof(float));
    robot_ctrl_data.new_data = 1;
}

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    
    // 根据数据包头字节判断数据类型
    uint8_t header = vision_usart_instance->recv_buff[0];
    uint16_t recv_len = vision_usart_instance->recv_buff_size;
    
    switch (header)
    {
    case PACKET_HEADER_VISION_RX:  // 0xA5 - 视觉目标数据 (原seasky协议)
        if (get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch))
        {
            VisionDecodeFlags(flag_register, &recv_data);
            recv_data.new_data = 1;
        }
        break;
        
    case PACKET_HEADER_ROBOT_CTRL:  // 0xA3 - 机器人控制命令 (云台扫描等)
        DecodeRobotCtrlUART(vision_usart_instance->recv_buff, recv_len);
        break;
        
    // 注：0xA4 (PACKET_HEADER_TWIST) 底盘速度控制由 radar_comm 模块处理
        
    default:
        // 尝试用seasky协议解析 (兼容旧协议)
        if (get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch))
        {
            VisionDecodeFlags(flag_register, &recv_data);
            recv_data.new_data = 1;
        }
        break;
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    flag_register = VisionEncodeFlags(&send_data);
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

/**
 * @brief 解析上位机下发的机器人控制命令 (Header: 0xA3)
 * @note 导航速度命令 (vx,vy,wz) 由 radar_comm 模块处理
 */
static void DecodeRobotCtrl(uint8_t *data, uint16_t len)
{
    // 数据包格式: [0xA3][stop_gimbal_scan(1B)][chassis_spin_vel(4B)][CRC16(2B)] = 8字节
    if (len < 8) return;
    
    // 验证CRC16
    uint16_t crc_calc = crc_16(data, len - 2);
    uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
    if (crc_calc != crc_recv) return;
    
    robot_ctrl_data.stop_gimbal_scan = data[1];
    memcpy(&robot_ctrl_data.chassis_spin_vel, &data[2], sizeof(float));
    robot_ctrl_data.new_data = 1;
}

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    
    // 根据数据包头字节判断数据类型
    uint8_t header = vis_recv_buff[0];
    
    switch (header)
    {
    case PACKET_HEADER_VISION_RX:  // 0xA5 - 视觉目标数据 (原seasky协议)
        if (get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch))
        {
            VisionDecodeFlags(flag_register, &recv_data);
            recv_data.new_data = 1;
        }
        break;
        
    case PACKET_HEADER_ROBOT_CTRL:  // 0xA3 - 机器人控制命令 (云台扫描等)
        DecodeRobotCtrl(vis_recv_buff, recv_len);
        break;
        
    // 注：0xA4 (PACKET_HEADER_TWIST) 底盘速度控制由 radar_comm 模块处理
        
    default:
        // 尝试用seasky协议解析 (兼容旧协议)
        if (get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch))
        {
            VisionDecodeFlags(flag_register, &recv_data);
            recv_data.new_data = 1;
        }
        break;
    }
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    flag_register = VisionEncodeFlags(&send_data);
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP

/* ================== 新增：裁判系统数据发送实现 ================== */
/* 使用简单二进制协议，与SMBU兼容: [Header(1B)][Data][CRC16(2B)] */

/**
 * @brief 计算CRC16校验值并附加到数据包末尾
 */
static void AppendCRC16(uint8_t *data, uint16_t len)
{
    uint16_t crc = crc_16(data, len);
    data[len] = crc & 0xff;
    data[len + 1] = (crc >> 8) & 0xff;
}

/**
 * @brief 发送所有机器人血量数据到上位机
 */
void SendAllRobotHP(const All_Robot_HP_Send_s *hp)
{
    if (!hp) return;
    
    static uint8_t send_buff[64];
    uint16_t data_len = sizeof(All_Robot_HP_Send_s);
    
    send_buff[0] = PACKET_HEADER_ALL_ROBOT_HP;  // 0x5B
    memcpy(&send_buff[1], hp, data_len);
    AppendCRC16(send_buff, 1 + data_len);
    
#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance, send_buff, 1 + data_len + 2, USART_TRANSFER_DMA);
#endif
#ifdef VISION_USE_VCP
    USBTransmit(send_buff, 1 + data_len + 2);
#endif
}

/**
 * @brief 发送比赛状态数据到上位机
 */
void SendGameStatus(const Game_Status_Send_s *status)
{
    if (!status) return;
    
    static uint8_t send_buff[16];
    uint16_t data_len = sizeof(Game_Status_Send_s);
    
    send_buff[0] = PACKET_HEADER_GAME_STATUS;  // 0x5C
    memcpy(&send_buff[1], status, data_len);
    AppendCRC16(send_buff, 1 + data_len);
    
#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance, send_buff, 1 + data_len + 2, USART_TRANSFER_DMA);
#endif
#ifdef VISION_USE_VCP
    USBTransmit(send_buff, 1 + data_len + 2);
#endif
}

/**
 * @brief 发送机器人状态数据到上位机
 */
void SendRobotStatus(const Robot_Status_Send_s *status)
{
    if (!status) return;
    
    static uint8_t send_buff[16];
    uint16_t data_len = sizeof(Robot_Status_Send_s);
    
    send_buff[0] = PACKET_HEADER_ROBOT_STATUS;  // 0x5D
    memcpy(&send_buff[1], status, data_len);
    AppendCRC16(send_buff, 1 + data_len);
    
#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance, send_buff, 1 + data_len + 2, USART_TRANSFER_DMA);
#endif
#ifdef VISION_USE_VCP
    USBTransmit(send_buff, 1 + data_len + 2);
#endif
}
