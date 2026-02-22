/**
 * @file radar_comm.h
 * @brief Radar communication module (USB CDC) - NUC/Jetson sends vx,vy,wz,yaw over USB
 *
 * Protocol: [SYNC1=0xA5] [SYNC2=0x5A] [vx:4B] [vy:4B] [wz:4B] [yaw:4B] [CRC8:1B]
 * Total: 19 bytes (2 + 16 + 1)
 * yaw: 底盘在世界/雷达系下的 yaw，度，用于固定正方向（雷达 IMU）
 * CRC8: polynomial 0x07, computed over first 18 bytes
 *
 * NUC 需在 cmd_vel 基础上扩展发送 yaw（哨兵底盘朝向，度）
 */

#ifndef RADAR_COMM_H
#define RADAR_COMM_H

#include <stdint.h>

#pragma pack(1)

typedef struct
{
    float vx;        // m/s
    float vy;        // m/s
    float wz;        // rad/s
    float yaw_deg;   // 底盘在雷达/世界系下的 yaw（度），雷达 IMU 给出，用于固定正方向
    uint32_t ts_ms;  // HAL_GetTick() timestamp when frame was parsed
    uint8_t valid;   // 1 = valid, 0 = timeout or error
} Radar_Recv_s;

// Frame format: [SYNC1=0xA5] [SYNC2=0x5A] [vx:4] [vy:4] [wz:4] [yaw:4] [CRC8:1B]
#define RADAR_FRAME_SYNC1 0xA5u
#define RADAR_FRAME_SYNC2 0x5Au
#define RADAR_FRAME_SIZE 19u
#define RADAR_FRAME_DATA_SIZE 16u  // 4 floats
#define RADAR_RX_BUFFER_SIZE 128u  // Circular buffer for USB CDC data

#pragma pack()

/** Initialize radar comm (USB CDC). Returns pointer to receive struct. */
Radar_Recv_s *RadarComm_Init(void);

/** Start reception (no-op for USB CDC). */
void RadarComm_StartReceive(void);

/** USB CDC receive callback (called from CDC_Receive_FS) - writes to ring buffer, ISR-safe */
void RadarComm_RxCallback(uint8_t *buf, uint32_t len);

/** Process pending data in ring buffer. Call periodically (e.g. 200Hz in RobotCMDTask). */
void RadarComm_Task(void);

/** Get pointer to latest data */
Radar_Recv_s *RadarComm_GetData(void);

#endif // RADAR_COMM_H
