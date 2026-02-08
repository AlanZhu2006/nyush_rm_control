/**
 * @file radar_comm.h
 * @brief Radar communication module (USB CDC) - NUC/Jetson sends vx,vy,wz over USB
 *
 * Protocol: [SYNC1=0xA5] [SYNC2=0x5A] [vx:4B] [vy:4B] [wz:4B] [CRC8:1B]
 * Total: 15 bytes (2 + 12 + 1)
 * CRC8: polynomial 0x07, computed over first 14 bytes
 *
 * Compatible with robomaster-control cmd_vel_forwarder.py
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
    uint32_t ts_ms;  // HAL_GetTick() timestamp when frame was parsed
    uint8_t valid;   // 1 = valid, 0 = timeout or error
} Radar_Recv_s;

// Frame format: [SYNC1=0xA5] [SYNC2=0x5A] [vx:4B] [vy:4B] [wz:4B] [CRC8:1B]
#define RADAR_FRAME_SYNC1 0xA5u
#define RADAR_FRAME_SYNC2 0x5Au
#define RADAR_FRAME_SIZE 15u
#define RADAR_FRAME_DATA_SIZE 12u  // 3 floats
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
