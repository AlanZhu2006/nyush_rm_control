/**
 * @file radar_comm.c
 * @brief Radar communication module (USB CDC) - framing, CRC8, ring buffer
 *
 * Protocol: [0xA5][0x5A][vx:f32][vy:f32][wz:f32][crc8]
 * CRC8 polynomial: 0x07
 *
 * Data flow: CDC_Receive_FS -> RadarComm_RxCallback -> ring buffer
 *            RadarComm_Task -> parse frames -> PubPushMessage("radar_cmd")
 */

#include "radar_comm.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// CRC8: polynomial 0x07 (same as robomaster cmd_vel_forwarder.py)
static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0x07;
        else
            crc = (crc << 1);
    }
    return crc;
}

static uint8_t crc8_calc(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++)
        crc = crc8_update(crc, data[i]);
    return crc;
}

typedef struct
{
    uint8_t buffer[RADAR_RX_BUFFER_SIZE];
    volatile uint32_t write_idx;
    uint32_t read_idx;
} RingBuffer_t;

static RingBuffer_t ring_buffer;
static Radar_Recv_s recv_data;
static uint32_t last_valid_time_ms = 0u;

#define RADAR_DATA_TIMEOUT_MS 1000u

static void ring_buffer_write(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0)
        return;
    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t next_idx = (ring_buffer.write_idx + 1) % RADAR_RX_BUFFER_SIZE;
        ring_buffer.buffer[ring_buffer.write_idx] = data[i];
        ring_buffer.write_idx = next_idx;
    }
}

static int ring_buffer_read_one(uint8_t *byte)
{
    if (ring_buffer.read_idx == ring_buffer.write_idx)
        return -1;
    *byte = ring_buffer.buffer[ring_buffer.read_idx];
    ring_buffer.read_idx = (ring_buffer.read_idx + 1) % RADAR_RX_BUFFER_SIZE;
    return 0;
}

static int ring_buffer_peek(uint32_t offset, uint8_t *byte)
{
    if (offset >= RADAR_RX_BUFFER_SIZE)
        return -1;
    uint32_t idx = (ring_buffer.read_idx + offset) % RADAR_RX_BUFFER_SIZE;
    if (idx == ring_buffer.write_idx && offset > 0)
        return -1;
    *byte = ring_buffer.buffer[idx];
    return 0;
}

static void ring_buffer_skip(uint32_t count)
{
    ring_buffer.read_idx = (ring_buffer.read_idx + count) % RADAR_RX_BUFFER_SIZE;
}

void RadarComm_RxCallback(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0)
        return;
    ring_buffer_write(buf, len);
}

void RadarComm_Task(void)
{
    uint8_t byte = 0;

    while (ring_buffer_read_one(&byte) == 0)
    {
        if (byte != RADAR_FRAME_SYNC1)
            continue;

        if (ring_buffer_peek(0, &byte) != 0)
            break;
        if (byte != RADAR_FRAME_SYNC2)
            continue;

        if (ring_buffer.read_idx == ring_buffer.write_idx)
        {
            ring_buffer.read_idx = (ring_buffer.read_idx - 1 + RADAR_RX_BUFFER_SIZE) % RADAR_RX_BUFFER_SIZE;
            break;
        }

        uint32_t available = ring_buffer.write_idx >= ring_buffer.read_idx
                                 ? (ring_buffer.write_idx - ring_buffer.read_idx + 1)
                                 : (RADAR_RX_BUFFER_SIZE - ring_buffer.read_idx + ring_buffer.write_idx + 1);

        if (available < RADAR_FRAME_SIZE)
        {
            ring_buffer.read_idx = (ring_buffer.read_idx - 1 + RADAR_RX_BUFFER_SIZE) % RADAR_RX_BUFFER_SIZE;
            break;
        }

        uint8_t frame[RADAR_FRAME_SIZE];
        frame[0] = RADAR_FRAME_SYNC1;
        frame[1] = RADAR_FRAME_SYNC2;
        ring_buffer_skip(1);

        for (int i = 2; i < RADAR_FRAME_SIZE; i++)
        {
            if (ring_buffer_read_one(&frame[i]) != 0)
                return;
        }

        uint8_t crc_calc = crc8_calc(frame, RADAR_FRAME_SIZE - 1);
        uint8_t crc_recv = frame[RADAR_FRAME_SIZE - 1];
        if (crc_calc != crc_recv)
            continue;

        float f_vx = 0.0f, f_vy = 0.0f, f_wz = 0.0f;
        memcpy(&f_vx, &frame[2], sizeof(float));
        memcpy(&f_vy, &frame[6], sizeof(float));
        memcpy(&f_wz, &frame[10], sizeof(float));

        recv_data.vx = f_vx;
        recv_data.vy = f_vy;
        recv_data.wz = f_wz;
        recv_data.ts_ms = HAL_GetTick();
        recv_data.valid = 1;
        last_valid_time_ms = recv_data.ts_ms;
    }

    uint32_t now = HAL_GetTick();
    if (recv_data.valid && (now - last_valid_time_ms > RADAR_DATA_TIMEOUT_MS))
        recv_data.valid = 0;
}

void RadarComm_StartReceive(void)
{
}

Radar_Recv_s *RadarComm_Init(void)
{
    memset(&ring_buffer, 0, sizeof(ring_buffer));
    memset(&recv_data, 0, sizeof(recv_data));
    return &recv_data;
}

Radar_Recv_s *RadarComm_GetData(void)
{
    return &recv_data;
}
