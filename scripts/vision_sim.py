#!/usr/bin/env python3
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


CRC8_TABLE = [
    0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
    67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
    134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
    197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
    61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
    126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
    187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
    248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
    122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
    57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
    252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
    191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
    71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
    4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
    193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
    130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172,
]


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc = CRC8_TABLE[b ^ crc]
    return crc & 0xFF


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0x00FF]
    return crc & 0xFFFF


def init_crc16_table() -> list:
    table = []
    poly = 0xA001
    for i in range(256):
        crc = 0
        c = i
        for _ in range(8):
            if (crc ^ c) & 0x0001:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            c >>= 1
        table.append(crc & 0xFFFF)
    return table


CRC16_TABLE = init_crc16_table()


def build_flags(
    fire_mode: int,
    target_state: int,
    target_type: int,
    enemy_color: int,
    work_mode: int,
    bullet_speed_code: int,
) -> int:
    flags = 0
    flags |= (fire_mode & 0x3) << 0
    flags |= (target_state & 0x3) << 2
    flags |= (target_type & 0xF) << 4
    flags |= (enemy_color & 0x3) << 8
    flags |= (work_mode & 0x3) << 10
    flags |= (bullet_speed_code & 0xF) << 12
    return flags & 0xFFFF


def build_frame(cmd_id: int, flags: int, pitch: float, yaw: float) -> bytes:
    float_data = struct.pack("<ff", pitch, yaw)
    data_len = 2 + len(float_data)

    frame = bytearray()
    frame.append(0xA5)
    frame += struct.pack("<H", data_len)
    frame.append(crc8(frame[0:3]))
    frame += struct.pack("<H", cmd_id)
    frame += struct.pack("<H", flags)
    frame += float_data
    frame += struct.pack("<H", crc16(frame))
    return bytes(frame)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision UART simulator for seasky protocol.")
    parser.add_argument("--port", required=True, default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch error in radians")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw error in radians")
    parser.add_argument("--fire-mode", type=int, default=0, help="0..3")
    parser.add_argument("--target-state", type=int, default=0, help="0..3")
    parser.add_argument("--target-type", type=int, default=0, help="0..15")
    parser.add_argument("--enemy-color", type=int, default=0, help="0..3")
    parser.add_argument("--work-mode", type=int, default=0, help="0..3")
    parser.add_argument("--bullet-speed-code", type=int, default=0, help="0..15")
    parser.add_argument("--cmd-id", type=lambda x: int(x, 0), default=0x0001, help="Command ID (hex or dec)")
    parser.add_argument("--rate", type=float, default=20.0, help="Send rate Hz (default: 20)")
    parser.add_argument("--once", action="store_true", help="Send once and exit")
    parser.add_argument("--print", action="store_true", help="Print hex frame")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    flags = build_flags(
        args.fire_mode,
        args.target_state,
        args.target_type,
        args.enemy_color,
        args.work_mode,
        args.bullet_speed_code,
    )
    frame = build_frame(args.cmd_id, flags, args.pitch, args.yaw)

    if args.print:
        print(frame.hex())

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        if args.once:
            ser.write(frame)
            ser.flush()
            return 0

        interval = 1.0 / max(args.rate, 0.1)
        next_time = time.time()
        while True:
            now = time.time()
            if now >= next_time:
                ser.write(frame)
                next_time = now + interval
            time.sleep(0.001)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
