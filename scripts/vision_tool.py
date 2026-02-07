#!/usr/bin/env python3
"""Interactive vision communication tool for seasky protocol.

Communicates with the STM32 MCU over UART/VCP. Supports:
  - Read mode:  display MCU attitude data (yaw/pitch/roll in degrees)
  - Write mode: send vision commands to MCU interactively

    python3 vision_tool.py
"""
import struct
import sys
from datetime import datetime

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:
    print("pyserial is required.  pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# CRC (must match firmware crc8.c / crc16.c)
# ---------------------------------------------------------------------------
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


def _init_crc16_table():
    table, poly = [], 0xA001
    for i in range(256):
        crc, c = 0, i
        for _ in range(8):
            crc = (crc >> 1) ^ poly if (crc ^ c) & 1 else crc >> 1
            c >>= 1
        table.append(crc & 0xFFFF)
    return table


CRC16_TABLE = _init_crc16_table()


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


# ---------------------------------------------------------------------------
# Enum labels
# ---------------------------------------------------------------------------
SOF = 0xA5
OFFSET_BYTE = 8

FIRE_MODE = {0: "NO_FIRE", 1: "AUTO_FIRE", 2: "AUTO_AIM"}
TARGET_STATE = {0: "NO_TARGET", 1: "CONVERGING", 2: "READY_TO_FIRE"}
TARGET_TYPE = {
    0: "NONE", 1: "HERO1", 2: "ENGINEER2", 3: "INFANTRY3",
    4: "INFANTRY4", 5: "INFANTRY5", 6: "OUTPOST", 7: "SENTRY", 8: "BASE",
}
ENEMY_COLOR = {0: "NONE", 1: "BLUE", 2: "RED"}
WORK_MODE = {0: "AIM", 1: "SMALL_BUFF", 2: "BIG_BUFF"}
BULLET_SPEED = {
    0: "NONE", 1: "10 m/s", 2: "15 m/s", 3: "16 m/s",
    4: "18 m/s", 5: "30 m/s",
}

# ---------------------------------------------------------------------------
# ANSI colors (safe: degrades gracefully on dumb terminals)
# ---------------------------------------------------------------------------
BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
RED = "\033[31m"
RESET = "\033[0m"


# ---------------------------------------------------------------------------
# Frame build / parse
# ---------------------------------------------------------------------------
def build_frame(cmd_id: int, flags: int, floats: list) -> bytes:
    float_data = struct.pack(f"<{len(floats)}f", *floats)
    data_len = 2 + len(float_data)
    buf = bytearray()
    buf.append(SOF)
    buf += struct.pack("<H", data_len)
    buf.append(crc8(buf[:3]))
    buf += struct.pack("<H", cmd_id)
    buf += struct.pack("<H", flags)
    buf += float_data
    buf += struct.pack("<H", crc16(buf))
    return bytes(buf)


def try_parse_frame(buf: bytearray):
    """Try to extract one frame from *buf*.
    Returns (info_dict, consumed) on success, (None, skip) otherwise.
    """
    if len(buf) < 4:
        return None, 0
    if buf[0] != SOF:
        return None, 1
    if buf[3] != crc8(bytes(buf[:3])):
        return None, 1
    data_len = buf[1] | (buf[2] << 8)
    total = data_len + OFFSET_BYTE
    if len(buf) < total:
        return None, 0
    frame = bytes(buf[:total])
    if crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
        return None, 1
    cmd_id = frame[4] | (frame[5] << 8)
    flags = frame[6] | (frame[7] << 8)
    fb = frame[8:-2]
    n = len(fb) // 4
    floats = list(struct.unpack(f"<{n}f", fb[: n * 4]))
    return {"cmd_id": cmd_id, "flags": flags, "floats": floats}, total


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def prompt(msg: str, default: str = "") -> str:
    try:
        hint = f" {DIM}[{default}]{RESET}" if default else ""
        val = input(f"  {msg}{hint}: ").strip()
        return val if val else default
    except (EOFError, KeyboardInterrupt):
        print()
        return ""


def pick_port() -> str:
    ports = sorted(comports(), key=lambda p: p.device)
    if not ports:
        print(f"  {YELLOW}No serial ports detected.{RESET}")
        return prompt("Enter port manually", "/dev/ttyACM0")

    print(f"  {BOLD}Available ports:{RESET}")
    for i, p in enumerate(ports, 1):
        desc = f" {DIM}- {p.description}{RESET}" if p.description and p.description != "n/a" else ""
        print(f"    {CYAN}[{i}]{RESET} {p.device}{desc}")
    print()

    choice = prompt("Select port number or type path", "1")
    if choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(ports):
            return ports[idx].device
    return choice



def separator():
    print(f"  {DIM}{'─' * 56}{RESET}")


# ---------------------------------------------------------------------------
# Read mode
# ---------------------------------------------------------------------------
def fmt_frame(info: dict) -> str:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    cmd = info["cmd_id"]
    fl = info["floats"]

    if cmd == 0x02:  # MCU -> PC  (yaw, pitch, roll in degrees)
        flags = info["flags"]
        ec = ENEMY_COLOR.get((flags >> 8) & 0x3, "?")
        wm = WORK_MODE.get((flags >> 10) & 0x3, "?")
        bs = BULLET_SPEED.get((flags >> 12) & 0xF, "?")
        yaw = fl[0] if len(fl) > 0 else 0.0
        pitch = fl[1] if len(fl) > 1 else 0.0
        roll = fl[2] if len(fl) > 2 else 0.0
        return (
            f"  {DIM}[{ts}]{RESET}  "
            f"yaw={GREEN}{yaw:+9.4f}deg{RESET}  "
            f"pitch={GREEN}{pitch:+9.4f}deg{RESET}  "
            f"roll={GREEN}{roll:+9.4f}deg{RESET}  "
            f"{DIM}enemy={ec} work={wm} bullet={bs}{RESET}"
        )
    return (
        f"  {DIM}[{ts}]{RESET}  "
        f"cmd=0x{cmd:04X}  flags=0x{info['flags']:04X}  data={fl}"
    )


def run_read(ser: serial.Serial):
    separator()
    print(f"  {BOLD}Read Mode{RESET} - receiving MCU attitude data")
    print(f"  {DIM}Note: MCU outputs yaw/pitch/roll in degrees.{RESET}")
    print(f"  {DIM}Press Ctrl+C to stop.{RESET}")
    separator()
    print()

    buf = bytearray()
    try:
        while True:
            chunk = ser.read(max(ser.in_waiting, 1))
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(SOF)
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    buf = buf[idx:]
                info, consumed = try_parse_frame(buf)
                if consumed == 0:
                    break
                if info is None:
                    buf = buf[consumed:]
                    continue
                print(fmt_frame(info))
                buf = buf[consumed:]
    except KeyboardInterrupt:
        print(f"\n  {DIM}Stopped.{RESET}\n")


# ---------------------------------------------------------------------------
# Write mode
# ---------------------------------------------------------------------------
FIELDS = [
    # key,            label,                  type, enum_map
    ("yaw",           "Yaw offset  (rad)",    float, None),
    ("pitch",         "Pitch offset (rad)",   float, None),
    ("fire_mode",     "Fire mode",            int,   FIRE_MODE),
    ("target_state",  "Target state",         int,   TARGET_STATE),
    ("target_type",   "Target type",          int,   TARGET_TYPE),
]


def build_send_flags(s: dict) -> int:
    flags = 0
    flags |= (s["fire_mode"] & 0x3)
    flags |= (s["target_state"] & 0x3) << 2
    flags |= (s["target_type"] & 0xF) << 4
    return flags


def show_state(s: dict):
    print()
    for i, (key, label, _, emap) in enumerate(FIELDS, 1):
        v = s[key]
        if emap:
            name = emap.get(v, "?")
            opts = " | ".join(f"{k}={n}" for k, n in emap.items())
            print(f"    {CYAN}[{i}]{RESET} {label:22s}:  {BOLD}{v}{RESET}  ({name})")
            print(f"        {DIM}{opts}{RESET}")
        else:
            print(f"    {CYAN}[{i}]{RESET} {label:22s}:  {BOLD}{v}{RESET}")


def do_send(ser: serial.Serial, s: dict):
    flags = build_send_flags(s)
    frame = build_frame(0x0001, flags, [s["pitch"], s["yaw"]])
    ser.write(frame)
    ser.flush()
    print(f"  {GREEN}-> Sent {len(frame)} bytes{RESET}  {DIM}[{frame.hex()}]{RESET}")


def run_write(ser: serial.Serial):
    state = {
        "yaw": 0.0,
        "pitch": 0.0,
        "fire_mode": 0,
        "target_state": 2,
        "target_type": 0,
    }

    separator()
    print(f"  {BOLD}Write Mode{RESET} - send vision commands to MCU")
    separator()
    print()
    print(f"  {YELLOW}IMPORTANT - for the MCU gimbal to respond:{RESET}")
    print(f"    1. RC left  switch -> {BOLD}MID{RESET} position  (enables vision mode)")
    print(f"    2. RC right switch -> {BOLD}DOWN{RESET} position (GYRO_MODE)")
    print(f"       or {BOLD}MID{RESET} position (FREE_MODE)")
    print(f"    3. Set {BOLD}target_state{RESET} to 1 (CONVERGING) or 2 (READY_TO_FIRE)")
    print(f"       {DIM}If target_state is 0 (NO_TARGET), vision data is ignored.{RESET}")
    print()
    print(f"  {DIM}yaw/pitch are one-shot offsets in radians,")
    print(f"  accumulated onto the current gimbal target angle.{RESET}")
    separator()
    print()
    print(f"  Commands:  {CYAN}1-5{RESET} edit field   "
          f"{CYAN}s{RESET} send   {CYAN}q{RESET} quit")

    while True:
        show_state(state)
        cmd = prompt(f"\n  {BOLD}>{RESET} ")
        if not cmd:
            continue
        if cmd == "q":
            break
        elif cmd == "s":
            do_send(ser, state)
        elif cmd.isdigit():
            idx = int(cmd) - 1
            if 0 <= idx < len(FIELDS):
                key, label, typ, _ = FIELDS[idx]
                raw = prompt(f"  New value for {label}", str(state[key]))
                if raw:
                    try:
                        state[key] = typ(raw)
                    except ValueError:
                        print(f"    {RED}Invalid input, value unchanged.{RESET}")
            else:
                print(f"    {RED}Invalid field number.{RESET}")
        else:
            print(f"    {DIM}Unknown command. Use 1-5, s, or q.{RESET}")
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print()
    print(f"  {BOLD}╔══════════════════════════════════════════╗{RESET}")
    print(f"  {BOLD}║   Vision Tool  -  seasky protocol v1.0   ║{RESET}")
    print(f"  {BOLD}╚══════════════════════════════════════════╝{RESET}")
    print()

    port = pick_port()
    if not port:
        return
    baud = 921600

    print()
    print(f"  {BOLD}Select mode:{RESET}")
    print(f"    {CYAN}[1]{RESET} Read  - receive & display MCU attitude (yaw/pitch/roll in deg)")
    print(f"    {CYAN}[2]{RESET} Write - send vision commands to MCU")
    mode = prompt("Mode", "1")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"\n  {RED}Failed to open {port}: {e}{RESET}\n")
        return

    with ser:
        print(f"\n  {GREEN}Opened {port} @ {baud}{RESET}")
        if mode == "2":
            run_write(ser)
        else:
            run_read(ser)

    print(f"  {DIM}Port closed. Bye!{RESET}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n  {DIM}Interrupted. Bye!{RESET}")
