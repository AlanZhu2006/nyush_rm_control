# NYUSH RoboMaster Control

NYUSH Robotics Club RoboMaster firmware based on Hunan University YueLu team's **basic_framework**. Supports infantry, hero, and **sentry** (哨兵) robot types via compile-time config.

- **Target:** STM32F407IGH6 (RoboMaster Type C Development Board)
- **RTOS:** FreeRTOS
- **Toolchain:** ARM GNU Toolchain (arm-none-eabi-gcc), Makefile / CMake
- **Original framework:** [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)

Quick start: [Setup Guide](docs/setup-guide.md) | [Flashing Guide](docs/flashing-guide.md)

---

## Documentation

| Doc | Description |
|-----|-------------|
| [Setup Guide](docs/setup-guide.md) | Dev environment (Mac/Windows/Linux) |
| [Flashing Guide](docs/flashing-guide.md) | DFU, OpenOCD, ST-Link, J-Link |
| [CAN Communication](docs/can.md) | CAN protocol for GM6020 / C620 |
| [GitHub Commands](docs/github-commands.md) | Git workflow |
| [Basic Framework README](docs/basic_framework/README.md) | Framework intro (Chinese) |
| [Architecture Guide](docs/basic_framework/架构介绍与开发指南.md) | 3-layer architecture (Chinese) |

---

## Repository structure (high level)

```
├── application/          # Layer 3: robot logic (cmd, gimbal, chassis, shoot)
│   ├── robot_def.h       # Shared types, CMD/feedback structs
│   ├── robot_config_select.h
│   ├── robot_configs/   # Robot-specific config (robot_sentry.h, etc.)
│   ├── cmd/              # Command center (RC + vision → commands)
│   ├── gimbal/           # 2-DOF gimbal
│   ├── chassis/          # Chassis (mecanum or sentry swerve)
│   └── shoot/            # Friction wheel + loader
├── modules/              # Layer 2: hardware-agnostic drivers & algorithms
├── bsp/                   # Layer 1: HAL-based hardware abstraction
├── docs/
├── Makefile
└── Src/ Inc/ Drivers/ Middlewares/  # CubeMX & HAL
```

---

## Architecture (three layers)

All application-layer communication goes through **message_center** (pub-sub). Apps do not include each other.

### Layer 1: BSP (`bsp/`)

Hardware abstraction on STM32 HAL. Each peripheral uses an **instance + callback** pattern.

| BSP | Role |
|-----|------|
| `can/` | CAN bus (hcan1, hcan2, etc.) |
| `usart/` | UART (RC, vision, log) |
| `usb/` | USB VCP (e.g. vision) |
| `spi/`, `iic/` | SPI/I2C (IMU, sensors) |
| `pwm/`, `gpio/`, `adc/` | PWM, GPIO, ADC |
| `dwt/` | Timing (DWT_Delay) |
| `log/` | Segger RTT logging |
| `flash/` | On-chip flash |

### Layer 2: Module (`modules/`)

| Module | Role |
|--------|------|
| **Motor** | `motor/DJImotor/` (M3508, M2006, GM6020), `HTmotor/`, `LKmotor/`, `servo_motor/`, `step_motor/` |
| **Sensors** | `imu/` + `BMI088/` (attitude), `ist8310/`, `TFminiPlus/` |
| **Communication** | `can_comm/`, `referee/`, `remote/` (DT7-DR16), **`master_machine/`** (vision SeaSky/VCP/UART), `radar_comm/`, `unicomm/` |
| **Algorithms** | `algorithm/` — PID (`controller`), Kalman, QuaternionEKF, `user_lib`, CRC |
| **System** | `message_center/` (pub-sub), `daemon/` (watchdog/offline), `alarm/` (buzzer/LED), `super_cap/` |

### Layer 3: Application (`application/`)

| App | Role |
|-----|------|
| **cmd** (`cmd/`) | Receives RC + vision; publishes `Chassis_Ctrl_Cmd_s`, `Gimbal_Ctrl_Cmd_s`, `Shoot_Ctrl_Cmd_s`; subscribes to chassis/gimbal/shoot feedback |
| **gimbal** (`gimbal/`) | Subscribes to gimbal cmd; runs yaw/pitch control (e.g. GM6020); publishes `Gimbal_Upload_Data_s` |
| **chassis** (`chassis/`) | Subscribes to chassis cmd; kinematics → motor refs; publishes `Chassis_Upload_Data_s`. For sentry, uses `sentry_controller` instead of `chassis.c` |
| **shoot** (`shoot/`) | Subscribes to shoot cmd; friction wheel + loader; publishes `Shoot_Upload_Data_s` |

**Data flow:**

```
RC / Vision  →  cmd  →  message_center  →  gimbal / chassis / shoot
                     ←  message_center  ←  (feedback)
```

**Robot type selection:** At compile time, one of `ROBOT_TYPE_infantry`, `ROBOT_TYPE_hero`, or `ROBOT_TYPE_sentry` is defined. `robot_config_select.h` includes the corresponding `robot_configs/robot_*.h`, which defines board type (`ONE_BOARD` etc.), chassis type (`CHASSIS_TYPE_MECANUM` / `CHASSIS_TYPE_SWERVE`), vision interface (`VISION_USE_VCP` / `VISION_USE_UART`), and all motor IDs, PID, dimensions.

**Config:** `application/robot_def.h` holds shared enums/structs; robot-specific numbers live in `application/robot_configs/robot_<type>.h`.

---

## Sentry (哨兵) — detailed

The sentry build uses **swerve chassis** (舵轮), vision-driven commands, and the same gimbal/shoot stack as the rest of the framework.

### Build selection

- In `application/robot_configs/robot_sentry.h`: `ROBOT_TYPE_sentry` is implied by including this file via `robot_config_select.h` (which is driven by Makefile/CMake `ROBOT_TYPE=sentry` or equivalent).
- `robot.c` and `chassis` wiring: when `ROBOT_TYPE_sentry` is defined, `SentryChassisInit()` / `SentryChassisTask()` are used instead of `ChassisInit()` / `ChassisTask()`.

### Sentry config file: `application/robot_configs/robot_sentry.h`

- **Board:** `ONE_BOARD` (single MCU).
- **Vision:** `VISION_USE_VCP` (USB virtual COM) or `VISION_USE_UART`.
- **Chassis type:** `CHASSIS_TYPE_SWERVE`.
- **Mechanical:** `WHEEL_BASE`, `TRACK_WIDTH`, `RADIUS_WHEEL`, `REDUCTION_RATIO_WHEEL`; gimbal alignment `YAW_CHASSIS_ALIGN_ECD`, `PITCH_HORIZON_ECD`, pitch limits; shooter `ONE_BULLET_DELTA_ANGLE`, `NUM_PER_CIRCLE`, etc.
- **Motor directions:** `GIMBAL_*_REVERSE`, `CHASSIS_MOTOR_*_REVERSE`, `STEER_MOTOR_A/B_REVERSE`, `SHOOT_*_REVERSE`.
- **CAN IDs:** chassis drive (LF/RF/LB/RB), steer A/B, gimbal yaw/pitch, shoot friction/loader — all in this header.
- **PID:** chassis speed/current; steer angle/speed/current; gimbal yaw/pitch angle/speed; shoot friction/loader (with comments referencing robomaster sentry_swerve).

### Sentry chassis: swerve (舵轮)

- **Hardware:** Two **swerve modules** on the diagonal (e.g. left-front and right-rear). Each has:
  - One **GM6020** for steering (angle loop).
  - One **M3508** for drive (speed loop).
- The other two corners (e.g. RF, LB) can be passive or omitted in config; sentry code uses only **two drive + two steer** motors.
- **CAN layout (typical):**
  - **CAN1 (hcan1):** chassis drive LF/RF/LB/RB (M3508), steer A (GM6020 ID 5), steer B (GM6020 ID 6), gimbal yaw (GM6020 ID 7).
  - **CAN2 (hcan2):** gimbal pitch (GM6020 ID 5), shoot loader (M3508 ID 1), friction L/R (M3508 ID 6, 8).

### Sentry chassis logic: `application/chassis/sentry_controller.c` + `steering.c`

- **Startup:** `SentrySteerAlign()` — both steer motors move to a defined “forward” position (e.g. `STEER_MOTOR_A_INIT_ANGLE`, `STEER_MOTOR_B_INIT_ANGLE` in encoder ticks). Drive motors stay at zero. After alignment (or timeout), normal control runs.
- **Modes (conceptually):**
  - **Translation only:** `SentryTranslationCalculate(vx, vy)` — uses `SteeringCalculate()` in `steering.c` to compute steer angles (and shortest path) and drive direction; both drive motors get the same scaled speed.
  - **Spin only:** `SentrySpinCalculate(wz)` — steer angles set to tangential direction for rotation around chassis center; both drives get the same signed speed for spin.
  - **Spin + translation:** `SentrySpinWithTranslationCalculate(wz, vx, vy)` — each wheel’s velocity vector = translation + tangential rotation; steer angle = direction of that vector; drive speed = magnitude.
- **Input:** Subscribes to `chassis_cmd` (e.g. `vx`, `vy`, `wz`, `chassis_mode`). Speed deadband and scaling (e.g. `CHASSIS_DRIVE_SPEED_SCALE`, `CHASSIS_SPIN_SPEED_SCALE`) applied in sentry_controller.
- **Output:** `SentryLimitChassisOutput()` sets M3508 refs from `vt_drive_a` / `vt_drive_b`; GM6020 refs set in the calculate functions (angle or angle increment for shortest path).
- **Steering math:** `application/chassis/steering.c` / `steering.h` — `SteeringCalculate(vx, vy, init_angle_a/b, current_angle_a/b, out_angle_a/b, out_direction)` for translation; tick/degree conversion helpers.

### Vision (上位机)

- **Module:** `modules/master_machine/` — receives vision data (SeaSky protocol) over **VCP** or **UART** (see `robot_sentry.h`).
- **Usage:** `master_process` fills `Robot_Ctrl_Recv_s`; cmd layer reads it (and/or radar_comm) to produce chassis/gimbal/shoot commands. So sentry can be driven by vision + RC.
- **Referee / UI:** Sentry uses the same referee task and UI as the rest of the framework (e.g. game state, health); chassis feedback can be used for UI.

### Gimbal & shoot on sentry

- Same **gimbal** and **shoot** applications as other robot types: gimbal uses yaw/pitch GM6020; shoot uses friction wheels + loader. All parameters and CAN IDs for sentry are in `robot_sentry.h`.

### Summary table (sentry)

| Item | Detail |
|------|--------|
| Config header | `application/robot_configs/robot_sentry.h` |
| Chassis entry | `application/chassis/sentry_controller.c` (`SentryChassisInit`, `SentryChassisTask`) |
| Steer math | `application/chassis/steering.c` / `steering.h` |
| Drive motors | 2× M3508 (e.g. LB, RB) |
| Steer motors | 2× GM6020 (steer A, B) |
| Gimbal | 2× GM6020 (yaw on CAN1, pitch on CAN2) |
| Shoot | M3508 loader + 2× M3508 friction |
| Vision | `master_machine` (SeaSky), VCP or UART |
| CMD | Same `robot_cmd`; subscribes/publishes same topics; sentry-specific handling behind `#ifdef ROBOT_TYPE_sentry` where needed |

---

## Build & flash

**Build:**

```bash
make -j12              # macOS/Linux
mingw32-make -j12      # Windows (MSYS2/MinGW64)
# Or: Ctrl+Shift+B / Cmd+Shift+B in VSCode
```

**Flash:**

```bash
make flash_dfu         # DFU (hold BOOT0, press RESET, release BOOT0)
make flash_dap        # OpenOCD + DAP-Link
make flash_stlink     # OpenOCD + ST-Link
```

Artifacts: `build/basic_framework.{elf,hex,bin}`. See [Flashing Guide](docs/flashing-guide.md).

---

## Remotes & sync

- **origin** — Upstream (e.g. NYUSH-Robotics-Club); pull updates from here.
- **newrepo** — Your fork; push your branch here only, e.g. `git push newrepo sentry_alan`.

To sync from origin **without losing local-only commits** (e.g. this README): use **merge**, not reset:

```bash
git fetch origin
git merge origin/sentry_alan
git push newrepo sentry_alan
```

Do **not** run `git reset --hard origin/sentry_alan` if you want to keep commits that exist only on newrepo.

---

## Credits

- **Original framework:** Hunan University YueLu RoboMaster Team (2022–2023)
- **NYUSH adaptation:** NYUSH Robotics Club  
- License: MIT
