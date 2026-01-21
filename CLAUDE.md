# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **basic_framework**, an embedded control framework for RoboMaster competition robots developed by Hunan University YueLu RoboMaster team. The framework runs on STM32F407IGH6 MCU with FreeRTOS and provides a layered architecture for robot control systems (infantry, hero, sentry, balance infantry, etc.).

**Target Hardware:** STM32F407IGH6 (RoboMaster Type C Development Board)
**RTOS:** FreeRTOS
**Build System:** Makefile (primary), CMake (supported)
**Toolchain:** ARM GNU Toolchain (arm-none-eabi-gcc)

## Build and Flash Commands

### Building

**macOS/Linux:**
```bash
make -j12          # Parallel build with 12 jobs
make clean         # Clean build artifacts
make rebuild       # Clean and rebuild
make help          # Show all available targets
```

**Windows:**
```bash
mingw32-make -j12     # Build (requires MSYS2/MinGW64)
mingw32-make clean    # Clean
mingw32-make rebuild  # Rebuild
mingw32-make help     # Help
```

**VSCode:** Press `Cmd+Shift+B` (Mac) or `Ctrl+Shift+B` (Windows/Linux)

**CMake (Alternative):**
```bash
cmake -B build -G Ninja
cmake --build build -j12
```

### Flashing Firmware

The framework supports cross-platform flashing with 4 methods:

**1. DFU-Util (Recommended - no external debugger needed):**
```bash
make flash_dfu           # Default method
# Or: make flash         # Alias for flash_dfu
```
To enter DFU mode: Hold BOOT0 → Press RESET → Release BOOT0

**2. OpenOCD + DAP-Link:**
```bash
make flash_dap          # Uses openocd_dap.cfg
```

**3. OpenOCD + ST-Link:**
```bash
make flash_stlink
```

**4. J-Link (Windows):**
```bash
make flash_jlink        # Uses stm32.jflash config
```

**Build artifacts:** `build/basic_framework.{elf,hex,bin}`

## Architecture

The framework uses a **3-layer architecture** with strict separation of concerns:

### Layer 1: BSP (Board Support Package) - `bsp/`

Hardware abstraction layer built on STM32 HAL. Each peripheral has an `XXXInstance` structure pattern.

**Key BSPs:**
- **Communication:** `can/`, `usart/`, `spi/`, `iic/`, `usb/`
- **Function:** `pwm/`, `gpio/`, `adc/`, `dwt/` (timing)
- **System:** `log/` (Segger RTT), `flash/`

**Design Pattern:** Instance-based with callback registration. BSP instances require module-provided callbacks for data handling (avoids upward dependencies).

### Layer 2: Module - `modules/`

Hardware-agnostic modules that use BSP underneath. Provides clean interfaces to application layer.

**Motor Drivers:** `motor/{DJImotor,HTmotor,LKmotor,DMmotor,servo_motor,step_motor}`
- DJI: M3508, M2006, GM6020 (RoboMaster motors with CAN interface)
- HT: HT04 high-torque motors
- LK: Lingkong motors
- Servo/Stepper support

**Sensors:**
- `imu/` + `BMI088/` - IMU sensor with quaternion EKF attitude estimation
- `ist8310/` - Magnetometer
- `TFminiPlus/` - Distance sensor

**Communication:**
- `can_comm/` - Multi-board CAN communication
- `referee/` - DJI referee system (game data, UI, robot-to-robot comm)
- `remote/` - DT7-DR16 RC receiver
- `master_machine/` - Vision system communication (SeaSky protocol/VCP/UART)
- `unicomm/` - Universal communication interface

**Algorithms:** `algorithm/`
- `controller.{c,h}` - PID controllers
- `kalman_filter.{c,h}` - Kalman filtering
- `QuaternionEKF.{c,h}` - Quaternion-based attitude estimation
- `user_lib.{c,h}` - Math utilities
- CRC validation (`crc8.c`, `crc16.c`)

**System Support:**
- `daemon/` - Watchdog/offline detection for modules
- `message_center/` - Pub-sub message passing between apps
- `alarm/` - Buzzer/LED alerts

### Layer 3: Application - `application/`

High-level robot control logic. Uses **pub-sub pattern** via `message_center` (NO direct cross-references between apps).

**Core Applications:**
1. **`cmd/`** (robot_cmd) - Command center
   - Receives input from RC/vision system
   - Translates to control commands
   - Publishes commands → Subscribes to feedback

2. **`gimbal/`** - 2-DOF gimbal control
   - Modes: ZERO_FORCE, FREE_MODE, GYRO_MODE
   - Controls yaw/pitch motors (typically GM6020)

3. **`chassis/`** - Mobile base control
   - Supports: Mecanum, omni-directional wheels
   - Modes: ZERO_FORCE, ROTATE, NO_FOLLOW, FOLLOW_GIMBAL_YAW
   - Handles supercapacitor integration

4. **`shoot/`** - Launching mechanism
   - Friction wheels (typically M3508) + loader (M2006)
   - Supports continuous/burst fire modes
   - Magazine servo control

**Configuration:** `application/robot_def.h` contains all robot-specific parameters (encoder alignment values, mechanical dimensions, gear ratios, etc.)

**Board Configuration:** Supports single-board or multi-board (gimbal board + chassis board) via compile-time macros:
- `ONE_BOARD` - Single MCU controls entire robot
- `GIMBAL_BOARD` - Cloud platform board
- `CHASSIS_BOARD` - Chassis board

## Data Flow

**Initialization Flow:**
1. `main()` calls `RobotInit()` (from `robot.c`)
2. `RobotInit()` → BSP init → App init
3. Each app subscribes to message topics via `message_center`
4. FreeRTOS starts → Tasks run periodically

**Runtime Flow:**
```
RC/Vision Input → robot_cmd (parse & publish)
                      ↓
          message_center (pub-sub hub)
                      ↓
    ┌─────────────┬────────────┬─────────────┐
    ↓             ↓            ↓             ↓
 gimbal       chassis      shoot     (feedback)
    │             │            │             │
    └─────────────┴────────────┴─────────────┘
              (publish feedback)
                      ↓
          message_center → robot_cmd
```

**Task Frequencies:**
- `INStask`: 1000 Hz (mandatory)
- `MotorTask`: 200-1000 Hz (motor control loop)
- `RobotTask`: ≥150 Hz (app layer)
- `MonitorTask`: 100 Hz (daemon checks)

## Development Guidelines

### Code Style

**Functions:** PascalCase verb-object phrases (≤4 words)
```c
void SetMotorControl();
void UpdateChassisState();
```

**Variables:** snake_case
```c
uint8_t gimbal_recv_cmd;
float chassis_speed_x;
```

**Type Definitions:**
- `_t` suffix: Simple data types (e.g., `IMU_Data_t`)
- `_s` suffix: Complex structures (e.g., `Motor_Controller_s`)
- `_e` suffix: Enums (e.g., `chassis_mode_e`)
- Instance types: `XXXInstance` (e.g., `CANInstance`, `USARTInstance`)

**Constants/Enums:** UPPER_SNAKE_CASE

### Important Constraints

1. **Standard Units Only:** All physical quantities use SI units (meters, seconds, radians, etc.). Use macros in `modules/general_def.h` for conversions if needed.

2. **UTF-8 Encoding:** All source files must be UTF-8.

3. **User Code Blocks:** When modifying CubeMX-generated files (especially `Src/main.c`, `Src/freertos.c`), place code within `/* USER CODE BEGIN */` and `/* USER CODE END */` blocks to survive regeneration.

4. **Layer Separation:**
   - App layer: NO HAL/peripheral code allowed (hardware-agnostic)
   - Module layer: Uses BSP instances only (no direct HAL calls)
   - BSP layer: Only place for HAL interaction

5. **Cross-App Communication:** Apps MUST use `message_center` pub-sub. Never include one app's header in another app.

6. **Safety Checks:** Always validate inputs - "treat your users as idiots"

### File Organization

```
├── application/          # App layer (robot-specific logic)
│   ├── robot_def.h      # Robot parameters (CRITICAL CONFIG FILE)
│   ├── robot_task.h     # Task scheduling
│   ├── cmd/             # Command processing
│   ├── gimbal/          # Gimbal control
│   ├── chassis/         # Chassis control
│   └── shoot/           # Launcher control
├── modules/             # Module layer (hardware-agnostic)
│   ├── motor/           # Motor drivers (DJI/HT/LK/etc)
│   ├── algorithm/       # Control algorithms & math
│   ├── imu/             # IMU integration
│   ├── referee/         # Referee system
│   ├── message_center/  # Pub-sub messaging
│   └── daemon/          # Offline detection
├── bsp/                 # BSP layer (hardware abstraction)
│   ├── can/             # CAN bus
│   ├── usart/           # UART
│   ├── dwt/             # Timing utilities
│   └── log/             # Logging (Segger RTT)
├── Drivers/             # STM32 HAL & CMSIS
├── Middlewares/         # FreeRTOS, Segger RTT, CMSIS-DSP
├── Src/                 # CubeMX-generated main sources
├── Inc/                 # CubeMX-generated headers
├── docs/basic_framework/ # Detailed documentation
├── Makefile             # Build system
├── CMakeLists.txt       # Alternative build (CMake)
├── STM32F407IGHx_FLASH.ld  # Linker script
└── basic_framework.ioc  # STM32CubeMX project
```

## Key Configuration Files

**`application/robot_def.h`** - Robot parameters (modify for each robot):
- Board type: `ONE_BOARD` / `CHASSIS_BOARD` / `GIMBAL_BOARD`
- Vision interface: `VISION_USE_VCP` / `VISION_USE_UART`
- Gimbal alignment: `YAW_CHASSIS_ALIGN_ECD`, `PITCH_HORIZON_ECD`
- Mechanical: `WHEEL_BASE`, `TRACK_WIDTH`, `RADIUS_WHEEL`
- Launcher: `ONE_BULLET_DELTA_ANGLE`, `NUM_PER_CIRCLE`
- Gyro orientation: `GYRO2GIMBAL_DIR_{YAW,PITCH,ROLL}`

**`.vscode/tasks.json`** - Cross-platform build tasks (auto-detects Windows/Mac/Linux)

**`openocd_{dap,jlink}.cfg`** - OpenOCD debug configurations

**`stm32.jflash`** - J-Link flash project

## Testing Individual Modules

To test BSP/modules without full robot code:

1. Comment out `RoboInit()` in `Src/main.c`
2. Add `#include "bsp_init.h"` and call `BSPInit()`
3. Include target module header and initialize per its documentation
4. Test in `main()` while loop or use `bsp_tim.h` periodic tasks
5. Optionally disable FreeRTOS initialization for bare-metal testing

**Each BSP/module folder contains `.md` documentation with:**
- Interface descriptions
- Usage examples
- Test cases

## Debugging

**Tooling:**
- **VSCode + Cortex-Debug:** Basic debugging (launch.json configured)
- **Segger Ozone:** Advanced debugging, live watch, data plotting
- **Segger SystemView:** Real-time task profiling
- **FreeMaster:** Alternative data visualization
- **bsp_dwt:** Inline performance measurement

**Logging:** Uses Segger RTT (`bsp/log/`) - real-time logging without UART overhead. View with:
```bash
JLinkRTTClient
```
Or VSCode task: "log"

**Note:** LOG system can be disabled via `-DDISABLE_LOG_SYSTEM` in Makefile (currently enabled by default).

## Common Workflows

### Adding a New Motor
1. Identify motor type (DJI/HT/LK/etc) in `modules/motor/`
2. Create instance in relevant app (`gimbal/chassis/shoot`)
3. Configure CAN ID, PID parameters, control mode
4. Register with motor task in `robot_task.h`

### Supporting a New Robot Type
1. Fork repository
2. Modify `application/robot_def.h` with robot dimensions/config
3. Adjust app initialization configs (motor IDs, comms ports)
4. Rebuild and flash
5. Use `git cherry-pick` to pull framework updates without losing customizations

### Adding a New Sensor/Module
1. Create `modules/new_module/` with `.c`, `.h`, `.md`
2. Use BSP instances (CAN/UART/SPI/I2C) for communication
3. Provide clean init function: `NewModuleInstance* NewModuleRegister(config)`
4. Document interface in `.md` with example
5. If needed, integrate daemon for offline detection

### Multi-Board Communication
- Enable CAN-based inter-board comm via `modules/can_comm/`
- Avoid CAN ID conflicts between boards
- Monitor CAN bus load (especially at 1 kHz motor control)

## Useful References

Detailed docs in `docs/basic_framework/`:
- **`VSCode+Ozone使用方法.md`** - Development environment setup (CRITICAL)
- **`架构介绍与开发指南.md`** - Full architecture guide with file tree (CRITICAL)
- **`合理地进行PID参数整定.md`** - PID tuning methodology
- **`如何定位bug.md`** - Debugging techniques
- **`必须做&禁止做.md`** - Do's and don'ts
- **`让VSCode成为更称手的IDE.md`** - VSCode productivity tips

**Setup Guide:** `docs/SETUP_GUIDE.md` - Platform-specific installation (Mac/Windows/Linux)

**Framework Tutorial Videos:** [bilibili collection](https://space.bilibili.com/522795884/channel/collectiondetail)

## Dependencies

**Required:**
- ARM GNU Toolchain (≥v10.3)
- Make (or mingw32-make on Windows)
- OpenOCD (for debug/flash)
- DFU-Util (for USB flashing)

**Included (Middlewares/):**
- FreeRTOS
- CMSIS-DSP (precompiled `libCMSISDSP.a`)
- Segger RTT

**Hardware Abstraction:**
- STM32 HAL (`Drivers/STM32F4xx_HAL_Driver/`)
- CMSIS (`Drivers/CMSIS/`)

## Build System Details

**Compilation:**
- Parallel source discovery via `find` (Unix) or `dir /s` (Windows CMD)
- Recursive include directory auto-discovery
- Optimizations: `-Og` (debug), `-Ofast` (release), `-flto` (link-time optimization)
- Output: `build/` directory with `.elf`, `.hex`, `.bin`, map files

**Linker Script:** `STM32F407IGHx_FLASH.ld` (128KB RAM, 1MB Flash)

**Math Library:** Links against CMSIS-DSP for optimized ARM math functions

## Notes for Claude Code

- When modifying robot parameters, always check `application/robot_def.h` first
- Pay attention to board configuration macros - incorrect settings can cause runtime issues
- Motor CAN IDs are configured in app initialization, not centralized
- Vision communication uses either VCP (USB) or UART - check `VISION_USE_*` macro
- FreeRTOS heap/stack sizes are in CubeMX config - regenerate if tasks need more memory
- Use `message_center` for all inter-app data exchange (enforced design pattern)
- All angles use radians internally unless explicitly stated otherwise
- When reading/modifying control loops, note the task frequency requirements
