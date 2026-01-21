# NYUSH RoboMaster Control

NYUSH Robotics Club RoboMaster firmware based on Hunan University YueLu team's **basic_framework**.

- Quick start: see the [Setup Guide](docs/setup-guide.md) and [Flashing Guide](docs/flashing-guide.md)
- Toolchain: Makefile, ARM GNU Toolchain
- Target: STM32F407IGH6 (RoboMaster Type C Development Board)
- RTOS: FreeRTOS
- Original framework: [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)

> **For comprehensive framework documentation, features, and architecture details, see [docs/basic_framework/README.md](docs/basic_framework/README.md)**

## Documentation

- **[Setup Guide](docs/setup-guide.md)** - Development environment setup (Mac/Windows/Linux)
- **[Flashing Guide](docs/flashing-guide.md)** - Firmware flashing methods
- **[CAN Communication](docs/can.md)** - CAN bus protocol for GM6020 and C620 motors
- **[GitHub Commands](docs/github-commands.md)** - Git workflow reference

### Framework Documentation
- **[Basic Framework README](docs/basic_framework/README.md)** - Full framework introduction (Chinese)
- **[Architecture Guide](docs/basic_framework/架构介绍与开发指南.md)** - 3-layer architecture details (Chinese)
- **[VSCode + Ozone Setup](docs/basic_framework/VSCode+Ozone使用方法.md)** - Development workflow (Chinese)
- `docs/user-guides/` - Official hardware manuals (GM6020, C620, C610, C Board)

## Repository Structure

- `application/` - Robot control logic (cmd, gimbal, chassis, shoot)
- `modules/` - Hardware-agnostic drivers (motors, sensors, algorithms)
- `bsp/` - Hardware abstraction layer (CAN, UART, peripherals)
- `docs/` - Guides and references
- `Makefile` - Build system

## Build & Flash

### Build
```bash
make -j12              # macOS/Linux
mingw32-make -j12      # Windows (MSYS2/MinGW64)
```
Or use VSCode: `Ctrl+Shift+B` / `Cmd+Shift+B`

### Flash
```bash
make flash_dfu         # DFU-Util (USB, no debugger needed)
make flash_dap         # OpenOCD + DAP-Link
make flash_stlink      # OpenOCD + ST-Link
```
Or use **STM32CubeProgrammer** (GUI) - see [Flashing Guide](docs/flashing-guide.md)

## Architecture

3-layer design with pub-sub messaging:
- **BSP** - Hardware abstraction (CAN, UART, peripherals)
- **Modules** - Hardware-agnostic drivers (motors, sensors, algorithms)
- **Application** - Robot control logic (gimbal, chassis, shooter, command)

Configuration: `application/robot_def.h`

## Credits

- **Original framework:** Hunan University YueLu RoboMaster Team (2022-2023)
- **NYUSH adaptation:** NYUSH Robotics Club
- License: MIT
