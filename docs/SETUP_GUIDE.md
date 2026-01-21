# Setup Guide for Basic Framework

---

## Table of Contents

- [System Requirements](#system-requirements)
- [Quick Start by Platform](#quick-start-by-platform)
  - [macOS Setup](#macos-setup)
  - [Windows Setup](#windows-setup)
  - [Linux Setup](#linux-setup)
- [VSCode Configuration](#vscode-configuration)
- [Compilation](#compilation)
- [Flashing Methods](#flashing-methods)
- [Troubleshooting](#troubleshooting)

---

## System Requirements

### Common Requirements (All Platforms)
- **VSCode**(latest version recommended)
- **ARM GNU Toolchain**(v10.3 or later)
- **Make** build tool
- **OpenOCD**(v0.11 or later) for debugging/flashing
- **DFU-Util** for USB flashing (recommended)

### Hardware Requirements
- STM32F407IGH6 development board (RoboMaster Type C board)
- USB cable for programming
- Optional: Debug probe (DAP-Link, ST-Link, or J-Link)


## Quick Start by Platform


## macOS Setup

### Step 1: Install Homebrew (if not installed)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Step 2: Install Development Tools
```bash
# Install ARM toolchain
brew install --cask gcc-arm-embedded

# Install build tools and utilities
brew install openocd dfu-util

# Verify installations
arm-none-eabi-gcc --version
openocd --version
dfu-util --version
```

### Step 3: Install VSCode
Download from [code.visualstudio.com](https://code.visualstudio.com/)

### Step 4: Install Required VSCode Extensions
Open VSCode and install:
- **C/C++**(Microsoft)
- **Cortex-Debug**(marus25)
- **Cortex-Debug: Device Support Pack - STM32F4**
- **Makefile Tools**(Microsoft)

### Step 5: Clone and Open Project
```bash
cd ~/your-workspace
git clone https://github.com/NYUSH-Robotics-Club/nyush-rm-control.git
cd nyush-rm-control
code .
```

### Step 6: Verify Setup
```bash
# Test compilation
make -j12

# Check generated files
ls -lh build/
```

**macOS setup complete!** The current configuration is already optimized for macOS.

## Windows Setup

### Step 1: Install MSYS2

1. **Download MSYS2**
   - Visit: https://www.msys2.org/
   - Download the installer (msys2-x86_64-*.exe)
   - Install to default location (do not change it): `C:\msys64`

2. **Launch MSYS2 MSYS**
   - Find "MSYS2 MSYS" in Start Menu
   - A terminal window will open

### Step 2: Install Development Tools

In the MSYS2 terminal, run:

```bash
# Update package database
pacman -Syu

# If prompted, close terminal and reopen MSYS2 MSYS, then run again:
pacman -Syu

# Install toolchain and utilities
pacman -S mingw-w64-x86_64-toolchain \
          mingw-w64-x86_64-arm-none-eabi-toolchain \
          mingw-w64-x86_64-openocd \
          mingw-w64-x86_64-dfu-util \
          git
```

Press Enter when prompted to select all packages.

### Step 3: Add to Windows PATH

1. Open **System Environment Variables**:
   - Press `Win` key, type "environment"
   - Click "Edit the system environment variables"
   - Click "Environment Variables" button

2. Edit **Path**variable:
   - In "System variables" section, find "Path"
   - Click "Edit"
   - Click "New"
   - Add: `C:\msys64\mingw64\bin`
   - Click "OK" on all dialogs

3. **Restart Windows**(or at least restart VSCode)

### Step 4: Verify Installation

Open **Command Prompt**(not MSYS2) and verify:

```cmd
arm-none-eabi-gcc --version
mingw32-make --version
openocd --version
dfu-util --version
```

All commands should display version information.

### Step 5: Install VSCode

Download from [code.visualstudio.com](https://code.visualstudio.com/)

### Step 6: Install VSCode Extensions

Open VSCode and install:
- **C/C++**(Microsoft)
- **Cortex-Debug**(marus25)
- **Cortex-Debug: Device Support Pack - STM32F4**
- **Makefile Tools**(Microsoft)

### Step 7: Configure VSCode Settings

Open your project in VSCode, then update `.vscode/settings.json`:

```json
{
  "cortex-debug.armToolchainPath.windows": "C:\\msys64\\mingw64\\bin",
  "cortex-debug.openocdPath.windows": "C:\\msys64\\mingw64\\bin\\openocd.exe",
  "makefile.makePath.windows": "mingw32-make"
}
```

### Step 8: Test Compilation

In VSCode terminal or Command Prompt:

```cmd
cd path\to\basic_framework
mingw32-make -j12
```

Or use VSCode shortcut: `Ctrl+Shift+B`

**Windows setup complete!**


## Linux Setup

### Ubuntu/Debian-based Systems

### Step 1: Install Development Tools

```bash
# Add ARM toolchain repository
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt update

# Install toolchain and utilities
sudo apt install gcc-arm-none-eabi \
                 openocd \
                 dfu-util \
                 build-essential \
                 git

# Verify installations
arm-none-eabi-gcc --version
openocd --version
dfu-util --version
```

### Fedora/RHEL-based Systems

```bash
# Install toolchain and utilities
sudo dnf install arm-none-eabi-gcc-cs \
                 openocd \
                 dfu-util \
                 make \
                 git

# Verify installations
arm-none-eabi-gcc --version
openocd --version
dfu-util --version
```

### Step 2: Install VSCode

**Ubuntu/Debian:**
```bash
# Download and install VSCode
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code
```

### Step 3: Install VSCode Extensions

Same as macOS/Windows:
- **C/C++**
- **Cortex-Debug**
- **Cortex-Debug: Device Support Pack - STM32F4**
- **Makefile Tools**

### Step 4: Set USB Permissions (for DFU)

```bash
# Create udev rules file
sudo nano /etc/udev/rules.d/50-stm32.rules
```

Add these lines:
```
# STM32 DFU Mode
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"

# ST-Link
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"
```

Save and reload:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 5: Test Compilation

```bash
cd ~/your-workspace/basic_framework
make -j12
```

Or use VSCode: `Ctrl+Shift+B`

**Linux setup complete!**


## VSCode Configuration

### Project Structure

The `.vscode` folder contains platform-aware configurations:

```
.vscode/
├── settings.json    # Toolchain paths (platform-specific)
├── tasks.json       # Build and flash tasks (auto-detects OS)
└── launch.json      # Debug configurations
```

### Platform-Specific Paths

The `settings.json` automatically uses the correct paths based on your OS:

```json
{
  // macOS paths
  "cortex-debug.armToolchainPath.osx": "/opt/homebrew/bin",
  "cortex-debug.openocdPath.osx": "/opt/homebrew/bin/openocd",

  // Windows paths (update if needed)
  "cortex-debug.armToolchainPath.windows": "C:\\msys64\\mingw64\\bin",
  "cortex-debug.openocdPath.windows": "C:\\msys64\\mingw64\\bin\\openocd.exe",

  // Linux paths
  "cortex-debug.armToolchainPath.linux": "/usr/bin",
  "cortex-debug.openocdPath.linux": "/usr/bin/openocd"
}
```

**No manual switching needed**- VSCode detects your OS automatically!


## Compilation

### Command Line

**macOS / Linux:**
```bash
make -j12        # Compile with 12 parallel jobs
make clean       # Clean build files
make rebuild     # Clean and rebuild
make help        # Show all available commands
```

**Windows:**
```cmd
mingw32-make -j12     # Compile
mingw32-make clean    # Clean
mingw32-make rebuild  # Rebuild
mingw32-make help     # Show help
```

### VSCode Integration

**Keyboard Shortcuts:**
- **Compile**: `Cmd+Shift+B` (Mac) / `Ctrl+Shift+B` (Windows/Linux)
- **Run Task**: `Cmd+Shift+P` (Mac) / `Ctrl+Shift+P` (Windows/Linux) → Type "Tasks: Run Task"

**Available Tasks:**
- `build` - Compile project
- `clean` - Remove build files
- `rebuild` - Clean and rebuild
- `flash: dfu-util` - Flash via DFU (recommended)
- `flash: dap-link` - Flash via DAP-Link
- `flash: stlink` - Flash via ST-Link
- `flash: jlink` - Flash via J-Link

### Build Output

Successful compilation generates:
```
build/
├── basic_framework.elf    # Debug format
├── basic_framework.hex    # Intel HEX format
├── basic_framework.bin    # Binary format
└── *.o, *.d, *.lst        # Object files and listings
```

---

## Flashing Methods

The project supports 4 flashing methods. **DFU-Util is recommended**for cross-platform compatibility.

---

### Method 1: DFU-Util (Recommended)

**Advantages:**
-  Works on all platforms (Windows/Mac/Linux)
-  No external debugger required (USB cable only)
-  Fast and reliable
-  Built-in bootloader support

#### Entering DFU Mode

1. **Press and hold**the **BOOT0**button on the board
2. **Press and release**the **RESET**button
3. **Release**the **BOOT0**button

The STM32 is now in DFU mode.

#### Verify DFU Device

**All Platforms:**
```bash
dfu-util -l
```

You should see output like:
```
Found DFU: [0483:df11] ver=2200, devnum=X, cfg=1, intf=0, ...
```

#### Flash Firmware

**macOS / Linux:**
```bash
make flash_dfu
```

**Windows:**
```cmd
mingw32-make flash_dfu
```

**VSCode (All Platforms):**
1. Press `Cmd/Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Select **"flash: dfu-util"**

#### Expected Output

```
dfu-util 0.11
Opening DFU capable USB device...
Device ID 0483:df11
...
Downloading to address = 0x08000000
Download        [=========================] 100%
File downloaded successfully
```

#### Exit DFU Mode

Press **RESET**button on the board to start your application.

#### Windows-Specific: DFU Driver Installation

If `dfu-util -l` doesn't detect the device:

1. Download **Zadig**: https://zadig.akeo.ie/
2. Connect board in DFU mode
3. In Zadig:
   - Select "STM32 BOOTLOADER" device
   - Select "WinUSB" driver
   - Click "Replace Driver"

### Method 2: OpenOCD + DAP-Link

**Hardware Required:**DAP-Link debugger (e.g., wireless debugger)

**Configuration File:**`openocd_dap.cfg` (provided in project root)

#### Flash Command

**macOS / Linux:**
```bash
make flash_dap
```

**Windows:**
```cmd
mingw32-make flash_dap
```

**VSCode:**
- Task: **"flash: dap-link"**

#### Expected Output
```
Open On-Chip Debugger 0.12.0
...
Warn : JTAG tap: stm32f4x.bs tap/device found: 0x06413041
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
...
**Programming Finished **
```


### Method 3: OpenOCD + ST-Link

**Hardware Required:**ST-Link V2/V3 debugger

#### Flash Command

**macOS / Linux:**
```bash
make flash_stlink
```

**Windows:**
```cmd
mingw32-make flash_stlink
```

**VSCode:**
- Task: **"flash: stlink"**

#### Notes
- Uses OpenOCD's built-in ST-Link configuration
- No additional config file needed
- Supports ST-Link V2, V2-1, and V3

---

### Method 4: J-Link

**Hardware Required:**Segger J-Link debugger

**Software Required:**J-Link Software Package
- Download: https://www.segger.com/downloads/jlink/

**Configuration File:**`stm32.jflash` (provided)

#### Flash Command

**Windows:**
```cmd
mingw32-make flash_jlink
```

**macOS (if J-Link installed):**
```bash
make flash_jlink
```

**VSCode:**
- Task: **"flash: jlink"**

#### Notes
- Primarily tested on Windows
- Requires JFlash executable in PATH
- Fast programming speed


## Troubleshooting

### Common Issues

#### Issue 1: "arm-none-eabi-gcc: command not found"

**Cause:** Toolchain not in PATH

**Solution:**

**macOS:**
```bash
# Check installation
which arm-none-eabi-gcc

# If not found, reinstall
brew reinstall gcc-arm-embedded
```

**Windows:**
```cmd
# Check PATH includes C:\msys64\mingw64\bin
echo %PATH%

# Verify in MSYS2
where arm-none-eabi-gcc
```

**Linux:**
```bash
# Reinstall toolchain
sudo apt install --reinstall gcc-arm-none-eabi
```


#### Issue 2: "make: command not found" (Windows)

**Cause:** Using wrong command

**Solution:** On Windows, use `mingw32-make` instead of `make`:
```cmd
mingw32-make -j12
```

Or use VSCode tasks which handle this automatically.


#### Issue 3: DFU Device Not Detected

**Symptoms:**
```bash
dfu-util -l
# No DFU capable USB device available
```

**Solutions:**

1. **Verify DFU Mode Entry:**
   - Hold BOOT0, press RESET, release BOOT0
   - Check LED indicators (board-specific)

2. **Check USB Connection:**
   - Try different USB cable
   - Try different USB port
   - Avoid USB hubs

3. **Windows: Install DFU Driver:**
   - Use Zadig tool (see DFU section above)
   - Select "WinUSB" driver

4. **Linux: Check Permissions:**
   ```bash
   # Check if device visible
   lsusb | grep 0483

   # Add udev rules (see Linux setup)
   sudo nano /etc/udev/rules.d/50-stm32.rules
   ```

5. **macOS: Check System Extensions:**
   - Go to System Preferences → Security & Privacy
   - Allow any blocked USB drivers


#### Issue 4: OpenOCD Connection Failed

**Symptoms:**
```
Error: unable to open ftdi device with vid 0403, pid 6014, description '*', serial '*'
```

**Solutions:**

1. **Check Debugger Connection:**
   - Ensure debugger is connected to board
   - Check SWD/JTAG pins (SWDIO, SWCLK, GND)

2. **Linux: USB Permissions:**
   ```bash
   sudo usermod -a -G plugdev $USER
   # Log out and back in
   ```

3. **Windows: Driver Issues:**
   - Install appropriate driver for your debugger
   - Use Zadig to install WinUSB driver if needed

4. **Verify OpenOCD Config:**
   ```bash
   # Test OpenOCD connection
   openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
   ```

#### Issue 5: Compilation Warnings About _write, _read, etc.

**Example:**
```
warning: _write is not implemented and will always fail
```

**This is normal!** These warnings are expected for bare-metal embedded systems and can be safely ignored. They occur because standard I/O functions are not implemented in the embedded environment.

#### Issue 6: VSCode Cannot Find Includes

**Symptoms:**
- Red squiggles under `#include` statements
- IntelliSense not working

**Solution:**

1. **Configure C/C++ Extension:**
   Press `Cmd/Ctrl+Shift+P` → "C/C++: Edit Configurations (JSON)"

2. **Verify Compiler Path:**
   ```json
   {
     "compilerPath": "/opt/homebrew/bin/arm-none-eabi-gcc",  // macOS
     "compilerPath": "C:\\msys64\\mingw64\\bin\\arm-none-eabi-gcc.exe",  // Windows
     "intelliSenseMode": "gcc-arm"
   }
   ```

3. **Reload VSCode:**
   `Cmd/Ctrl+Shift+P` → "Developer: Reload Window"


#### Issue 7: "Permission Denied" on macOS

**Cause:**Security restrictions on downloaded binaries

**Solution:**
```bash
# Remove quarantine attribute
sudo xattr -rd com.apple.quarantine /opt/homebrew/bin/openocd
sudo xattr -rd com.apple.quarantine /opt/homebrew/bin/dfu-util

# Or for entire toolchain
sudo xattr -rd com.apple.quarantine /opt/homebrew/Caskroom/gcc-arm-embedded
```

#### Issue 8: Line Ending Issues (Windows ↔ Mac)

**Symptoms:**
- Build errors when switching between platforms
- Git shows unnecessary changes

**Solution:**

The project includes `.gitattributes` which enforces LF line endings:

```bash
# On Windows, configure Git
git config --global core.autocrlf true

# Refresh repository
git rm --cached -r .
git reset --hard
```

## Conclusion

You now have a complete cross-platform development environment for STM32 firmware development!

**Next Steps:**
1. Configure robot parameters in `application/robot_def.h`
2. Review framework documentation in `docs/basic_framework/` folder
3. Start developing your application!

**Recommended Workflow:**
1. Edit code in VSCode
2. Press `Cmd/Ctrl+Shift+B` to compile
3. Enter DFU mode on board
4. Run `flash: dfu-util` task
5. Press RESET to run firmware
6. Debug with `F5` if needed