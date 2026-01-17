# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32F103xE-based robot chassis firmware project featuring four-wheel drive, IMU attitude measurement (MPU6050 with DMP), servo control, and serial communication. The codebase uses a modular architecture with clear separation between hardware abstraction (HAL) and application logic.

**Hardware**: STM32F103xE (72MHz, 512KB Flash, 64KB RAM)
**Build System**: CMake + Ninja
**Toolchain**: arm-none-eabi-gcc

---

## Build and Development Commands

### Building

```bash
# Configure and build (Debug)
cmake --preset Debug
cmake --build build/Debug

# Configure and build (Release)
cmake --preset Release
cmake --build build/Release

# Clean build
rm -rf build/Debug
cmake --preset Debug
cmake --build build/Debug
```

### Flashing and Debugging

```bash
# Flash using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/Debug/RobotChassis_Firmware.elf verify reset exit"

# Debug with GDB (in separate terminal)
arm-none-eabi-gdb build/Debug/RobotChassis_Firmware.elf
(gdb) target remote :3333
```

### Project Configuration

- **CMakePresets.json**: Defines Debug/Release build configurations
- **CMakeLists.txt**: Main build file - add new source files here
- **STM32F103XX_FLASH.ld**: Linker script for memory layout

---

## Architecture Overview

### Four-Layer Architecture

```
Application Layer (USER/*/):     Business logic, task scheduling
      ↓
Driver Wrapper Layer (USER/*/): Device drivers, interface abstraction
      ↓
HAL Layer (Core/):               STM32 HAL library (CubeMX generated)
      ↓
Hardware Layer:                  STM32F103 peripherals
```

### Key Design Patterns

**Modular Organization**: Each functional module has its own directory under `USER/` with dedicated `.c/.h` files.

**Real-time Task Scheduling**: Fixed-period tasks coordinated by TIM6 interrupt (10ms base):
- **10ms**: LED updates, IMU data transmission, communication processing
- **40ms**: Motor PID control, encoder readings
- **200ms**: Battery voltage monitoring

**Interrupt-Driven Communication**: UART2 uses interrupt-driven reception with `Comm_RxCallback()` for protocol parsing.

---

## Critical Architecture Details

### Module Communication

**Data Flow**:
```
main.c (10ms loop)
├── Comm_ProcessControlData()  ← UART2 RX interrupt
├── IMU_Update()               ← MPU6050 I2C (GPIO bit-bang)
├── Motor_PIDControl()         ← Encoder TIM2/3/4/5
├── Comm_Send*()               → UART2 TX
└── LED_Update()               → GPIO toggling
```

**Serial Protocol Format**:
```
[0xFC][FuncCode][Data10Bytes][XORChecksum][0xDF]
```
- Function codes: Battery(0x01), Encoder(0x02), Gyro(0x03), Accel(0x04), Euler(0x05), MotorSpeed(0x06), PID(0x07), Servo(0x08)
- XOR checksum validation

### Motor Control Architecture

**Motor IDs**: A(0), B(1), C(2), D(3) - corresponds to physical position

**Control Flow**:
1. `Motor_SetTargetSpeed(id, speed)` - Set target (units: encoder ticks/40ms)
2. `Motor_UpdateEncoderDelta()` - Read encoder counters every 40ms
3. `Motor_PIDControl(id, target, actual)` - PID computation, returns PWM value
4. `Motor_SetSpeed(id, pwm)` - Output PWM to TIM8 (0-1439 range)

**Hardware Mapping**:
- PWM: TIM8 CH1/2/3/4 → PC6/7/8/9 (25kHz frequency)
- Direction: GPIO pins IN1/IN2 per motor
- Encoders: TIM2/3/4/5 in quadrature encoder mode (4x counting)

**PID Parameters**:
- Configurable via serial protocol (FuncCode 0x07)
- Scale factor: 0.01 (all values ×100)
- Output clamped to PWM range [0, 1439]

### IMU (MPU6050 + DMP) Architecture

**Three-Layer Design**:
```
Application: imu_data.c         - Unified interface (Euler/Accel/Gyro)
    ↓
Algorithm: eMPL/inv_mpu.c       - DMP firmware, sensor fusion
    ↓
Driver: MPU6050/mpu6050.c       - Register operations, I2C
```

**Critical Implementation Details**:
- I2C is bit-banged via GPIO (PB12-SCL, PB13-SDA), NOT hardware I2C
- DMP firmware loaded at runtime via `dmp_load_motion_driver_firmware()`
- Output data format: quaternion (q30) → converted to Euler angles (degrees)
- All sensor data ×100 for precision preservation

**IMU Initialization Sequence** (must be followed exactly):
1. `MPU_Init()` - Basic hardware init (gyro ±2000dps, accel ±2g)
2. `mpu_dmp_init()` - 11-step DMP initialization
   - Load DMP firmware
   - Set orientation matrix
   - Enable features (6X_LP_QUAT, GYRO_CAL)
   - Run self-test
   - Enable DMP
3. `IMU_Update()` - Blocking call to read DMP data

### Timer Resource Allocation

**7/8 Timers Used**:
- **TIM1**: Servo PWM (50Hz, 20ms period, CH2N/CH3N → PB14/15)
- **TIM2-5**: Quadrature encoders (4× counting, 16-bit range)
- **TIM6**: System timebase (100Hz interrupt → 10ms period)
- **TIM8**: Motor PWM (25kHz, 1440 steps, CH1-4 → PC6-9)
- **TIM7**: Unused (available for future expansion)

### Interrupt Priority Configuration

All peripheral interrupts set to priority 0 (highest preemption):
- USART1/2: Serial communication
- TIM6: System timebase
- EXTI: External interrupts (if used)

**Note**: High interrupt priority may affect other real-time requirements.

---

## Important Conventions

### Code Organization Rules

**Adding New Modules**:
1. Create directory under `USER/ModuleName/`
2. Add `module_name.c` and `module_name.h`
3. Update `CMakeLists.txt`:
   - Add source to `target_sources()`
   - Add include dir to `target_include_directories()`
4. Use prefix convention: `Module_FunctionName()`
5. Document in `USER/README.md`

**File Modifications**:
- **Core/** files: Generated by STM32CubeMX - only modify within `/* USER CODE BEGIN */` and `/* USER CODE END */` markers
- **USER/** files: Fully user-controlled

### Naming Conventions

- **Functions**: `Module_Action()` (e.g., `Motor_SetSpeed()`, `IMU_Update()`)
- **Header Guards**: `__MODULE_NAME_H__` format
- **Macros**: `UPPER_CASE_WITH_UNDERSCORES`
- **GPIO**: `PERIPHERAL_PIN_Pin` and `PERIPHERAL_PIN_GPIO_Port` (auto-generated)

### Data Precision Handling

**All IMU and control data is multiplied by 100**:
- Euler angles: 0.01° precision
- Gyroscope: 0.01 rad/s precision
- Acceleration: 0.01G precision
- Motor speed: encoder ticks/40ms
- Divide by 100 when using actual values

### PWM Control Ranges

**Motor PWM (TIM8)**: 0-1439 (12-bit effective resolution, 25kHz)
**Servo PWM (TIM1)**: 250-1250 (corresponds to 0.5ms-2.5ms pulse, 50Hz)

---

## Common Development Tasks

### Adding a New UART Command

1. Define function code in `USER/Communication/comm_protocol.h`
2. Add handler in `Comm_ProcessControlData()` switch statement
3. Implement response packing function following existing pattern
4. Document in protocol section

### Modifying PID Parameters

**Via Serial Protocol**:
```
Send: [FC][0x07][motor_id][Kp][Ki][Kd][checksum][DF]
Values are ×100 (e.g., Kp=1.50 → send 150)
```

**In Code**: Edit `user_config.h` PID initial values.

### Debugging with printf

Printf is redirected to USART1 (115200 baud). Usage:
```c
#include "System/retarget.h"
// After RetargetInit(&huart1) in main()
printf("Debug: value = %d\n", value);
```

**Note**: Disable in production by removing `#define Debug` in main.c.

### Updating MPU6050 I2C Pins

The MPU6050 uses GPIO bit-bang I2C, NOT hardware I2C. To change pins:
1. Edit `USER/IMU/MPU6050/mpuiic.c` - modify `MPU_IIC_Init()` GPIO configuration
2. No need to change I2C peripheral settings

---

## Hardware-Specific Notes

### Pin Multiplexing Conflicts

**Encoders vs JTAG**:
- Encoder TIM2 uses PA15 (JTDI) and PB3 (JTDO)
- Encoder TIM4 uses PB6/7
- **Use SWD debug interface instead of JTAG** (PA13/PA14)

**All TIM8 PWM Channels Used** (PC6-9). For additional PWM:
- Use TIM1 (has CH2N/CH3N on PB14/15)
- Or reconfigure timer usage

### Power Considerations

**Operating Voltage**: 3.3V (VDD range: 2.0-3.6V)
**Battery Monitoring**: ADC2 Channel 8 (PB0) - requires voltage divider circuit
**Motor Driver**: External H-bridge controlled by direction GPIOs + TIM8 PWM

### Clock Configuration

```
SYSCLK: 72MHz (max for STM32F103)
AHB (HCLK): 72MHz
APB1: 36MHz (TIM2-7, USART2-3, I2C1-2)
APB2: 72MHz (TIM1,8, ADC1-2, USART1, SPI1)
ADC: 12MHz (6x prescale from APB2)
```

Timer clocks auto ×2 when APB prescaler > 1 (TIM2-7 get 72MHz despite APB1=36MHz).

---

## Project-Specific Files

### Configuration Files

- **`USER/user_config.h`**: Global parameters, motor definitions, PID defaults
- **`Core/Inc/main.h`**: GPIO pin definitions (auto-generated)
- **`Documentation/`**: Detailed hardware and module documentation

### Debug Utilities

- **`pid_command_generator.py`**: Python script to generate PID configuration commands
- **`System/retarget.c`**: Printf redirect implementation
- **`Communication/comm_protocol.c`**: XOR checksum and frame packing

---

## Testing and Validation

### Module Testing Strategy

**Unit Tests**: Test individual modules (motor control, IMU, etc.)
**Integration Tests**: Verify inter-module communication (encoder → motor PID)
**Hardware-in-Loop**: Test on actual robot chassis

### Common Failure Points

1. **DMP initialization fails**: Check I2C GPIO connections, ensure AD0 pin grounded (0x68 address)
2. **Motor doesn't respond**: Verify TIM8 PWM started, direction GPIOs configured correctly
3. **Encoder noise**: Add filtering in hardware (RC filter) or software (moving average)
4. **UART communication errors**: Verify baud rate mismatch, check XOR checksum calculation

---

## Documentation References

Detailed documentation available in `Documentation/`:
- **`Core外设配置详解.md`**: Complete peripheral configuration guide
- **`Core外设配置速查表.md`**: Quick reference for timers/GPIO/PWM
- **`MPU6050_IMU架构详解.md`**: In-depth IMU and DMP explanation
- **`USER/README.md`**: Module organization and dependencies
- **`USER/System/README.md`**: System infrastructure

---

## Important Constraints

**Timer Resources**: Only TIM7 available for new features (all others used)
**GPIO Availability**: ~50% utilized (32/64 pins), check conflicts before adding peripherals
**Flash Memory**: ~50KB used, 450KB available for expansion
**Stack Size**: 1KB configured - monitor stack usage if adding deep call stacks
**Heap Size**: 512B configured - minimize dynamic allocation

---

## Maintenance Notes

When modifying CubeMX-generated files in `Core/`:
- Always use STM32CubeMX to regenerate, then manually port USER CODE sections
- Never modify code outside `/* USER CODE BEGIN */ ... /* USER CODE END */` markers
- Test thoroughly after regeneration

When adding new modules to `USER/`:
- Follow existing module structure (init, update, get/set functions)
- Add Doxygen-style comments
- Update this CLAUDE.md if introducing architectural changes
