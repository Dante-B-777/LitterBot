# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
# Claude.md for LitterBot STM32 Project

## Project Overview

**LitterBot** is a senior robotics engineering project implementing an autonomous mobile robot for litter collection. This is a **real hardware system** where accuracy, electrical correctness, timing, and safety are critical.

The system uses a two-layer compute architecture with strict separation of responsibilities between low-level real-time control (STM32) and high-level planning (Jetson + ROS 2).

---

## System Architecture

### Low-Level Control Layer (STM32L4A6ZG)

**Hardware:**
- MCU: STM32L4A6ZG on NUCLEO-144 development board
- Programming: ST-LINK via USB
- IDE: STM32CubeIDE with CubeMX
- Package: LQFP144

**Core Responsibilities:**
- DC motor control (4-wheel drive via Cytron MD10C drivers)
- Stepper motor control (NEMA 23 closed-loop via CL57T drivers)
- Bus servo control (HX-65HM via UART protocol)
- Quadrature encoder capture and counting
- Odometry data generation from wheel encoders
- Real-time I/O and deterministic timing
- Safety handling (E-stop, fault monitoring)

**Software Constraints:**
- Bare-metal C or STM32 HAL only (no Arduino abstractions)
- Modular file structure with clear separation of concerns
- No blocking delays in control paths
- Interrupt-driven and timer-based real-time behavior
- Must be safe to run with only STM32 powered (no dependency on Jetson)

### High-Level Control Layer (Jetson + ROS 2)

**Environment:**
- Ubuntu Linux with ROS 2
- Python/C++ nodes

**Responsibilities:**
- Navigation and path planning
- Sensor fusion (LiDAR, cameras)
- Mapping and localization (SLAM)
- Object detection and perception
- High-level motion commands (cmd_vel messages)

**Critical:** The Jetson does NOT directly control hardware pins. It issues commands to the STM32 and consumes telemetry.

---

## Hardware Components

### Motors and Drivers

**DC Drive Motors:**
- Type: Brushed DC gear motors with integrated quadrature encoders
- Application: 4-wheel drive chassis (FL, FR, RL, RR)
- Driver: Cytron MD10C (sign-magnitude PWM control)
- Control Interface: PWM + DIR signals from STM32
- Logic Compatibility: 3.3V/5V tolerant
- Current: ~13A continuous per driver
- Encoder Feedback: A/B channels for odometry

**Stepper Motors:**
- Model: NEMA 23 closed-loop stepper (e.g., 23HS45-4204D-E1000)
- Application: Precision actuation (scoop, collection mechanisms)
- Driver: CL57T closed-loop stepper driver
- Control Interface: Step/Direction/Enable signals
- Motor Supply: 24-48 VDC
- Logic Input: 5V or 24V opto-isolated
- Encoder: Integrated 1000 CPR, connected directly to driver
- STM32 Role: Generates step/dir signals only (position loop closed in driver)

**Bus Servo Motors:**
- Model: Hiwonder HX-65HM Magnetic Encoder Bus Servo
- Application: Precision joint control (collection mechanism actuation)
- Control: UART serial command (half-duplex asynchronous)
- Baud Rate: 115200 (configurable, default for embedded systems)
- Communication Protocol: Packet-based with ID addressing (0-253)
- Operating Voltage: 9-12.6V
- Torque: 50KG·cm @ 9.6V, 65KG·cm @ 12.6V
- Rotation Range: 0°-360° (0-4095 position steps)
- Precision: 0.3° (12-bit magnetic encoder)
- Connector: 5264-3P (GND, VIN, SIG)
- Daisy-Chain: Up to 253 servos on single bus
- Operating Modes: Position, Velocity (Closed-Loop), PWM (Open-Loop)
- Feedback: Position, speed, load, temperature, voltage
- Protection: Stall detection, overheat, overcurrent
- Current: 80mA no-load, 5A stall
- STM32 Interface: UART TX/RX (half-duplex requires direction control)

**Critical Safety Notes:**
- All motor drivers require common ground with STM32
- E-stop must disable motor outputs at STM32 level
- Never assume timing or polarity without checking datasheets
- PWM frequency and pulse widths must respect driver specifications
- Bus servos require shared ground between servo power supply and STM32
- Do not manually rotate powered servos (risk of internal damage)

---

## Communication Architecture

### STM32 ↔ Jetson Protocol

**Physical Layer:**
- Peripheral: LPUART1, 115200 baud, **8N1**
- Pins: PG7 (TX) / PG8 (RX) — these are the NUCLEO ST-LINK VCP pins, so the same
  USB port is used for development (PC serial monitor) and deployment (Jetson)
- **NOTE: CubeMX fix required before use:** Change LPUART1 word length from 9 Bits → **8 Bits**

**Commands (Jetson → STM32):**

| Command | Format | Gate | Description |
|---------|--------|------|-------------|
| Motor | `M <fl> <fr> <rl> <rr>\n` | GATED | Set wheel speeds. Values -1000 to +1000. Firmware divides by 10 → % PWM. |
| E-stop | `X\n` | ALWAYS | Emergency stop — halts all motors immediately. |
| Scoop | `S SCOOP\n` | GATED | Run the full scoop sequence (6 phases with coordinated servo). |
| Calibrate | `C CALIBRATE\n` | GATED | Arm calibration — extend/retract slowly, print step counts. B1 = latched e-stop. |
| Deploy | `D DEPLOY\n` | STANDBY/ACTIVE | Arm homing → transition to ACTIVE state. |
| Dev mode | `DEV ON\n` / `DEV OFF\n` | ALWAYS | Toggle dev mode — bypasses all command gating. |
| State query | `Q\n` | ALWAYS | Responds `STATE <STANDBY\|DEPLOYING\|ACTIVE> [DEV]\r\n`. |
| Encoder test | `T\n` | ALWAYS | Print raw encoder tick counts over LPUART1 (debug use only). |

GATED commands are blocked in STANDBY unless dev mode is on. STM32 responds `ERR:STANDBY\r\n` when rejected.

Example motor command: `M 500 500 500 500\n` → all wheels forward at 50% PWM.

**Telemetry (STM32 → Jetson):**
- Not yet implemented. Planned: encoder tick counts at ~50 Hz.

**Watchdog:**
- STM32 expects a motor command every 250 ms. If no command arrives, it zeros all motors.
- The ROS2 `stm32_bridge` sends `M 0 0 0 0\n` (pause) to satisfy the watchdog without moving.

---

## Project Structure
```
firmware/stm32/LitterBot/
├── Core/
│   ├── Inc/                    # Header files
│   │   ├── main.h
│   │   ├── stm32l4xx_hal_conf.h
│   │   └── stm32l4xx_it.h
│   ├── Src/                    # Source files
│   │   ├── main.c
│   │   ├── stm32l4xx_hal_msp.c
│   │   └── stm32l4xx_it.c
│   └── Startup/                # Startup code
│
├── Drivers/                    # STM32 HAL and CMSIS
│   ├── STM32L4xx_HAL_Driver/
│   └── CMSIS/
│
├── LitterBot.ioc              # CubeMX configuration
├── STM32L4A6ZGTX_FLASH.ld     # Linker script
└── .project / .cproject        # Eclipse project files
```

### Current Modular Structure

All custom files live in `Core/Inc/` (headers) and `Core/Src/` (implementations):

```
Core/
├── Inc/
│   ├── cytron_md10c.h         # DC motor driver API — 4 wheels, TIM2 PWM, sign-magnitude
│   ├── cl57t_stepper.h        # Stepper driver API — step/dir GPIO bit-bang via DWT timer
│   ├── hx65_servo.h           # Bus servo API — USART3 half-duplex, packet protocol
│   ├── encoder.h              # Quadrature encoder — TIM1/3/4/8 in encoder mode
│   ├── odometry.h             # Mecanum dead-reckoning from encoder ticks
│   ├── uart_cmd.h             # UART command parser — M/X/S SCOOP/C CALIBRATE/D DEPLOY/DEV/Q/T
│   ├── scoop_sequence.h       # Scoop motion constants + Scoop_Execute() declaration
│   │                          # NOTE: ALL NUMERIC CONSTANTS ARE PLACEHOLDERS — measure on hardware
│   ├── limit_switch.h         # Limit switch driver — PD15 (home), PE10 (extend), NC to GND
│   ├── arm_control.h          # Arm control — calibration, extend/retract with servo interp, Arm_Home()
│   ├── deploy_state.h         # Deploy/standby state machine — STANDBY/DEPLOYING/ACTIVE, dev mode
│   ├── servo_test.h           # Servo sweep test (B1 button)
│   ├── dc_motor_test.h        # DC motor sequential test (B1 button)
│   ├── stepper_test.h         # Stepper 3-phase speed test (B1 button)
│   ├── combined_test.h        # All-peripheral test (B1 button)
│   └── encoder_test.h         # Encoder count test (B1 button, output on LD3)
│
└── Src/
    ├── cytron_md10c.c         # DC motor implementation
    ├── cl57t_stepper.c        # Stepper implementation
    ├── hx65_servo.c           # Bus servo implementation
    ├── encoder.c              # Encoder implementation
    ├── odometry.c             # Odometry implementation
    ├── uart_cmd.c             # Command parser — M/X/S SCOOP/C CALIBRATE/D DEPLOY/DEV ON|OFF/Q/T
    ├── scoop_sequence.c       # 6-phase scoop sequence with coordinated servo (placeholder values)
    ├── limit_switch.c         # Limit switch implementation
    ├── arm_control.c          # Arm calibration + extend/retract with safety and servo interpolation
    ├── deploy_state.c         # Deploy state machine implementation
    ├── servo_test.c
    ├── dc_motor_test.c
    ├── stepper_test.c
    ├── combined_test.c
    └── encoder_test.c
```

### Not Yet Built (firmware)

| Module | Purpose |
|--------|---------|
| `safety.c/h` | Dedicated E-stop / fault monitoring module |
| `motion_control.c/h` | Closed-loop velocity controllers |
| `system_state.c/h` | Robot state machine |
| `telemetry.c/h` | Push encoder counts to Jetson at fixed rate |

---

## Development Guidelines

### 1. Safety First

- **E-stop is mandatory**: Must immediately disable all motor outputs
- **Fault detection**: Monitor stepper driver alarm signals, motor current limits
- **Fail-safe defaults**: System should enter safe state on communication loss
- **Independent operation**: STM32 code must not crash or hang if Jetson is disconnected

### 2. Real-Time Constraints

- **No blocking delays** in control loops (use timers and interrupts)
- **Deterministic timing** for motor control (PWM updates, step generation)
- **Predictable interrupt latency** (prioritize critical ISRs)
- **Timer-based scheduling** for periodic tasks (odometry updates, telemetry)

### 3. Hardware Interfacing

- **Always check datasheets** before connecting or configuring hardware
- **Verify voltage levels**: 3.3V STM32 logic vs 5V/24V driver inputs
- **Confirm signal polarity**: Active-high vs active-low, step pulse requirements
- **Validate timing**: Minimum pulse widths, setup/hold times for signals
- **Use pull-ups/pull-downs** appropriately for encoder inputs and safety signals

### 4. Code Quality

- **Modular design**: Clear separation of hardware drivers, application logic, communication
- **Documented interfaces**: Function headers with parameter descriptions
- **Clear ownership**: Each peripheral and pin should have one responsible module
- **Explicit initialization**: Clear startup sequence, peripheral configuration order
- **Error handling**: Check HAL return values, handle peripheral failures gracefully

### 5. Testing Strategy

- **Unit testing**: Test each driver module independently
- **Integration testing**: Verify communication protocol with mock Jetson
- **Hardware-in-loop**: Test motor control with actual drivers (no motors initially)
- **Safety testing**: Verify E-stop behavior under all conditions
- **Incremental bring-up**: Enable subsystems one at a time
- **Incremental bring-up**: Enable subsystems one at a time, test before adding more

---

## Key Subsystems

### Motor Control (DC Drive Motors)

**Peripherals Required:**
- TIM (Timer): 4 channels for PWM generation (one per motor)
- GPIO: 4 direction pins
- Frequency: Typically 20 kHz PWM

**Control Mode:**
- Sign-magnitude: PWM controls speed, DIR controls direction
- Dead-band handling in firmware (avoid simultaneous FWD/REV)

**Implementation Notes:**
- Use hardware PWM via TIM peripheral
- Update duty cycle via `__HAL_TIM_SET_COMPARE()`
- Direction changes should include brief delay or zero-PWM transition

### Encoder Capture (Odometry)

**Peripherals Required:**
- TIM (Encoder Mode): 4 timers configured as quadrature decoders
- One timer per wheel encoder

**Configuration:**
- Encoder mode TI1 and TI2
- Count on both edges for 4x resolution
- 16-bit or 32-bit counter (check for overflow)

**Odometry Calculation:**
- Sample encoder counts at fixed rate (e.g., 50 Hz)
- Compute wheel velocities from count deltas
- Apply wheel kinematics to estimate robot velocity and pose

### Stepper Control (Closed-Loop Steppers)

**Peripherals Required:**
- TIM (PWM): Generate step pulse trains
- GPIO: Direction and Enable signals

**Control Mode:**
- Step/Direction interface
- Step pulse: Active-high or active-low (check CL57T datasheet)
- Minimum pulse width: Typically 2.5 µs (verify with datasheet)

**Implementation Notes:**
- Use timer with adjustable frequency for variable step rate
- Direction must be stable before step pulse (setup time requirement)
- Monitor driver ALARM output for fault detection

### Bus Servo Control (HX-65HM)

**Peripherals Required:**
- USART/UART: One channel for servo bus communication
- GPIO: Optional direction control pin for half-duplex operation

**Control Mode:**
- Half-duplex UART: Single wire for TX/RX (requires direction switching)
- Packet-based protocol with checksums
- Each servo has unique ID (0-253)
- Broadcast mode available (ID 254)

**Implementation Notes:**
- Baud rate: 115200 (configurable in servo)
- Packet structure: Header (0xFF 0xFF) + ID + Length + Instruction + Parameters + Checksum
- Common instructions: PING, READ_DATA, WRITE_DATA, SYNC_WRITE
- Half-duplex requires either hardware support or GPIO-based TX/RX switching
- Position range: 0-4095 (maps to 0°-360°)
- Can daisy-chain multiple servos on same bus
- Monitor feedback: position, speed, load, temperature, voltage

### UART Communication

**Peripherals Required:**
- USART/UART: One channel for Jetson communication



## Pin Assignment Strategy

### Timer Allocation (Actual)

| Timer | Function | Pins | Mode |
|-------|----------|------|------|
| TIM2 | DC motor PWM (4 channels) | PA0, PA1, PA2, PA3 | PWM output |
| TIM4 | Front Left encoder | PB6, PB7 | Quadrature decoder |
| TIM3 | Front Right encoder | PA6, PA7 | Quadrature decoder |
| TIM8 | Rear Left encoder | PC6, PC7 | Quadrature decoder |
| TIM1 | Rear Right encoder | PE9, PE11 | Quadrature decoder |

**Note:** Stepper uses GPIO bit-bang (PD14/PF15) with DWT cycle counter for microsecond delays, not a timer.

### UART Allocation (Actual)

| Peripheral | Function | Pins | Mode |
|------------|----------|------|------|
| LPUART1 | Jetson/PC communication (115200 8N1) | PG7 (TX), PG8 (RX) | Full duplex |
| USART3 | Bus servo (HX-65HM, 1 Mbaud) | PC10 (TX only) | Half-duplex |

Note: LPUART1 pins PG7/PG8 are the NUCLEO ST-LINK VCP. During development the PC sees
them as a COM port. On the Jetson, the same pins connect via USB-UART adapter.

USART3 TX is on PC10 (PB10 was originally soldered but pin was damaged; rewired to PC10).

### GPIO Pin Assignments (Actual)

| Pin | Port | Function | Subsystem |
|-----|------|----------|-----------|
| PA0 | GPIOA | TIM2_CH1 PWM | DC Motor FL |
| PA1 | GPIOA | TIM2_CH2 PWM | DC Motor FR |
| PA2 | GPIOA | TIM2_CH3 PWM | DC Motor RL |
| PA3 | GPIOA | TIM2_CH4 PWM | DC Motor RR |
| PA6 | GPIOA | TIM3_CH1 Encoder CHA | FR Encoder |
| PA7 | GPIOA | TIM3_CH2 Encoder CHB | FR Encoder |
| PB0 | GPIOB | DIR output | DC Motor FL |
| PB1 | GPIOB | DIR output | DC Motor FR |
| PB2 | GPIOB | DIR output | DC Motor RL |
| PB6 | GPIOB | TIM4_CH1 Encoder CHA | FL Encoder |
| PB7 | GPIOB | TIM4_CH2 Encoder CHB | FL Encoder (**conflicts with LD2 LED**) |
| PC10 | GPIOC | USART3 TX (half-duplex) | Servo |
| PC6 | GPIOC | TIM8_CH1 Encoder CHA | RL Encoder |
| PC7 | GPIOC | TIM8_CH2 Encoder CHB | RL Encoder |
| PC13 | GPIOC | B1 user button | Input (active LOW) |
| PD14 | GPIOD | PUL+ (step pulse) | Stepper |
| PD15 | GPIOD | Input w/ pull-up (NC to GND) | Limit Switch — HOME (retracted) |
| PE9 | GPIOE | TIM1_CH1 Encoder CHA | RR Encoder |
| PE10 | GPIOE | Input w/ pull-up (NC to GND) | Limit Switch — EXTEND (forward) |
| PE11 | GPIOE | TIM1_CH2 Encoder CHB | RR Encoder |
| PF14 | GPIOF | DIR output | DC Motor RR |
| PF15 | GPIOF | DIR+ (direction) | Stepper |

**Best Practice:** Document pin assignments in `hardware_config.h` with clear labels.

---

## Common Pitfalls to Avoid

1. **Blocking in interrupts**: Keep ISRs short, set flags for main loop processing
2. **Floating inputs**: Always configure pull-up/pull-down for unused or critical pins
3. **Uninitialized peripherals**: Follow proper clock enable → configure → enable sequence
4. **Timer overflow**: Monitor 16-bit encoder timers, implement overflow handling
5. **CubeMX regeneration**: User code between `USER CODE BEGIN/END` tags is preserved
6. **Race conditions**: Use volatile for variables shared between ISRs and main loop
7. **Interrupt priorities**: Higher priority (lower number) for time-critical tasks
8. **Debugging with motors**: Always test with motors disconnected first

---

## Build and Debug Workflow

### Using STM32CubeIDE

1. **Import Project:**
   - `File → Import → Existing Projects into Workspace`
   - Select: `C:\Users\benji\Documents\GitHub\LitterBot_STM32\firmware\stm32\LitterBot`
   - **Uncheck** "Copy projects into workspace"

2. **Build:**
   - Right-click project → `Build Project`
   - Or press `Ctrl+B`

3. **Flash and Debug:**
   - Connect NUCLEO-144 via USB (ST-LINK)
   - Click Debug button (bug icon) or `F11`
   - Step through code, set breakpoints, inspect variables

4. **Serial Monitor:**
   - Use built-in SWV (Serial Wire Viewer) for `printf` debugging
   - Or external terminal (PuTTY, Tera Term) connected to UART

---

## Key Questions to Ask Before Implementation

When implementing any subsystem:

1. **Do I have the component datasheet?**
   - Motor driver timing requirements
   - Encoder signal specifications
   - Communication protocol details

2. **What are the electrical characteristics?**
   - Voltage levels (3.3V, 5V, 24V?)
   - Current requirements
   - Signal polarity (active-high/low?)

3. **What are the timing requirements?**
   - Minimum pulse widths
   - Setup and hold times
   - Maximum switching frequencies

4. **How does this affect safety?**
   - What happens if this fails?
   - How do we detect faults?
   - Can the system recover?

5. **Is this deterministic?**
   - Does timing need to be precise?
   - Should this be interrupt-driven?
   - What's the worst-case latency?

---

## Resources and Documentation

### Project Documentation

- **Hardware datasheets**: Store in `docs/datasheets/`
  - DC Motor & Driver: `docs/datasheets/Planetary Gear Motor*.pdf`
  - Stepper Motor & Driver: `docs/datasheets/Stepper Motor/`
  - Bus Servo: `docs/datasheets/Servo Motor/HX-65*.pdf`
  - STM32: `docs/datasheets/STM32L4A6ZG Nucleo/`
- **Schematics**: Store in `docs/schematics/`
- **Meeting notes**: Keep in OneDrive (not Git)

### STM32 References

- **Reference Manual**: STM32L4A6 RM0351 (detailed peripheral descriptions)
- **Datasheet**: STM32L4A6ZG (electrical characteristics, pinout)
- **HAL Documentation**: UM1884 (STM32L4 HAL driver API)
- **NUCLEO User Manual**: UM2179 (board-specific info)

### Online Resources

- STM32 Community: https://community.st.com
- CubeMX Documentation: Built into CubeMX (`Help → User Manual`)
- HAL Driver Examples: STM32CubeL4 package

---

## Expected Interaction Style

When working on this project with Claude Code:

- **Provide context**: Which subsystem you're working on
- **Share datasheets**: When implementing drivers for specific hardware
- **Ask for code reviews**: Before committing major changes
- **Request explanations**: For HAL functions or register-level operations
- **Verify pin assignments**: Before connecting hardware
- **Discuss trade-offs**: Between different implementation approaches

**Claude Code should:**
- Explain reasoning behind design decisions
- Flag potential safety issues explicitly
- Ask clarifying questions when requirements are ambiguous
- Provide register-level detail when needed
- Suggest testing strategies for new features
- Reference datasheets and specifications

---

## Project Status

**Last updated: 2026-04-01**

**What is written and builds clean:**
- CubeMX configured: TIM2 PWM (4ch), TIM1/3/4/8 encoder mode, USART3 half-duplex, LPUART1 VCP
- Drivers: `cytron_md10c`, `cl57t_stepper`, `hx65_servo`, `encoder`, `odometry`
- Communication: `uart_cmd` — handles `M`/`X`/`S SCOOP`/`C CALIBRATE`/`D DEPLOY`/`DEV`/`Q`/`T` with deploy state gating
- Safety: `deploy_state` — STANDBY/DEPLOYING/ACTIVE state machine, dev mode bypass, command gating
- Arm: `arm_control` — calibration mode (B1 e-stop), normal extend/retract, servo interpolation, Arm_Home()
- Limit switches: `limit_switch` — PD15 (home), PE10 (extend), NC-to-GND fail-safe wiring
- Scoop sequence: `scoop_sequence.c/h` — 6-phase coordinated stepper+servo motion (**all constants are PLACEHOLDERS**)
- Test modules: `servo_test`, `dc_motor_test`, `stepper_test`, `combined_test`, `encoder_test`

**What has NOT been tested on hardware (blocked — no board available):**
- All of the above. Every module is written but none have been run on real hardware.

**Hardware notes:**
- First NUCLEO-144 board fried (LD5 overcurrent after servo wiring incident) — using identical spare
- USART3 TX rewired from PB10 (damaged) to PC10
- PB7 (FL encoder CHB) conflicts with LD2 LED — watch for interference during encoder testing

**NOTE: ACTION REQUIRED before any hardware testing:**
- Fix LPUART1 word length in CubeMX: 9 Bits → **8 Bits** (standard 8N1). Currently misconfigured.

**Hardware bring-up order (do in this sequence):**
1. Fix LPUART1 in CubeMX, regenerate code
2. Flash and test servo (`servo_test.c` — B1 button → servo sweep)
3. Flash and test DC motors (`dc_motor_test.c`)
4. Flash and test stepper (`stepper_test.c`)
5. Flash and run combined test (`combined_test.c`)
6. Flash and test encoders (`encoder_test.c` — spin wheels by hand, verify counts)
7. Test UART teleop end-to-end: connect to WSL2, run `stm32_bridge` in real mode, drive with WASD
8. Measure scoop values, fill in `scoop_sequence.h` (see placeholders and comments in that file)


## Notes for Claude Code

- This is a **real hardware project** with physical safety implications
- Prioritize **correctness and safety** over speed of implementation
- When unsure about electrical characteristics, **always ask for the datasheet**
- Assume the user has moderate embedded systems experience
- Provide **detailed explanations** for complex concepts
- Use **register-level detail** when HAL abstraction is insufficient
- Suggest **incremental testing** strategies for each new feature
- Respect **real-time constraints** and deterministic timing requirements

**Never guess about:**
- Voltage levels or current limits
- Signal timing requirements
- Pin polarity or electrical characteristics
- Communication protocol details
- Motor driver configuration.


Many of the concepts in here I have not yet learned about, so I would like you to help teach me along the way.
This includes propper use of github. I would like this document to stay live, and edit it as you learn more, and feel free to make changes to things that are not true

**Always verify with datasheets or explicit user confirmation.**