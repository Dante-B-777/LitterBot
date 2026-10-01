# STM32 System Testing

This document tracks all hardware test procedures for the LitterBot STM32 firmware. Each test is designed to validate a specific subsystem independently before integration.

---

## Master Pin Assignment Table

All STM32 pins currently in use across every subsystem. Reference this when wiring or troubleshooting.

### DC Motors (Cytron MD10C)

| Pin  | Port  | Function       | Motor | Peripheral | Wire Color | Notes                              |
|------|-------|----------------|-------|------------|------------|------------------------------------|
| PA0  | GPIOA | PWM (TIM2_CH1) | FL    | TIM2       | Red        | Speed control                      |
| PB0  | GPIOB | DIR output     | FL    | GPIO       | Red        | Direction control                  |
| PA1  | GPIOA | PWM (TIM2_CH2) | FR    | TIM2       | Blue       | Speed control                      |
| PB1  | GPIOB | DIR output     | FR    | GPIO       | Blue       | Direction control                  |
| PA2  | GPIOA | PWM (TIM2_CH3) | RL    | TIM2       | Green      | Speed control                      |
| PB2  | GPIOB | DIR output     | RL    | GPIO       | Green      | Direction control                  |
| PA3  | GPIOA | PWM (TIM2_CH4) | RR    | TIM2       | Yellow     | Speed control                      |
| PF14 | GPIOF | DIR output     | RR    | GPIO       | Yellow     | Was PB10, moved to free servo UART |

### Encoders (goBILDA Quadrature)

| Pin  | Port  | Function              | Motor | Peripheral | Wire Color | Notes                         |
|------|-------|-----------------------|-------|------------|------------|-------------------------------|
| PD12 | GPIOD | Encoder CHA (TIM4_CH1)| FL    | TIM4       | White      | Quadrature channel A          |
| PD13 | GPIOD | Encoder CHB (TIM4_CH2)| FL    | TIM4       | Blue       | Quadrature channel B          |
| PE3  | GPIOE | Encoder CHA (TIM3_CH1)| FR    | TIM3       | White      | Quadrature channel A          |
| PE4  | GPIOE | Encoder CHB (TIM3_CH2)| FR    | TIM3       | Blue       | Quadrature channel B          |
| PC6  | GPIOC | Encoder CHA (TIM8_CH1)| RL    | TIM8       | White      | Quadrature channel A          |
| PC7  | GPIOC | Encoder CHB (TIM8_CH2)| RL    | TIM8       | Blue       | Quadrature channel B          |
| PE9  | GPIOE | Encoder CHA (TIM1_CH1)| RR    | TIM1       | White      | Quadrature channel A          |
| PE11 | GPIOE | Encoder CHB (TIM1_CH2)| RR    | TIM1       | Blue       | Quadrature channel B          |

### Stepper (CL57T)

| Pin  | Port  | Function    | Peripheral | Wire Color | Notes            |
|------|-------|-------------|------------|------------|------------------|
| PD14 | GPIOD | PUL+ output | GPIO       | Green      | CL57T step pulse |
| PF15 | GPIOF | DIR+ output | GPIO       | Blue       | CL57T direction  |

### Servo (HX-65HM)

| Pin  | Port  | Function              | Peripheral | Wire Color | Notes                          |
|------|-------|-----------------------|------------|------------|--------------------------------|
| PC10 | GPIOC | UART TX (half-duplex) | USART3     | --         | HX-65HM signal wire (PB10 damaged, rewired to PC10) |

### Board / System

| Pin  | Port  | Function          | Peripheral | Notes                          |
|------|-------|-------------------|------------|--------------------------------|
| PC13 | GPIOC | Button input      | GPIO       | B1 user button (active LOW)    |
| PB14 | GPIOB | LD3 output        | GPIO       | Red LED (used by encoder test) |
| PG5  | GPIOG | USB OverCurrent   | GPIO       | Input                          |
| PG6  | GPIOG | USB PowerSwitchOn | GPIO       | Output                         |

### Ground Connections

| Device | Connect To | Wire Color |
|--------|-----------|------------|
| CL57T PUL- | STM32 GND | Black |
| CL57T DIR- | STM32 GND | Black |
| Cytron MD10C GND (x4) | STM32 GND | -- |
| HX-65HM GND | STM32 GND | -- |

### Power Summary (External, NOT from STM32)

| Device | Supply Voltage | Source |
|--------|---------------|--------|
| Cytron MD10C (x4) | Motor supply voltage | External PSU |
| CL57T stepper driver | 24-48V DC | External PSU |
| HX-65HM servo | 9-12.6V DC | External PSU |

---

## Servo Motor Test (HX-65HM)

### Purpose

Validate that the STM32 can communicate with the HX-65HM bus servo over half-duplex UART and command position changes across the full range of motion. This is the first hardware communication test -- it confirms the UART wiring, protocol implementation, and servo response.

### Prerequisites

- **Servo powered**: 9-12.6V supply connected to servo (the Nucleo board **cannot** power the servo)
- **Common ground**: Servo GND and STM32 GND must be connected together
- **Signal wire**: Servo SIG pin connected to the USART3 half-duplex TX pin
- **Servo ID**: Factory default is typically 1 (configurable in `servo_test.h` via `SERVO_TEST_ID`)
- **Firmware flashed**: Build and flash the LitterBot project via ST-LINK

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `servo_test.h` | `Core/Inc/servo_test.h` | Test function declarations, servo ID config |
| `servo_test.c` | `Core/Src/servo_test.c` | Test implementation (sweep sequence) |
| `hx65_servo.h` | `Core/Inc/hx65_servo.h` | Servo driver interface |
| `hx65_servo.c` | `Core/Src/hx65_servo.c` | Servo driver implementation (UART protocol) |
| `main.c` | `Core/Src/main.c` | `ServoTest_Init()` and `ServoTest_Poll()` calls in USER CODE sections |

### How to Run

1. Connect servo hardware (power, ground, signal)
2. Flash firmware to the Nucleo-144 via USB/ST-LINK
3. System powers up -- **LD2 is off** (idle)
4. Press the **blue B1 button** (PC13) to start the test
5. Observe the servo and LD2 LED for results

### Test Sequence

When B1 is pressed, the servo sweeps through six positions with 1 second of movement time between each and a 1.5 second hold at each position:

| Step | Position | Angle | Description |
|------|----------|-------|-------------|
| 1 | 0 | 0 degrees | One end of travel |
| 2 | 1024 | 90 degrees | Quarter turn |
| 3 | 2048 | 180 degrees | Midpoint |
| 4 | 3072 | 270 degrees | Three-quarter turn |
| 5 | 4095 | 360 degrees | Full rotation end |
| 6 | 2048 | 180 degrees | Return to center |

Total test duration: approximately 15 seconds.

### LED Feedback

| LD2 Pattern | Meaning |
|-------------|---------|
| Off | Idle, waiting for button press |
| Solid on | Test is running |
| 3 slow blinks | Test completed successfully |
| 10 fast blinks | Error -- servo not responding |

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| LD2 fast-blinks immediately | Servo not connected, wrong ID, or wiring issue |
| LD2 stays solid, servo doesn't move | UART TX pin not connected to servo SIG |
| Servo moves erratically | Baud rate mismatch (should be 115200) or noise on signal line |
| Nothing happens on button press | Firmware not flashed, or button debounce issue |
| Servo vibrates but doesn't sweep | Power supply voltage too low (needs 9-12.6V) |

### Configuration

Key parameters in `servo_test.h`:

```c
#define SERVO_TEST_ID       1       // Servo bus address (change if yours differs)
```

Key parameters in `servo_test.c`:

```c
#define SWEEP_HOLD_MS       1500    // Hold time at each position (ms)
#define SWEEP_MOVE_MS       1000    // Travel time between positions (ms)
#define DEBOUNCE_MS         200     // Button debounce delay (ms)
```

### Peripheral Configuration

| Peripheral | Setting | Configured In |
|------------|---------|---------------|
| USART3 | 115200 baud, 8N1, half-duplex | CubeMX (`LitterBot.ioc`) |
| TIM2 | PSC=0, ARR=2366, 4ch PWM | CubeMX (not used by this test) |
| B1 Button | PC13, rising edge interrupt (polled) | CubeMX |
| LD2 | PB7, push-pull output | CubeMX |

### UART Protocol Summary

The HX-65HM uses a packet-based protocol over half-duplex UART:

```
[0xFF] [0xFF] [ID] [Length] [Command] [Params...] [Checksum]
```

- **Checksum** = `~(ID + Length + Command + Params) & 0xFF`
- **Half-duplex**: STM32 switches between transmitter and receiver mode on the same wire
- **Move command** (CMD 1): 4 parameter bytes -- position (16-bit LE) + time (16-bit LE)

---

## DC Motor Test (Cytron MD10C)

### Purpose

Validate that the STM32 can drive all four Cytron MD10C motor controllers via TIM2 PWM and GPIO direction pins. This test verifies PWM generation on PA0-PA3, direction control on PB0/PB1/PB2/PF14, and confirms each motor responds to forward and reverse commands. Speed is capped at 25% for safety during bench testing.

### Prerequisites

- **Motor drivers powered**: Each Cytron MD10C needs its own motor supply voltage (matching your motors)
- **Common ground**: Motor driver GND, motor GND, and STM32 GND must all be connected together
- **PWM wiring**: PA0→FL PWM, PA1→FR PWM, PA2→RL PWM, PA3→RR PWM
- **DIR wiring**: PB0→FL DIR, PB1→FR DIR, PB2→RL DIR, PF14→RR DIR
- **Motors connected** (or test with oscilloscope on PWM/DIR pins if no motors available)
- **Firmware flashed**: Build with `dc_motor_test.h` active (see "How to Run")

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `dc_motor_test.h` | `Core/Inc/dc_motor_test.h` | Test function declarations, speed cap define |
| `dc_motor_test.c` | `Core/Src/dc_motor_test.c` | Test implementation (sequential + all-motors sequence) |
| `cytron_md10c.h` | `Core/Inc/cytron_md10c.h` | DC motor driver interface |
| `cytron_md10c.c` | `Core/Src/cytron_md10c.c` | DC motor driver implementation (PWM + GPIO) |
| `main.c` | `Core/Src/main.c` | `DCMotorTest_Init()` and `DCMotorTest_Poll()` calls in USER CODE sections |

### How to Run

1. Connect motor driver hardware (power, ground, PWM, DIR)
2. In `main.c`, comment out the servo test and uncomment the DC motor test:
   ```c
   /* USER CODE BEGIN Includes */
   // #include "servo_test.h"
   #include "dc_motor_test.h"
   // #include "stepper_test.h"
   /* USER CODE END Includes */
   ```
   Do the same for `Init()` in USER CODE BEGIN 2 and `Poll()` in USER CODE BEGIN 3.
3. Build and flash firmware to the Nucleo-144 via USB/ST-LINK
4. System powers up -- **LD2 is off** (idle)
5. Press the **blue B1 button** (PC13) to start the test
6. Observe motors and LD2 LED for results

### Test Sequence

When B1 is pressed, the test runs three phases (~20 seconds total):

**Phase 1 -- Individual Motor Test:**

| Step | Motor | Direction | Speed | Duration |
|------|-------|-----------|-------|----------|
| 1 | FL | Forward | 25% | 1.5s |
| 2 | FL | (pause) | 0% | 0.5s |
| 3 | FL | Reverse | 25% | 1.5s |
| 4 | FL | (pause) | 0% | 0.5s |
| 5 | FR | Forward | 25% | 1.5s |
| 6 | FR | (pause) | 0% | 0.5s |
| 7 | FR | Reverse | 25% | 1.5s |
| 8 | FR | (pause) | 0% | 0.5s |
| 9 | RL | Forward | 25% | 1.5s |
| 10 | RL | (pause) | 0% | 0.5s |
| 11 | RL | Reverse | 25% | 1.5s |
| 12 | RL | (pause) | 0% | 0.5s |
| 13 | RR | Forward | 25% | 1.5s |
| 14 | RR | (pause) | 0% | 0.5s |
| 15 | RR | Reverse | 25% | 1.5s |
| 16 | RR | (pause) | 0% | 0.5s |

**Phase 2 -- All Forward:**

| Step | Motors | Direction | Speed | Duration |
|------|--------|-----------|-------|----------|
| 17 | All 4 | Forward | 25% | 1.5s |
| 18 | All 4 | (pause) | 0% | 0.5s |

**Phase 3 -- All Reverse:**

| Step | Motors | Direction | Speed | Duration |
|------|--------|-----------|-------|----------|
| 19 | All 4 | Reverse | 25% | 1.5s |
| 20 | All 4 | Stop | 0% | -- |

### LED Feedback

| LD2 Pattern | Meaning |
|-------------|---------|
| Off | Idle, waiting for button press |
| Solid on | Test is running |
| 3 slow blinks | Test completed successfully |
| 10 fast blinks | Error -- `MD10C_SetSpeed()` returned HAL_ERROR |

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| LD2 fast-blinks immediately | `MD10C_Init()` failed -- TIM2 not configured in CubeMX, or HAL_TIM not enabled |
| Nothing happens on button press | Firmware not flashed, or wrong test active in `main.c` |
| Motor spins but doesn't reverse | DIR pin not connected or wrong GPIO port/pin |
| Motor doesn't spin at all | PWM pin not connected, motor driver not powered, or driver fault |
| Only some motors work | Check individual PWM/DIR wiring for the non-working motor |
| Motors run at wrong speed | Verify TIM2 ARR=2366 and PSC=0 in CubeMX |

### Configuration

Key parameter in `dc_motor_test.h`:

```c
#define TEST_SPEED_PCT      25.0f   // Max speed during test (safety cap)
```

Key parameters in `dc_motor_test.c`:

```c
#define RUN_MS              1500    // Time each motor runs at speed (ms)
#define PAUSE_MS            500     // Pause between motor actions (ms)
#define DEBOUNCE_MS         200     // Button debounce delay (ms)
```

### Peripheral Configuration

| Peripheral | Setting | Configured In |
|------------|---------|---------------|
| TIM2 | PSC=0, ARR=2366, 4ch PWM on PA0-PA3 | CubeMX (`LitterBot.ioc`) |
| GPIOB | PB0/PB1/PB2 push-pull outputs + PF14 (DIR pins) | `cytron_md10c.c` `MD10C_Init()` |
| B1 Button | PC13, rising edge interrupt (polled) | CubeMX |
| LD2 | PB7, push-pull output | CubeMX |

### Pin Assignment Summary

| Motor | PWM Pin | Timer Channel | DIR Pin |
|-------|---------|---------------|---------|
| FL | PA0 | TIM2_CH1 | PB0 |
| FR | PA1 | TIM2_CH2 | PB1 |
| RL | PA2 | TIM2_CH3 | PB2 |
| RR | PA3 | TIM2_CH4 | PF14 |

### PWM Details

- Timer clock: ~47.333 MHz (APB1 timer clock with x2 doubler)
- PWM frequency: 47,333,333 / (2366 + 1) = ~19,996 Hz (~20 kHz, above audible range)
- Duty cycle at 25%: compare value = 2366 × 0.25 = ~591 counts
- Sign-magnitude control: PWM duty sets speed, DIR GPIO sets forward/reverse

---

## Stepper Motor Test (CL57T)

### Purpose

Validate that the STM32 can generate step/direction signals for the CL57T closed-loop stepper driver via GPIO bit-banging on PC6 (PUL+) and PC7 (DIR+). This test verifies pulse timing, direction control, and variable speed operation. The stepper returns to its starting position after each phase to confirm step accuracy.

### Prerequisites

- **Stepper driver powered**: CL57T needs 24-48V motor supply
- **Common ground**: CL57T GND and STM32 GND must be connected together
- **Signal wiring**: PD14→PUL+ (green wire), PF15→DIR+ (blue wire)
- **PUL-/DIR- connection**: Both to STM32 GND (black wires, common-anode wiring)
- **Motor connected**: NEMA 23 stepper wired to CL57T A+/A-/B+/B- terminals
- **Firmware flashed**: Build with `stepper_test.h` active (see "How to Run")

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `stepper_test.h` | `Core/Inc/stepper_test.h` | Test function declarations |
| `stepper_test.c` | `Core/Src/stepper_test.c` | Test implementation (3-phase speed sequence) |
| `cl57t_stepper.h` | `Core/Inc/cl57t_stepper.h` | Stepper driver interface |
| `cl57t_stepper.c` | `Core/Src/cl57t_stepper.c` | Stepper driver implementation (GPIO bit-bang, DWT delays) |
| `main.c` | `Core/Src/main.c` | `StepperTest_Init()` and `StepperTest_Poll()` calls in USER CODE sections |

### How to Run

1. Connect stepper driver hardware (power, ground, PUL, DIR)
2. In `main.c`, comment out the servo test and uncomment the stepper test:
   ```c
   /* USER CODE BEGIN Includes */
   // #include "servo_test.h"
   // #include "dc_motor_test.h"
   #include "stepper_test.h"
   /* USER CODE END Includes */
   ```
   Do the same for `Init()` in USER CODE BEGIN 2 and `Poll()` in USER CODE BEGIN 3.
3. Build and flash firmware to the Nucleo-144 via USB/ST-LINK
4. System powers up -- **LD2 is off** (idle)
5. Press the **blue B1 button** (PC13) to start the test
6. Observe stepper motor and LD2 LED for results

### Test Sequence

When B1 is pressed, the test runs three phases (~7 seconds total):

| Phase | Speed | Step Delay | Steps CW | Steps CCW | Phase Duration |
|-------|-------|-----------|-----------|-----------|----------------|
| 1 (Slow) | ~250 steps/s | 2000 us | 200 | 200 | ~1.0s |
| 2 (Medium) | ~500 steps/s | 1000 us | 200 | 200 | ~0.6s |
| 3 (Fast) | ~1000 steps/s | 500 us | 200 | 200 | ~0.4s |

Each phase:
1. Moves 200 steps clockwise
2. Brief 200ms pause
3. Moves 200 steps counter-clockwise (returns to start)
4. 1 second pause before next phase (except after last phase)

The motor should return to exactly the same position after the full test.

### LED Feedback

| LD2 Pattern | Meaning |
|-------------|---------|
| Off | Idle, waiting for button press |
| Solid on | Test is running |
| 3 slow blinks | Test completed successfully |
| 10 fast blinks | Error (reserved for future fault detection) |

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Motor doesn't move at all | PUL+/DIR+ not connected, driver not powered, or motor not wired |
| Motor vibrates but doesn't rotate | Wrong motor winding order (swap A+/A- or B+/B-) |
| Motor moves CW but not CCW (or vice versa) | DIR pin (PC7) not connected or wrong polarity |
| Motor doesn't return to exact start position | Step pulses being missed -- check PUL-/DIR- ground connection |
| LD2 stays solid, nothing happens | DWT cycle counter not initializing -- check `CL57T_Init()`. Also verify GPIOD/GPIOF clocks enabled. |
| Motor runs but skips steps | Motor load too high for speed, or driver current setting too low |
| Nothing happens on button press | Firmware not flashed, or wrong test active in `main.c` |

### Configuration

Key parameters in `stepper_test.c`:

```c
#define STEPS_PER_MOVE      200     // Steps per direction per phase
#define DELAY_SLOW_US       2000    // Step delay for slow phase (us)
#define DELAY_MEDIUM_US     1000    // Step delay for medium phase (us)
#define DELAY_FAST_US       500     // Step delay for fast phase (us)
#define PHASE_PAUSE_MS      1000    // Pause between phases (ms)
#define DEBOUNCE_MS         200     // Button debounce delay (ms)
```

### Peripheral Configuration

| Peripheral | Setting | Configured In |
|------------|---------|---------------|
| GPIOD | PD14 push-pull output (PUL+) | `cl57t_stepper.c` `CL57T_Init()` |
| GPIOF | PF15 push-pull output (DIR+) | `cl57t_stepper.c` `CL57T_Init()` |
| DWT | Cycle counter enabled for microsecond delays | `cl57t_stepper.c` `CL57T_Init()` |
| B1 Button | PC13, rising edge interrupt (polled) | CubeMX |
| LD2 | PB7, push-pull output | CubeMX |

### Wiring Summary

| CL57T Terminal | STM32 Pin | Wire Color |
|----------------|-----------|------------|
| PUL+ | PD14 | Green |
| PUL- | GND | Black |
| DIR+ | PF15 | Blue |
| DIR- | GND | Black |

### Stepper Timing Details

- **Pulse generation**: GPIO bit-bang using DWT cycle counter for microsecond precision
- **Minimum PUL high time**: 2.5 us (driver uses 3 us for margin)
- **DIR setup time**: 5 us before PUL rising edge (driver uses 6 us for margin)
- **Step rate at 500us delay**: ~1000 steps/sec (well within CL57T's 200 kHz max)
- **Total step time**: pulse_width (3 us) + delay_us per step

---

## Combined Peripheral Test (All Subsystems)

### Purpose

Validate that all three motor subsystems (DC motors, stepper, servo) can operate correctly from a single firmware build. This is the integration test -- it exercises every peripheral in sequence on a single button press, confirming that there are no pin conflicts, timing issues, or initialization order problems when all drivers are active simultaneously.

### Prerequisites

- **All hardware connected**: DC motor drivers, stepper driver, and servo must all be wired and powered
- **DC motor power**: Each Cytron MD10C powered with motor supply voltage
- **Stepper power**: CL57T powered with 24-48V supply
- **Servo power**: HX-65HM powered with 9-12.6V supply
- **Common ground**: All driver GNDs and STM32 GND connected together
- **Signal wiring**: All PWM, DIR, PUL, DIR, and UART signal wires connected (see individual test sections above)
- **Firmware flashed**: Build with `combined_test.h` active (see "How to Run")

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `combined_test.h` | `Core/Inc/combined_test.h` | Test function declarations, speed cap, servo ID |
| `combined_test.c` | `Core/Src/combined_test.c` | Test implementation (7-phase sequence) |
| `cytron_md10c.h/.c` | `Core/Inc/` and `Core/Src/` | DC motor driver (TIM2 PWM + GPIO DIR) |
| `cl57t_stepper.h/.c` | `Core/Inc/` and `Core/Src/` | Stepper driver (GPIO bit-bang, DWT delays) |
| `hx65_servo.h/.c` | `Core/Inc/` and `Core/Src/` | Servo driver (half-duplex UART protocol) |
| `main.c` | `Core/Src/main.c` | `CombinedTest_Init()` and `CombinedTest_Poll()` calls in USER CODE sections |

### How to Run

1. Connect all hardware (DC motors, stepper, servo -- power, ground, signals)
2. In `main.c`, comment out all other tests and uncomment the combined test:
   ```c
   /* USER CODE BEGIN Includes */
   // #include "servo_test.h"
   // #include "dc_motor_test.h"
   // #include "stepper_test.h"
   #include "combined_test.h"
   /* USER CODE END Includes */
   ```
   Do the same for `Init(&huart3)` in USER CODE BEGIN 2 and `Poll()` in USER CODE BEGIN 3.
3. Build and flash firmware to the Nucleo-144 via USB/ST-LINK
4. System powers up -- **LD2 is off** (idle)
5. Press the **blue B1 button** (PC13) to start the test
6. Observe all peripherals and LD2 LED for results

### Test Sequence

When B1 is pressed, the test runs seven phases (~18 seconds total):

| Phase | Subsystem | Action | Speed/Params | Duration |
|-------|-----------|--------|--------------|----------|
| 1 | DC Motors | All forward | 25% duty cycle | 1.5s |
| (pause) | -- | Motors stopped | -- | 0.5s |
| 2 | DC Motors | All reverse | 25% duty cycle | 1.5s |
| (pause) | -- | Motors stopped | -- | 0.5s |
| 3 | DC Motors | Skid-steer left | FL/RL reverse, FR/RR forward at 25% | 1.5s |
| (pause) | -- | Motors stopped | -- | 0.5s |
| 4 | DC Motors | Skid-steer right | FL/RL forward, FR/RR reverse at 25% | 1.5s |
| (pause) | -- | Motors stopped | -- | 0.5s |
| 5 | Stepper | CW then CCW | 200 steps each at 1000us delay | ~0.6s |
| (pause) | -- | -- | -- | 0.5s |
| 6 | Servo | Move up | Position 3072 (270 deg), 1s travel | 2.0s |
| 7 | Servo | Move down | Position 1024 (90 deg), 1s travel | 2.0s |

After all phases complete, LD2 blinks 3 times (success). Press B1 again to repeat.

### LED Feedback

| LD2 Pattern | Meaning |
|-------------|---------|
| Off | Idle, waiting for button press |
| Solid on | Test is running |
| 3 slow blinks | Test completed successfully |
| 10 fast blinks | Error -- servo communication failed (DC motors auto-stopped) |

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| DC motors work but servo fast-blinks error | Servo not connected, wrong ID, or UART wiring issue |
| Servo works but DC motors don't spin | TIM2 not configured in CubeMX, or motor driver not powered |
| Stepper doesn't move but others work | PUL+/DIR+ not wired, or CL57T not powered |
| Nothing happens on button press | Firmware not flashed, or wrong test active in `main.c` |
| Only partial sequence runs | Check which phase fails -- corresponds to specific subsystem issue |
| Motors spin wrong direction on left/right turn | DIR pin wiring swapped on one side |
| System hangs during stepper phase | DWT cycle counter issue -- check `CL57T_Init()` |

### Configuration

Key parameters in `combined_test.h`:

```c
#define COMBINED_TEST_SPEED_PCT     25.0f   // DC motor speed cap (safety)
#define COMBINED_SERVO_ID           1       // Servo bus address
```

Key parameters in `combined_test.c`:

```c
#define DC_PHASE_MS         1500    // DC motor run time per phase (ms)
#define PHASE_PAUSE_MS      500     // Pause between phases (ms)
#define STEPPER_STEPS       200     // Steps per direction
#define STEPPER_DELAY_US    1000    // Step delay (us) -- medium speed
#define SERVO_POS_UP        3072    // Servo "up" position (270 deg)
#define SERVO_POS_DOWN      1024    // Servo "down" position (90 deg)
#define SERVO_MOVE_MS       1000    // Servo travel time (ms)
#define SERVO_HOLD_MS       1000    // Hold at each servo position (ms)
#define DEBOUNCE_MS         200     // Button debounce delay (ms)
```

### Peripheral Configuration

| Peripheral | Setting | Used By |
|------------|---------|---------|
| TIM2 | PSC=0, ARR=2366, 4ch PWM on PA0-PA3 | DC motors (speed) |
| GPIOB | PB0/PB1/PB2 push-pull outputs + PF14 | DC motors (direction) |
| GPIOD | PD14 push-pull output (PUL+) | Stepper (step pulses) |
| GPIOF | PF15 push-pull output (DIR+) | Stepper (direction) |
| USART3 | 115200 baud, 8N1, half-duplex on PC10 | Servo (UART protocol) |
| DWT | Cycle counter for microsecond delays | Stepper (pulse timing) |
| B1 Button | PC13, active LOW (polled) | Test trigger |
| LD2 | PB7, push-pull output | Status feedback |

### Pin Summary (All Subsystems)

| Subsystem | Pin | Function |
|-----------|-----|----------|
| DC Motor FL | PA0 / PB0 | PWM (TIM2_CH1) / DIR |
| DC Motor FR | PA1 / PB1 | PWM (TIM2_CH2) / DIR |
| DC Motor RL | PA2 / PB2 | PWM (TIM2_CH3) / DIR |
| DC Motor RR | PA3 / PF14 | PWM (TIM2_CH4) / DIR |
| Stepper PUL+ | PD14 | Step pulse (GPIO) |
| Stepper DIR+ | PF15 | Direction (GPIO) |
| Servo SIG | PC10 | USART3 TX (half-duplex) |
| User Button | PC13 | Test trigger (B1) |
| Status LED | PB7 | LD2 feedback |

---

## Encoder Test (Quadrature Encoders + Odometry)

### Purpose

Validate that the STM32 can read all four goBILDA Yellow Jacket motor encoders via hardware quadrature counting (timer encoder mode). This test confirms encoder wiring, timer configuration, tick counting accuracy, and basic odometry integration. The test drives all motors forward then reverse at 25% while sampling encoders, and checks that all four report non-zero ticks.

### Prerequisites

- **Encoders connected**: Each goBILDA motor has an integrated quadrature encoder (White = CHA, Blue = CHB)
- **Encoder wiring**: FL→PD12/PD13 (TIM4), FR→PE3/PE4 (TIM3), RL→PC6/PC7 (TIM8), RR→PE9/PE11 (TIM1)
- **DC motor drivers powered**: Cytron MD10C needs motor supply voltage (test drives motors at 25%)
- **Common ground**: All encoder GNDs, motor driver GNDs, and STM32 GND connected
- **CubeMX configured**: TIM1, TIM3, TIM4, TIM8 must be set to Encoder Mode (TI1 and TI2, both edges)
- **Serial terminal**: Connect to ST-LINK VCP (LPUART1) to view test output
- **Firmware flashed**: Build with `encoder_test.h` active (see "How to Run")

**Important**: The current CubeMX `.ioc` maps the FL encoder to `PD12/PD13`, so `PB7` remains available for LD2 unless the `.ioc` is changed again. The encoder test uses **LD3 (PB14, red LED)** for feedback instead.

### Encoder Hardware Specs

| Parameter | Value |
|-----------|-------|
| Motor | goBILDA Yellow Jacket 5203-2402-0019 |
| Encoder type | Hall-effect quadrature |
| CPR (motor shaft) | 28 |
| Gear ratio | 19.2:1 |
| Output shaft resolution | 537.7 ticks/rev (4x counting) |
| Wheel diameter | 104mm (52mm radius) |
| Max RPM (no-load, 12V) | 312 RPM |

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `encoder_test.h` | `Core/Inc/encoder_test.h` | Test function declarations |
| `encoder_test.c` | `Core/Src/encoder_test.c` | Test implementation (fwd/rev + print ticks) |
| `encoder.h` | `Core/Inc/encoder.h` | Encoder driver interface |
| `encoder.c` | `Core/Src/encoder.c` | Encoder driver implementation (timer encoder mode) |
| `odometry.h` | `Core/Inc/odometry.h` | Odometry interface (mecanum kinematics) |
| `odometry.c` | `Core/Src/odometry.c` | Odometry implementation (dead reckoning) |
| `cytron_md10c.h/.c` | `Core/Inc/` and `Core/Src/` | DC motor driver (used to spin wheels) |
| `main.c` | `Core/Src/main.c` | `EncoderTest_Init()` and `EncoderTest_Poll()` calls |

### How to Run

**Option A: With ROS2 teleop (recommended)**

The encoder test runs alongside UART teleop. Your bridge node sends `T\n` when you press an unused teleop key (e.g., `t`).

1. Connect encoder and motor driver hardware
2. In `main.c`, uncomment the "UART teleop + encoder test (ROS2 mode)" block:
   - Uncomment `static uint8_t rx_byte;` in USER CODE PV
   - Uncomment `Encoder_Init()`, `Odometry_Init()`, `UartCmd_Init(&hlpuart1)`, `HAL_UART_Receive_IT(...)` in USER CODE BEGIN 2
   - Uncomment `Encoder_Update()`, `Odometry_Update()`, `UartCmd_Task(...)` in USER CODE BEGIN 3
   - Uncomment the `HAL_UART_RxCpltCallback` in USER CODE BEGIN 4
3. Build and flash
4. Drive with teleop, send `T\n` over serial to trigger the encoder test

**Option B: Standalone (B1 button only)**

1. Connect encoder and motor driver hardware
2. In `main.c`, uncomment `EncoderTest_Init(&hlpuart1)` and `EncoderTest_Poll()` (standalone lines)
3. Build and flash
4. Press B1 button to trigger the test
4. Build and flash firmware to the Nucleo-144 via USB/ST-LINK
5. Open a serial terminal (PuTTY, Tera Term) on the ST-LINK COM port
6. System powers up -- **LD3 is off** (idle)
7. Press the **blue B1 button** (PC13) to start the test
8. Observe LD3 LED and serial output for results

### Test Sequence

When B1 is pressed, the test runs two phases (~5 seconds total):

| Phase | Action | Speed | Duration | Expected Encoder Result |
|-------|--------|-------|----------|------------------------|
| Reset | Zero all encoder counts | -- | instant | All ticks = 0 |
| 1 (Forward) | All motors forward | 25% | 2s | All ticks positive, similar magnitude |
| (settle) | Motors stopped | 0% | 0.2s | Ticks stabilize |
| 2 (Reverse) | All motors reverse | 25% | 2s | Ticks return toward 0 |
| (settle) | Motors stopped | 0% | 0.2s | Ticks near 0 |

After both phases, the test prints odometry pose (x, y, theta) and signals result via LD3.

### Serial Output Example

```
=== ENCODER TEST START ===
After reset: FL=0  FR=0  RL=0  RR=0

--- Forward 25% for 2s ---
After forward: FL=1423  FR=1398  RL=1410  RR=1435

--- Reverse 25% for 2s ---
After reverse: FL=12  FR=-8  RL=5  RR=-15
Odom: x=0.003 y=-0.001 theta=0.15 deg

=== ENCODER TEST END ===
RESULT: OK -- all encoders responding
```

### LED Feedback

| LD3 Pattern | Meaning |
|-------------|---------|
| Off | Idle, waiting for button press |
| Solid on | Test is running |
| 3 slow blinks (red) | Success -- all encoders responding |
| 10 fast blinks (red) | Error -- one or more encoders read 0 (no signal) |

**Note**: In the current CubeMX `.ioc`, LD2 stays on `PB7` because the FL encoder is mapped to `PD12/PD13`.

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| LD3 fast-blinks (error) | One or more encoders not connected or CubeMX timers not in encoder mode |
| All ticks are 0 | Encoder timers not configured in CubeMX, or HAL_TIM_Encoder_Start() failing |
| One encoder reads 0, others work | That specific encoder not wired, or wrong timer/pin in CubeMX |
| Ticks are negative when going forward | CHA and CHB wires swapped for that motor (swap White/Blue) |
| Ticks don't return to ~0 after reverse | Motor speeds not balanced, wheel slipping, or encoder miscounting |
| No serial output | LPUART1 not configured, wrong COM port, or wrong baud rate in terminal |
| Motors don't spin | Motor drivers not powered, PWM/DIR wiring issue (same as DC motor test) |
| LD2 doesn't light up | Not expected in the current `.ioc`; PB7 is still configured as LD2 |

### Configuration

Key parameters in `encoder_test.c`:

```c
#define TEST_SPEED_PCT      25.0f   // Motor speed during test (safety cap)
#define RUN_MS              2000    // Duration per direction (ms)
#define SETTLE_MS           200     // Settle time after stopping (ms)
#define UPDATE_INTERVAL_MS  20      // Encoder sampling interval (ms)
#define DEBOUNCE_MS         200     // Button debounce delay (ms)
```

Key constants in `encoder.h`:

```c
#define ENCODER_TICKS_PER_REV   537.7f  // 28 CPR * 19.2 gear ratio
#define ENCODER_WHEEL_RADIUS_M  0.052f  // 104mm wheel / 2
```

### Peripheral Configuration

| Peripheral | Setting | Configured In |
|------------|---------|---------------|
| TIM4 | Encoder Mode TI1+TI2, CH1=PD12, CH2=PD13 | CubeMX (FL encoder) |
| TIM3 | Encoder Mode TI1+TI2, CH1=PE3, CH2=PE4 | CubeMX (FR encoder) |
| TIM8 | Encoder Mode TI1+TI2, CH1=PC6, CH2=PC7 | CubeMX (RL encoder) |
| TIM1 | Encoder Mode TI1+TI2, CH1=PE9, CH2=PE11 | CubeMX (RR encoder) |
| TIM2 | PSC=0, ARR=2366, 4ch PWM on PA0-PA3 | CubeMX (motor PWM) |
| LPUART1 | ST-LINK VCP for serial output | CubeMX |
| B1 Button | PC13, active LOW (polled) | CubeMX |
| LD3 | PB14, push-pull output (red LED) | CubeMX |

### CubeMX Timer Encoder Mode Setup

Before this test will work, you must configure each encoder timer in CubeMX:

1. Open `LitterBot.ioc` in STM32CubeIDE
2. For each timer (TIM1, TIM3, TIM4, TIM8):
   - Set **Combined Channels** → **Encoder Mode**
   - Encoder Mode: **Encoder Mode TI1 and TI2** (counts on both edges = 4x resolution)
   - Input Filter: **0** (or small value like 4 for noise rejection)
3. Verify pin assignments match the table above
4. Generate code (this creates `htim1`, `htim3`, `htim4`, `htim8` handles)

### Encoder Pin Summary

| Motor | CHA Pin | CHB Pin | Timer | Wire Colors |
|-------|---------|---------|-------|-------------|
| FL | PD12 | PD13 | TIM4 | White (CHA), Blue (CHB) |
| FR | PE3 | PE4 | TIM3 | White (CHA), Blue (CHB) |
| RL | PC6 | PC7 | TIM8 | White (CHA), Blue (CHB) |
| RR | PE9 | PE11 | TIM1 | White (CHA), Blue (CHB) |

---

## Odometry (Mecanum Wheel Dead Reckoning)

### Purpose

The odometry module estimates the robot's position (x, y) and heading (theta) from encoder data using mecanum forward kinematics. This is essential for autonomous navigation -- it tells the Jetson where the robot thinks it is relative to where it started.

### How It Works

**Step 1: Encoder ticks → wheel angular velocity**

Each encoder update provides a tick delta. Convert to rad/s:

```
w_i = delta_ticks * (2*pi / TICKS_PER_REV) / dt
```

**Step 2: Wheel speeds → body velocity (mecanum forward kinematics)**

Mecanum wheels can move in any direction. The forward kinematics are:

```
vx    = (r/4) * ( w_fl + w_fr + w_rl + w_rr)     // forward
vy    = (r/4) * (-w_fl + w_fr + w_rl - w_rr)     // lateral (left+)
omega = (r / (4*(L+W))) * (-w_fl + w_fr - w_rl + w_rr)  // rotation (CCW+)
```

Where:
- `r` = wheel radius (0.052m)
- `L` = half wheelbase (front-to-rear axle center / 2)
- `W` = half track width (left-to-right wheel center / 2)
- `w_i` = wheel angular velocity (rad/s)

**Step 3: Body velocity → pose integration (Euler method)**

```
x     += (vx*cos(theta) - vy*sin(theta)) * dt
y     += (vx*sin(theta) + vy*cos(theta)) * dt
theta += omega * dt
```

### Constants You Must Measure

The wheelbase and track width must be measured on your actual chassis:

```c
#define ODOM_WHEELBASE_M    0.300f  // MEASURE: front-to-rear axle center / 2
#define ODOM_TRACK_WIDTH_M  0.300f  // MEASURE: left-to-right wheel center / 2
```

These are currently placeholder values (0.300m). Measure the actual distances and update in `odometry.h`.

### Coordinate Frame

| Axis | Direction | Positive |
|------|-----------|----------|
| x | Forward | Robot drives forward |
| y | Left | Robot strafes left |
| theta | Rotation | Counter-clockwise from start heading |

### Limitations

- **Dead reckoning only**: Drift accumulates over time (no absolute reference)
- **Assumes no wheel slip**: Accuracy degrades on slippery surfaces
- **Euler integration**: Acceptable at 50Hz update rate, could use midpoint method for better accuracy
- **Mecanum roller slip**: Lateral motion estimates are less accurate than forward motion

### File Location

| File | Path | Purpose |
|------|------|---------|
| `odometry.h` | `Core/Inc/odometry.h` | Interface and configurable constants |
| `odometry.c` | `Core/Src/odometry.c` | Mecanum kinematics + Euler integration |

---

## UART Teleop (Remote Motor Control)

### Purpose

The UART teleop module allows you to control the robot's motors from a PC serial terminal or from the Jetson. It receives ASCII commands over LPUART1 (ST-LINK VCP) and drives the DC motors via the `cytron_md10c` driver. This is adapted from the LB_FIRM `wheels4.c` protocol.

### Command Protocol

| Command | Format | Gate | Description |
|---------|--------|------|-------------|
| Motor set | `M <fl> <fr> <rl> <rr>\n` | GATED | Set motor speeds (-1000 to +1000) |
| E-stop | `X\n` | ALWAYS | Emergency stop all motors |
| Scoop | `S SCOOP\n` | GATED | Run full scoop sequence |
| Calibrate | `C CALIBRATE\n` | GATED | Arm calibration (B1 = e-stop) |
| Deploy | `D DEPLOY\n` | STANDBY/ACTIVE | Arm homing, transition to ACTIVE |
| Dev mode | `DEV ON\n` / `DEV OFF\n` | ALWAYS | Toggle dev mode (bypass gate) |
| State query | `Q\n` | ALWAYS | Responds `STATE <state> [DEV]` |
| Encoder test | `T\n` | ALWAYS | Run encoder test (~5 sec, blocking) |

GATED commands are blocked in STANDBY unless dev mode is enabled. Send `DEV ON\n` to bypass for testing. See [[Deploy and Standby System]] for full details.

**Motor speed mapping**: The -1000..+1000 range maps to -100%..+100% duty cycle. For example, `M 250 250 250 250\n` drives all motors forward at 25%.

**Timeout safety**: If no command is received for 250ms, all motors automatically stop.

### How to Test with Serial Terminal

1. In `main.c`, uncomment the UART teleop mode lines (see comments in USER CODE sections)
2. Build and flash firmware
3. Open serial terminal (PuTTY or Tera Term) on the ST-LINK COM port
4. Baud rate settings must match LPUART1 configuration
5. Type commands and press Enter:

| Example Command | Effect |
|----------------|--------|
| `M 250 250 250 250` | Drive forward at 25% |
| `M -250 -250 -250 -250` | Drive reverse at 25% |
| `M -250 250 250 -250` | Strafe left at 25% |
| `M 250 -250 -250 250` | Strafe right at 25% |
| `M -250 250 -250 250` | Rotate CCW at 25% |
| `M 0 0 0 0` | Stop |
| `X` | Emergency stop |

### Test Location in Code

| File | Path | Purpose |
|------|------|---------|
| `uart_cmd.h` | `Core/Inc/uart_cmd.h` | Command parser interface |
| `uart_cmd.c` | `Core/Src/uart_cmd.c` | Parser implementation (adapted from LB_FIRM wheels4.c) |
| `cytron_md10c.h/.c` | `Core/Inc/` and `Core/Src/` | DC motor driver (called by parser) |
| `main.c` | `Core/Src/main.c` | Init, task loop, and UART RX callback |

### How to Run

1. Connect DC motor drivers (power, ground, PWM, DIR)
2. In `main.c`:
   - Uncomment `#include "uart_cmd.h"` in Includes
   - Uncomment `static uint8_t rx_byte;` in PV
   - Uncomment `MD10C_Init();`, `UartCmd_Init(&hlpuart1);`, and `HAL_UART_Receive_IT(...)` in USER CODE BEGIN 2
   - Uncomment `UartCmd_Task(HAL_GetTick());` in USER CODE BEGIN 3
   - Uncomment the `HAL_UART_RxCpltCallback` function in USER CODE BEGIN 4
   - Comment out all test module code
3. Build and flash
4. Open serial terminal and send commands

### Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Motors don't respond to commands | UART callback not uncommented, or `rx_byte` not declared |
| Motors stop after ~0.25 seconds | Command timeout -- must send commands continuously (or at least every 250ms) |
| Motors spin at wrong speed | Check speed mapping: 1000 = 100%, 500 = 50%, 250 = 25% |
| Garbage characters cause erratic behavior | Baud rate mismatch between terminal and LPUART1 |
| Only some motors respond | Check individual PWM/DIR wiring for non-responding motors |

### Configuration

Key parameters in `uart_cmd.c`:

```c
#define LINE_BUF_SZ         64      // Command line buffer size
#define DRIVE_TIMEOUT_MS    250     // Motor stop timeout (ms)
#define CMD_MAX             1000    // Max command value
```
